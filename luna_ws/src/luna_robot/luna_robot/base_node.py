"""Track base: velocity mux (dock > teleop/nav) → safety gate → skid-steer kinematics → MCU; odometry back."""
import math
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8
from tf2_ros import TransformBroadcaster

from . import topics as T
from .entry import run
from .core import kinematics as K
from .core.safety import SafetyConfig, gate


class BaseNode(Node):
    def __init__(self):
        super().__init__('luna_base')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('turn_slip', 1.35)
        self.declare_parameter('floor_nominal_m', 0.055)
        self.limits = K.DriveLimits(turn_slip=self.get_parameter('turn_slip').value)
        self.safety = SafetyConfig(floor_nominal_m=self.get_parameter('floor_nominal_m').value)
        self.odo = K.Odometry(self.limits)
        self.cmd = {'teleop': (Twist(), 0.0), 'dock': (Twist(), 0.0)}
        self.estop, self.bumpers, self.cliff, self.vbat = True, 0, [0.0] * 4, 0.0   # fail safe until MCU speaks
        self.create_subscription(Twist, T.CMD_VEL, lambda m: self._cmd('teleop', m), 10)
        self.create_subscription(Twist, T.DOCK_CMD_VEL, lambda m: self._cmd('dock', m), 10)
        self.create_subscription(Bool, T.MCU_ESTOP, lambda m: setattr(self, 'estop', m.data), 10)
        self.create_subscription(UInt8, T.MCU_BUMPERS, lambda m: setattr(self, 'bumpers', m.data), 10)
        self.create_subscription(Float32MultiArray, T.MCU_CLIFF, lambda m: setattr(self, 'cliff', list(m.data)), 10)
        self.create_subscription(BatteryState, T.MCU_BATTERY, lambda m: setattr(self, 'vbat', m.voltage), 10)
        self.create_subscription(Float32MultiArray, T.MCU_WHEELS, self._wheels, 20)
        self.pub_target = self.create_publisher(Float32MultiArray, T.MCU_WHEEL_TARGET, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_safety = self.create_publisher(String, T.SAFETY, 10)
        self.tf = TransformBroadcaster(self)
        self.last_reason = ''
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self._tick)

    def _cmd(self, src, msg):
        self.cmd[src] = (msg, time.monotonic())

    def _tick(self):
        now = time.monotonic()
        dock_msg, dock_t = self.cmd['dock']
        src = 'dock' if now - dock_t < 0.3 else 'teleop'
        msg, t = self.cmd[src]
        v = gate(msg.linear.x, msg.angular.z, estop=self.estop, bumpers=self.bumpers,
                 cliff=self.cliff, cmd_age_s=now - t, cfg=self.safety)
        left, right = K.twist_to_tracks(v.linear, v.angular, self.limits)
        self.pub_target.publish(Float32MultiArray(data=[left, right, K.pwm_cap(self.vbat)]))
        if v.reason != self.last_reason:
            self.pub_safety.publish(String(data=f'{src}:{v.reason}'))
            self.last_reason = v.reason

    def _wheels(self, msg):
        if len(msg.data) < 4:
            return
        x, y, yaw = self.odo.update(msg.data[0], msg.data[1])
        lin, ang = K.tracks_to_twist(msg.data[2], msg.data[3], self.limits)
        stamp = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.angular.z = lin, ang
        self.pub_odom.publish(o)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation = o.pose.pose.orientation
        self.tf.sendTransform(tf)


def main():
    run(BaseNode)


if __name__ == '__main__':
    main()
