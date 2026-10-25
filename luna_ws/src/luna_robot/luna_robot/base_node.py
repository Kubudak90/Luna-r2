"""Wheel base: velocity mux (dock > teleop/nav > attention) → safety gate → diff-drive kinematics → MCU; odometry back.

Odometry: wheel travel from the encoders; heading from the depth camera's gyro while it is fresh (its rotation into
base_link comes from TF once), from the wheel difference otherwise. The gyro bias is learnt whenever the wheels stand
still (core/kinematics.GyroBias)."""
import math
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from . import topics as T
from .entry import run
from .core import kinematics as K
from .core.frames import quat_to_rot
from .core.safety import SafetyConfig, gate

CMD_FRESH_S = 0.3
GYRO_FRESH_S = 0.1


class BaseNode(Node):
    def __init__(self):
        super().__init__('luna_base')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('turn_slip', 1.0)
        self.declare_parameter('floor_nominal_m', 0.060)
        self.declare_parameter('use_gyro', True)
        self.declare_parameter('imu_topic', T.CAMERA_GYRO)
        self.limits = K.DriveLimits(turn_slip=self.get_parameter('turn_slip').value)
        self.safety = SafetyConfig(floor_nominal_m=self.get_parameter('floor_nominal_m').value)
        self.odo = K.Odometry(self.limits)
        self.cmd = {'teleop': (Twist(), 0.0), 'dock': (Twist(), 0.0), 'attention': (Twist(), 0.0)}
        self.estop, self.bumpers, self.cliff, self.vbat = True, 0, [0.0] * 4, 0.0   # fail safe until MCU speaks
        self.create_subscription(Twist, T.CMD_VEL, lambda m: self._cmd('teleop', m), 10)
        self.create_subscription(Twist, T.DOCK_CMD_VEL, lambda m: self._cmd('dock', m), 10)
        self.create_subscription(Twist, T.ATTENTION_CMD_VEL, lambda m: self._cmd('attention', m), 10)
        self.create_subscription(Bool, T.MCU_ESTOP, lambda m: setattr(self, 'estop', m.data), 10)
        self.create_subscription(UInt8, T.MCU_BUMPERS, lambda m: setattr(self, 'bumpers', m.data), 10)
        self.create_subscription(Float32MultiArray, T.MCU_CLIFF, lambda m: setattr(self, 'cliff', list(m.data)), 10)
        self.create_subscription(BatteryState, T.MCU_BATTERY, lambda m: setattr(self, 'vbat', m.voltage), 10)
        self.create_subscription(Float32MultiArray, T.MCU_WHEELS, self._wheels, 20)
        self.gyro = K.GyroBias()
        self.wheel_speed = (0.0, 0.0)
        self.gyro_rot, self.gyro_t, self.gyro_dyaw, self.gyro_rate, self.gyro_last = None, None, 0.0, 0.0, 0.0
        if self.get_parameter('use_gyro').value:
            self.tf_buf = Buffer()
            self.tf_listener = TransformListener(self.tf_buf, self)
            self.create_subscription(Imu, self.get_parameter('imu_topic').value, self._imu, qos_profile_sensor_data)
        self.pub_target = self.create_publisher(Float32MultiArray, T.MCU_WHEEL_TARGET, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_safety = self.create_publisher(String, T.SAFETY, 10)
        self.tf = TransformBroadcaster(self)
        self.last_reason = ''
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self._tick)

    def _cmd(self, src, msg):
        self.cmd[src] = (msg, time.monotonic())

    def _source(self, now):
        for src in ('dock', 'teleop', 'attention'):
            if now - self.cmd[src][1] < CMD_FRESH_S:
                return src
        return 'teleop'                                  # stale: the gate's timeout stops the wheels

    def _tick(self):
        now = time.monotonic()
        src = self._source(now)
        msg, t = self.cmd[src]
        v = gate(msg.linear.x, msg.angular.z, estop=self.estop, bumpers=self.bumpers,
                 cliff=self.cliff, cmd_age_s=now - t, cfg=self.safety)
        left, right = K.twist_to_wheels(v.linear, v.angular, self.limits)
        self.pub_target.publish(Float32MultiArray(data=[left, right, K.pwm_cap(self.vbat)]))
        if v.reason != self.last_reason:
            self.pub_safety.publish(String(data=f'{src}:{v.reason}'))
            self.last_reason = v.reason

    def _imu(self, msg):
        if self.gyro_rot is None:
            try:
                tf = self.tf_buf.lookup_transform('base_link', msg.header.frame_id, Time())
            except TransformException:
                return                                     # robot_state_publisher / camera driver not up yet
            q = tf.transform.rotation
            self.gyro_rot = quat_to_rot(q.x, q.y, q.z, q.w)
        w = msg.angular_velocity
        rate = sum(self.gyro_rot[2][i] * c for i, c in enumerate((w.x, w.y, w.z)))   # base_link z of the rate
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.gyro_t is None else t - self.gyro_t
        self.gyro_t = t
        if not 0.0 < dt < 0.2:
            return
        self.gyro_rate = self.gyro.update(rate, *self.wheel_speed, dt)
        self.gyro_dyaw += self.gyro_rate * dt
        self.gyro_last = time.monotonic()

    def _wheels(self, msg):
        if len(msg.data) < 4:
            return
        self.wheel_speed = (msg.data[2], msg.data[3])
        fresh = time.monotonic() - self.gyro_last < GYRO_FRESH_S and self.gyro.learnt_s > 1.0
        dyaw = self.gyro_dyaw if fresh else None
        self.gyro_dyaw = 0.0
        x, y, yaw = self.odo.update(msg.data[0], msg.data[1], dyaw)
        lin, ang = K.wheels_to_twist(msg.data[2], msg.data[3], self.limits)
        if fresh:
            ang = self.gyro_rate
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
