"""2.5D simulator: replaces mock_mcu_node with a room, furniture, a LiDAR and the dock on a wall (core/sim2d.py,
rooms in core/sim_worlds.py, parameter `world`), so SLAM, Nav2, docking and obstacle courses run with no hardware.
Same MCU topics as the Teensy firmware (corner ToF at its 5 Hz), plus /scan in the 'laser' frame, the ground truth
on /luna/sim/truth (frame 'world') and every contact with an obstacle on /luna/sim/contact; both for tests only,
nothing in the robot stack may use them. /luna/sim/estop (Bool) presses the simulated E-stop."""
import math

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8

from . import topics as T
from .entry import run
from .core.dock_nav import Pose2D
from .core.sim2d import Sim2D, SimConfig
from .core.sim_worlds import world


class Sim2DNode(Node):
    def __init__(self):
        super().__init__('luna_sim2d')
        self.declare_parameter('world', 'room')
        self.declare_parameter('start_soc', 0.6)
        self.declare_parameter('start_pose', [0.0, 0.0, 0.0])     # world frame; map = world only when (0, 0, 0)
        self.declare_parameter('drain_w', 44.0)
        self.declare_parameter('true_turn_slip', SimConfig.true_turn_slip)
        self.declare_parameter('linear_slip', SimConfig.linear_slip)
        p = lambda name: self.get_parameter(name).value
        cfg = SimConfig(start_soc=p('start_soc'), drain_w=p('drain_w'), true_turn_slip=p('true_turn_slip'),
                        linear_slip=p('linear_slip'))
        self.sim = Sim2D(world(p('world')), cfg, start=Pose2D(*p('start_pose')))
        self.contacts_sent = 0
        self.create_subscription(Float32MultiArray, T.MCU_WHEEL_TARGET, self._target, 10)
        self.create_subscription(Float32MultiArray, T.MCU_HEAD_US, lambda m: None, 10)
        self.estop = False
        self.create_subscription(Bool, T.SIM_ESTOP, lambda m: setattr(self, 'estop', m.data), 10)
        self.p_wheels = self.create_publisher(Float32MultiArray, T.MCU_WHEELS, 10)
        self.p_bump = self.create_publisher(UInt8, T.MCU_BUMPERS, 10)
        self.p_cliff = self.create_publisher(Float32MultiArray, T.MCU_CLIFF, 10)
        self.p_corner = self.create_publisher(Float32MultiArray, T.MCU_CORNERS, 10)
        self.p_estop = self.create_publisher(Bool, T.MCU_ESTOP, 10)
        self.p_sense = self.create_publisher(Bool, T.MCU_DOCK_SENSE, 10)
        self.p_batt = self.create_publisher(BatteryState, T.MCU_BATTERY, 10)
        self.p_tag = self.create_publisher(PointStamped, T.DOCK_TAG, 10)
        self.p_scan = self.create_publisher(LaserScan, T.SCAN, 10)
        self.p_truth = self.create_publisher(Odometry, T.SIM_TRUTH, 10)
        self.p_contact = self.create_publisher(String, T.SIM_CONTACT, 50)
        self.dt, self.ticks = 0.02, 0
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.1, self._scan)                         # RPLIDAR C1: 10 Hz

    def _target(self, m):
        if len(m.data) >= 2:
            self.sim.target = [m.data[0], m.data[1]]

    def _tick(self):
        s = self.sim
        s.step(self.dt)
        self.ticks += 1
        self.p_wheels.publish(Float32MultiArray(data=[*s.pos, *s.vel]))
        self.p_bump.publish(UInt8(data=s.bumpers))
        self.p_cliff.publish(Float32MultiArray(data=[0.055] * 4))
        self.p_estop.publish(Bool(data=self.estop))      # like the real one, it only reports: base_node must stop
        self.p_sense.publish(Bool(data=s.docked))
        if self.ticks % 10 == 0:                         # firmware: corner ToF with the battery at 5 Hz
            self.p_corner.publish(Float32MultiArray(data=s.corners()))
        for c in s.contacts[self.contacts_sent:]:
            self.get_logger().warn(f'contact: {c.name} ({c.kind}) {"/".join(c.bands)}')
            self.p_contact.publish(String(data=f'{c.t:.2f} {c.name} {c.kind} {",".join(c.bands) or "-"}'))
        self.contacts_sent = len(s.contacts)
        if self.ticks % 10 == 0:                         # the JBD BMS answers at ~5 Hz
            self._battery()
        self._tag_and_truth()

    def _battery(self):
        s = self.sim
        b = BatteryState()
        b.voltage = 12.0 + 4.8 * s.soc
        b.percentage = s.soc
        b.current = 3.0 if s.charging else -3.0
        b.temperature = 28.0
        b.present = True
        self.p_batt.publish(b)

    def _tag_and_truth(self):
        s = self.sim
        stamp = self.get_clock().now().to_msg()
        tag = s.tag()
        if tag:
            t = PointStamped()
            t.header.stamp = stamp
            t.point.x, t.point.z = tag
            self.p_tag.publish(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'world', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = s.pose.x, s.pose.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(s.pose.yaw / 2), math.cos(s.pose.yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.angular.z = s.twist
        self.p_truth.publish(o)

    def _scan(self):
        c = self.sim.cfg
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser'
        m.angle_increment = 2 * math.pi / c.beams
        m.angle_min = -math.pi
        m.angle_max = -math.pi + (c.beams - 1) * m.angle_increment
        m.scan_time, m.time_increment = 0.1, 0.1 / c.beams
        m.range_min, m.range_max = 0.05, c.range_max
        m.ranges = self.sim.scan()
        self.p_scan.publish(m)


def main():
    run(Sim2DNode)


if __name__ == '__main__':
    main()
