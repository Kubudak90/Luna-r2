"""Mock Teensy: lets the whole stack run with no hardware. Same topics as the real micro-ROS firmware.

Simulates (core/mock_plant.py): wheel motion (first-order lag), the R2 dock 1.2 m ahead of the start pose with
a visible tag, dock contact + charging (off while /luna/mcu/charge_enable is false; the real MCU also drops
a false after 1 s, the mock keeps the last value) and battery drain/charge.

publish_tag: true  → fakes /luna/dock/tag itself (no cameras needed).
publish_tag: false → only publishes the true pose on /luna/mock/truth; sim_camera_node renders the tag from it
                     and the real apriltag_ros → dock_tag_node pipeline produces /luna/dock/tag."""
import math

from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32MultiArray, UInt8

from . import topics as T
from .entry import run
from .core.mock_plant import DockPlant, PlantConfig
from .core.robot_model import load_frames


class MockMcu(Node):
    def __init__(self):
        super().__init__('luna_mock_mcu')
        self.declare_parameter('start_soc', 0.6)
        self.declare_parameter('dock_distance_m', 1.2)
        self.declare_parameter('drain_w', PlantConfig.drain_w)
        self.declare_parameter('start_y_m', 0.0)
        self.declare_parameter('start_yaw_deg', 0.0)
        self.declare_parameter('publish_tag', True)
        p = self.get_parameter
        self.plant = DockPlant(PlantConfig(dock_distance_m=p('dock_distance_m').value,
                                           start_soc=p('start_soc').value, drain_w=p('drain_w').value,
                                           start_y_m=p('start_y_m').value, start_yaw_deg=p('start_yaw_deg').value))
        self.publish_tag = p('publish_tag').value
        self.create_subscription(Float32MultiArray, T.MCU_WHEEL_TARGET, self._target, 10)
        self.create_subscription(Bool, T.MCU_CHARGE_ENABLE, lambda m: setattr(self.plant, 'charge_enable', m.data), 10)
        self.p_wheels = self.create_publisher(Float32MultiArray, T.MCU_WHEELS, 10)
        self.p_bump = self.create_publisher(UInt8, T.MCU_BUMPERS, 10)
        self.p_cliff = self.create_publisher(Float32MultiArray, T.MCU_CLIFF, 10)
        self.p_corner = self.create_publisher(Float32MultiArray, T.MCU_CORNERS, 10)
        self.p_estop = self.create_publisher(Bool, T.MCU_ESTOP, 10)
        self.p_sense = self.create_publisher(Bool, T.MCU_DOCK_SENSE, 10)
        self.p_batt = self.create_publisher(BatteryState, T.MCU_BATTERY, 10)
        self.p_tag = self.create_publisher(PointStamped, T.DOCK_TAG, 10)
        self.p_truth = self.create_publisher(PoseStamped, T.MOCK_TRUTH, 10)
        self.floor_m = load_frames().origin('tof_down_fl')[2]       # flat floor under the cliff sensors
        self.dt = 0.02
        self.create_timer(self.dt, self._tick)

    def _target(self, m):
        if len(m.data) >= 2:
            self.plant.target = [m.data[0], m.data[1]]

    def _tick(self):
        s = self.plant
        s.step(self.dt)
        self.p_wheels.publish(Float32MultiArray(data=[*s.pos, *s.vel]))
        self.p_bump.publish(UInt8(data=0))
        self.p_cliff.publish(Float32MultiArray(data=[self.floor_m] * 4))
        self.p_corner.publish(Float32MultiArray(data=[2.0, 2.0, 2.0, 2.0]))   # the dock plate is under the cones
        self.p_estop.publish(Bool(data=False))
        self.p_sense.publish(Bool(data=s.docked))
        b = BatteryState()
        b.voltage = 12.4 + 1.0 * s.soc + (0.8 if s.charging else 0.0)   # LiFePO4 4S: flat, the BMS gives the SoC
        b.percentage = s.soc
        b.current = 3.0 if s.charging else -3.0
        b.temperature = 28.0
        b.present = True
        self.p_batt.publish(b)
        truth = PoseStamped()
        truth.header.stamp, truth.header.frame_id = self.get_clock().now().to_msg(), 'dock'
        truth.pose.position.x, truth.pose.position.y = s.x - s.cfg.dock_distance_m, s.y   # dock frame: seated = 0
        truth.pose.orientation.z, truth.pose.orientation.w = math.sin(s.yaw / 2), math.cos(s.yaw / 2)
        self.p_truth.publish(truth)
        tag = s.tag() if self.publish_tag else None
        if tag:                                       # tag inside the colour camera's view
            t = PointStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.point.x, t.point.z = tag
            self.p_tag.publish(t)


def main():
    run(MockMcu)


if __name__ == '__main__':
    main()
