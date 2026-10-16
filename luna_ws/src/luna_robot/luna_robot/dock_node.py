"""Docking state machine + battery/power policy. Publishes a dock twist that the base node prefers while active."""
import time

from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Int32, String, UInt8

from . import topics as T
from .entry import run
from .core.battery import decide, soc_from_voltage
from .core.dock import Dock, Obs

ACTIVE = {'SEEK', 'ALIGN', 'APPROACH', 'CONTACT', 'BACKOFF', 'UNDOCK'}


class DockNode(Node):
    def __init__(self):
        super().__init__('luna_dock')
        self.declare_parameter('auto_dock', True)
        self.dock = Dock()
        self.obs = Obs()
        self.tag_t = 0.0
        self.batt = BatteryState()
        self.create_subscription(String, T.DOCK_REQUEST, lambda m: self.dock.request(m.data), 10)
        self.create_subscription(PointStamped, T.DOCK_TAG, self._tag, 10)
        self.create_subscription(Bool, T.MCU_DOCK_SENSE, lambda m: setattr(self.obs, 'dock_sense', m.data), 10)
        self.create_subscription(UInt8, T.MCU_BUMPERS, lambda m: setattr(self.obs, 'front_bumper', bool(m.data & 0b0011)), 10)
        self.create_subscription(BatteryState, T.MCU_BATTERY, self._battery, 10)
        self.pub_cmd = self.create_publisher(Twist, T.DOCK_CMD_VEL, 10)
        self.pub_state = self.create_publisher(String, T.DOCK_STATE, 10)
        self.pub_action = self.create_publisher(String, T.POWER_ACTION, 10)
        self.pub_mode = self.create_publisher(Int32, T.POWER_MODE_W, 10)
        self.dt = 0.05
        self.create_timer(self.dt, self._tick)

    def _tag(self, msg):
        self.obs.tag_offset, self.obs.tag_distance, self.tag_t = msg.point.x, msg.point.z, time.monotonic()

    def _battery(self, msg):
        self.batt = msg
        self.obs.charging = msg.current > 0.05      # BatteryState: positive current = charging

    def _tick(self):
        self.obs.tag_visible = time.monotonic() - self.tag_t < 0.5
        if self.batt.present:      # no power decisions before the BMS has reported once
            soc = self.batt.percentage * 100 if self.batt.percentage == self.batt.percentage and self.batt.percentage > 0 else soc_from_voltage(self.batt.voltage)
            temp = self.batt.temperature if self.batt.temperature == self.batt.temperature else None
            action, mode_w = decide(soc, charging=self.obs.charging, docked=self.dock.state in ('CHARGING', 'DOCKED', 'CONTACT'),
                                    temp_c=temp, seeking=self.dock.state in ACTIVE)
        else:
            action, mode_w = 'unknown', 15
        if self.get_parameter('auto_dock').value and action in ('seek_dock', 'critical') and self.dock.state == 'IDLE':
            self.dock.request('dock')
        lin, ang = self.dock.step(self.obs, self.dt)
        if self.dock.state in ACTIVE:
            t = Twist()
            t.linear.x, t.angular.z = lin, ang
            self.pub_cmd.publish(t)
        self.pub_state.publish(String(data=self.dock.state))
        self.pub_action.publish(String(data=action))
        self.pub_mode.publish(Int32(data=mode_w))


def main():
    run(DockNode)


if __name__ == '__main__':
    main()
