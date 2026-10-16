"""Pan/tilt head: degree targets → slew-limited servo pulse widths for the MCU, plus gaze for the face."""
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from . import topics as T
from .entry import run
from .core.head import Axis, Head


class HeadNode(Node):
    def __init__(self):
        super().__init__('luna_head')
        for name, default in [('pan_center_us', 1500.0), ('tilt_center_us', 1500.0), ('pan_direction', 1), ('tilt_direction', 1)]:
            self.declare_parameter(name, default)
        p = self.get_parameter
        self.head = Head(Axis('pan', 60.0, p('pan_center_us').value, p('pan_direction').value),
                         Axis('tilt', 12.0, p('tilt_center_us').value, p('tilt_direction').value, max_rate_deg_s=45.0))
        self.create_subscription(Vector3, T.HEAD_TARGET, lambda m: self.head.set_target(m.x, m.y), 10)
        self.pub_us = self.create_publisher(Float32MultiArray, T.MCU_HEAD_US, 10)
        self.pub_gaze = self.create_publisher(Vector3, T.HEAD_GAZE, 10)
        self.dt = 0.02
        self.create_timer(self.dt, self._tick)

    def _tick(self):
        pan_us, tilt_us = self.head.step(self.dt)
        self.pub_us.publish(Float32MultiArray(data=[pan_us, tilt_us]))
        gx, gy = self.head.gaze()
        self.pub_gaze.publish(Vector3(x=gx, y=gy, z=0.0))


def main():
    run(HeadNode)


if __name__ == '__main__':
    main()
