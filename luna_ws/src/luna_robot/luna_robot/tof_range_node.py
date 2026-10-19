"""Corner ToF for Nav2: /luna/mcu/corners [FL, FR, BL, BR] (raw VL53L1X metres from the Teensy, the mock or sim2d)
→ per sensor a sensor_msgs/Range on /luna/tof/{fl,fr,bl,br} (collision_monitor) and the same reading as an arc of
points on /luna/tof/<name>/points (costmaps), both in the urdf tof_* frame. A reading at or beyond max_m is sent as
range = max_range ("nothing nearer": the arc then only clears); an invalid one (≤ 0, NaN) is not sent (core/tof.py).
Stamp: arrival − age_s. The reading is older than its arrival (50 ms timing budget, read at 5 Hz, micro-ROS), and
the TF lookups at the stamp then do not ask for odometry that has not been published yet."""
import math
import struct

from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Range
from std_msgs.msg import Float32MultiArray

from . import topics as T
from .entry import run
from .core.tof import FRAMES, TofConfig, arc_points, to_range

FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]


class TofRangeNode(Node):
    def __init__(self):
        super().__init__('luna_tof_range')
        self.declare_parameter('max_m', TofConfig.max_m)
        self.declare_parameter('min_m', TofConfig.min_m)
        self.declare_parameter('fov_deg', 27.0)
        self.declare_parameter('age_s', 0.1)
        p = lambda name: self.get_parameter(name).value
        self.cfg = TofConfig(fov_rad=math.radians(p('fov_deg')), min_m=p('min_m'), max_m=p('max_m'))
        self.age = Duration(seconds=p('age_s'))
        self.pubs = [self.create_publisher(Range, topic, 10) for topic in T.TOF_RANGES]
        self.cloud_pubs = [self.create_publisher(PointCloud2, topic, 10) for topic in T.TOF_POINTS]
        self.create_subscription(Float32MultiArray, T.MCU_CORNERS, self._corners, 10)

    def _corners(self, m):
        stamp = (self.get_clock().now() - self.age).to_msg()
        for pub, cloud_pub, frame, raw in zip(self.pubs, self.cloud_pubs, FRAMES, m.data):
            r = to_range(raw, self.cfg)
            if r is None:
                continue
            msg = Range()
            msg.header.stamp, msg.header.frame_id = stamp, frame
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = self.cfg.fov_rad
            msg.min_range, msg.max_range, msg.range = self.cfg.min_m, self.cfg.max_m, r
            pub.publish(msg)
            pts = arc_points(r, self.cfg)
            c = PointCloud2()
            c.header = msg.header
            c.height, c.width, c.fields, c.is_bigendian = 1, len(pts), FIELDS, False
            c.point_step, c.row_step, c.is_dense = 12, 12 * len(pts), True
            c.data = struct.pack(f'<{3 * len(pts)}f', *(v for p in pts for v in p))
            cloud_pub.publish(c)


def main():
    run(TofRangeNode)


if __name__ == '__main__':
    main()
