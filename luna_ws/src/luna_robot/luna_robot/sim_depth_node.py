"""Simulated Gemini 336 depth for the 2.5D world (depth:=sim): renders the sim2d room (same `world`) from the true
robot pose on /luna/sim/truth and publishes the points on /camera/depth/points in camera_depth_optical_frame, stamped
with the truth sample's time so TF places them where the robot's own estimate was then. Only for the simulator: the
robot stack still never reads the truth itself. Model and its limits: core/depth_camera.py."""
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from . import topics as T
from .entry import run
from .core.depth_camera import FRAME, DepthConfig, pixel_rays, render
from .core.dock_nav import Pose2D
from .core.robot_model import load_frames
from .core.sim_worlds import world

FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]


class SimDepthNode(Node):
    def __init__(self):
        super().__init__('luna_sim_depth')
        self.declare_parameter('world', 'room')
        self.declare_parameter('rate_hz', DepthConfig.rate_hz)
        self.declare_parameter('width', DepthConfig.width)
        self.declare_parameter('height', DepthConfig.height)
        self.declare_parameter('noise_k', DepthConfig.noise_k)
        self.declare_parameter('seed', 2)
        p = lambda name: self.get_parameter(name).value
        self.cfg = DepthConfig(width=p('width'), height=p('height'), noise_k=p('noise_k'), rate_hz=p('rate_hz'))
        self.room = world(p('world'))
        self.frames = load_frames()
        self.rays = pixel_rays(self.cfg)
        self.rng = np.random.default_rng(p('seed'))
        self.truth = None
        self.create_subscription(Odometry, T.SIM_TRUTH, lambda m: setattr(self, 'truth', m), 10)
        self.pub = self.create_publisher(PointCloud2, T.DEPTH_POINTS, 5)
        self.create_timer(1.0 / self.cfg.rate_hz, self._render)

    def _render(self):
        m = self.truth
        if m is None:
            return
        q = m.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pts = render(Pose2D(m.pose.pose.position.x, m.pose.pose.position.y, float(yaw)), self.room, self.frames,
                     self.cfg, self.rng, self.rays)
        c = PointCloud2()
        c.header.stamp, c.header.frame_id = m.header.stamp, FRAME
        c.height, c.width = 1, len(pts)
        c.fields, c.is_bigendian, c.point_step = FIELDS, False, 12
        c.row_step, c.is_dense = 12 * len(pts), True
        c.data = pts.tobytes()
        self.pub.publish(c)


def main():
    run(SimDepthNode)


if __name__ == '__main__':
    main()
