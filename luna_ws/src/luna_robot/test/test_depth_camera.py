"""Simulated OAK-D Lite (core/depth_camera.py). Needs numpy (in the ROS image; skipped on a bare Mac Python).
Run from luna_ws/src/luna_robot:  python3 -m pytest test"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import numpy as np
except ImportError:
    np = None

from luna_robot.core.dock_nav import Pose2D
from luna_robot.core.robot_model import load_frames
from luna_robot.core.sim2d import Block, Room, box

if np is not None:
    from luna_robot.core.depth_camera import DepthConfig, depth_along, pixel_rays, render

FAR_DOCK = Pose2D(8.0, 4.7, math.pi / 2)


def to_base(points):
    """Optical-frame points → base_link (robot at the world origin facing +x)."""
    t, r = load_frames().pose('oakd_optical_frame')
    return points.astype(float) @ np.array(r).T + np.array(t)


@unittest.skipIf(np is None, 'numpy not installed')
class DepthCamera(unittest.TestCase):
    def setUp(self):
        self.frames = load_frames()
        self.cfg = DepthConfig(noise_k=0.0)

    def test_rays_cover_the_field_of_view_with_unit_depth(self):
        rays = pixel_rays(self.cfg)
        self.assertEqual(rays.shape, (64 * 48, 3))
        self.assertTrue(np.all(rays[:, 2] == 1.0))
        self.assertAlmostEqual(math.degrees(2 * math.atan(rays[:, 0].max())), 73.0 * 63 / 64, delta=1.0)
        self.assertAlmostEqual(math.degrees(2 * math.atan(rays[:, 1].max())), 58.0 * 47 / 48, delta=1.0)

    def test_depth_along_hits_floor_wall_and_block(self):
        room = Room(box(-1.0, -1.0, 3.0, 1.0), FAR_DOCK, (Block('b', 1.0, -0.1, 1.2, 0.1, 0.0, 0.2),))
        dirs = np.array([[1.0, 0.0, -0.5], [1.0, 0.0, 0.0], [1.0, 0.0, 0.3], [0.0, 1.0, 0.0]])
        t = depth_along((0.0, 0.0, 0.1), dirs, room)
        self.assertAlmostEqual(t[0], 0.2)                    # floor at x = 0.2
        self.assertAlmostEqual(t[1], 1.0)                    # the block's face
        self.assertAlmostEqual(t[2], 3.0)                    # over the block to the wall (z 1.0 < 2.5)
        self.assertAlmostEqual(t[3], 1.0)                    # side wall

    def test_empty_room_is_floor_and_walls_beyond_min_z(self):
        room = Room(box(-1.0, -3.0, 6.0, 3.0), FAR_DOCK)
        pts = render(Pose2D(0.0, 0.0, 0.0), room, self.frames, self.cfg)
        self.assertGreater(len(pts), 0.9 * 64 * 48)
        self.assertGreaterEqual(pts[:, 2].min(), self.cfg.min_z_m)
        base = to_base(pts)
        low = base[base[:, 2] < 0.03]
        floor = low[np.abs(low[:, 2]) < 1e-4]                  # float32 points: µm rounding
        wall_foot = low[np.abs(low[:, 2]) >= 1e-4]
        self.assertTrue(np.all((np.abs(np.abs(wall_foot[:, 1]) - 3.0) < 1e-3) | (np.abs(wall_foot[:, 0] - 6.0) < 1e-3)))
        self.assertGreater(len(floor), len(pts) / 2)
        nearest = floor[:, 0].min()
        self.assertAlmostEqual(nearest, 0.105 + 0.365 / math.tan(math.radians(15 + 29)), delta=0.03)  # blind zone

    def test_low_toy_box_rises_out_of_the_floor(self):
        toy = Block('toy', 1.0, -0.075, 1.2, 0.075, 0.0, 0.08)
        room = Room(box(-1.0, -3.0, 6.0, 3.0), FAR_DOCK, (toy,))
        base = to_base(render(Pose2D(0.0, 0.0, 0.0), room, self.frames, self.cfg))
        on_toy = base[(base[:, 2] > 0.03) & (base[:, 2] <= 0.0801) & (base[:, 0] < 1.25)]
        self.assertGreater(len(on_toy), 5)
        self.assertTrue(np.all((on_toy[:, 0] > 0.999) & (np.abs(on_toy[:, 1]) < 0.076)))

    def test_table_top_seen_from_below_the_robot_top(self):
        top = Block('top', 1.2, -0.5, 1.7, 0.5, 0.40, 0.45)
        room = Room(box(-1.0, -3.0, 6.0, 3.0), FAR_DOCK, (top,))
        base = to_base(render(Pose2D(0.0, 0.0, 0.0), room, self.frames, self.cfg))
        hits = base[(base[:, 2] >= 0.40 - 1e-6) & (base[:, 2] <= 0.45 + 1e-6) & (base[:, 0] < 1.71)]
        self.assertGreater(len(hits), 3)
        self.assertLessEqual(hits[:, 2].max(), 0.62)          # within the costmap's obstacle band

    def test_noise_grows_with_depth(self):
        room = Room(box(-1.0, -3.0, 6.0, 3.0), FAR_DOCK)
        clean = render(Pose2D(0.0, 0.0, 0.0), room, self.frames, self.cfg)
        noisy = render(Pose2D(0.0, 0.0, 0.0), room, self.frames, DepthConfig(), np.random.default_rng(0))
        self.assertEqual(clean.shape, noisy.shape)
        err = np.abs(noisy[:, 2] - clean[:, 2]) / clean[:, 2] ** 2
        self.assertLess(np.median(err), 3 * DepthConfig().noise_k)

    def test_robot_heading_turns_the_view(self):
        toy = Block('toy', -1.2, -0.075, -1.0, 0.075, 0.0, 0.08)
        room = Room(box(-3.0, -3.0, 3.0, 3.0), FAR_DOCK, (toy,))
        ahead = to_base(render(Pose2D(0.0, 0.0, math.pi), room, self.frames, self.cfg))
        self.assertTrue(np.any((ahead[:, 2] > 0.03) & (ahead[:, 2] < 0.081) & (ahead[:, 0] < 1.25)))
        away = to_base(render(Pose2D(0.0, 0.0, 0.0), room, self.frames, self.cfg))
        self.assertFalse(np.any((away[:, 2] > 0.03) & (away[:, 2] < 0.081) & (away[:, 0] < 2.0)))


if __name__ == '__main__':
    unittest.main()
