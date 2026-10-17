"""Head cameras, URDF frames and the dock tag observation. Run from luna_ws/src/luna_robot:
python3 -m unittest discover -s test"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core import kinematics as K
from luna_robot.core import tag36h11
from luna_robot.core.camera_model import B0630_720P
from luna_robot.core.dock import Dock, Obs
from luna_robot.core.dock_tag import TagFuser, TagGeometry, observe, tag_in_base
from luna_robot.core.frames import Pose, UrdfChain
from luna_robot.core.mock_plant import DockPlant, PlantConfig

try:
    import numpy
except ImportError:          # the Mac has no numpy; the ROS images do
    numpy = None

URDF = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'luna.urdf')
G = TagGeometry()


def chain():
    with open(URDF) as f:
        return UrdfChain(f.read())


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


class Frames(unittest.TestCase):
    def setUp(self):
        self.c = chain()

    def axes(self, camera, joints=None):
        p = self.c.pose('base_link', f'camera_{camera}_optical_frame', joints)
        o = p.apply((0, 0, 0))
        return o, [tuple(a - b for a, b in zip(p.apply(e), o)) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]

    def test_optical_frame_convention(self):
        _, (x, y, z) = self.axes('left')
        self.assertTrue(close(z, (1, 0, 0)), z)         # looks forward
        self.assertTrue(close(x, (0, -1, 0)), x)        # image right = robot right
        self.assertTrue(close(y, (0, 0, -1)), y)        # image down = down

    def test_cameras_at_r06_cad_positions(self):
        for cam, y in (('left', 0.062), ('right', -0.062)):
            o, _ = self.axes(cam)
            self.assertTrue(close(o, (0.034, y, 0.475)), (cam, o))

    def test_pan_left_and_tilt_down(self):
        _, (_, _, z) = self.axes('left', {'head_pan': math.radians(30)})
        self.assertAlmostEqual(math.degrees(math.atan2(z[1], z[0])), 30.0)      # + pan = left, URDF +z
        _, (_, _, z) = self.axes('left', {'head_tilt': math.radians(10)})
        self.assertAlmostEqual(math.degrees(math.atan2(-z[2], z[0])), 10.0)     # + tilt = nose down, URDF +y

    def test_pose_inverse_and_compose(self):
        a = Pose.planar(0.3, -0.2, 0.7, z=0.1)
        p = (0.5, 0.25, -1.0)
        self.assertTrue(close((a.inverse() * a).apply(p), p))
        self.assertTrue(close(self.c.pose('camera_left_link', 'base_link').apply(self.c.pose('base_link', 'camera_left_link').apply(p)), p))


class TagPattern(unittest.TestCase):
    def test_layout(self):
        for tag_id in tag36h11.CODES:
            im = tag36h11.bitmap(tag_id)
            ring = [im[0][i] for i in range(10)] + [im[9][i] for i in range(10)] + [im[i][0] for i in range(10)] + [im[i][9] for i in range(10)]
            border = [im[1][i] for i in range(1, 9)] + [im[8][i] for i in range(1, 9)] + [im[i][1] for i in range(1, 9)] + [im[i][8] for i in range(1, 9)]
            self.assertEqual(set(ring), {1})
            self.assertEqual(set(border), {0})
            self.assertEqual(sum(map(sum, (r[2:8] for r in im[2:8]))), bin(tag36h11.CODES[tag_id]).count('1'))

    def test_bit_layout_covers_the_data_square_once(self):
        self.assertEqual(sorted(zip(tag36h11.BIT_X, tag36h11.BIT_Y)), sorted((x, y) for x in range(1, 7) for y in range(1, 7)))

    def test_printed_size_fits_the_plate(self):
        printed = G.size_m / tag36h11.BLACK_WIDTH * tag36h11.TOTAL_WIDTH
        self.assertAlmostEqual(printed, 0.060)
        self.assertLessEqual(printed, 0.064 - 2 * 0.002 + 1e-9)            # 2 mm cutting margin on REF_R02_TAG


class Observation(unittest.TestCase):
    def test_seated(self):
        off, dist = observe(tag_in_base(0, 0, 0))
        self.assertAlmostEqual(off, 0.0)
        self.assertAlmostEqual(dist, 0.371 - 0.170 - 0.217)                  # nose tip 16 mm inside the mouth

    def test_distance_ahead_of_the_dock(self):
        for d in (0.05, 0.3, 1.2, 2.0):
            self.assertAlmostEqual(observe(tag_in_base(-d, 0, 0))[1], d - 0.016)

    def test_offset_sign_matches_mock_and_dock(self):
        # tag on the robot's right: positive offset, and ALIGN turns right (negative angular rate)
        for y, yaw in ((0.15, 0.0), (0.0, 0.3), (-0.1, -0.2)):
            plant = DockPlant(PlantConfig(dock_distance_m=1.0))
            plant.y, plant.yaw, plant.dist = y, yaw, 1.0
            mock_off, _ = plant.tag()
            off, _ = observe(tag_in_base(-1.0, y, yaw))
            self.assertEqual(math.copysign(1, off), math.copysign(1, mock_off), (y, yaw))
        d = Dock()
        d.state = 'ALIGN'
        _, ang = d.step(Obs(tag_visible=True, tag_offset=observe(tag_in_base(-1.0, 0.2, 0))[0]), 0.05)
        self.assertLess(ang, 0)

    def test_offset_scale_and_clamp(self):
        x = 1.0
        off, _ = observe((x, -x * math.tan(math.radians(17.5)), G.height_m))
        self.assertAlmostEqual(off, 0.5)
        self.assertEqual(observe((0.1, -1.0, G.height_m))[0], 1.0)            # far right, beyond 35°: clamped

    def test_rejects_what_cannot_be_the_dock_tag(self):
        self.assertIsNone(observe((1.0, 0.0, G.height_m + 0.15)))            # wrong height: another tag or bad pose
        self.assertIsNone(observe((0.1, 0.0, G.height_m)))                   # nearer than the seated pose allows
        self.assertIsNone(observe((4.0, 0.0, G.height_m)))
        self.assertIsNone(observe((float('nan'), 0.0, G.height_m)))


class HeadPanCompensation(unittest.TestCase):
    """The tag position goes camera optical frame → base_link through the URDF chain (what TF does in the node):
    the tag's place in the image moves with the head, the published observation does not."""

    def setUp(self):
        self.c = chain()

    def seen_from(self, camera, pan, p_base):
        joints = {'head_pan': math.radians(pan)}
        p_opt = self.c.pose(f'camera_{camera}_optical_frame', 'base_link', joints).apply(p_base)
        return p_opt, self.c.pose('base_link', f'camera_{camera}_optical_frame', joints).apply(p_opt)

    def test_observation_independent_of_head_pan(self):
        p_base = tag_in_base(-0.8, 0.12, 0.15)
        ref = observe(p_base)
        us = []
        for pan in (-30, -10, 0, 20):
            p_opt, back = self.seen_from('left', pan, p_base)
            self.assertTrue(B0630_720P.sees(p_opt, 0.030), pan)
            us.append(B0630_720P.project(p_opt)[0])
            self.assertTrue(close(observe(back), ref, 1e-9), pan)
        self.assertEqual(us, sorted(us))       # head turns left → tag moves right in the image

    def test_tag_leaves_the_image_when_the_head_is_turned_too_far(self):
        p_opt, _ = self.seen_from('left', 60, tag_in_base(-0.8, 0.0, 0.0))
        self.assertFalse(B0630_720P.sees(p_opt, 0.030))


class Fusion(unittest.TestCase):
    def test_average_of_fresh_cameras(self):
        f = TagFuser()
        f.add('left', 10.0, (1.0, 0.02, 0.38))
        f.add('right', 10.05, (1.02, 0.00, 0.384))
        self.assertTrue(close(f.position(10.05), (1.01, 0.01, 0.382), 1e-12))

    def test_stale_and_implausible_are_ignored(self):
        f = TagFuser(max_age_s=0.3)
        f.add('left', 9.0, (1.0, 0.0, 0.38))
        self.assertIsNone(f.observation(10.0))
        f.add('right', 10.0, (1.0, 0.0, 0.70))
        self.assertIsNone(f.observation(10.0))
        f.add('left', 10.0, (1.2, 0.0, 0.38))
        self.assertAlmostEqual(f.observation(10.0)[1], math.hypot(1.2 - 0.17, 0) - 0.217)

    def test_disagreeing_cameras_keep_the_one_at_tag_height(self):
        f = TagFuser(max_spread_m=0.08)
        f.add('left', 10.0, (1.0, 0.0, 0.39))
        f.add('right', 10.0, (1.3, 0.0, 0.45))
        self.assertEqual(f.position(10.0), (1.0, 0.0, 0.39))


class DockingThroughTheCameras(unittest.TestCase):
    """Dock state machine closed around the mock plant, but the tag reaches it the way the real one will: tag
    position in base_link → each head camera (URDF chain, head pan held) → visible only if it lands in the
    B0630 720p image → back to base_link → fuser → observe."""

    def run_dock(self, pan_deg, seconds=45, **start):
        c = chain()
        joints = {'head_pan': math.radians(pan_deg)}
        cams = {cam: (c.pose(f'camera_{cam}_optical_frame', 'base_link', joints),
                      c.pose('base_link', f'camera_{cam}_optical_frame', joints)) for cam in ('left', 'right')}
        plant, dock, fuser = DockPlant(PlantConfig(dock_distance_m=1.2, **start)), Dock(), TagFuser()
        dock.request('dock')
        states, seen, next_dock = [], set(), 0.0
        for i in range(int(seconds / 0.02)):
            t = i * 0.02
            if t >= next_dock - 1e-9:
                p_base = tag_in_base(plant.x - 1.2, plant.y, plant.yaw)
                for cam, (opt_T_base, base_T_opt) in cams.items():
                    p_opt = opt_T_base.apply(p_base)
                    if B0630_720P.sees(p_opt, 0.030):
                        fuser.add(cam, t, base_T_opt.apply(p_opt))
                        seen.add(cam)
                tag = fuser.observation(t)
                obs = Obs(tag_visible=tag is not None, dock_sense=plant.docked, charging=plant.charging)
                if tag:
                    obs.tag_offset, obs.tag_distance = tag
                lin, ang = dock.step(obs, 0.05)
                plant.target = list(K.twist_to_tracks(lin, ang))
                states.append(dock.state)
                next_dock += 0.05
            plant.step(0.02)
        return [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]], seen

    def test_docks_with_head_straight_and_turned(self):
        for pan in (0, 35, -35):
            with self.subTest(pan=pan):
                path, seen = self.run_dock(pan)
                self.assertEqual(path, ['ALIGN', 'APPROACH', 'CONTACT', 'CHARGING'], pan)
                self.assertEqual(seen, {'left', 'right'})

    def test_head_turned_past_the_fov_loses_the_tag(self):
        # 41° half FOV: lined up with the tag, a head turned 45° has it outside both images (README limit ±35°)
        path, _ = self.run_dock(45, seconds=60)
        self.assertNotIn('CHARGING', path)

    def test_turned_start_aligns_first(self):
        path, _ = self.run_dock(0, start_yaw_deg=20)          # tag 20° to the right at the start
        self.assertEqual(path, ['ALIGN', 'APPROACH', 'CONTACT', 'CHARGING'])

    def test_start_beside_the_dock_axis_is_not_docked_by_bearing_alone(self):
        # Known limit of the current state machine, not of the tag pipeline: it steers at the tag, never onto
        # the dock axis, so a robot starting 15 cm to the side arrives at an angle, misses the 2 cm mouth
        # tolerance of the mock and ends in FAULT after its retries.
        path, _ = self.run_dock(0, seconds=150, start_y_m=0.15)
        self.assertNotIn('CHARGING', path)


@unittest.skipUnless(numpy, 'numpy not installed')
class SyntheticImage(unittest.TestCase):
    def test_tag_lands_where_the_pinhole_puts_it(self):
        from luna_robot.core.tag_scene import dock_T_tag, render
        c = chain()
        base_T_opt = c.pose('base_link', 'camera_left_optical_frame')
        dock_T_base = Pose.planar(-0.6, 0.05, 0.1)
        opt_T_tag = (dock_T_base * base_T_opt).inverse() * dock_T_tag()
        img = render(B0630_720P, opt_T_tag, noise=0.0)
        u, v = B0630_720P.project(opt_T_tag.apply((0, 0, 0)))
        dark = numpy.argwhere(img < 75)
        self.assertGreater(len(dark), 100)
        cv, cu = dark.mean(axis=0)
        self.assertLess(math.hypot(cu - u, cv - v), 6.0)        # black cells and plate centred on the projection
        self.assertTrue(close(opt_T_tag.apply((0, 0, 0)), c.pose('camera_left_optical_frame', 'base_link').apply(
            tag_in_base(-0.6, 0.05, 0.1)), 1e-9))


if __name__ == '__main__':
    unittest.main()
