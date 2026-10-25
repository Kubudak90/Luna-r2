"""Colour camera, URDF frames and the dock tag observation. Run from luna_ws/src/luna_robot:
python3 -m unittest discover -s test"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core import kinematics as K
from luna_robot.core import tag36h11
from luna_robot.core.camera_model import GEMINI_COLOR_720P as CAM
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
OPT = 'camera_color_optical_frame'


def chain():
    with open(URDF) as f:
        return UrdfChain(f.read())


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


class Frames(unittest.TestCase):
    def setUp(self):
        self.c = chain()

    def axes(self, frame):
        p = self.c.pose('base_link', frame)
        o = p.apply((0, 0, 0))
        return o, [tuple(a - b for a, b in zip(p.apply(e), o)) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]

    def test_optical_frame_convention(self):
        _, (x, y, z) = self.axes(OPT)
        t = math.radians(15)
        self.assertTrue(close(z, (math.cos(t), 0, -math.sin(t)), 1e-4), z)   # looks forward, 15° nose down
        self.assertTrue(close(x, (0, -1, 0)), x)                          # image right = robot right
        self.assertTrue(close(y, (-math.sin(t), 0, -math.cos(t)), 1e-4), y)  # image down = (tilted) down

    def test_cameras_at_r2_cad_positions(self):
        o, _ = self.axes(OPT)
        self.assertTrue(close(o, (0.165, 0.008, 0.270)), o)              # Gemini 336 colour lens, 8 mm left of centre
        o, (_, _, z) = self.axes('face_camera_optical_frame')
        self.assertTrue(close(o, (0.072, 0.0, 0.505)), o)                # Pi camera over the screen
        self.assertAlmostEqual(math.degrees(math.atan2(z[2], z[0])), 12.0, places=2)   # tilted up to the children's faces

    def test_pose_inverse_and_compose(self):
        a = Pose.planar(0.3, -0.2, 0.7, z=0.1)
        p = (0.5, 0.25, -1.0)
        self.assertTrue(close((a.inverse() * a).apply(p), p))
        self.assertTrue(close(self.c.pose(OPT, 'base_link').apply(self.c.pose('base_link', OPT).apply(p)), p))


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
        self.assertAlmostEqual(printed, 0.100)
        self.assertLessEqual(printed, G.plate_m - 2 * 0.002 + 1e-9)          # 2 mm cutting margin on the mast plate


class Observation(unittest.TestCase):
    def test_seated(self):
        off, dist = observe(tag_in_base(0, 0, 0))
        self.assertAlmostEqual(off, 0.0)
        self.assertAlmostEqual(dist, 0.0)                                    # wheels against their stops

    def test_distance_ahead_of_the_dock(self):
        for d in (0.05, 0.3, 1.2, 2.0):
            self.assertAlmostEqual(observe(tag_in_base(-d, 0, 0))[1], d)

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
        self.assertIsNone(observe((0.2, 0.0, G.height_m)))                   # nearer than the seated pose allows
        self.assertIsNone(observe((4.5, 0.0, G.height_m)))
        self.assertIsNone(observe((float('nan'), 0.0, G.height_m)))


class ColourCameraView(unittest.TestCase):
    """The tag position goes base_link → colour optical frame through the URDF (what TF does in the node), and must
    land fully inside the 1280 × 720 image all the way from the pre-dock pose to the seat: the body carries the
    camera, so ALIGN turning the body is what brings the tag to the image centre."""

    def setUp(self):
        c = chain()
        self.opt_T_base, self.base_T_opt = c.pose(OPT, 'base_link'), c.pose('base_link', OPT)

    def test_in_view_from_far_away_down_to_the_seat(self):
        vs = []
        for d in (2.5, 2.0, 1.2, 0.6, 0.3, 0.1, 0.0):
            p_opt = self.opt_T_base.apply(tag_in_base(-d, 0, 0))
            self.assertTrue(CAM.sees(p_opt, G.plate_m / 2), d)
            vs.append(CAM.project(p_opt)[1])
            self.assertTrue(close(observe(self.base_T_opt.apply(p_opt)), observe(tag_in_base(-d, 0, 0))), d)
        self.assertEqual(vs, sorted(vs))           # driving in, the tag (below the lens) climbs down the image

    def test_tag_leaves_the_image_when_the_body_is_turned_too_far(self):
        # 43° half FOV, seen from the lens 0.165 m ahead of the turning centre: at the 0.6 m pre-dock pose the body
        # may be turned 30° from the tag, at 37° the tag is out of the image. The mock agrees (40° from the lens).
        for yaw, seen in ((30, True), (-30, True), (37, False), (-37, False)):
            p_base = tag_in_base(-0.6, 0.0, math.radians(yaw))
            self.assertEqual(CAM.sees(self.opt_T_base.apply(p_base), G.plate_m / 2), seen, yaw)
            plant = DockPlant(PlantConfig(dock_distance_m=0.6))
            plant.yaw = math.radians(yaw)
            self.assertEqual(plant.tag() is not None, seen, yaw)


class Fusion(unittest.TestCase):
    def test_average_of_fresh_cameras(self):
        f = TagFuser()
        f.add('color', 10.0, (1.0, 0.02, 0.25))
        f.add('second', 10.05, (1.02, 0.00, 0.254))
        self.assertTrue(close(f.position(10.05), (1.01, 0.01, 0.252), 1e-12))

    def test_stale_and_implausible_are_ignored(self):
        f = TagFuser(max_age_s=0.3)
        f.add('color', 9.0, (1.0, 0.0, 0.25))
        self.assertIsNone(f.observation(10.0))
        f.add('second', 10.0, (1.0, 0.0, 0.40))
        self.assertIsNone(f.observation(10.0))
        f.add('color', 10.0, (1.2, 0.0, 0.25))
        self.assertAlmostEqual(f.observation(10.0)[1], 1.2 - G.seat_m)

    def test_disagreeing_cameras_keep_the_one_at_tag_height(self):
        f = TagFuser(max_spread_m=0.08)
        f.add('color', 10.0, (1.0, 0.0, 0.26))
        f.add('second', 10.0, (1.3, 0.0, 0.32))
        self.assertEqual(f.position(10.0), (1.0, 0.0, 0.26))


class DockingThroughTheCamera(unittest.TestCase):
    """Dock state machine closed around the mock plant, but the tag reaches it the way the real one will: tag
    position in base_link → colour optical frame (URDF) → visible only if it lands in the Gemini 720p image → back
    to base_link → fuser → observe."""

    def run_dock(self, seconds=45, **start):
        c = chain()
        opt_T_base, base_T_opt = c.pose(OPT, 'base_link'), c.pose('base_link', OPT)
        plant, dock, fuser = DockPlant(PlantConfig(dock_distance_m=1.2, **start)), Dock(), TagFuser()
        dock.request('dock')
        states, next_dock = [], 0.0
        for i in range(int(seconds / 0.02)):
            t = i * 0.02
            if t >= next_dock - 1e-9:
                p_opt = opt_T_base.apply(tag_in_base(plant.x - 1.2, plant.y, plant.yaw))
                if CAM.sees(p_opt, G.plate_m / 2):
                    fuser.add('color', t, base_T_opt.apply(p_opt))
                tag = fuser.observation(t)
                obs = Obs(tag_visible=tag is not None, dock_sense=plant.docked, charging=plant.charging)
                if tag:
                    obs.tag_offset, obs.tag_distance = tag
                lin, ang = dock.step(obs, 0.05)
                plant.target = list(K.twist_to_wheels(lin, ang))
                states.append(dock.state)
                next_dock += 0.05
            plant.step(0.02)
        return [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]]

    def test_docks_straight_and_turned(self):
        for yaw in (0, 20, -30):
            with self.subTest(yaw=yaw):
                self.assertEqual(self.run_dock(start_yaw_deg=yaw), ['ALIGN', 'APPROACH', 'CONTACT', 'CHARGING'])

    def test_tag_out_of_view_at_the_start_is_found_by_seeking(self):
        self.assertEqual(self.run_dock(seconds=60, start_yaw_deg=50), ['SEEK', 'ALIGN', 'APPROACH', 'CONTACT', 'CHARGING'])

    def test_start_beside_the_dock_axis(self):
        # Steering at the tag 0.45 m beyond the seat, a robot starting 1.2 m out and y0 beside the axis reaches the
        # seat about 0.45 / 1.65 ≈ 0.27·y0 off (0.43·y0 from the 0.6 m pre-dock pose, hence dock_nav's handover
        # cone). The wheel funnel takes 4.5 cm: 15 cm still docks, 20 cm ends in FAULT after the retries.
        self.assertEqual(self.run_dock(seconds=60, start_y_m=0.15), ['ALIGN', 'APPROACH', 'CONTACT', 'CHARGING'])
        self.assertEqual(self.run_dock(seconds=150, start_y_m=0.20)[-1], 'FAULT')


@unittest.skipUnless(numpy, 'numpy not installed')
class SyntheticImage(unittest.TestCase):
    def test_tag_lands_where_the_pinhole_puts_it(self):
        from luna_robot.core.tag_scene import dock_T_tag, render
        c = chain()
        base_T_opt = c.pose('base_link', OPT)
        dock_T_base = Pose.planar(-0.6, 0.05, 0.1)
        opt_T_tag = (dock_T_base * base_T_opt).inverse() * dock_T_tag()
        img = render(CAM, opt_T_tag, noise=0.0)
        u, v = CAM.project(opt_T_tag.apply((0, 0, 0)))
        dark = numpy.argwhere(img < 75)
        self.assertGreater(len(dark), 100)
        cv, cu = dark.mean(axis=0)
        self.assertLess(math.hypot(cu - u, cv - v), 6.0)        # black cells and plate centred on the projection
        self.assertTrue(close(opt_T_tag.apply((0, 0, 0)), c.pose(OPT, 'base_link').apply(
            tag_in_base(-0.6, 0.05, 0.1)), 1e-9))


if __name__ == '__main__':
    unittest.main()
