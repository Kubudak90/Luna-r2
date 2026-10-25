"""Obstacles below the LiDAR plane: the 2.5D simulator (core/sim2d.py, core/sim_worlds.py), sensor poses from the
URDF (core/robot_model.py) and the corner ToF → Range conversion (core/tof.py). Pure Python.
Run from luna_ws/src/luna_robot:  python3 -m pytest test   (or python3 -m unittest discover -s test)"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core import kinematics as K
from luna_robot.core.dock_nav import Pose2D, pre_dock_pose
from luna_robot.core.frames import Pose, rot_rpy
from luna_robot.core.robot_model import Frames, drop_links, load_frames
from luna_robot.core.sim2d import (BUMPER_REACH, BUMPER_TRAVEL, C1_BASE_Z, LASER_X, LASER_YAW, LASER_Z, ROBOT_RADIUS,
                                   Block, Cylinder, Room, Sim2D, SimConfig, beam_toward, box, clearance, default_room,
                                   ray_enters, raycast, raycast3d, robot_bands, slice_at)
from luna_robot.core.sim_worlds import WORLDS, world
from luna_robot.core.tof import FRAMES, TofConfig, arc_points, to_range

OPEN = box(-5.0, -5.0, 9.0, 5.0)            # walls far away from everything below
FAR_DOCK = Pose2D(8.0, 4.7, math.pi / 2)
QUIET = SimConfig(range_noise_m=0.0, tof_noise_m=0.0)


def room_with(*obstacles):
    return Room(OPEN, FAR_DOCK, obstacles)


def drive(sim, v, w, seconds, dt=0.02):
    sim.target = list(K.twist_to_wheels(v, w))
    for _ in range(round(seconds / dt)):
        sim.step(dt)


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


class RobotModel(unittest.TestCase):
    def test_rpy_is_fixed_axis_roll_pitch_yaw(self):
        self.assertTrue(close(Pose(rot_rpy(0, 0, math.pi / 2)).apply((1, 0, 0)), (0, 1, 0)))
        self.assertTrue(close(Pose(rot_rpy(0, math.pi / 2, 0)).apply((1, 0, 0)), (0, 0, -1)))   # +pitch: nose down
        self.assertTrue(close(Pose(rot_rpy(math.pi / 2, 0, 0)).apply((0, 1, 0)), (0, 0, 1)))

    def test_chain_composes_parent_after_child(self):
        f = Frames('<robot name="t"><link name="base_link"/><link name="a"/><link name="b"/>'
                   '<joint name="j1" type="fixed"><parent link="base_link"/><child link="a"/>'
                   '<origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/></joint>'
                   '<joint name="j2" type="fixed"><parent link="a"/><child link="b"/><origin xyz="2 0 0"/></joint>'
                   '</robot>')
        self.assertTrue(close(f.origin('b'), (1.0, 2.0, 0.0)))
        self.assertAlmostEqual(f.yaw('b'), math.pi / 2)
        with self.assertRaises(KeyError):
            f.pose('nowhere')

    def test_laser_comes_from_the_urdf(self):
        f = load_frames()
        x, y, z = f.origin('laser')
        self.assertEqual((LASER_X, LASER_Z, LASER_YAW), (x, z, f.yaw('laser')))
        self.assertEqual((x, y), (0.0, 0.0))             # on the turning axis: the scan turns about its own origin
        self.assertAlmostEqual(z, C1_BASE_Z + 0.0298)    # C1 datasheet: laser plane 29.8 mm above its base
        self.assertAlmostEqual(z, 0.5498)
        self.assertAlmostEqual(LASER_YAW, -math.pi / 2, places=4)   # sllidar_ros2: +x to the cable exit (robot right)
        self.assertAlmostEqual(max(b.z1 for b in robot_bands()), 0.5613)   # R2 height 561.3 mm, top of the C1

    def test_r2_frames_are_all_there(self):
        f = load_frames()
        for name in ('camera_link', 'camera_color_optical_frame', 'camera_depth_optical_frame', 'camera_imu_frame',
                     'face_camera_optical_frame', 'face_display', 'mic_array', 'dock_contact',
                     'tof_down_fl', 'tof_down_fr', 'tof_down_bl', 'tof_down_br'):
            with self.subTest(frame=name):
                f.origin(name)
        for name in ('head_pan_link', 'camera_left_link', 'oakd_link'):     # R1 parts that R2 does not have
            with self.subTest(frame=name), self.assertRaises(KeyError):
                f.origin(name)
        t, r = f.pose('tof_down_fl')
        self.assertTrue(close((r[0][0], r[1][0], r[2][0]), (0.0, 0.0, -1.0), 1e-4))   # floor ToF look down

    def test_drop_links_removes_the_frames_a_driver_publishes(self):
        urdf = open(os.path.join(os.path.dirname(__file__), '..', 'urdf', 'luna.urdf')).read()
        names = ('camera_color_optical_frame', 'camera_depth_optical_frame')    # OrbbecSDK_ROS2 publishes these
        f = Frames(drop_links(urdf, names))
        for name in names:
            with self.assertRaises(KeyError):
                f.pose(name)
        self.assertEqual(f.origin('camera_link'), load_frames().origin('camera_link'))
        self.assertEqual(f.origin('laser'), load_frames().origin('laser'))

    def test_tof_frames_look_out_of_the_ring_at_30_deg(self):
        f = load_frames()
        for name, sx, sy in (('tof_fl', 1, 1), ('tof_fr', 1, -1), ('tof_bl', -1, 1), ('tof_br', -1, -1)):
            with self.subTest(frame=name):
                x, y, z = f.origin(name)
                self.assertEqual((math.copysign(1, x), math.copysign(1, y)), (sx, sy))   # tof_fl on Luna's left
                self.assertAlmostEqual(math.hypot(x, y), 0.175, places=3)               # windows in the shell, R175
                self.assertAlmostEqual(z, 0.100)                                       # just under the ring top
                want = math.radians(30) if sx > 0 else math.radians(150)
                self.assertAlmostEqual(f.yaw(name), sy * want, places=4)               # 30° off the axis, outwards
        self.assertEqual(FRAMES, ('tof_fl', 'tof_fr', 'tof_bl', 'tof_br'))

    def test_depth_camera_optical_axis_points_forward_and_15_deg_down(self):
        t, r = load_frames().pose('camera_depth_optical_frame')
        z_axis = (r[0][2], r[1][2], r[2][2])
        x_axis = (r[0][0], r[1][0], r[2][0])
        self.assertAlmostEqual(math.degrees(math.atan2(-z_axis[2], z_axis[0])), 15.0, places=2)   # 4-digit rpy
        self.assertTrue(close(x_axis, (0.0, -1.0, 0.0), 1e-4))     # image x = robot's right
        self.assertGreater(t[0], ROBOT_RADIUS - 0.02)        # behind the torso window, inside the outline
        self.assertLess(t[0], ROBOT_RADIUS)
        self.assertLess(t[2], 0.345)                         # in the torso, below the neck


class Rays(unittest.TestCase):
    def test_raycast_hits_a_circle(self):
        self.assertAlmostEqual(raycast(0.0, 0.0, 0.0, (), 10.0, circles=((2.0, 0.0, 0.5),)), 1.5)
        self.assertEqual(raycast(0.0, 0.0, math.pi, (), 10.0, circles=((2.0, 0.0, 0.5),)), math.inf)

    def test_ray_enters_blocks_and_cylinders_in_3d(self):
        top = Block('top', 1.0, -0.5, 1.5, 0.5, 0.40, 0.45)
        self.assertAlmostEqual(ray_enters(top, (0.0, 0.0, 0.42), (1.0, 0.0, 0.0)), 1.0)
        self.assertEqual(ray_enters(top, (0.0, 0.0, LASER_Z), (1.0, 0.0, 0.0)), math.inf)   # the scan plane passes over
        d = (math.cos(0.2), 0.0, math.sin(0.2))
        self.assertAlmostEqual(ray_enters(top, (0.6, 0.0, 0.30), d), (0.40 - 0.30) / math.sin(0.2), places=9)  # from below
        leg = Cylinder('leg', 2.0, 0.0, 0.02, 0.0, 0.40)
        self.assertAlmostEqual(ray_enters(leg, (0.0, 0.0, 0.1), (1.0, 0.0, 0.0)), 1.98)
        self.assertEqual(ray_enters(leg, (0.0, 0.0, 0.5), (1.0, 0.0, 0.0)), math.inf)

    def test_raycast3d_sees_the_floor_and_walls_only_up_to_their_height(self):
        room = Room(box(-1.0, -1.0, 3.0, 1.0), FAR_DOCK)
        d = (math.cos(math.radians(-10)), 0.0, math.sin(math.radians(-10)))
        self.assertAlmostEqual(raycast3d((0.0, 0.0, 0.1), d, room, 10.0), 0.1 / math.sin(math.radians(10)))
        up = (math.cos(math.radians(60)), 0.0, math.sin(math.radians(60)))
        self.assertEqual(raycast3d((0.0, 0.0, 0.1), up, room, 10.0), 10.0)   # over the 2.5 m wall at 3 m

    def test_lidar_plane_only_meets_what_spans_it(self):
        segs, circles = slice_at(world('table'), LASER_Z)
        self.assertEqual(len(segs), len(world('table').walls))                 # table: nothing in the scan plane
        self.assertEqual(circles, ())
        segs, circles = slice_at(world('chair'), LASER_Z)
        self.assertEqual(len(segs), len(world('chair').walls) + 4)             # the chair back
        self.assertEqual(len(circles), 2)                                      # its two tall legs

    def test_scan_goes_under_the_table(self):
        sim = Sim2D(world('table'), QUIET)
        scan = sim.scan()
        self.assertAlmostEqual(scan[beam_toward(0.0, len(scan))], 5.0 - LASER_X, places=6)  # to the far wall

    def test_dock_clutter_leaves_the_scan_of_the_mapped_room_unchanged(self):
        self.assertEqual(Sim2D(default_room()).scan(), Sim2D(world('dock')).scan())


class CornerTof(unittest.TestCase):
    def test_open_floor_reads_beyond_the_usable_range(self):
        r = Sim2D(room_with(), QUIET).corners()
        for v in r:
            self.assertGreater(v, TofConfig().max_m)         # floor at ≈ 0.77 m with the 16×4 ROI
            self.assertLess(v, 0.80)
            self.assertEqual(to_range(v), TofConfig().max_m)

    def test_full_roi_would_see_the_floor_at_0_43_m(self):
        r = Sim2D(room_with(), SimConfig(tof_noise_m=0.0, tof_fov_v=math.radians(27))).corners()
        for v in r:
            self.assertAlmostEqual(v, 0.100 / math.sin(math.radians(13.5)), delta=0.01)

    def nearest_in_cone(self, face_x, frame='tof_fl'):
        """Range the FL/FR cone reads to a wide face at robot x = face_x: its inner edge ray is the nearest."""
        f = load_frames()
        x = f.origin(frame)[0]
        return (face_x - x) / math.cos(abs(f.yaw(frame)) - TofConfig().fov_rad / 2)

    def test_front_sensors_see_a_low_toy_box_the_lidar_misses(self):
        toy = Block('toy', ROBOT_RADIUS + 0.30, -0.40, ROBOT_RADIUS + 0.50, 0.40, 0.0, 0.08)
        sim = Sim2D(room_with(toy), QUIET)
        fl, fr, bl, br = sim.corners()
        for v in (fl, fr):
            self.assertAlmostEqual(v, self.nearest_in_cone(ROBOT_RADIUS + 0.30), delta=0.01)
            self.assertLess(v, TofConfig().max_m)
        self.assertGreater(min(bl, br), TofConfig().max_m)
        scan = sim.scan()
        self.assertGreater(scan[beam_toward(0.0, len(scan))], 5.0)

    def test_corner_frames_follow_the_robot_heading(self):
        wall = Block('box', -0.7, -0.3, -0.5, 0.3, 0.0, 0.3)
        sim = Sim2D(room_with(wall), QUIET, start=Pose2D(0.0, 0.0, math.pi))     # facing the box
        fl, fr, bl, br = sim.corners()
        self.assertAlmostEqual(fl, self.nearest_in_cone(0.5), delta=0.01)
        self.assertAlmostEqual(fr, fl, delta=0.005)
        self.assertGreater(min(bl, br), TofConfig().max_m)


class TofToRange(unittest.TestCase):
    def test_conversion(self):
        cfg = TofConfig()
        self.assertIsNone(to_range(0.0))                  # not ranging: no message, so nothing gets cleared
        self.assertIsNone(to_range(-1.0))
        self.assertIsNone(to_range(math.nan))
        self.assertEqual(to_range(math.inf), cfg.max_m)
        self.assertEqual(to_range(4.0), cfg.max_m)
        self.assertEqual(to_range(cfg.max_m), cfg.max_m)
        self.assertEqual(to_range(0.3), 0.3)
        self.assertEqual(to_range(0.01), cfg.min_m)
        self.assertEqual(to_range(0.9, TofConfig(max_m=1.2)), 0.9)

    def test_arc_spans_the_field_of_view_at_the_range(self):
        pts = arc_points(0.5)
        self.assertEqual(len(pts), TofConfig().arc_points)
        for x, y, z in pts:
            self.assertAlmostEqual(math.hypot(x, y), 0.5)
            self.assertEqual(z, 0.0)
        self.assertAlmostEqual(math.degrees(math.atan2(pts[-1][1], pts[-1][0])), 13.5)
        self.assertAlmostEqual(pts[len(pts) // 2][1], 0.0)


class Contacts(unittest.TestCase):
    def test_low_toy_presses_the_bumper_ring(self):
        # R2's ring reaches down to Z15, so a toy box only 8 cm tall is felt (R1's strips were at Z125–149).
        sim = Sim2D(room_with(Block('toy', 1.40, -0.075, 1.60, 0.075, 0.0, 0.08)), start=Pose2D(1.0, 0.0, 0.0))
        drive(sim, 0.2, 0.0, 4.0)
        self.assertAlmostEqual(sim.pose.x, 1.40 - (ROBOT_RADIUS - BUMPER_TRAVEL), delta=0.01)
        self.assertEqual(sim.bumpers, 0b0011)
        self.assertEqual([(c.name, c.kind, c.bands) for c in sim.contacts], [('toy', 'bumper', ())])   # felt before it was hard

    def test_a_threshold_under_the_ring_is_not_felt(self):
        sim = Sim2D(room_with(Block('sill', 1.40, -1.0, 1.45, 1.0, 0.0, 0.012)), start=Pose2D(1.0, 0.0, 0.0))
        drive(sim, 0.2, 0.0, 4.0)
        self.assertAlmostEqual(sim.pose.x, 1.40 - 0.165, delta=0.01)     # the chassis plate under the ring
        self.assertEqual(sim.bumpers, 0)
        self.assertEqual([(c.name, c.kind, c.bands) for c in sim.contacts], [('sill', 'crash', ('chassis',))])

    def test_table_top_hits_the_head(self):
        sim = Sim2D(room_with(Block('top', 1.55, -0.5, 2.05, 0.5, 0.40, 0.45)), start=Pose2D(0.8, 0.0, 0.0))
        drive(sim, 0.2, 0.0, 6.0)
        self.assertAlmostEqual(sim.pose.x, 1.55 - 0.070, delta=0.01)        # screen face of the head, X70
        self.assertEqual(sim.bumpers, 0)
        self.assertEqual([(c.name, c.kind, c.bands) for c in sim.contacts], [('top', 'crash', ('head',))])

    def test_shelf_above_the_robot_is_no_obstacle(self):
        sim = Sim2D(room_with(Block('shelf', 1.55, -0.5, 2.05, 0.5, 0.60, 0.65)), start=Pose2D(0.8, 0.0, 0.0))
        drive(sim, 0.2, 0.0, 8.0)
        self.assertGreater(sim.pose.x, 2.2)
        self.assertEqual(sim.contacts, [])

    def test_leg_in_front_presses_the_bumper_first(self):
        sim = Sim2D(room_with(Cylinder('leg', 1.5, 0.10, 0.0125, 0.0, 0.42)), start=Pose2D(1.0, 0.0, 0.0))
        sim.target = list(K.twist_to_wheels(0.05, 0.0))
        while not sim.bumpers and sim.t < 10.0:
            sim.step(0.02)
        self.assertEqual(sim.bumpers, 0b0001)                # FL: the leg is ≈ 30° to the robot's left
        self.assertFalse(sim.collides(sim.pose))             # pressed before the hard contact
        self.assertEqual([(c.name, c.kind) for c in sim.contacts], [('leg', 'bumper')])

    def test_leg_beside_the_robot_is_felt_too(self):
        # The ring goes all the way round: nothing like R1's unfelt track ends.
        sim = Sim2D(room_with(Cylinder('leg', 1.5, 0.16, 0.0125, 0.0, 0.42)), start=Pose2D(1.0, 0.0, 0.0))
        drive(sim, 0.1, 0.0, 6.0)
        self.assertEqual(sim.bumpers, 0b0001)
        self.assertEqual([(c.name, c.kind, c.bands) for c in sim.contacts], [('leg', 'bumper', ())])

    def test_turning_in_place_beside_a_wall_touches_nothing(self):
        sim = Sim2D(Room(box(-2.0, -2.0, 2.0, 2.0), FAR_DOCK), start=Pose2D(0.0, 2.0 - ROBOT_RADIUS - BUMPER_REACH - 0.005, 0.0))
        drive(sim, 0.0, 1.0, 8.0)
        self.assertEqual(sim.contacts, [])
        self.assertGreater(abs(sim.pose.yaw), 0.5)          # it did turn

    def test_wall_contact_is_felt_and_logged_once_per_touch(self):
        sim = Sim2D(Room(box(-2.0, -2.0, 2.0, 2.0), FAR_DOCK))
        drive(sim, 0.3, 0.0, 10.0)
        drive(sim, 0.0, 0.0, 1.0)
        self.assertEqual([(c.name, c.kind) for c in sim.contacts], [('wall', 'bumper')])
        drive(sim, -0.1, 0.0, 1.5)                           # away, then back after the release time
        drive(sim, 0.3, 0.0, 3.0)
        self.assertEqual([(c.name, c.kind) for c in sim.contacts], [('wall', 'bumper')] * 2)

    def test_sensor_only_obstacles_do_not_stop_the_robot(self):
        sim = Sim2D(room_with(Block('ghost', 1.4, -0.5, 1.6, 0.5, 0.0, 0.3, contact=False)),
                    start=Pose2D(1.0, 0.0, 0.0))
        drive(sim, 0.2, 0.0, 4.0)
        self.assertGreater(sim.pose.x, 1.6)
        self.assertEqual(sim.contacts, [])


class Clearance(unittest.TestCase):
    def test_gap_to_blocks_and_cylinders(self):
        b = Block('b', 1.0, -0.1, 1.2, 0.1, 0.0, 0.1)
        self.assertAlmostEqual(clearance(Pose2D(0.0, 0.0, 0.0), b), 1.0 - ROBOT_RADIUS)
        self.assertAlmostEqual(clearance(Pose2D(1.1, 0.5, 0.0), b), 0.5 - 0.1 - ROBOT_RADIUS)
        self.assertAlmostEqual(clearance(Pose2D(0.0, 0.0, math.pi / 2), b), 1.0 - ROBOT_RADIUS)   # round: heading-free
        self.assertEqual(clearance(Pose2D(1.0, 0.0, 0.3), b), 0.0)
        c = Cylinder('c', 1.0, 0.0, 0.05, 0.0, 0.4)
        self.assertAlmostEqual(clearance(Pose2D(0.0, 0.0, 0.0), c), 1.0 - ROBOT_RADIUS - 0.05)
        d = ROBOT_RADIUS + 0.05
        touching = Pose2D(1.0 - 0.6 * d, -0.8 * d, 0.0)                     # 3-4-5 diagonal, just touching
        self.assertAlmostEqual(clearance(touching, c), 0.0)
        self.assertAlmostEqual(clearance(Pose2D(touching.x - 0.03, touching.y - 0.04, 0.0), c), 0.05)


class Worlds(unittest.TestCase):
    def test_start_goal_and_dock_poses_are_free(self):
        for name in WORLDS:
            room = world(name)
            for pose in (Pose2D(0.0, 0.0, 0.0), room.dock, pre_dock_pose(room.dock, 0.6), Pose2D(3.5, 0.0, 0.0)):
                with self.subTest(world=name, pose=pose):
                    self.assertFalse(Sim2D(room).collides(pose))

    def test_course_obstacles_are_below_the_scan_plane_except_the_chair_back(self):
        for name in ('toy', 'table', 'door', 'dock'):
            with self.subTest(world=name):
                self.assertTrue(all(o.z1 < LASER_Z for o in world(name).obstacles))
        tall = {o.name for o in world('chair').obstacles if o.z0 <= LASER_Z <= o.z1}
        self.assertEqual(tall, {'chair_back', 'chair_leg'})

    def test_table_legs_leave_room_for_the_body_but_the_top_catches_the_head(self):
        legs = [o for o in world('table').obstacles if o.name == 'table_leg']
        gap = min(abs(a.y - b.y) for a in legs for b in legs if a.y != b.y) - 2 * legs[0].r
        self.assertGreater(gap, 2 * ROBOT_RADIUS)
        top = next(o for o in world('table').obstacles if o.name == 'table_top')
        head = next(b for b in robot_bands() if b.name == 'head')
        body = next(b for b in robot_bands() if b.name == 'body')
        self.assertTrue(body.z1 < top.z0 < head.z1)           # over the torso, into the head band Z360–520

    def test_unknown_world_is_an_error(self):
        with self.assertRaises(ValueError):
            world('garden')


if __name__ == '__main__':
    unittest.main()
