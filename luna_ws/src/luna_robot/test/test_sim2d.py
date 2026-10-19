"""2D simulator world (core/sim2d.py) used to test mapping, navigation and docking without hardware.
Run from luna_ws/src/luna_robot:  python3 -m unittest discover -s test"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core import kinematics as K
from luna_robot.core.dock import Dock, Obs
from luna_robot.core.dock_nav import Pose2D, front_points, pre_dock_pose, wall_normal_yaw
from luna_robot.core.sim2d import LASER_X, LASER_YAW, Room, Sim2D, SimConfig, beam_toward, box, default_room, raycast

SQUARE = Room(walls=box(-2.0, -2.0, 2.0, 2.0), dock=Pose2D(0.0, 1.75, math.pi / 2))


def drive(sim, v, w, seconds, dt=0.02):
    sim.target = list(K.twist_to_tracks(v, w))
    for _ in range(round(seconds / dt)):
        sim.step(dt)


class Lidar(unittest.TestCase):
    def test_raycast_hits_nearest_wall(self):
        self.assertAlmostEqual(raycast(0.0, 0.0, 0.0, SQUARE.walls, 12.0), 2.0, places=9)
        self.assertAlmostEqual(raycast(0.0, 0.0, math.pi / 4, SQUARE.walls, 12.0), 2 * math.sqrt(2), places=9)

    def test_raycast_beyond_range_is_inf(self):
        self.assertEqual(raycast(0.0, 0.0, 0.0, SQUARE.walls, 1.5), math.inf)

    def test_scan_is_taken_from_the_laser_frame(self):
        sim = Sim2D(SQUARE, SimConfig(range_noise_m=0.0))
        scan = sim.scan()
        n = len(scan)
        self.assertEqual(n, SimConfig().beams)
        ahead = scan[beam_toward(0.0, n)]             # R08: the laser frame is turned −90°, so not the middle beam
        behind = scan[beam_toward(math.pi, n)]
        self.assertAlmostEqual(ahead, 2.0 - LASER_X, places=6)
        self.assertAlmostEqual(behind, 2.0 + LASER_X, places=6)

    def test_scan_rotates_with_the_robot(self):
        sim = Sim2D(SQUARE, SimConfig(range_noise_m=0.0), start=Pose2D(1.0, 0.0, math.pi / 2))
        scan = sim.scan()
        self.assertAlmostEqual(scan[beam_toward(0.0, len(scan))], 2.0 - LASER_X, places=6)  # facing the north wall


class Motion(unittest.TestCase):
    def test_odometry_matches_truth_without_slip(self):
        sim = Sim2D(SQUARE, SimConfig(true_turn_slip=K.DriveLimits().turn_slip, linear_slip=0.0))
        odo = K.Odometry()
        odo.update(*sim.pos)
        sim.target = list(K.twist_to_tracks(0.2, 0.4))
        for _ in range(100):                                                 # base_node integrates every MCU sample
            sim.step(0.02)
            x, y, yaw = odo.update(*sim.pos)
        self.assertAlmostEqual(sim.pose.x, x, places=6)
        self.assertAlmostEqual(sim.pose.y, y, places=6)
        self.assertAlmostEqual(sim.pose.yaw, yaw, places=6)

    def test_unmodelled_slip_makes_odometry_overestimate(self):
        sim = Sim2D(SQUARE, SimConfig(true_turn_slip=1.45, linear_slip=0.03))
        odo = K.Odometry()
        odo.update(*sim.pos)
        drive(sim, 0.0, 0.8, 1.5)
        ox, oy, oyaw = odo.update(*sim.pos)
        self.assertGreater(oyaw, sim.pose.yaw * 1.05)                        # SLAM has to correct this
        tx, ty = sim.pose.x, sim.pose.y
        drive(sim, 0.2, 0.0, 2.0)
        x, y, _ = odo.update(*sim.pos)
        self.assertGreater(math.hypot(x - ox, y - oy), 1.02 * math.hypot(sim.pose.x - tx, sim.pose.y - ty))

    def test_twist_is_the_true_body_velocity(self):
        sim = Sim2D(SQUARE, SimConfig(linear_slip=0.0, true_turn_slip=K.DriveLimits().turn_slip))
        drive(sim, 0.2, 0.5, 2.0)
        v, w = sim.twist
        self.assertAlmostEqual(v, 0.2, places=3)
        self.assertAlmostEqual(w, 0.5, places=3)
        drive(sim, 0.3, 0.0, 10.0)                                           # against the east wall
        self.assertEqual(sim.twist, (0.0, 0.0))

    def test_wall_stops_the_robot_and_presses_the_front_bumper(self):
        sim = Sim2D(SQUARE)
        drive(sim, 0.3, 0.0, 10.0)
        self.assertAlmostEqual(sim.pose.x, 2.0 - 0.17, delta=0.02)          # front face against the east wall
        self.assertEqual(sim.bumpers & 0b0011, 0b0011)
        self.assertEqual(sim.bumpers & 0b1100, 0)
        drive(sim, -0.1, 0.0, 1.0)
        self.assertEqual(sim.bumpers, 0)

    def test_rear_bumper_when_reversing_into_a_wall(self):
        sim = Sim2D(SQUARE)
        drive(sim, -0.2, 0.0, 15.0)
        self.assertEqual(sim.bumpers & 0b1100, 0b1100)


class DockScene(unittest.TestCase):
    def test_default_room_poses_are_free(self):
        room = default_room()
        for pose in (Pose2D(0, 0, 0), room.dock, pre_dock_pose(room.dock, 0.6)):
            with self.subTest(pose=pose):
                self.assertFalse(Sim2D(room, start=pose).collides(pose))

    def test_tag_seen_from_the_pre_dock_pose(self):
        room = default_room()
        sim = Sim2D(room, start=pre_dock_pose(room.dock, 0.6))
        offset, dist = sim.tag()
        self.assertAlmostEqual(offset, 0.0, places=6)
        self.assertAlmostEqual(dist, 0.6, places=6)

    def test_tag_hidden_when_facing_away_or_from_the_start(self):
        room = default_room()
        pre = pre_dock_pose(room.dock, 0.6)
        self.assertIsNone(Sim2D(room, start=Pose2D(pre.x, pre.y, pre.yaw + math.pi)).tag())
        self.assertIsNone(Sim2D(room, start=Pose2D(0.0, 0.0, 0.0)).tag())

    def test_tag_occluded_by_the_cabinet(self):
        room = default_room()
        d = room.dock
        across = Pose2D(0.7, 0.2, math.atan2(d.y - 0.2, d.x - 0.7))         # facing the dock, cabinet in between
        self.assertIsNone(Sim2D(room, start=across).tag())
        no_cabinet = Room(walls=box(-1.0, -2.0, 4.0, 2.0), dock=d)
        self.assertIsNotNone(Sim2D(no_cabinet, start=across).tag())

    def test_dock_state_machine_docks_from_an_imperfect_pre_dock_pose(self):
        room = default_room()
        pre = pre_dock_pose(room.dock, 0.6)
        start = Pose2D(pre.x + 0.08, pre.y - 0.05, pre.yaw + 0.12)          # Nav2 goal tolerance and then some
        sim, dock = Sim2D(room, start=start), Dock()
        dock.request('dock')
        states = []
        for i in range(int(40 / 0.01)):
            if i % 5 == 0:                                                   # dock_node at 20 Hz
                tag = sim.tag()
                o = Obs(tag_visible=tag is not None, dock_sense=sim.docked, charging=sim.charging,
                        front_bumper=bool(sim.bumpers & 0b0011))
                if tag:
                    o.tag_offset, o.tag_distance = tag
                lin, ang = dock.step(o, 0.05)
                sim.target = list(K.twist_to_tracks(lin, ang))
                states.append(dock.state)
            sim.step(0.01)
        self.assertEqual(dock.state, 'CHARGING')
        self.assertNotIn('BACKOFF', states)
        err = math.hypot(sim.pose.x - room.dock.x, sim.pose.y - room.dock.y)
        self.assertLess(err, 0.02)

    def test_misaligned_robot_hits_the_dock_with_its_bumper(self):
        room = default_room()
        d = room.dock
        sim = Sim2D(room, start=Pose2D(d.x + 0.08, d.y - 0.3, d.yaw))      # 8 cm beside the dock axis
        drive(sim, 0.06, 0.0, 8.0)
        self.assertFalse(sim.docked)
        self.assertEqual(sim.bumpers & 0b0011, 0b0011)

    def test_skewed_robot_does_not_make_contact(self):
        room = default_room()
        d = room.dock
        sim = Sim2D(room, start=Pose2D(d.x, d.y - 0.01, d.yaw + 0.35))      # on the axis, 20° skewed
        drive(sim, 0.02, 0.0, 2.0)
        self.assertFalse(sim.docked)
        self.assertEqual(sim.bumpers & 0b0011, 0b0011)

    def test_wall_fit_on_the_dock_recovers_the_dock_axis(self):
        # Docked 0.15 rad skewed (the funnel allows it): the noisy scan of the wall still gives the true axis.
        room = default_room()
        d = room.dock
        sim = Sim2D(room, start=Pose2D(d.x, d.y, d.yaw - 0.15))
        c = sim.cfg
        pts = front_points(sim.scan(), -math.pi, 2 * math.pi / c.beams, LASER_X, 1.0, math.radians(35), LASER_YAW)
        self.assertGreater(len(pts), 50)
        self.assertAlmostEqual(wall_normal_yaw(pts), 0.15, delta=0.01)

    def test_charging_on_the_dock_fills_the_pack(self):
        room = default_room()
        sim = Sim2D(room, SimConfig(start_soc=0.3), start=Pose2D(room.dock.x, room.dock.y - 0.05, room.dock.yaw))
        drive(sim, 0.05, 0.0, 3.0)
        self.assertTrue(sim.docked and sim.charging)
        soc = sim.soc
        drive(sim, 0.0, 0.0, 60.0)
        self.assertGreater(sim.soc, soc)


if __name__ == '__main__':
    unittest.main()
