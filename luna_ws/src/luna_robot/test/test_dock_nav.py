"""Go-to-dock coordinator (core/dock_nav.py): Nav2 to the pre-dock pose, then hand over to dock_node.
Run from luna_ws/src/luna_robot:  python3 -m unittest discover -s test"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core.dock_nav import (DockNav, DockNavConfig, NavObs, Pose2D, format_dock_pose, front_points,
                                      parse_dock_pose, pose_error, pre_dock_pose, wall_normal_yaw)

DOCK = Pose2D(3.0, 1.75, math.pi / 2)          # robot pose when docked, facing the wall (+y)
PRE = pre_dock_pose(DOCK, 0.6)                 # 0.6 m back along the dock axis: (3.0, 1.15, pi/2)
FAR = Pose2D(0.0, 0.0, 0.0)
DT = 0.1


def obs(dock_state='IDLE', pose=FAR, power='run', result=None, wall_yaw=None):
    return NavObs(dock_state=dock_state, power_action=power, pose=pose, nav_result=result, wall_yaw=wall_yaw)


def run(nav, o, seconds):
    out = []
    for _ in range(round(seconds / DT)):
        out += nav.step(o, DT)
    return out


class Geometry(unittest.TestCase):
    def test_pre_dock_pose_backs_off_along_the_dock_axis(self):
        self.assertAlmostEqual(PRE.x, 3.0, places=9)
        self.assertAlmostEqual(PRE.y, 1.15, places=9)
        self.assertAlmostEqual(PRE.yaw, DOCK.yaw, places=9)

    def test_pose_error_wraps_yaw(self):
        d, dyaw = pose_error(Pose2D(0, 0, 3.1), Pose2D(0.3, 0.4, -3.1))
        self.assertAlmostEqual(d, 0.5, places=9)
        self.assertAlmostEqual(dyaw, 2 * math.pi - 6.2, places=9)

    def test_wall_normal_of_a_skewed_wall(self):
        # Wall 0.4 m ahead, turned 0.1 rad: its normal (into the wall) points 0.1 rad left of straight ahead.
        a = 0.1
        pts = [(0.4 * math.cos(a) - t * math.sin(a), 0.4 * math.sin(a) + t * math.cos(a))
               for t in [i * 0.02 - 0.3 for i in range(31)]]
        self.assertAlmostEqual(wall_normal_yaw(pts), a, places=6)

    def test_wall_normal_rejects_a_corner_or_too_few_points(self):
        corner = [(0.4, y * 0.02) for y in range(-10, 1)] + [(0.4 - x * 0.02, 0.0) for x in range(1, 11)]
        self.assertIsNone(wall_normal_yaw(corner))
        self.assertIsNone(wall_normal_yaw([(0.4, 0.0), (0.4, 0.1)]))

    def test_front_points_from_a_turned_laser(self):
        # R08: the C1's +x points to the robot's right (yaw −90°); straight ahead is then laser angle +90°.
        inc = 2 * math.pi / 8
        ranges = [1.0] * 8
        ranges[6] = 0.5                                              # -pi + 6 * pi/4 = +pi/2 in the laser frame
        pts = front_points(ranges, -math.pi, inc, laser_x=-0.07, max_range=0.8, half_angle=math.radians(20),
                           laser_yaw=-math.pi / 2, laser_y=0.01)
        self.assertEqual(len(pts), 1)
        self.assertAlmostEqual(pts[0][0], 0.43, places=9)
        self.assertAlmostEqual(pts[0][1], 0.01, places=9)

    def test_front_points_from_a_scan_in_the_laser_frame(self):
        # 8 beams from -pi, laser 0.07 m behind the centre; only the forward sector within range is kept.
        inc = 2 * math.pi / 8
        ranges = [1.0, 1.0, 1.0, 1.0, 0.5, 0.6, 1.0, 1.0]           # index 4 looks straight ahead
        pts = front_points(ranges, -math.pi, inc, laser_x=-0.07, max_range=0.8, half_angle=math.radians(50))
        self.assertEqual(len(pts), 2)
        self.assertAlmostEqual(pts[0][0], 0.43, places=9)            # 0.5 m from the laser = 0.43 m from the centre
        self.assertAlmostEqual(pts[0][1], 0.0, places=9)
        self.assertAlmostEqual(pts[1][0], -0.07 + 0.6 * math.cos(inc), places=9)

    def test_dock_file_round_trip(self):
        p = parse_dock_pose(format_dock_pose(Pose2D(-1.25, 0.5, -2.0)))
        self.assertEqual((round(p.x, 4), round(p.y, 4), round(p.yaw, 4)), (-1.25, 0.5, -2.0))

    def test_dock_file_rejects_missing_keys(self):
        with self.assertRaises(ValueError):
            parse_dock_pose('x: 1.0\ny: 2.0\n')


class GoToDock(unittest.TestCase):
    def test_happy_path_navigates_then_hands_over_then_learns(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        self.assertEqual(nav.step(obs(), DT), [('navigate', 1, PRE)])
        self.assertEqual(nav.state, 'NAVIGATE')
        self.assertEqual(run(nav, obs(pose=Pose2D(1.5, 0.5, 1.0)), 5), [])      # Nav2 driving: nothing new
        at_pre = Pose2D(3.02, 1.16, math.pi / 2 - 0.05)
        self.assertEqual(nav.step(obs(pose=at_pre, result=(1, 'succeeded')), DT), [('dock', 'dock')])
        self.assertEqual(nav.state, 'HANDOVER')
        self.assertEqual(nav.step(obs('ALIGN', at_pre), DT), [])
        self.assertEqual(nav.state, 'DOCKING')
        run(nav, obs('APPROACH', at_pre), 5)
        run(nav, obs('CONTACT', DOCK), 1)
        self.assertEqual(nav.state, 'DOCKING')                                 # contact can still fail
        docked = Pose2D(3.01, 1.76, math.pi / 2)
        self.assertEqual(nav.step(obs('CHARGING', docked), DT), [('save_dock', docked)])
        self.assertEqual(nav.state, 'DOCKED')
        self.assertEqual(nav.dock, docked)
        self.assertEqual(run(nav, obs('CHARGING', docked), 5), [])             # learned once per docking

    def test_already_at_pre_dock_skips_nav2(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        self.assertEqual(nav.step(obs(pose=PRE), DT), [('dock', 'dock')])
        self.assertEqual(nav.state, 'HANDOVER')

    def test_already_docked_does_nothing(self):
        for state in ('CHARGING', 'DOCKED'):
            with self.subTest(state=state):
                nav = DockNav(dock=DOCK)
                nav.request('go')
                self.assertEqual(nav.step(obs(state, DOCK), DT), [])
                self.assertEqual(nav.state, 'DOCKED')

    def test_dock_node_already_working_is_monitored(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        self.assertEqual(nav.step(obs('APPROACH', PRE), DT), [])
        self.assertEqual(nav.state, 'DOCKING')

    def test_unknown_dock_falls_back_to_dock_node_seek(self):
        # Before any map/dock pose exists, behave like the old auto-dock: dock_node searches for the tag.
        nav = DockNav(dock=None)
        nav.request('go')
        self.assertEqual(nav.step(obs(), DT), [('dock', 'dock')])
        self.assertEqual((nav.state, nav.reason), ('HANDOVER', 'no_dock_pose'))

    def test_waits_for_localization_then_navigates(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        self.assertEqual(run(nav, obs(pose=None), 3), [])
        self.assertEqual(nav.state, 'WAIT')
        self.assertEqual(nav.step(obs(), DT), [('navigate', 1, PRE)])

    def test_never_localized_fails(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        run(nav, obs(pose=None), nav.cfg.start_timeout_s + 0.5)
        self.assertEqual((nav.state, nav.reason), ('FAILED', 'not_localized'))

    def test_undock_in_progress_is_waited_out(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        self.assertEqual(run(nav, obs('UNDOCK', DOCK), 2), [])
        self.assertEqual(nav.step(obs('IDLE', Pose2D(3.0, 1.6, math.pi / 2)), DT), [('dock', 'dock')])


class NavFailures(unittest.TestCase):
    def start(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        self.assertEqual(nav.step(obs(), DT), [('navigate', 1, PRE)])
        return nav

    def test_aborted_goal_is_retried_after_a_delay(self):
        nav = self.start()
        self.assertEqual(nav.step(obs(result=(1, 'aborted')), DT), [])
        out = run(nav, obs(), nav.cfg.retry_delay_s + 0.2)
        self.assertEqual(out, [('navigate', 2, PRE)])

    def test_nav2_still_starting_is_not_a_failed_attempt(self):
        # At boot the battery can ask for the dock before Nav2's lifecycle manager has activated the servers.
        nav = self.start()
        seq = 1
        for _ in range(10):
            nav.step(obs(result=(seq, 'unavailable')), DT)
            out = run(nav, obs(), nav.cfg.retry_delay_s + 0.2)
            seq += 1
            self.assertEqual(out, [('navigate', seq, PRE)])
        self.assertEqual(nav.state, 'NAVIGATE')
        nav.step(obs(pose=PRE, result=(seq, 'succeeded')), DT)
        self.assertEqual(nav.state, 'HANDOVER')

    def test_nav2_that_never_comes_up_fails(self):
        nav = self.start()
        seq = 1
        for _ in range(round(nav.cfg.nav2_wait_s / nav.cfg.retry_delay_s) + 1):
            nav.step(obs(result=(seq, 'unavailable')), DT)
            run(nav, obs(), nav.cfg.retry_delay_s + 0.2)
            seq += 1
        self.assertEqual((nav.state, nav.reason), ('FAILED', 'nav2_unavailable'))

    def test_gives_up_after_max_attempts(self):
        nav = self.start()
        for seq in range(1, nav.cfg.max_nav_attempts + 1):
            nav.step(obs(result=(seq, 'aborted')), DT)
            run(nav, obs(), nav.cfg.retry_delay_s + 0.2)
        self.assertEqual((nav.state, nav.reason), ('FAILED', 'nav_failed'))

    def test_success_far_from_pre_dock_is_not_handed_over(self):
        # Nav2 said "succeeded" but localization jumped: the tag would not be in view, so drive again.
        nav = self.start()
        self.assertEqual(nav.step(obs(pose=Pose2D(2.5, 1.0, 0.2), result=(1, 'succeeded')), DT), [])
        self.assertIn(('navigate', 2, PRE), run(nav, obs(), nav.cfg.retry_delay_s + 0.2))
        self.assertEqual(nav.reason, 'pose_off')

    def test_handover_needs_the_dock_axis_and_heading(self):
        for pose in (Pose2D(PRE.x + 0.3, PRE.y, PRE.yaw),          # beside the axis: tag at the edge of view
                     Pose2D(PRE.x, PRE.y, PRE.yaw + 0.6),          # on the axis but looking away
                     Pose2D(PRE.x, PRE.y - 0.4, PRE.yaw)):         # too far back for the tag approach
            with self.subTest(pose=pose):
                nav = self.start()
                self.assertEqual(nav.step(obs(pose=pose, result=(1, 'succeeded')), DT), [])
                self.assertEqual(nav.reason, 'pose_off')

    def test_handover_cone_narrows_towards_the_dock(self):
        # dock_node steers at the dock point, so it arrives with heading error ≈ lateral / distance.
        nav = DockNav(dock=DOCK)
        on_axis = lambda along, lateral: Pose2D(DOCK.x - lateral, DOCK.y + along, DOCK.yaw)
        self.assertTrue(nav.ready_for_handover(on_axis(-0.6, 0.06)))
        self.assertFalse(nav.ready_for_handover(on_axis(-0.6, 0.10)))
        self.assertFalse(nav.ready_for_handover(on_axis(-0.2, 0.05)))
        self.assertTrue(nav.ready_for_handover(on_axis(-0.15, 0.01)))

    def test_stale_result_of_an_old_goal_is_ignored(self):
        nav = self.start()
        nav.step(obs(result=(1, 'aborted')), DT)
        run(nav, obs(), nav.cfg.retry_delay_s + 0.2)                           # goal 2 sent
        self.assertEqual(nav.step(obs(pose=PRE, result=(1, 'succeeded')), DT), [])
        self.assertEqual(nav.state, 'NAVIGATE')

    def test_goal_canceled_by_someone_else_stops_without_retry(self):
        nav = self.start()
        self.assertEqual(nav.step(obs(result=(1, 'canceled')), DT), [])
        self.assertEqual((nav.state, nav.reason), ('IDLE', 'nav_canceled'))
        self.assertEqual(run(nav, obs(), 5), [])

    def test_timeout_cancels_and_retries(self):
        nav = self.start()
        out = run(nav, obs(), nav.cfg.nav_timeout_s + 0.2)
        self.assertEqual(out[0], ('cancel_nav', 1))
        out = run(nav, obs(), nav.cfg.retry_delay_s + 0.2)
        self.assertEqual(out, [('navigate', 2, PRE)])

    def test_dock_node_starting_on_its_own_takes_over_from_nav2(self):
        # Two velocity sources must not fight: base_node prefers the dock twist, so Nav2 would only stall.
        nav = self.start()
        self.assertEqual(nav.step(obs('SEEK', Pose2D(3.0, 1.3, 1.5)), DT), [('cancel_nav', 1)])
        self.assertEqual(nav.state, 'DOCKING')
        self.assertEqual(nav.step(obs('CHARGING', DOCK), DT), [('save_dock', DOCK)])
        self.assertEqual(nav.state, 'DOCKED')


class DockFailures(unittest.TestCase):
    def handed_over(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        nav.step(obs(pose=PRE), DT)
        nav.step(obs('SEEK', PRE), DT)
        self.assertEqual(nav.state, 'DOCKING')
        return nav

    def test_dock_fault_backs_off_then_drives_back_to_pre_dock_once(self):
        # dock_node gives up with Luna pressed against the dock face, inside the costmap's lethal band: Nav2 can
        # neither plan nor turn there, so first reverse straight out (Nav2 BackUp, still gated by base_node).
        nav = self.handed_over()
        at_face = Pose2D(3.0, 1.74, math.pi / 2)
        self.assertEqual(nav.step(obs('FAULT', at_face), DT), [('backup', 1, nav.cfg.backup_m)])
        self.assertEqual(nav.state, 'BACKUP')
        self.assertEqual(nav.step(obs('FAULT', Pose2D(3.0, 1.34, 1.5), result=(1, 'succeeded')), DT),
                         [('navigate', 2, PRE)])
        nav.step(obs('FAULT', PRE, result=(2, 'succeeded')), DT)
        self.assertEqual(nav.state, 'HANDOVER')
        nav.step(obs('SEEK', PRE), DT)
        nav.step(obs('FAULT', PRE), DT)
        self.assertEqual((nav.state, nav.reason), ('FAILED', 'dock_fault'))

    def test_failed_backup_gives_up(self):
        nav = self.handed_over()
        nav.step(obs('FAULT', PRE), DT)
        nav.step(obs('FAULT', PRE, result=(1, 'aborted')), DT)
        self.assertEqual((nav.state, nav.reason), ('FAILED', 'backup_failed'))

    def test_cancel_during_backup_cancels_it(self):
        nav = self.handed_over()
        nav.step(obs('FAULT', PRE), DT)
        nav.request('cancel')
        self.assertEqual(nav.step(obs('FAULT', PRE), DT), [('cancel_nav', 1)])
        self.assertEqual(nav.state, 'IDLE')

    def test_handover_from_dock_fault_waits_for_dock_node(self):
        # dock_node still reports FAULT for a tick or two after accepting the new request.
        nav = self.handed_over()
        nav.step(obs('FAULT', PRE), DT)                                        # back up
        nav.step(obs('FAULT', PRE, result=(1, 'succeeded')), DT)               # re-navigate
        nav.step(obs('FAULT', PRE, result=(2, 'succeeded')), DT)
        self.assertEqual(nav.step(obs('FAULT', PRE), DT), [])
        self.assertEqual(nav.state, 'HANDOVER')

    def test_silent_dock_node_fails_the_handover(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        nav.step(obs(pose=PRE), DT)
        run(nav, obs('IDLE', PRE), nav.cfg.handover_timeout_s + 0.2)
        self.assertEqual((nav.state, nav.reason), ('FAILED', 'dock_node_silent'))

    def test_dock_canceled_elsewhere_ends_docking(self):
        nav = self.handed_over()
        nav.step(obs('IDLE', PRE), DT)
        self.assertEqual(nav.state, 'IDLE')

    def test_lifted_off_the_dock_returns_to_idle(self):
        nav = self.handed_over()
        nav.step(obs('CHARGING', DOCK), DT)
        nav.step(obs('IDLE', DOCK), DT)
        self.assertEqual(nav.state, 'IDLE')


class Cancel(unittest.TestCase):
    def test_cancel_while_navigating_cancels_the_goal(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        nav.step(obs(), DT)
        nav.request('cancel')
        self.assertEqual(nav.step(obs(), DT), [('cancel_nav', 1)])
        self.assertEqual(nav.state, 'IDLE')

    def test_cancel_while_docking_cancels_dock_node(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        nav.step(obs(pose=PRE), DT)
        nav.step(obs('APPROACH', PRE), DT)
        nav.request('cancel')
        self.assertEqual(nav.step(obs('APPROACH', PRE), DT), [('dock', 'cancel')])
        self.assertEqual(nav.state, 'IDLE')


class AutoDock(unittest.TestCase):
    def test_low_battery_starts_go_to_dock(self):
        for action in ('seek_dock', 'critical'):
            with self.subTest(action=action):
                nav = DockNav(dock=DOCK)
                self.assertEqual(nav.step(obs(power=action), DT), [('navigate', 1, PRE)])

    def test_auto_off_ignores_the_battery(self):
        nav = DockNav(DockNavConfig(auto=False), dock=DOCK)
        self.assertEqual(run(nav, obs(power='seek_dock'), 2), [])

    def test_no_auto_trigger_while_docked(self):
        nav = DockNav(dock=DOCK)
        self.assertEqual(run(nav, obs('CHARGING', DOCK, power='seek_dock'), 2), [])

    def test_failure_holds_off_auto_retry_for_the_cooldown(self):
        nav = DockNav(dock=DOCK)
        nav.step(obs(pose=None, power='seek_dock'), DT)
        run(nav, obs(pose=None, power='seek_dock'), nav.cfg.start_timeout_s + 0.5)
        self.assertEqual(nav.state, 'FAILED')
        self.assertEqual(run(nav, obs(power='seek_dock'), nav.cfg.retry_cooldown_s - 20), [])
        self.assertEqual(run(nav, obs(power='seek_dock'), 25), [('navigate', 1, PRE)])

    def test_user_cancel_holds_off_auto_retry(self):
        nav = DockNav(dock=DOCK)
        nav.step(obs(power='seek_dock'), DT)
        nav.request('cancel')
        self.assertEqual(nav.step(obs(power='seek_dock'), DT), [('cancel_nav', 1)])
        self.assertEqual(nav.state, 'IDLE')
        self.assertEqual(run(nav, obs(power='seek_dock'), 10), [])

    def test_explicit_go_ignores_the_cooldown(self):
        nav = DockNav(dock=DOCK)
        nav.request('go')
        nav.step(obs(), DT)
        nav.request('cancel')
        nav.step(obs(), DT)
        nav.request('go')
        self.assertEqual(nav.step(obs(), DT), [('navigate', 2, PRE)])


class Learning(unittest.TestCase):
    def test_first_charge_learns_the_dock(self):
        nav = DockNav(dock=None)
        self.assertEqual(nav.step(obs('APPROACH', PRE), DT), [])
        self.assertEqual(nav.step(obs('CHARGING', DOCK), DT), [('save_dock', DOCK)])
        self.assertEqual(nav.dock, DOCK)

    def test_dock_axis_comes_from_the_wall_not_the_docked_heading(self):
        # The receiver funnel accepts a skewed robot, so its heading on the dock is not the dock axis. The wall
        # behind the dock, seen by the LiDAR while charging, gives the axis.
        nav = DockNav(dock=None)
        nav.step(obs('APPROACH', PRE), DT)
        skewed = Pose2D(3.0, 1.75, math.pi / 2 - 0.12)
        out = nav.step(obs('CHARGING', skewed, wall_yaw=0.12), DT)
        self.assertEqual(len(out), 1)
        kind, learned = out[0]
        self.assertEqual(kind, 'save_dock')
        self.assertEqual((learned.x, learned.y), (skewed.x, skewed.y))
        self.assertAlmostEqual(learned.yaw, math.pi / 2, places=9)

    def test_large_jump_is_not_learned(self):
        # Charging while badly localized must not overwrite a good dock pose.
        nav = DockNav(dock=DOCK)
        nav.step(obs('APPROACH', PRE), DT)
        wrong = Pose2D(DOCK.x + 1.0, DOCK.y, DOCK.yaw)
        self.assertEqual(nav.step(obs('CHARGING', wrong), DT), [('reject_dock', wrong)])
        self.assertEqual(nav.dock, DOCK)

    def test_no_pose_no_learning(self):
        nav = DockNav(dock=None)
        nav.step(obs('APPROACH', None), DT)
        self.assertEqual(nav.step(obs('CHARGING', None), DT), [])


if __name__ == '__main__':
    unittest.main()
