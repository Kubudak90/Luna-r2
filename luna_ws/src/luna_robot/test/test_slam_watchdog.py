"""slam_toolbox watchdog decisions (core/slam_watchdog.py). Pure Python.
Run from luna_ws/src/luna_robot:  python3 -m pytest test"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core.dock_nav import Pose2D
from luna_robot.core.slam_watchdog import SlamWatchdog, WatchdogConfig, compose

CFG = WatchdogConfig()


def run(dog, t0, t1, advanced_at, scans=True, listening=True, answers=False, dt=0.5):
    """Step every dt from t0 to t1; scans arrive continuously if `scans`; slam_toolbox answers probes right away if
    `answers`. Returns [(t, action)]."""
    out, t = [], t0
    while t <= t1 + 1e-9:
        adv = advanced_at(t) if callable(advanced_at) else advanced_at
        for a in dog.step(t, adv, t if scans else None, listening, t if answers else None):
            out.append((round(t, 1), a))
        t += dt
    return out


class Watchdog(unittest.TestCase):
    def test_healthy_slam_is_left_alone(self):
        dog = SlamWatchdog()
        self.assertEqual(run(dog, 0.0, 120.0, lambda t: t), [])
        self.assertEqual(dog.state, 'WATCH')

    def test_frozen_map_to_odom_with_scans_restarts_then_hands_over_the_pose(self):
        dog = SlamWatchdog()
        frozen_at = 20.0
        probe_at = frozen_at + CFG.stall_s + 0.5
        t = probe_at + CFG.probe_timeout_s + 0.5
        acts = run(dog, 0.0, t, lambda u: min(u, frozen_at))
        self.assertEqual(acts, [(probe_at, 'probe'), (t, 'restart')])
        acts = run(dog, t + 0.5, t + 10.0, frozen_at)              # new slam_toolbox listens, nothing published yet
        poses = [a for a in acts if a[1] == 'initial_pose']
        self.assertEqual(len(poses), CFG.pose_repeats)
        self.assertGreaterEqual(poses[0][0] - t, CFG.respawn_s)    # not to the dying node
        self.assertEqual(dog.state, 'RECOVERING')
        acts = run(dog, t + 10.5, t + 20.0, lambda u: u)           # map→odom moves again
        self.assertEqual(acts, [])
        self.assertEqual(dog.state, 'WATCH')
        self.assertEqual(dog.restarts, 1)

    def test_slow_but_answering_slam_is_not_restarted(self):
        dog = SlamWatchdog()
        acts = run(dog, 0.0, 60.0, lambda t: min(t, 20.0), answers=True)
        self.assertNotIn('restart', [a for _, a in acts])
        probes = [t for t, a in acts if a == 'probe']
        self.assertGreater(len(probes), 1)
        self.assertGreater(min(b - a for a, b in zip(probes, probes[1:])), CFG.stall_s)   # not a probe storm

    def test_no_pose_until_the_respawned_node_listens(self):
        dog = SlamWatchdog()
        run(dog, 0.0, 30.0, lambda t: min(t, 10.0), listening=False)
        self.assertEqual(dog.state, 'RESTARTED')
        self.assertEqual(run(dog, 30.5, 40.0, 10.0, listening=False), [])

    def test_restarts_again_when_slam_never_recovers(self):
        dog = SlamWatchdog()
        acts = run(dog, 0.0, 20.0 + CFG.stall_s + 2 * CFG.grace_s + 10.0, lambda t: min(t, 20.0))
        restarts = [t for t, a in acts if a == 'restart']
        self.assertEqual(len(restarts), 2)                         # retried, but no faster than grace_s
        self.assertGreater(restarts[1] - restarts[0], CFG.grace_s)

    def test_no_restart_without_scans(self):
        # LiDAR unplugged or the simulator gone: not slam_toolbox's fault, a restart would not help
        dog = SlamWatchdog()
        self.assertEqual(run(dog, 0.0, 60.0, lambda t: min(t, 5.0), scans=False), [])

    def test_stuck_from_the_start(self):
        dog = SlamWatchdog()
        acts = run(dog, 0.0, CFG.startup_s + CFG.probe_timeout_s + 1.0, None)
        self.assertEqual([a for _, a in acts], ['probe', 'restart'])
        self.assertGreater(acts[0][0], CFG.startup_s)

    def test_compose(self):
        p = compose(Pose2D(1.0, 2.0, math.pi / 2), Pose2D(0.5, 0.0, 0.1))
        self.assertAlmostEqual(p.x, 1.0)
        self.assertAlmostEqual(p.y, 2.5)
        self.assertAlmostEqual(p.yaw, math.pi / 2 + 0.1)


if __name__ == '__main__':
    unittest.main()
