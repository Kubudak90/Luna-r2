"""Watchdog for slam_toolbox in localization mode (slam_watchdog_node).

slam_toolbox 2.6.10's localization node sometimes locks up for good: its executor stops (the parameter service no
longer answers), every thread sleeps on a futex, and map→odom is never published again while /scan keeps coming.
In the simulator that happened in about 1 of 12 localization runs, with the R07 and the R08 LiDAR pose alike and
with obstacle_sensors lidar and all alike (README_TR.md, "Engel algılama"). Nav2 then stops for good: the robot is
lost on a map it knows. The cause inside slam_toolbox was not found, so the watchdog recovers instead:

  WATCH       map→odom's stamp has not moved for stall_s (or never appeared within startup_s) while scans arrive
              → 'probe' (ask slam_toolbox's parameter service)
  PROBING     it answers → only slow (a long scan match, a loaded CPU): WATCH again, from now; no answer within
              probe_timeout_s → 'restart' (kill slam_toolbox; the launch respawns it with the same saved map)
  RESTARTED   once the new slam_toolbox listens on /initialpose → 'initial_pose' (hand it the robot's pose:
              the last good map→odom composed with the current odometry), repeated pose_repeats times
  RECOVERING  map→odom moves again → WATCH; still stuck after grace_s → 'restart' again

Pure logic: the node feeds wall times and the /initialpose subscriber count, and executes the returned actions."""
import math
from dataclasses import dataclass

from .dock_nav import Pose2D, wrap


@dataclass(frozen=True)
class WatchdogConfig:
    stall_s: float = 4.0          # slam_toolbox's own hiccups stay under ~2 s (transform_timeout is 1.5 s)
    scan_fresh_s: float = 1.0     # only a stall while scans arrive is slam_toolbox's fault
    probe_timeout_s: float = 8.0  # a locked node never answers; a busy one answers after its scan (1–2 s)
    respawn_s: float = 3.0        # the killed node's /initialpose subscription can linger in the graph this long
    pose_period_s: float = 1.0
    pose_repeats: int = 3
    grace_s: float = 40.0         # posegraph load + first match after a restart
    startup_s: float = 40.0       # no map→odom at all this long after start, while scans arrive: stuck from the start


def compose(map_odom, odom_base):
    """map→base_footprint from map→odom and odom→base_footprint (planar poses)."""
    c, s = math.cos(map_odom.yaw), math.sin(map_odom.yaw)
    return Pose2D(map_odom.x + c * odom_base.x - s * odom_base.y, map_odom.y + s * odom_base.x + c * odom_base.y,
                  wrap(map_odom.yaw + odom_base.yaw))


class SlamWatchdog:
    def __init__(self, cfg=WatchdogConfig()):
        self.cfg = cfg
        self.state = 'WATCH'
        self.since = 0.0              # when the current state began
        self.poses_sent = 0
        self.last_pose_at = -math.inf
        self.restarts = 0
        self.started = None

    def step(self, now, advanced_at, scan_at, slam_listening, answered_at=None):
        """advanced_at: wall time map→odom's stamp last moved (None before the first one); scan_at: last /scan
        (None if none yet); slam_listening: slam_toolbox subscribes /initialpose; answered_at: when slam_toolbox last
        answered a probe (None if never). Returns a list of actions."""
        c = self.cfg
        if self.started is None:
            self.started = now
        if self.state == 'WATCH':
            scans = scan_at is not None and now - scan_at < c.scan_fresh_s
            ref = self.started if advanced_at is None else max(advanced_at, self.since)
            limit = c.startup_s if advanced_at is None else c.stall_s
            if scans and now - ref > limit:
                self._go('PROBING', now)
                return ['probe']
            return []
        if self.state == 'PROBING':
            if answered_at is not None and answered_at >= self.since:
                self._go('WATCH', now)                       # alive, just slow: give it another stall_s
                return []
            return self._restart(now) if now - self.since > c.probe_timeout_s else []
        if self.state == 'RESTARTED':
            if now - self.since > c.grace_s:
                return self._restart(now)
            if now - self.since < c.respawn_s or not slam_listening or now - self.last_pose_at < c.pose_period_s:
                return []
            self.poses_sent += 1
            self.last_pose_at = now
            if self.poses_sent >= c.pose_repeats:
                self._go('RECOVERING', now)
            return ['initial_pose']
        # RECOVERING
        if advanced_at is not None and advanced_at > self.since:
            self._go('WATCH', now)
            return []
        if now - self.since > c.grace_s:
            return self._restart(now)
        return []

    def _restart(self, now):
        self.restarts += 1
        self.poses_sent, self.last_pose_at = 0, -math.inf
        self._go('RESTARTED', now)
        return ['restart']

    def _go(self, state, now):
        self.state, self.since = state, now
