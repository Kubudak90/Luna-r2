"""Skid-steer track kinematics for Luna. Pure Python: no ROS, no I/O, unit-testable anywhere.

Geometry comes from the R05 CAD and the drive candidates in Donanım ve Güç R01:
- track centre lines at X = ±159 mm (REF_TRACK_L/R X134..184)  → gauge 0.318 m
- goBILDA 16T 24 mm-pitch sprocket, pitch diameter 24 / sin(π/16) ≈ 123.0 mm
- Pololu 4755: 6533 encoder counts per output revolution (64 CPR × 102.08)
"""
from dataclasses import dataclass
import math

PITCH = 0.024
SPROCKET_TEETH = 16
SPROCKET_RADIUS = PITCH / (2 * math.sin(math.pi / SPROCKET_TEETH))
TRACK_GAUGE = 0.318
COUNTS_PER_REV = 6533.0
MOTOR_RATED_V = 12.0


@dataclass(frozen=True)
class DriveLimits:
    max_linear: float = 0.35        # m/s, indoor companion speed
    max_angular: float = 1.2        # rad/s
    turn_slip: float = 1.35         # tracks skid when turning: commanded differential is scaled up
    max_wheel_rad_s: float = 100 / 60 * 2 * math.pi * 0.95  # 100 rpm no-load at 12 V, 5 % margin


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def twist_to_tracks(v, w, limits=DriveLimits()):
    """Body twist (m/s, rad/s) → (left, right) sprocket speeds in rad/s.

    Skid steering needs more track differential than an ideal diff drive for the same
    yaw rate; `turn_slip` is an empirical factor to be calibrated on the real floor."""
    v = clamp(v, -limits.max_linear, limits.max_linear)
    w = clamp(w, -limits.max_angular, limits.max_angular)
    half = TRACK_GAUGE / 2 * limits.turn_slip
    left = (v - w * half) / SPROCKET_RADIUS
    right = (v + w * half) / SPROCKET_RADIUS
    peak = max(abs(left), abs(right))
    if peak > limits.max_wheel_rad_s:          # keep the turn radius, slow both tracks
        k = limits.max_wheel_rad_s / peak
        left, right = left * k, right * k
    return left, right


def tracks_to_twist(left, right, limits=DriveLimits()):
    """Inverse of twist_to_tracks (rad/s → m/s, rad/s) for odometry."""
    vl, vr = left * SPROCKET_RADIUS, right * SPROCKET_RADIUS
    return (vl + vr) / 2, (vr - vl) / (TRACK_GAUGE * limits.turn_slip)


def counts_to_rad(counts):
    return counts / COUNTS_PER_REV * 2 * math.pi


def pwm_cap(vbat):
    """Max PWM duty so a 12 V motor never sees more than its rating from a 16.8 V pack."""
    if vbat <= 0:
        return 0.0
    return clamp(MOTOR_RATED_V / vbat, 0.0, 1.0)


class Odometry:
    """Planar dead reckoning from sprocket angles (rad). Tracks slip: fuse with LiDAR/IMU later."""

    def __init__(self, limits=DriveLimits()):
        self.limits = limits
        self.x = self.y = self.yaw = 0.0
        self._last = None

    def update(self, left_rad, right_rad):
        if self._last is None:
            self._last = (left_rad, right_rad)
            return self.x, self.y, self.yaw
        dl = (left_rad - self._last[0]) * SPROCKET_RADIUS
        dr = (right_rad - self._last[1]) * SPROCKET_RADIUS
        self._last = (left_rad, right_rad)
        ds = (dl + dr) / 2
        dyaw = (dr - dl) / (TRACK_GAUGE * self.limits.turn_slip)
        mid = self.yaw + dyaw / 2
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
        return self.x, self.y, self.yaw
