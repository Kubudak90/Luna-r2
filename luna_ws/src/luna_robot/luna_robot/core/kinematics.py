"""Differential-drive kinematics for Luna R2. Pure Python: no ROS, no I/O, unit-testable anywhere.

Geometry (urdf/luna.urdf, docs/TASARIM_R2.md):
- two Pololu 80 × 10 mm wheels, radius 0.040 m, centres at y = ±0.150 m → separation 0.300 m
- Pololu 37D 70:1 12 V gearmotors with 64 CPR encoders: 64 × 70 = 4480 counts per wheel revolution, 150 rpm
  no-load at 12 V
- base_link is the midpoint of the wheels, so the robot turns in place about its own centre.
Two wheels and two casters barely slip, unlike R1's tracks: `turn_slip` stays a calibration knob (CALIBRATE, ≈ 1.0)
and the yaw comes from the depth camera's gyro when it is fresh (Odometry.update, base_node)."""
from dataclasses import dataclass
import math

WHEEL_RADIUS = 0.040
WHEEL_SEPARATION = 0.300
COUNTS_PER_REV = 4480.0
MOTOR_RATED_V = 12.0
NO_LOAD_RPM = 150.0


@dataclass(frozen=True)
class DriveLimits:
    max_linear: float = 0.30        # m/s, indoor companion speed; the cliff sensors are placed for it (TASARIM_R2)
    max_angular: float = 1.5        # rad/s
    turn_slip: float = 1.0          # CALIBRATE: > 1 if the real floor makes turns come out short
    max_wheel_rad_s: float = NO_LOAD_RPM / 60 * 2 * math.pi * 0.95   # 5 % below no-load at 12 V


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def twist_to_wheels(v, w, limits=DriveLimits()):
    """Body twist (m/s, rad/s) → (left, right) wheel speeds in rad/s. When a wheel would exceed its limit both are
    slowed by the same factor, so the turn radius is kept."""
    v = clamp(v, -limits.max_linear, limits.max_linear)
    w = clamp(w, -limits.max_angular, limits.max_angular)
    half = WHEEL_SEPARATION / 2 * limits.turn_slip
    left = (v - w * half) / WHEEL_RADIUS
    right = (v + w * half) / WHEEL_RADIUS
    peak = max(abs(left), abs(right))
    if peak > limits.max_wheel_rad_s:
        k = limits.max_wheel_rad_s / peak
        left, right = left * k, right * k
    return left, right


def wheels_to_twist(left, right, limits=DriveLimits()):
    """Inverse of twist_to_wheels (rad/s → m/s, rad/s) for odometry."""
    vl, vr = left * WHEEL_RADIUS, right * WHEEL_RADIUS
    return (vl + vr) / 2, (vr - vl) / (WHEEL_SEPARATION * limits.turn_slip)


def counts_to_rad(counts):
    return counts / COUNTS_PER_REV * 2 * math.pi


def pwm_cap(vbat):
    """Max PWM duty so a 12 V motor never sees more than its rating (LiFePO4 4S: up to 14.6 V on the charger)."""
    if not vbat > 0:        # also NaN (no BMS reading): never let NaN through as a duty cap
        return 0.0
    return clamp(MOTOR_RATED_V / vbat, 0.0, 1.0)


class GyroBias:
    """Gyro z bias, learnt while both wheels stand still (a MEMS gyro drifts with temperature)."""

    def __init__(self, tau_s=5.0, still_rad_s=0.02):
        self.tau_s, self.still_rad_s = tau_s, still_rad_s
        self.bias = 0.0
        self.learnt_s = 0.0

    def update(self, gyro_z, wheel_left, wheel_right, dt):
        """Bias-corrected rate (rad/s); learns the bias when the wheels (rad/s) stand still."""
        if abs(wheel_left) < self.still_rad_s and abs(wheel_right) < self.still_rad_s and dt > 0:
            a = min(1.0, dt / self.tau_s)
            self.bias += a * (gyro_z - self.bias)
            self.learnt_s += dt
        return gyro_z - self.bias


class Odometry:
    """Planar dead reckoning from wheel angles (rad). The heading change comes from the gyro when the caller gives
    one for the interval (dyaw_gyro, rad), otherwise from the wheel difference."""

    def __init__(self, limits=DriveLimits()):
        self.limits = limits
        self.x = self.y = self.yaw = 0.0
        self._last = None

    def update(self, left_rad, right_rad, dyaw_gyro=None):
        if self._last is None:
            self._last = (left_rad, right_rad)
            return self.x, self.y, self.yaw
        dl = (left_rad - self._last[0]) * WHEEL_RADIUS
        dr = (right_rad - self._last[1]) * WHEEL_RADIUS
        self._last = (left_rad, right_rad)
        ds = (dl + dr) / 2
        dyaw = (dr - dl) / (WHEEL_SEPARATION * self.limits.turn_slip) if dyaw_gyro is None else dyaw_gyro
        mid = self.yaw + dyaw / 2
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
        return self.x, self.y, self.yaw
