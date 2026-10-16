"""Motion safety gate between any velocity source and the tracks. Pure function, unit-tested.

Inputs mirror the MCU topics: E-stop, four bumper segments (bit 0 FL, 1 FR, 2 BL, 3 BR),
four floor-facing VL53L4CD ranges (FL, BL, FR, BR in metres) and command age.
The hardware E-stop also cuts the motor driver SLP line; this gate is the software layer."""
from dataclasses import dataclass

FRONT_BUMPERS = 0b0011
REAR_BUMPERS = 0b1100


@dataclass(frozen=True)
class SafetyConfig:
    cmd_timeout_s: float = 0.3
    floor_nominal_m: float = 0.055      # sensor window Z52..60 → floor ≈ 55 mm (to be measured)
    cliff_margin_m: float = 0.04        # >95 mm means no floor below that corner
    sensor_invalid_m: float = 0.0       # 0 or negative = no reading → treat as cliff


@dataclass(frozen=True)
class Verdict:
    linear: float
    angular: float
    reason: str


def gate(linear, angular, *, estop, bumpers, cliff, cmd_age_s, cfg=SafetyConfig()):
    if estop:
        return Verdict(0.0, 0.0, 'estop')
    if cmd_age_s > cfg.cmd_timeout_s:
        return Verdict(0.0, 0.0, 'timeout')
    limit = cfg.floor_nominal_m + cfg.cliff_margin_m
    drop = [r <= cfg.sensor_invalid_m or r > limit for r in cliff]
    front_drop = drop[0] or drop[2]
    rear_drop = drop[1] or drop[3]
    reason = 'ok'
    if (bumpers & FRONT_BUMPERS or front_drop) and linear > 0:
        linear, reason = 0.0, 'front_blocked'
    if (bumpers & REAR_BUMPERS or rear_drop) and linear < 0:
        linear, reason = 0.0, 'rear_blocked'
    if front_drop and rear_drop:
        return Verdict(0.0, 0.0, 'lifted_or_cliff')   # likely picked up: stop everything
    if any(drop) and angular != 0 and reason == 'ok':
        reason = 'turn_near_edge'                         # turning in place is allowed but flagged
    return Verdict(linear, angular, reason)
