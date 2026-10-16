"""4S3P Li-ion (12 × Supex 26650) state estimate and power policy.

Prefers the BMS state of charge when present (JBD UART); otherwise uses a rested-voltage
curve for NMC cells. Voltage SoC under load is only a rough guide — the thresholds below
include hysteresis so the robot does not flap between modes."""
from dataclasses import dataclass

SERIES = 4
# Typical NMC open-circuit curve, V/cell → %. Replace with a curve measured on the real cells.
OCV = [(3.30, 0), (3.50, 5), (3.60, 12), (3.68, 25), (3.74, 40), (3.80, 55),
       (3.87, 68), (3.95, 80), (4.05, 90), (4.15, 98), (4.20, 100)]


def soc_from_voltage(pack_v):
    v = pack_v / SERIES
    if v <= OCV[0][0]:
        return 0.0
    for (v0, s0), (v1, s1) in zip(OCV, OCV[1:]):
        if v <= v1:
            return s0 + (s1 - s0) * (v - v0) / (v1 - v0)
    return 100.0


@dataclass(frozen=True)
class Policy:
    seek_dock_below: float = 25.0
    critical_below: float = 10.0
    resume_above: float = 90.0
    cell_overtemp_c: float = 50.0


def decide(soc, *, charging, docked, temp_c, seeking, policy=Policy()):
    """Return (action, agx_power_mode_w). action ∈ run, seek_dock, critical, charge, overtemp."""
    if temp_c is not None and temp_c >= policy.cell_overtemp_c:
        return 'overtemp', 10
    if docked and charging:
        return ('charge' if soc < policy.resume_above else 'run'), 10
    if soc < policy.critical_below:
        return 'critical', 10
    if soc < policy.seek_dock_below or (seeking and soc < policy.seek_dock_below + 5):
        return 'seek_dock', 15
    return 'run', 15
