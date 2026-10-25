"""4S2P LiFePO4 (8 × 32700, 3.2 V 6 Ah: 12.8 V nominal, 12 Ah, 154 Wh) state estimate and power policy.

LiFePO4 instead of R1's NMC: much harder to push into thermal runaway, which matters in a child's room, and the
Daly smart BMS that reads it out over UART is sold in Turkey (docs/MALZEME_LISTESI.md). Its voltage curve is flat
between 10 % and 90 %, so the state of charge comes from the BMS (coulomb counting); the voltage curve below is only a
fallback for when the BMS sends none. The thresholds include hysteresis so the robot does not flap between modes."""
from dataclasses import dataclass

SERIES = 4
# Typical LiFePO4 rested curve, V/cell → %. Replace with a curve measured on the real cells.
OCV = [(2.80, 0), (3.00, 2), (3.15, 6), (3.22, 12), (3.25, 20), (3.28, 30), (3.30, 40), (3.31, 50),
       (3.32, 60), (3.33, 70), (3.34, 80), (3.36, 90), (3.40, 96), (3.45, 99), (3.60, 100)]


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
    cell_overtemp_c: float = 50.0     # LiFePO4 discharges to 60 °C; the Daly BMS stops charging above 45 °C itself


def decide(soc, *, charging, docked, temp_c, seeking, policy=Policy()):
    """Return (action, jetson_power_mode_w). action ∈ run, seek_dock, critical, charge, overtemp. The power modes are
    Orin NX 16 GB nvpmodel budgets: 10 W while charging, low or hot, 15 W otherwise (25 W only by hand)."""
    if temp_c is not None and temp_c >= policy.cell_overtemp_c:
        return 'overtemp', 10
    if docked and charging:
        return ('charge' if soc < policy.resume_above else 'run'), 10
    if soc < policy.critical_below:
        return 'critical', 10
    if soc < policy.seek_dock_below or (seeking and soc < policy.seek_dock_below + 5):
        return 'seek_dock', 15
    return 'run', 15
