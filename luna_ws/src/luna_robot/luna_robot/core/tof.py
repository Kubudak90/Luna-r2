"""Corner ToF (4 × VL53L1X) as sensor_msgs/Range for Nav2. The MCU publishes the raw readings on /luna/mcu/corners
[FL, FR, BL, BR] (m); tof_range_node turns each one into a Range in its urdf tof_* frame (x along the beam).

Usable range: the cone of a window 0.101 m above the floor reaches the floor too. With the full 16×16 SPAD ROI
(27° × 27°) that happens 0.42 m out, so every reading would be the floor. The simulator assumes the ROI narrowed to
16 × 4 SPADs (27° wide, ≈ 15° tall: floor from ≈ 0.77 m) and only readings below max_m count as an obstacle; a
reading at or beyond it means "nothing nearer" (range = max_range, which clears RangeSensorLayer's cone).
The firmware does not set that ROI yet (see README_TR.md, "Engel algılama")."""
import math
from dataclasses import dataclass

NAMES = ('fl', 'fr', 'bl', 'br')            # /luna/mcu/corners order
FRAMES = tuple(f'tof_{n}' for n in NAMES)  # urdf frames


@dataclass(frozen=True)
class TofConfig:
    fov_rad: float = math.radians(27.0)    # horizontal width of the ROI; RangeSensorLayer draws a 2D cone of this
    min_m: float = 0.04                     # VL53L1X minimum
    max_m: float = 0.70                     # below the floor return of the 16×4 ROI (≈ 0.77 m)


def to_range(raw, cfg=TofConfig()):
    """Range to publish for one raw reading (m), or None for no message (≤ 0 or NaN: sensor dead or not ranging; a
    broken sensor must not keep clearing the costmap). Readings ≥ max_m become max_m, closer than min_m become min_m."""
    if raw is None or math.isnan(raw) or raw <= 0.0:
        return None
    if raw >= cfg.max_m:
        return cfg.max_m
    return max(raw, cfg.min_m)
