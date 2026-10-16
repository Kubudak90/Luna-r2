"""Pan/tilt head mapping for two goBILDA 2000-0025-0002 servos (servo mode 500–2500 µs ↔ 300°).

Mechanical limits come from R03/R05 CAD: pan ±60°, tilt ±12° (hard stops reviewed at ±62°/±14°).
The CAD spline phase (pan 42.84°, tilt 42.36°) is a CAD mating phase, not a live calibration:
`center_us` must be measured on the assembled head before the limits are trusted."""
from dataclasses import dataclass

US_PER_DEG = 2000 / 300  # 6.667 µs per degree


@dataclass
class Axis:
    name: str
    limit_deg: float
    center_us: float = 1500.0
    direction: int = 1               # +1 or -1, set during calibration
    max_rate_deg_s: float = 90.0

    def to_us(self, deg):
        deg = max(-self.limit_deg, min(self.limit_deg, deg))
        return self.center_us + self.direction * deg * US_PER_DEG


class Head:
    def __init__(self, pan=None, tilt=None):
        self.pan = pan or Axis('pan', 60.0)
        self.tilt = tilt or Axis('tilt', 12.0, max_rate_deg_s=45.0)
        self.state = [0.0, 0.0]
        self.target = [0.0, 0.0]

    def set_target(self, pan_deg, tilt_deg):
        self.target = [max(-self.pan.limit_deg, min(self.pan.limit_deg, pan_deg)),
                       max(-self.tilt.limit_deg, min(self.tilt.limit_deg, tilt_deg))]

    def step(self, dt):
        """Slew-rate-limited motion; returns (pan_us, tilt_us) to send to the MCU."""
        for i, axis in enumerate((self.pan, self.tilt)):
            err = self.target[i] - self.state[i]
            lim = axis.max_rate_deg_s * dt
            self.state[i] += max(-lim, min(lim, err))
        return self.pan.to_us(self.state[0]), self.tilt.to_us(self.state[1])

    def gaze(self):
        """Normalised gaze for the face (-1..1): eyes follow where the head points."""
        return self.state[0] / self.pan.limit_deg, -self.state[1] / self.tilt.limit_deg
