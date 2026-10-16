"""Docking state machine for the front-centre recessed receiver and the wall-backed dock.

The robot drives forward into the dock: receiver mouth Y−170, dock nose face Y−187/−154.
Guidance is an AprilTag on the wall marker plate (REF_R02_TAG, centre ≈ 382 mm high) seen by the
head cameras, plus the DOCK_SENSE contact (P4) for confirmation. Pure logic: the ROS node feeds
observations and applies the returned twist through the safety gate."""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class DockConfig:
    seek_turn: float = 0.35           # rad/s while searching for the tag
    align_tol: float = 0.06           # normalised horizontal tag offset
    approach_speed: float = 0.06      # m/s
    creep_speed: float = 0.025        # m/s in the last 60 mm
    creep_distance: float = 0.06      # m of tag-to-mouth distance where creeping starts
    k_turn: float = 1.2               # rad/s per unit tag offset
    seek_timeout: float = 40.0
    approach_timeout: float = 25.0
    contact_settle: float = 1.0       # s of stable DOCK_SENSE before charging is expected
    charge_wait: float = 4.0          # s to see charge current after contact
    sense_loss_s: float = 0.2         # s of continuous DOCK_SENSE loss before contact counts as lost
    charge_loss_s: float = 1.0        # s without charge current before CHARGING parks as DOCKED (BMS: 5 Hz)
    backoff_time: float = 2.5
    max_retries: int = 3


@dataclass
class Obs:
    tag_visible: bool = False
    tag_offset: float = 0.0           # -1 (left) .. 1 (right) in the image
    tag_distance: float = 9.9         # m, mouth-to-nose estimate
    dock_sense: bool = False
    charging: bool = False
    front_bumper: bool = False


@dataclass
class Dock:
    cfg: DockConfig = field(default_factory=DockConfig)
    state: str = 'IDLE'
    t: float = 0.0
    retries: int = 0
    lost: float = 0.0                 # s DOCK_SENSE has been continuously false in CONTACT/CHARGING
    uncharged: float = 0.0            # s charge current has been continuously absent in CHARGING

    def _go(self, state):
        self.state, self.t, self.lost, self.uncharged = state, 0.0, 0.0, 0.0

    def request(self, what):
        if what == 'dock' and self.state in ('IDLE', 'FAULT'):
            self.retries = 0
            self._go('SEEK')
        elif what == 'undock' and self.state in ('DOCKED', 'CHARGING', 'CONTACT'):
            self._go('UNDOCK')
        elif what == 'cancel':
            self._go('IDLE')

    def step(self, o: Obs, dt):
        """Advance one tick. Returns (linear m/s, angular rad/s)."""
        c = self.cfg
        self.t += dt
        s = self.state
        if s in ('IDLE', 'FAULT', 'DOCKED'):
            return 0.0, 0.0
        if s == 'SEEK':
            if o.tag_visible:
                self._go('ALIGN')
                return 0.0, 0.0
            if self.t > c.seek_timeout:
                self._go('FAULT')
                return 0.0, 0.0
            return 0.0, c.seek_turn
        if s == 'ALIGN':
            if not o.tag_visible:
                self._go('SEEK')
                return 0.0, 0.0
            if abs(o.tag_offset) <= c.align_tol:
                self._go('APPROACH')
                return 0.0, 0.0
            return 0.0, -c.k_turn * o.tag_offset
        if s == 'APPROACH':
            if o.dock_sense:
                self._go('CONTACT')
                return 0.0, 0.0
            if o.front_bumper or self.t > c.approach_timeout or not o.tag_visible:
                return self._retry()
            speed = c.creep_speed if o.tag_distance < c.creep_distance else c.approach_speed
            return speed, -c.k_turn * o.tag_offset
        if s in ('CONTACT', 'CHARGING'):
            # The MCU publishes one undebounced ADC read per tick, so contact bounce must not end the
            # dock: only `sense_loss_s` of continuous loss counts, and any true sample restarts it.
            if not o.dock_sense:
                self.lost += dt
                return self._retry() if self.lost >= c.sense_loss_s else (0.0, 0.0)
            self.lost = 0.0
        if s == 'CONTACT':
            if o.charging and self.t >= c.contact_settle:
                self._go('CHARGING')
            elif self.t > c.contact_settle + c.charge_wait:
                return self._retry()
            return 0.0, 0.0
        if s == 'CHARGING':
            # Charge current drops with every contact bounce and the BMS reports it 5×/s, so it can come
            # back after DOCK_SENSE does: only `charge_loss_s` without it means full or charger paused.
            if o.charging:
                self.uncharged = 0.0
            else:
                self.uncharged += dt
                if self.uncharged >= c.charge_loss_s:
                    self._go('DOCKED')        # full or charger paused: stay parked
            return 0.0, 0.0
        if s == 'BACKOFF':
            if self.t >= c.backoff_time:
                self._go('SEEK')
                return 0.0, 0.0
            return -c.approach_speed, 0.0
        if s == 'UNDOCK':
            if self.t >= c.backoff_time:
                self._go('IDLE')
                return 0.0, 0.0
            return -c.approach_speed, 0.0
        raise AssertionError(s)

    def _retry(self):
        self.retries += 1
        self._go('FAULT' if self.retries > self.cfg.max_retries else 'BACKOFF')
        return 0.0, 0.0
