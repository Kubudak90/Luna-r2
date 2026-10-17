"""Turn toward a voice: XVF3800 direction of arrival → eye gaze for the face and a gentle head turn.

The eyes go to the voice at once; the head follows slowly (turn_rate_deg_s, well under head_node's own slew)
and only for a voice clearly off to the side (deadband_deg). Eye gaze uses the head gaze convention
(pan / pan limit, -tilt / tilt limit), so when attention lets go the face is already where head gaze points.

Bearings are base_link degrees (+ = left, the head pan convention). The array turns with the head (R07: under
the head top shell); audio_node turns its head-relative DoA into base_link with core/head_frame.py first.
While the head moves (servo noise next to a mic, R07) attention freezes what it heard: no update, no pause.

Who moves the head when:
- On the dock (CHARGING, DOCKED) Luna sleeps: no reaction.
- Only in IDLE does the head turn. While docking, the cameras need the head forward: the eyes glance, the head
  stays, and a head that was already turned is brought back to centre first.
- Luna's own voice (self_speaking) is not a speaker; during a reply Luna keeps facing the person instead.
Attention publishes head targets only while it is using the head and until the head is back at centre."""
import math
from dataclasses import dataclass, field

from .head_frame import base_to_head


@dataclass(frozen=True)
class AttentionConfig:
    pan_limit_deg: float = 60.0         # core/head.py mechanical limits: never exceeded
    tilt_limit_deg: float = 12.0
    max_pan_deg: float = 50.0           # attention's own, softer pan limit
    listen_tilt_deg: float = 4.0        # look slightly up at a standing person (CALIBRATE)
    turn_rate_deg_s: float = 30.0       # head_node slews at 90 °/s; this is the gentle part
    deadband_deg: float = 12.0          # a voice this close to where the head looks: eyes only
    min_speech_s: float = 0.3           # speech this long before Luna reacts
    min_resultant: float = 0.6          # 0..1 direction consistency of the smoothed DoA
    smoothing_s: float = 0.4            # time constant of the circular average
    gap_s: float = 0.6                  # a pause longer than this starts a new utterance
    hold_s: float = 4.0                 # keep facing the person after they stop talking
    head_states: frozenset = frozenset({'IDLE'})
    sleep_states: frozenset = frozenset({'CHARGING', 'DOCKED'})


@dataclass(frozen=True)
class AttentionOut:
    head_target: tuple = None           # (pan, tilt) deg for /luna/head/target, or None: leave the head alone
    gaze: tuple = None                  # (x, y) -1..1 for the face, or None: face uses the head gaze
    engaged: bool = False


def _clamp(v, lim):
    return max(-lim, min(lim, v))


@dataclass
class Attention:
    cfg: AttentionConfig = field(default_factory=AttentionConfig)

    def __post_init__(self):
        self.last_t = None
        self.ema = [0.0, 0.0]
        self.speech_start = self.last_speech = self.last_heard = None
        self.bearing = 0.0
        self.goal = [0.0, 0.0]
        self.cmd = [0.0, 0.0]

    def _forget(self):
        self.ema = [0.0, 0.0]
        self.speech_start = self.last_speech = None

    def update(self, now, *, bearing_deg=None, speech=False, dock_state='IDLE', self_speaking=False, head_moving=False):
        c = self.cfg
        # dt is capped: after a stalled timer (CPU load) the head catches up gently instead of jumping.
        dt = 0.0 if self.last_t is None else max(0.0, min(0.1, now - self.last_t))
        self.last_t = now
        asleep = dock_state in c.sleep_states
        hearing = bool(speech) and bearing_deg is not None and not self_speaking and not asleep
        engaged = False
        if head_moving and not asleep:
            if self.last_speech is not None:
                self.last_speech = now                 # frozen: the moving time is neither speech nor a pause
        elif hearing:
            b = math.radians(bearing_deg)
            a = 1.0 - math.exp(-dt / c.smoothing_s)
            self.ema = [self.ema[0] + a * (math.cos(b) - self.ema[0]), self.ema[1] + a * (math.sin(b) - self.ema[1])]
            self.last_speech = now
            if self.speech_start is None:
                self.speech_start = now
            if now - self.speech_start >= c.min_speech_s and math.hypot(*self.ema) >= c.min_resultant:
                engaged = True
                self.bearing = math.degrees(math.atan2(self.ema[1], self.ema[0]))
                self.last_heard = now
        elif self.last_speech is not None and now - self.last_speech > c.gap_s:
            self._forget()
        if asleep:
            self._forget()
            self.last_heard = None
        active = self.last_heard is not None and now - self.last_heard <= c.hold_s
        if active and self_speaking:
            self.last_heard = now                      # replying: keep facing the person
        tilt = _clamp(c.listen_tilt_deg, c.tilt_limit_deg)
        use_head = active and dock_state in c.head_states
        if use_head:
            desired = round(_clamp(self.bearing, min(c.max_pan_deg, c.pan_limit_deg)), 2)
            if abs(desired - self.goal[0]) > c.deadband_deg:
                self.goal[0] = desired
            self.goal[1] = tilt
        else:
            self.goal = [0.0, 0.0]
        was_off_centre = self.cmd != [0.0, 0.0]
        step = c.turn_rate_deg_s * dt
        for i in (0, 1):
            err = self.goal[i] - self.cmd[i]
            self.cmd[i] = self.goal[i] if abs(err) <= step else self.cmd[i] + math.copysign(step, err)
        head_target = tuple(self.cmd) if use_head or was_off_centre else None
        gaze = None
        if active:
            gaze = (round(_clamp(self.bearing, c.pan_limit_deg) / c.pan_limit_deg, 3), round(-tilt / c.tilt_limit_deg, 3))
        return AttentionOut(head_target, gaze, engaged)


class MockSpeakers:
    """Scripted voices for running without the array: [(start_s, duration_s, base_bearing_deg)] repeating every
    period_s, at elevation_deg. sample() answers like the head-mounted array would: azimuth in the head frame
    for the current pan/tilt, and the speech flag."""

    def __init__(self, script, period_s=20.0, elevation_deg=25.0):
        self.script, self.period_s, self.elevation_deg = script, period_s, elevation_deg

    def sample(self, now, pan_deg=0.0, tilt_deg=0.0):
        t = now % self.period_s
        for start, duration, bearing in self.script:
            if start <= t < start + duration:
                return base_to_head(bearing, self.elevation_deg, pan_deg, tilt_deg), True
        return None, False
