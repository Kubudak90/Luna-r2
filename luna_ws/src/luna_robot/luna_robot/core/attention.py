"""Turn toward a person: voice direction (mic array) and faces (face camera) → eye gaze on the screen and, when the
person is well off to the side, a slow turn of the whole body. R2 has a fixed head: the eyes do the small looks.

The eyes go to the person at once; the body follows slowly (turn_rate_deg_s) and only for someone clearly off to the
side: deadband_deg for a voice, face_deadband_deg for a face (the camera sees ±31°, so the body turns before the
face leaves the picture). Targets are kept as world bearings (odometry yaw + body bearing), so a voice heard before
the turn is still where it was while the body turns under it. Bearings are degrees, + = Luna's left.

Gaze for /luna/face/gaze: x = bearing / gaze_range_deg (+ = Luna's left, i.e. the viewer's right on the screen),
y = elevation / elevation_range_deg (+ = up), both clamped to ±1. A voice has no elevation (the array is planar):
listen_elevation_deg is assumed unless a face is seen close to the voice.

Who turns the body when:
- On the dock (CHARGING, DOCKED) Luna sleeps: no reaction at all.
- Only in dock state IDLE, and not while Nav2 or a teleop drives (`driving`), a bumper is pressed (`blocked`) or
  Luna takes a question someone asked by touching her face (`hold`: motor noise would be in the recording, and that
  person is right in front anyway). The eyes still go to the person.
- Luna's own voice (self_speaking) is not a speaker; during a reply Luna keeps facing the person.
- While the body turns (`turning`, measured), what the array hears is frozen: no update, no pause (motor noise, and
  the bearing moves under the array). The round body turns inside its own outline, so a turn can't sweep into
  someone next to it.
A turn ends with an explicit stop (turn = 0) for stop_ticks updates, so the wheels don't coast on a stale command."""
import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class AttentionConfig:
    gaze_range_deg: float = 60.0         # a person this far off centre puts the eyes at the screen edge
    elevation_range_deg: float = 30.0
    listen_elevation_deg: float = 15.0   # assumed for a voice with no face near it (CALIBRATE)
    turn_rate_deg_s: float = 30.0
    min_turn_rate_deg_s: float = 8.0     # slower than this the wheels only stall
    turn_gain: float = 2.0               # deg/s per deg left to turn, below turn_rate_deg_s
    deadband_deg: float = 12.0           # a voice this close to straight ahead: eyes only
    face_deadband_deg: float = 22.0
    settle_deg: float = 3.0              # a turn ends this close
    min_speech_s: float = 0.3            # speech this long before Luna reacts
    min_resultant: float = 0.6           # 0..1 direction consistency of the smoothed DoA
    smoothing_s: float = 0.4             # time constant of the circular average
    gap_s: float = 0.6                   # a pause longer than this starts a new utterance
    hold_s: float = 4.0                  # keep facing the person after they stop talking
    face_hold_s: float = 1.0             # ... and after their face leaves the picture
    face_join_deg: float = 20.0          # a face this close to the voice is the speaker
    stop_ticks: int = 3
    turn_states: frozenset = frozenset({'IDLE'})
    sleep_states: frozenset = frozenset({'CHARGING', 'DOCKED'})


@dataclass(frozen=True)
class AttentionOut:
    turn: float = None                   # rad/s for /luna/attention/cmd_vel, or None: leave the wheels alone
    gaze: tuple = None                   # (x, y) -1..1 for the face, or None: the face looks around by itself
    engaged: bool = False                # a voice is being followed now
    target: str = ''                     # 'voice' | 'face' | ''


def wrap_deg(a):
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def _clamp(v, lim):
    return max(-lim, min(lim, v))


@dataclass
class Attention:
    cfg: AttentionConfig = field(default_factory=AttentionConfig)

    def __post_init__(self):
        self.last_t = None
        self.ema = [0.0, 0.0]
        self.speech_start = self.last_speech = self.last_heard = self.last_face = None
        self.voice_world = self.face_world = 0.0
        self.face_elevation = self.cfg.listen_elevation_deg
        self.turning = False
        self.stops = 0

    def _forget_voice(self):
        self.ema = [0.0, 0.0]
        self.speech_start = self.last_speech = None

    def update(self, now, *, yaw_deg=0.0, bearing_deg=None, speech=False, face=None, dock_state='IDLE',
               self_speaking=False, turning=False, hold=False, driving=False, blocked=False):
        """face: (bearing deg, elevation deg) of the face to look at, or None. yaw_deg: odometry heading."""
        c = self.cfg
        dt = 0.0 if self.last_t is None else max(0.0, min(0.1, now - self.last_t))
        self.last_t = now
        asleep = dock_state in c.sleep_states
        engaged = False
        if asleep:
            self._forget_voice()
            self.last_heard = self.last_face = None
        elif turning:
            if self.last_speech is not None:
                self.last_speech = now                 # frozen: the turning time is neither speech nor a pause
        elif speech and bearing_deg is not None and not self_speaking:
            b = math.radians(yaw_deg + bearing_deg)
            a = 1.0 - math.exp(-dt / c.smoothing_s)
            self.ema = [self.ema[0] + a * (math.cos(b) - self.ema[0]), self.ema[1] + a * (math.sin(b) - self.ema[1])]
            self.last_speech = now
            if self.speech_start is None:
                self.speech_start = now
            if now - self.speech_start >= c.min_speech_s and math.hypot(*self.ema) >= c.min_resultant:
                engaged = True
                self.voice_world = math.degrees(math.atan2(self.ema[1], self.ema[0]))
                self.last_heard = now
        elif self.last_speech is not None and now - self.last_speech > c.gap_s:
            self._forget_voice()
        if face is not None and not asleep:
            self.face_world, self.face_elevation = yaw_deg + face[0], face[1]
            self.last_face = now
        voice = self.last_heard is not None and now - self.last_heard <= c.hold_s
        if voice and self_speaking:
            self.last_heard = now                      # replying: keep facing the person
        seen = self.last_face is not None and now - self.last_face <= c.face_hold_s
        target, world, elevation = '', 0.0, c.listen_elevation_deg
        if voice:
            target, world = 'voice', self.voice_world
            if seen and abs(wrap_deg(self.face_world - self.voice_world)) <= c.face_join_deg:
                world, elevation = self.face_world, self.face_elevation      # the speaker's own face
        elif seen:
            target, world, elevation = 'face', self.face_world, self.face_elevation
        if not target:
            self.turning = False
            return AttentionOut(self._stop(), None, engaged, '')
        rel = wrap_deg(world - yaw_deg)
        gaze = (round(_clamp(rel / c.gaze_range_deg, 1.0), 3), round(_clamp(elevation / c.elevation_range_deg, 1.0), 3))
        may_turn = dock_state in c.turn_states and not (hold or driving or blocked)
        if not may_turn:
            self.turning = False
            return AttentionOut(self._stop(), gaze, engaged, target)
        deadband = c.deadband_deg if target == 'voice' else c.face_deadband_deg
        if not self.turning and abs(rel) > deadband:
            self.turning = True
        if self.turning and abs(rel) <= c.settle_deg:
            self.turning = False
        if not self.turning:
            return AttentionOut(self._stop(), gaze, engaged, target)
        rate = min(c.turn_rate_deg_s, max(c.min_turn_rate_deg_s, c.turn_gain * abs(rel)))
        self.stops = c.stop_ticks
        return AttentionOut(math.radians(math.copysign(rate, rel)), gaze, engaged, target)

    def _stop(self):
        """0.0 for a few updates after a turn ends, then None."""
        if self.stops > 0:
            self.stops -= 1
            return 0.0
        return None


class MockSpeakers:
    """Scripted voices for running without the array: [(start_s, duration_s, world_bearing_deg)] repeating every
    period_s. sample() answers like the body-mounted array would: the bearing in base_link for the robot's current
    heading (yaw_deg), and the speech flag."""

    def __init__(self, script, period_s=20.0):
        self.script, self.period_s = script, period_s

    def sample(self, now, yaw_deg=0.0):
        t = now % self.period_s
        for start, duration, bearing in self.script:
            if start <= t < start + duration:
                return wrap_deg(bearing - yaw_deg), True
        return None, False
