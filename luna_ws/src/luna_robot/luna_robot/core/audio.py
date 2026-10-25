"""Microphone PCM helpers: pick one channel of the mic array stream, measure its level and cut it into utterances
for a speech-to-text plug-in, and gate listening by mute, body turning and a touch on the face (push-to-talk).
Signed 16-bit little-endian PCM
(ALSA S16_LE), standard library only."""
import math
import os
import sys
import time
import wave
from array import array
from collections import deque
from dataclasses import dataclass


def channel(raw, channels, index):
    """One channel of interleaved S16_LE bytes; a torn last frame is dropped."""
    frame = 2 * channels
    samples = array('h', raw[:len(raw) - len(raw) % frame])
    if sys.byteorder == 'big':
        samples.byteswap()
    return samples[index::channels]


def rms_dbfs(samples):
    if not samples:
        return -120.0
    rms = math.sqrt(sum(s * s for s in samples) / len(samples))
    return max(-120.0, 20 * math.log10(rms / 32768)) if rms > 0 else -120.0


@dataclass(frozen=True)
class SegmenterConfig:
    rate: int = 16000
    pre_roll_s: float = 0.3      # audio kept from before the speech flag rose (word onsets)
    hangover_s: float = 0.7      # silence that ends an utterance; shorter pauses stay inside it
    min_s: float = 0.4           # voiced time below this is a click or a cough, dropped
    max_s: float = 12.0          # an utterance is cut here even if speech goes on
    energy_dbfs: float = -42.0   # energy VAD, only when the array gives no speech flag (CALIBRATE)


class Segmenter:
    """feed(chunk, speech) → finished utterances (array('h')). speech=None falls back to the energy VAD."""

    def __init__(self, cfg=SegmenterConfig()):
        self.cfg = cfg
        self.pre = deque()
        self.pre_n = 0
        self.cur = None
        self.voiced = self.silence = 0

    @property
    def busy(self):
        """An utterance has started and not ended yet."""
        return self.cur is not None

    def reset(self):
        """Drop everything, including an utterance in progress (mute)."""
        self.pre.clear()
        self.pre_n = 0
        self.cur = None
        self.voiced = self.silence = 0

    def feed(self, chunk, speech=None, hold=False, freeze=False):
        """hold: no new utterance may start (body turning: motor noise); one already started runs to its end.
        freeze: the chunk is neither speech nor silence (body turning: the speech flag is not trusted), so a running
        utterance keeps it but neither ends nor counts it as voiced."""
        cfg, n = self.cfg, len(chunk)
        voiced = rms_dbfs(chunk) > cfg.energy_dbfs if speech is None else speech
        out = []
        if self.cur is None:
            if not voiced or hold or freeze:
                self.pre.append(chunk)
                self.pre_n += n
                while self.pre and self.pre_n - len(self.pre[0]) >= cfg.pre_roll_s * cfg.rate:
                    self.pre_n -= len(self.pre.popleft())
                return out
            self.cur = array('h')
            for c in self.pre:
                self.cur.extend(c)
            self.pre.clear()
            self.pre_n = 0
            self.voiced = self.silence = 0
        self.cur.extend(chunk)
        if freeze:
            pass
        elif voiced:
            self.voiced += n
            self.silence = 0
        else:
            self.silence += n
        if len(self.cur) >= cfg.max_s * cfg.rate or self.silence >= cfg.hangover_s * cfg.rate:
            if self.voiced >= cfg.min_s * cfg.rate:
                out.append(self.cur)
            self.cur = None
        return out


@dataclass(frozen=True)
class MicGateConfig:
    turning_rad_s: float = 0.15     # body yaw rate (odometry) faster than this = turning
    settle_s: float = 0.3           # gearbox noise and the bearing settling after the wheels stop


class MicGate:
    """When Luna may listen. Muted (software mute, or the privacy slider has cut the array's power): never.
    Body turning in place (gearmotor noise, and the bearing moving under the array) and settle_s after: the speech
    flag and DoA are not trusted and no new utterance starts. body() takes odometry yaw-rate samples (rad/s)."""

    def __init__(self, cfg=MicGateConfig(), muted=False):
        self.cfg, self.muted = cfg, muted
        self.moving_until = -1.0

    def body(self, now, yaw_rate):
        if abs(yaw_rate) > self.cfg.turning_rad_s:
            self.moving_until = now + self.cfg.settle_s

    def moving(self, now):
        return now < self.moving_until

    def speech(self, now, speech):
        return bool(speech) and not self.muted and not self.moving(now)

    def may_start_utterance(self, now):
        return not self.muted and not self.moving(now)


LISTEN_MODES = ('push_to_talk', 'always')


@dataclass(frozen=True)
class ListenConfig:
    mode: str = 'push_to_talk'      # push_to_talk: only after a touch on the face; always: every utterance
    window_s: float = 6.0           # after a touch: time to start speaking
    min_gap_s: float = 0.5          # touches closer than this are the same touch (a double tap, a tap then a hold)


class ListenWindow:
    """Which utterances become questions. push_to_talk: one utterance that starts within window_s of a touch on the
    face (talk()); it runs to its end even past the window, and the window closes once it is kept, so one touch is
    one question. always: every utterance (for a later wake word). Nothing outside the window is written anywhere.
    Times are time.monotonic() seconds."""

    def __init__(self, cfg=ListenConfig()):
        if cfg.mode not in LISTEN_MODES:
            raise ValueError(f'listen mode {cfg.mode!r}: expected one of {LISTEN_MODES}')
        self.cfg = cfg
        self.until = self.last_talk = float('-inf')
        self.active = False             # an accepted utterance is running

    @property
    def push_to_talk(self):
        return self.cfg.mode == 'push_to_talk'

    def talk(self, now):
        """A touch. True if it opened (or reopened) the window, False if it repeats the last touch."""
        if now - self.last_talk < self.cfg.min_gap_s:
            return False
        self.last_talk, self.until = now, now + self.cfg.window_s
        return True

    def may_start(self, now):
        return not self.push_to_talk or now < self.until

    def started(self):
        self.active = True

    def finished(self, kept):
        """The running utterance ended; kept = long enough to be a question (Segmenter returned it)."""
        self.active = False
        if kept and self.push_to_talk:
            self.until = float('-inf')

    def cancel(self):
        """Mute: close the window; the segmenter drops the running utterance itself."""
        self.until = float('-inf')
        self.active = False

    def listening(self, now):
        """Luna is taking a question: an accepted utterance runs, or (push_to_talk) the window is open."""
        return self.active or (self.push_to_talk and now < self.until)


class Listener:
    """Capture chunks → question utterances: the Segmenter, gated by the MicGate (mute, body turning), Luna's own
    voice and the ListenWindow. Not thread-safe: the caller serialises feed() with window.talk()."""

    def __init__(self, segmenter, gate, window):
        self.seg, self.gate, self.window = segmenter, gate, window

    def feed(self, samples, now, speech=None, speaking=False):
        """speech: the array's flag (None: energy VAD); speaking: Luna's voice is on the speaker."""
        if self.gate.muted:
            self.seg.reset()                     # muted: drop the audio, even an utterance in progress
            self.window.cancel()
            return []
        hold = not self.gate.may_start_utterance(now) or speaking or not self.window.may_start(now)
        was = self.seg.busy
        done = self.seg.feed(samples, speech=speech, hold=hold, freeze=self.gate.moving(now))
        if self.seg.busy and not was:
            self.window.started()
        elif was and not self.seg.busy:
            self.window.finished(kept=bool(done))
        return done


class UtteranceSpool:
    """Writes utterances as 16-bit mono WAV files and keeps only the newest `keep_last` of its own files.
    They are recordings of people at home: keep the spool small and local (default under /tmp)."""
    PREFIX = 'luna_utt_'

    def __init__(self, directory, rate=16000, keep_last=5):
        self.dir, self.rate, self.keep_last, self.seq = directory, rate, keep_last, 0
        os.makedirs(directory, exist_ok=True)

    def write(self, samples):
        self.seq += 1
        path = os.path.join(self.dir, f'{self.PREFIX}{int(time.time() * 1000):013d}_{self.seq:05d}.wav')
        data = array('h', samples)
        if sys.byteorder == 'big':
            data.byteswap()
        with wave.open(path, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(self.rate)
            w.writeframes(data.tobytes())
        ours = sorted(f for f in os.listdir(self.dir) if f.startswith(self.PREFIX) and f.endswith('.wav'))
        for old in ours[:max(0, len(ours) - self.keep_last)]:
            os.remove(os.path.join(self.dir, old))
        return path
