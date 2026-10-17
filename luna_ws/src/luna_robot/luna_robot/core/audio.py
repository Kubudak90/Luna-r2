"""Microphone PCM helpers: pick one channel of the XVF3800 stream, measure its level and cut it into utterances
for a speech-to-text plug-in. Signed 16-bit little-endian PCM (ALSA S16_LE), standard library only."""
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

    def feed(self, chunk, speech=None):
        cfg, n = self.cfg, len(chunk)
        voiced = rms_dbfs(chunk) > cfg.energy_dbfs if speech is None else speech
        out = []
        if self.cur is None:
            if not voiced:
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
        if voiced:
            self.voiced += n
            self.silence = 0
        else:
            self.silence += n
        if len(self.cur) >= cfg.max_s * cfg.rate or self.silence >= cfg.hangover_s * cfg.rate:
            if self.voiced >= cfg.min_s * cfg.rate:
                out.append(self.cur)
            self.cur = None
        return out


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
