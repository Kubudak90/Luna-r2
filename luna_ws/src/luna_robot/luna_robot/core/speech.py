"""Speaker output for assistant replies, with the text-to-speech engine left pluggable.

The speaker (50 mm 4 Ω 3 W in a closed box behind the torso grille, PAM8403 amplifier) hangs on the ReSpeaker Mic
Array v2.0's 3.5 mm output, so audio is played to the array's ALSA card: that playback is also what its echo
canceller hears as reference. With the privacy slider closed the array has no power, so Luna stays silent too.

Engines: NullTts (no voice, waits about as long as speaking would take, so listening/speaking states stay real),
CommandTts (any command line, e.g. espeak-ng, optionally piped into aplay) and HttpWavTts (Piper's HTTP server:
the voice stays loaded, each sentence comes back as a WAV and is played with aplay). Commands are split into argv
once and the text is substituted as a whole argument: no shell, so a reply cannot inject commands.

Piper (OHF-Voice/piper1-gpl, docs/API_HTTP.md, read 2026-10-25): `python3 -m piper.http_server -m <voice>`
serves POST /synthesize with JSON {"text", "length_scale", ...} → WAV. Turkish voices in rhasspy/piper-voices on
that day: only tr_TR-dfki-medium (fahrettin and fettah were removed at their contributors' request)."""
import json
import re
import shlex
import subprocess
import threading
from collections import deque

from .assistant import TransportError, urllib_transport

_LINK = re.compile(r'\[([^\]]*)\]\([^)]*\)')
_URL = re.compile(r'https?://\S+')
_MARK = re.compile(r'[*_`#>|~]+')
_BULLET = re.compile(r'(?m)^\s*(?:[-+•]|\d+[.)])\s+')
_SENTENCE = re.compile(r'(?<=[.!?…])\s+')


def speakable(text, max_chars=400):
    """Markdown and links out, whitespace collapsed, cut at the last sentence end that fits."""
    t = _BULLET.sub('', _URL.sub('', _LINK.sub(r'\1', text or '')))
    t = ' '.join(_MARK.sub('', t).split())
    if len(t) <= max_chars:
        return t
    cut = t[:max_chars]
    ends = [m.end() for m in re.finditer(r'[.!?…]', cut)]
    return cut[:ends[-1]] if ends else cut


def split_sentences(text):
    return [s for s in _SENTENCE.split(text.strip()) if s]


class SpeechQueue:
    """Sentences waiting to be spoken; when full, the oldest are dropped (a stale answer is worse than a cut one)."""

    def __init__(self, max_items=8):
        self.items = deque(maxlen=max_items)
        self.cond = threading.Condition()

    def push(self, text):
        with self.cond:
            self.items.extend(split_sentences(text))
            self.cond.notify()

    def pop(self, timeout=None):
        """Next sentence or None; waits up to `timeout` seconds when given."""
        with self.cond:
            if not self.items and timeout:
                self.cond.wait(timeout)
            return self.items.popleft() if self.items else None

    def clear(self):
        with self.cond:
            self.items.clear()


class NullTts:
    def __init__(self, chars_per_s=14.0, sleep=None):
        self.chars_per_s = chars_per_s
        self._stop = threading.Event()
        self._sleep = sleep or self._stop.wait

    def say(self, text):
        self._stop.clear()
        self._sleep(len(text) / self.chars_per_s)

    def stop(self):
        self._stop.set()


class CommandTts:
    """synth: argv template with {text} (and optionally {device}); player: argv template reading the synth's
    stdout (e.g. ['aplay', '-q', '-D', '{device}']), or None if the synth plays by itself."""

    def __init__(self, synth, player=None, device='default', popen=subprocess.Popen, timeout_s=60.0):
        self.synth, self.player, self.device, self.popen, self.timeout_s = synth, player, device, popen, timeout_s
        self.procs = []
        self.lock = threading.Lock()

    def _argv(self, template, text):
        return [a.replace('{text}', text).replace('{device}', self.device) for a in template]

    def say(self, text):
        if text.startswith('-'):
            text = ' ' + text                  # never parsed as an option by the synth
        with self.lock:
            synth = self.popen(self._argv(self.synth, text), stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE if self.player else subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.procs = [synth]
            if self.player:
                self.procs.append(self.popen(self._argv(self.player, text), stdin=synth.stdout,
                                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                if hasattr(synth.stdout, 'close'):
                    synth.stdout.close()           # the player owns the pipe now; synth gets SIGPIPE if it dies
        for p in reversed(self.procs):
            try:
                p.wait(timeout=self.timeout_s)
            except subprocess.TimeoutExpired:
                p.terminate()

    def stop(self):
        with self.lock:
            for p in self.procs:
                if p.poll() is None:
                    p.terminate()


class TtsError(Exception):
    pass


class HttpWavTts:
    """POST the sentence to a WAV-returning TTS server, then play the WAV with `player` (argv template reading WAV
    on stdin, e.g. ['aplay', '-q', '-D', '{device}']; aplay takes the rate from the WAV header). stop() kills
    the playback; a sentence still being synthesised is dropped when it arrives."""

    def __init__(self, url, player, device='default', length_scale=1.0, timeout_s=20.0,
                 transport=urllib_transport, popen=subprocess.Popen):
        self.url, self.player, self.device, self.length_scale = url, player, device, length_scale
        self.timeout_s, self.transport, self.popen = timeout_s, transport, popen
        self.proc, self.stopped = None, threading.Event()
        self.lock = threading.Lock()

    def say(self, text):
        self.stopped.clear()
        body = json.dumps({'text': text, 'length_scale': self.length_scale}, ensure_ascii=False).encode()
        try:
            res = self.transport(self.url, {'Content-Type': 'application/json'}, body, self.timeout_s)
        except TransportError as e:
            raise TtsError(f'TTS server {self.url} unreachable: {e}') from e
        if res.status != 200 or not res.body.startswith(b'RIFF'):
            raise TtsError(f'TTS server {self.url} answered {res.status}, {len(res.body)} bytes, not a WAV')
        with self.lock:
            if self.stopped.is_set():
                return
            self.proc = self.popen([a.replace('{device}', self.device) for a in self.player], stdin=subprocess.PIPE,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            self.proc.communicate(res.body, timeout=self.timeout_s + len(res.body) / 32000)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
        except (BrokenPipeError, OSError):
            pass                                   # stopped mid-sentence: the player is gone

    def stop(self):
        with self.lock:
            self.stopped.set()
            if self.proc is not None and self.proc.poll() is None:
                self.proc.terminate()


TTS_ENGINES = ('none', 'command', 'piper_http')


def make_tts(engine, command='', player='', device='default', url='', length_scale=1.0):
    """engine 'none' → NullTts; 'command' → CommandTts from command-line strings (split here, not by a shell);
    'piper_http' → HttpWavTts(url) played with `player`."""
    if engine not in TTS_ENGINES:
        raise ValueError(f'tts_engine {engine!r}: expected one of {TTS_ENGINES}')
    if engine == 'command' and command.strip():
        return CommandTts(shlex.split(command), shlex.split(player) if player.strip() else None, device)
    if engine == 'piper_http':
        return HttpWavTts(url, shlex.split(player), device, length_scale)
    return NullTts()
