"""Speech-to-text for one utterance WAV (16 kHz mono, core/audio.UtteranceSpool), with the engine left pluggable
like the TTS. Everything runs on the robot: the recording never leaves it.

Engines:
- WhisperServerStt: whisper.cpp's `whisper-server` (examples/server, read 2026-10-25). The model stays loaded
  between questions. POST {url} multipart: `file` (the WAV, 16 kHz), `language`, `response_format=json`,
  `temperature`, optionally `audio_ctx` (encoder window, 1500 = 30 s; 768 ≈ 15 s still holds the longest
  utterance, 12 s, and halved the time on an M4 at equal accuracy, 2026-10-25); the reply is {"text": "..."}.
- CommandStt: any command line printing the transcript on stdout, e.g. `whisper-cli -m … -l {lang} -nt -np
  -f {wav}`. Split into argv once, no shell; {wav} and {lang} are filled in as whole arguments. Loads the model
  for every question, so it is slower.
- NullStt: no engine; every utterance fails with reason 'no_engine'.

Whisper writes subtitle credits and bracketed sound labels into silence and noise, a habit from its training data
("Altyazı M.K." in Turkish). clean_transcript() drops those, so a cough is not sent to the assistant as a question."""
import json
import re
import shlex
import subprocess
import time
import uuid
from dataclasses import dataclass

from .assistant import TransportError, urllib_transport

_BRACKETS = re.compile(r'\[[^\]]*\]|\([^)]*\)|\*[^*]*\*|♪+')
_SENTENCE = re.compile(r'(?<=[.!?…])\s+')
_NOT_WORD = re.compile(r'[^\w]+')
_PUNCT = re.compile(r'[^\w\s]+')
# Whole sentences, compared without case and punctuation. Only phrases nobody says to a robot.
HALLUCINATIONS = frozenset({
    'altyazı mk', 'altyazı', 'altyazılar', 'izlediğiniz için teşekkür ederim', 'izlediğiniz için teşekkürler',
    'abone olmayı unutmayın', 'kanalıma abone olmayı unutmayın', 'beğenmeyi ve abone olmayı unutmayın',
    'bir sonraki videoda görüşmek üzere', 'bir sonraki videoda görüşmek üzere hoşça kalın',
    'thank you for watching', 'thanks for watching', 'subtitles by the amaraorg community',
})


def _keys(sentence):
    """The sentence without punctuation ("M.K." → "mk"), lowered the Turkish way (I → ı, İ → i) and the generic way."""
    tr = sentence.replace('İ', 'i').replace('I', 'ı').lower()
    return {' '.join(_PUNCT.sub('', s).split()) for s in (tr, sentence.casefold())}


def clean_transcript(text, min_chars=2):
    """Sound labels and known hallucinated sentences out, whitespace collapsed; '' when nothing real is left."""
    t = ' '.join(_BRACKETS.sub(' ', text or '').split())
    kept = [s for s in _SENTENCE.split(t) if s and not _keys(s) & HALLUCINATIONS]
    t = ' '.join(kept).strip()
    return t if len(_NOT_WORD.sub('', t)) >= min_chars else ''


@dataclass(frozen=True)
class Transcript:
    text: str                   # '' unless ok
    ok: bool
    reason: str = None          # why not: no_engine, empty, network, http_<code>, bad_reply, timeout, failed, missing
    seconds: float = 0.0        # engine time
    detail: str = None          # for the log; never the transcript


class NullStt:
    def transcribe(self, wav_path):
        return Transcript('', False, 'no_engine')


def _multipart(fields, file_field, filename, data, content_type='audio/wav'):
    boundary = f'luna{uuid.uuid4().hex}'
    out = []
    for name, value in fields.items():
        out += [f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode()]
    out += [f'--{boundary}\r\nContent-Disposition: form-data; name="{file_field}"; filename="{filename}"\r\n'
            f'Content-Type: {content_type}\r\n\r\n'.encode(), data, f'\r\n--{boundary}--\r\n'.encode()]
    return b''.join(out), f'multipart/form-data; boundary={boundary}'


class WhisperServerStt:
    def __init__(self, url='http://127.0.0.1:8178/inference', language='tr', timeout_s=20.0, prompt='',
                 min_chars=2, audio_ctx=0, transport=urllib_transport, clock=time.monotonic):
        self.url, self.language, self.timeout_s, self.prompt = url, language, timeout_s, prompt
        self.audio_ctx = audio_ctx
        self.min_chars, self.transport, self.clock = min_chars, transport, clock

    def transcribe(self, wav_path):
        try:
            with open(wav_path, 'rb') as f:
                data = f.read()
        except OSError as e:
            return Transcript('', False, 'missing', detail=str(e))
        fields = {'language': self.language, 'response_format': 'json', 'temperature': '0.0'}
        if self.prompt:
            fields['prompt'] = self.prompt
        if self.audio_ctx:
            fields['audio_ctx'] = str(self.audio_ctx)
        body, ctype = _multipart(fields, 'file', 'utterance.wav', data)
        t0 = self.clock()
        try:
            res = self.transport(self.url, {'Content-Type': ctype}, body, self.timeout_s)
        except TransportError as e:
            return Transcript('', False, 'network', self.clock() - t0, str(e))
        dt = self.clock() - t0
        if res.status != 200:
            return Transcript('', False, f'http_{res.status}', dt, res.body[:200].decode('utf-8', 'replace'))
        try:
            raw = json.loads(res.body.decode('utf-8'))['text']
        except (ValueError, KeyError, TypeError, UnicodeDecodeError) as e:
            return Transcript('', False, 'bad_reply', dt, f'{type(e).__name__}')
        text = clean_transcript(raw if isinstance(raw, str) else '', self.min_chars)
        return Transcript(text, bool(text), None if text else 'empty', dt)


class CommandStt:
    def __init__(self, argv, language='tr', timeout_s=30.0, min_chars=2, run=subprocess.run, clock=time.monotonic):
        self.argv, self.language, self.timeout_s, self.min_chars = argv, language, timeout_s, min_chars
        self.run, self.clock = run, clock

    def transcribe(self, wav_path):
        argv = [a.replace('{wav}', wav_path).replace('{lang}', self.language) for a in self.argv]
        t0 = self.clock()
        try:
            res = self.run(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           timeout=self.timeout_s)
        except FileNotFoundError:
            return Transcript('', False, 'missing', detail=f'{argv[0]} not found')
        except subprocess.TimeoutExpired:
            return Transcript('', False, 'timeout', self.clock() - t0)
        dt = self.clock() - t0
        if res.returncode != 0:
            err = (res.stderr or b'').decode('utf-8', 'replace').strip().splitlines()
            return Transcript('', False, 'failed', dt, f'exit {res.returncode}: {err[-1] if err else ""}'[:200])
        text = clean_transcript((res.stdout or b'').decode('utf-8', 'replace'), self.min_chars)
        return Transcript(text, bool(text), None if text else 'empty', dt)


STT_ENGINES = ('none', 'whisper_server', 'command')


def make_stt(engine, *, url='', command='', language='tr', timeout_s=20.0, prompt='', min_chars=2, audio_ctx=0):
    """engine 'whisper_server' → WhisperServerStt(url); 'command' → CommandStt (split here, not by a shell);
    'none' → NullStt. An unknown engine is an error: a typo must not silently turn Luna deaf."""
    if engine not in STT_ENGINES:
        raise ValueError(f'stt_engine {engine!r}: expected one of {STT_ENGINES}')
    if engine == 'whisper_server':
        return WhisperServerStt(url, language, timeout_s, prompt, min_chars, audio_ctx)
    if engine == 'command' and command.strip():
        return CommandStt(shlex.split(command), language, timeout_s, min_chars)
    return NullStt()
