"""Voice chain without hardware or models: push-to-talk listening window and its capture gate, speech-to-text
engines (whisper-server over a real local HTTP server, command line, hallucination filter), Piper HTTP TTS and its
cut-off, and the face bridge's POST /talk.
Run from luna_ws/src/luna_robot:  python3 -m unittest discover -s test"""
import json
import os
import subprocess
import sys
import tempfile
import threading
import unittest
import wave
from array import array
from email import policy
from email.parser import BytesParser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core.assistant import AssistantClient, HttpResult, TransportError  # noqa: E402
from luna_robot.core.attention import Attention, AttentionConfig  # noqa: E402
from luna_robot.core.audio import (ListenConfig, Listener, ListenWindow, MicGate, Segmenter,  # noqa: E402
                                   SegmenterConfig)
from luna_robot.core.speech import HttpWavTts, NullTts, TtsError, make_tts  # noqa: E402
from luna_robot.core.stt import CommandStt, NullStt, WhisperServerStt, clean_transcript, make_stt  # noqa: E402

RATE = 1000                                    # small rate: 20 ms chunks of 20 samples keep the tests quick
CHUNK = 0.02


def chunk(level=0):
    return array('h', [level] * int(RATE * CHUNK))


def run_listener(listener, script, t0=0.0, speaking=lambda t: False):
    """script: [(seconds, speech flag)] fed in 20 ms chunks from t0 → [(t, utterance seconds)] for kept utterances.
    The flag goes through the MicGate like audio_node's DoA does (0 while the body turns)."""
    t, out = t0, []
    for seconds, speech in script:
        for _ in range(round(seconds / CHUNK)):
            flag = listener.gate.speech(t, speech)
            for utt in listener.feed(chunk(), t, speech=flag, speaking=speaking(t)):
                out.append((round(t, 2), round(len(utt) / RATE, 2)))
            t += CHUNK
    return out, t


def make_listener(mode='push_to_talk', window_s=6.0):
    seg = Segmenter(SegmenterConfig(rate=RATE, pre_roll_s=0.2, hangover_s=0.6, min_s=0.4, max_s=12.0))
    return Listener(seg, MicGate(), ListenWindow(ListenConfig(mode, window_s)))


class ListenWindowLogic(unittest.TestCase):
    def test_touch_opens_a_window_that_closes_after_one_question(self):
        w = ListenWindow(ListenConfig(window_s=6.0))
        self.assertFalse(w.may_start(0.0))
        self.assertTrue(w.talk(1.0))
        self.assertTrue(w.may_start(6.9) and w.listening(6.9))
        self.assertFalse(w.may_start(7.0) or w.listening(7.0))
        w.talk(10.0)
        w.started()
        w.finished(kept=True)
        self.assertFalse(w.may_start(11.0) or w.listening(11.0))    # one touch, one question

    def test_a_cough_does_not_use_up_the_touch(self):
        w = ListenWindow(ListenConfig(window_s=6.0))
        w.talk(0.0)
        w.started()
        self.assertTrue(w.listening(9.0))                           # running utterance: listening past the window
        w.finished(kept=False)
        self.assertTrue(w.may_start(2.0))

    def test_repeated_touch_is_the_same_touch_and_a_later_one_reopens(self):
        w = ListenWindow(ListenConfig(window_s=6.0, min_gap_s=0.5))
        self.assertTrue(w.talk(0.0))
        self.assertFalse(w.talk(0.3))
        self.assertFalse(w.may_start(6.1))                          # the double tap did not extend the window
        self.assertTrue(w.talk(5.0))
        self.assertTrue(w.may_start(10.9))

    def test_always_mode_takes_every_utterance_and_mute_cancels(self):
        w = ListenWindow(ListenConfig(mode='always'))
        self.assertTrue(w.may_start(0.0))
        self.assertFalse(w.listening(0.0))
        w.started()
        self.assertTrue(w.listening(0.0))
        w.finished(kept=True)
        self.assertTrue(w.may_start(1.0))
        p = ListenWindow()
        p.talk(0.0)
        p.cancel()
        self.assertFalse(p.may_start(1.0) or p.listening(1.0))

    def test_unknown_mode_is_an_error(self):
        with self.assertRaises(ValueError):
            ListenWindow(ListenConfig(mode='wake_word'))


class ListenerGate(unittest.TestCase):
    def test_speech_without_a_touch_is_never_kept(self):
        out, _ = run_listener(make_listener(), [(1.0, False), (2.0, True), (1.0, False)])
        self.assertEqual(out, [])

    def test_one_question_after_a_touch_even_when_it_runs_past_the_window(self):
        listener = make_listener(window_s=1.0)
        listener.window.talk(0.0)
        out, _ = run_listener(listener, [(0.5, False), (3.0, True), (1.0, False), (2.0, True), (1.0, False)])
        self.assertEqual(len(out), 1)                                # the second sentence came after the question
        self.assertAlmostEqual(out[0][1], 0.2 + 3.0 + 0.6, places=1)  # pre-roll + speech + hangover

    def test_a_question_cannot_start_while_luna_speaks(self):
        listener = make_listener()
        listener.window.talk(0.0)
        out, _ = run_listener(listener, [(1.0, True), (1.0, False), (1.0, True), (1.0, False)],
                              speaking=lambda t: t < 1.5)
        self.assertEqual(len(out), 1)                                # only the voice after Luna stopped
        self.assertAlmostEqual(out[0][0], 3.6, places=1)
        self.assertAlmostEqual(out[0][1], 0.2 + 1.0 + 0.6, places=1)

    def test_mute_drops_the_running_question_and_closes_the_window(self):
        listener = make_listener()
        listener.window.talk(0.0)
        run_listener(listener, [(1.0, True)])
        listener.gate.muted = True
        out, t = run_listener(listener, [(0.2, True)], t0=1.0)
        listener.gate.muted = False
        out2, _ = run_listener(listener, [(1.0, True), (1.0, False)], t0=t)
        self.assertEqual(out + out2, [])
        self.assertFalse(listener.window.listening(t))

    def test_turning_holds_the_start(self):
        listener = make_listener()
        listener.window.talk(0.0)
        listener.gate.body(0.1, 0.5)                                  # turning: moving until 0.4 s
        out, _ = run_listener(listener, [(1.5, True), (1.0, False)])
        self.assertEqual(len(out), 1)
        self.assertLess(out[0][1], 1.5 + 0.6)                        # started only once the body settled

    def test_body_turning_mid_question_does_not_cut_it(self):
        # R1 live run 2026-10-25: attention turned the head 0.3 s into the question, the gated flag went to 0 and the
        # question was cut 0.7 s later (1.4 s WAVs for 2.5 s of speech). R2 turns the body instead; the same rule.
        listener = make_listener()
        listener.window.talk(0.0)
        out, t = run_listener(listener, [(0.2, False), (0.5, True)])
        for i in range(30):                                          # the body turns for 0.6 s
            listener.gate.body(t + i * CHUNK, 0.5)
        out2, _ = run_listener(listener, [(2.0, True), (1.0, False)], t0=t)
        self.assertEqual(out, [])
        self.assertEqual(len(out2), 1)
        self.assertAlmostEqual(out2[0][1], 0.2 + 2.5 + 0.6, places=1)

    def test_frozen_time_is_neither_speech_nor_silence(self):
        seg = Segmenter(SegmenterConfig(rate=RATE, pre_roll_s=0.0, hangover_s=0.6, min_s=0.4))
        feed = lambda n, **kw: [u for _ in range(n) for u in seg.feed(chunk(), **kw)]  # noqa: E731
        self.assertEqual(feed(5, speech=True) + feed(50, speech=False, freeze=True), [])   # 1 s frozen: still running
        self.assertTrue(seg.busy)
        self.assertEqual(feed(30, speech=False), [])                 # 0.6 s of real silence ends it...
        self.assertFalse(seg.busy)                                   # ...but 0.1 s voiced < min_s: dropped
        self.assertEqual(feed(10, speech=False, freeze=True), [])
        self.assertFalse(seg.busy)                                   # frozen time never starts one

    def test_always_mode_keeps_every_utterance(self):
        out, _ = run_listener(make_listener('always'), [(1.0, True), (1.0, False), (1.0, True), (1.0, False)])
        self.assertEqual(len(out), 2)


class BodyDuringAQuestion(unittest.TestCase):
    def test_body_stays_while_luna_takes_a_question_and_the_eyes_still_look(self):
        att, t, turns, gazes = Attention(), 0.0, [], []
        for _ in range(60):                                          # 3 s, voice at +35°, window open
            t += 0.05
            out = att.update(t, bearing_deg=35.0, speech=True, hold=True)
            turns.append(out.turn)
            gazes.append(out.gaze)
        self.assertEqual(set(turns), {None})
        self.assertAlmostEqual(gazes[-1][0], round(35 / 60, 3))
        for _ in range(20):                                          # question over: the body may turn now
            t += 0.05
            out = att.update(t, bearing_deg=35.0, speech=True)
        self.assertGreater(out.turn, 0.0)

    def test_a_turn_under_way_stops_when_a_question_starts(self):
        att, t = Attention(), 0.0
        for _ in range(12):
            t += 0.05
            out = att.update(t, bearing_deg=40.0, speech=True)
        self.assertGreater(out.turn, 0.0)
        stops = []
        for _ in range(200):                                         # silence long past hold_s, window still open
            t += 0.05
            stops.append(att.update(t, hold=True).turn)
        self.assertEqual(stops[0], 0.0)                              # stopped at once, then left alone
        self.assertEqual(set(stops[AttentionConfig().stop_ticks:]), {None})


class DroppedAnswer(unittest.TestCase):
    def test_an_answer_nobody_heard_leaves_the_context(self):
        replies = iter(['Bugün güneşli.', 'Ankara.'])

        def transport(url, headers, body, timeout):
            text = next(replies)
            return HttpResult(200, {}, json.dumps({'choices': [{'message': {'content': text}}]}).encode())
        client = AssistantClient(api_key='k', enabled=True, transport=transport)
        client.ask('Hava nasıl?')
        client.forget_last()                                        # cut off by a touch
        client.ask('Başkent neresi?')
        self.assertEqual([m['content'] for m in client.history], ['Başkent neresi?', 'Ankara.'])


class TranscriptCleaning(unittest.TestCase):
    def test_real_question_kept_and_whitespace_collapsed(self):
        self.assertEqual(clean_transcript('  Luna,   bugün hava nasıl?\n'), 'Luna, bugün hava nasıl?')

    def test_subtitle_credits_and_sound_labels_are_dropped(self):
        for text in ('Altyazı M.K.', ' altyazı: m.k. ', 'ALTYAZI M.K.', '[Müzik]', '(gülüşmeler)', '♪♪', '*alkış*',
                     'İzlediğiniz için teşekkür ederim.', 'İZLEDİĞİNİZ İÇİN TEŞEKKÜRLER!', 'Thank you for watching!',
                     'Abone olmayı unutmayın. Altyazı M.K.', '.', 'a'):
            self.assertEqual(clean_transcript(text), '', text)

    def test_hallucination_appended_to_a_real_sentence_is_cut(self):
        self.assertEqual(clean_transcript('Merhaba Luna. Altyazı M.K.'), 'Merhaba Luna.')
        self.assertEqual(clean_transcript('[Müzik] Şarkı söyler misin?'), 'Şarkı söyler misin?')

    def test_ordinary_thanks_are_not_a_hallucination(self):
        self.assertEqual(clean_transcript('Teşekkürler Luna.'), 'Teşekkürler Luna.')
        self.assertEqual(clean_transcript('Teşekkürler.'), 'Teşekkürler.')


def write_wav(path, seconds=0.5, rate=16000):
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(bytes(2 * int(rate * seconds)))


class FakeWhisperServer:
    """Real HTTP server answering like whisper.cpp's /inference; it records the multipart fields it got."""

    def __init__(self, reply=None, status=200):
        self.got, self.reply, self.status = [], reply if reply is not None else {'text': ' Merhaba Luna.\n'}, status
        fake = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_POST(self):
                body = self.rfile.read(int(self.headers['Content-Length']))
                msg = BytesParser(policy=policy.default).parsebytes(
                    f'Content-Type: {self.headers["Content-Type"]}\r\n\r\n'.encode() + body)
                fields = {}
                for part in msg.iter_parts():
                    name = part.get_param('name', header='content-disposition')
                    fields[name] = part.get_content()
                fake.got.append((self.path, fields))
                data = json.dumps(fake.reply).encode() if isinstance(fake.reply, dict) else fake.reply
                self.send_response(fake.status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.url = f'http://127.0.0.1:{self.server.server_address[1]}/inference'
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


class WhisperServerEngine(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.wav = os.path.join(self.dir.name, 'luna_utt_1.wav')
        write_wav(self.wav)

    def tearDown(self):
        self.dir.cleanup()

    def test_wav_and_turkish_go_to_the_server_and_the_text_comes_back(self):
        fake = FakeWhisperServer()
        try:
            r = WhisperServerStt(fake.url, 'tr', timeout_s=5).transcribe(self.wav)
        finally:
            fake.close()
        self.assertTrue(r.ok)
        self.assertEqual(r.text, 'Merhaba Luna.')
        path, fields = fake.got[0]
        self.assertEqual(path, '/inference')
        self.assertEqual(fields['language'], 'tr')
        self.assertEqual(fields['response_format'], 'json')
        self.assertEqual(fields['temperature'], '0.0')
        with open(self.wav, 'rb') as f:
            self.assertEqual(fields['file'], f.read())

    def test_hallucination_only_is_empty_not_a_question(self):
        fake = FakeWhisperServer({'text': ' Altyazı M.K.'})
        try:
            r = WhisperServerStt(fake.url, timeout_s=5).transcribe(self.wav)
        finally:
            fake.close()
        self.assertEqual((r.ok, r.text, r.reason), (False, '', 'empty'))

    def test_server_errors_and_garbage_are_reasons(self):
        for reply, status, reason in [({'error': 'failed to read audio data'}, 400, 'http_400'),
                                      (b'not json', 200, 'bad_reply'), ({'no_text': 1}, 200, 'bad_reply')]:
            fake = FakeWhisperServer(reply, status)
            try:
                r = WhisperServerStt(fake.url, timeout_s=5).transcribe(self.wav)
            finally:
                fake.close()
            self.assertEqual((r.ok, r.reason), (False, reason))

    def test_no_server_and_no_file(self):
        def down(url, headers, body, timeout):
            raise TransportError('connection refused')
        self.assertEqual(WhisperServerStt(transport=down).transcribe(self.wav).reason, 'network')
        self.assertEqual(WhisperServerStt().transcribe(os.path.join(self.dir.name, 'gone.wav')).reason, 'missing')

    def test_prompt_is_sent_only_when_set(self):
        seen = []

        def transport(url, headers, body, timeout):
            seen.append(body)
            return HttpResult(200, {}, b'{"text": "Evet"}')
        WhisperServerStt(transport=transport).transcribe(self.wav)
        WhisperServerStt(transport=transport, prompt='Luna').transcribe(self.wav)
        self.assertNotIn(b'name="prompt"', seen[0])
        self.assertIn(b'name="prompt"\r\n\r\nLuna\r\n', seen[1])
        self.assertNotIn(b'name="audio_ctx"', seen[0])
        stt = make_stt('whisper_server', url='u', audio_ctx=768)
        stt.transport = transport
        stt.transcribe(self.wav)
        self.assertIn(b'name="audio_ctx"\r\n\r\n768\r\n', seen[2])


class CommandEngine(unittest.TestCase):
    def test_wav_and_language_are_whole_arguments_without_a_shell(self):
        calls = []

        def run(argv, **kw):
            calls.append((argv, kw))
            return subprocess.CompletedProcess(argv, 0, b' Bu ne?\n', b'')
        stt = make_stt('command', command='whisper-cli -m m.bin -l {lang} -nt -f {wav}', language='tr')
        stt.run = run
        r = stt.transcribe('/tmp/a b; rm -rf x.wav')
        self.assertEqual(r.text, 'Bu ne?')
        self.assertEqual(calls[0][0], ['whisper-cli', '-m', 'm.bin', '-l', 'tr', '-nt', '-f', '/tmp/a b; rm -rf x.wav'])
        self.assertNotIn('shell', calls[0][1])

    def test_missing_binary_timeout_and_failure(self):
        def missing(argv, **kw):
            raise FileNotFoundError(argv[0])

        def slow(argv, **kw):
            raise subprocess.TimeoutExpired(argv, kw['timeout'])

        def fails(argv, **kw):
            return subprocess.CompletedProcess(argv, 1, b'', b'loading model\nerror: failed to open model\n')
        self.assertEqual(CommandStt(['x'], run=missing).transcribe('a.wav').reason, 'missing')
        self.assertEqual(CommandStt(['x'], run=slow).transcribe('a.wav').reason, 'timeout')
        r = CommandStt(['x'], run=fails).transcribe('a.wav')
        self.assertEqual(r.reason, 'failed')
        self.assertIn('failed to open model', r.detail)

    def test_engine_choice(self):
        self.assertIsInstance(make_stt('none'), NullStt)
        self.assertIsInstance(make_stt('command', command='  '), NullStt)
        self.assertIsInstance(make_stt('whisper_server', url='http://x'), WhisperServerStt)
        self.assertEqual(make_stt('none').transcribe('a.wav').reason, 'no_engine')
        with self.assertRaises(ValueError):
            make_stt('whisper')                                     # a typo must not turn Luna deaf silently


class FakePlayer:
    def __init__(self, argv, **kw):
        self.argv, self.kw, self.fed, self.terminated, self.code = argv, kw, None, False, None
        FakePlayer.made.append(self)

    def communicate(self, data, timeout=None):
        self.fed, self.code = data, 0

    def poll(self):
        return self.code

    def terminate(self):
        self.terminated, self.code = True, -15


WAV = b'RIFF\x24\x00\x00\x00WAVEfmt ' + bytes(28)


class PiperHttpTts(unittest.TestCase):
    def setUp(self):
        FakePlayer.made = []

    def test_sentence_goes_to_piper_and_the_wav_to_aplay(self):
        sent = []

        def transport(url, headers, body, timeout):
            sent.append((url, headers, json.loads(body.decode())))
            return HttpResult(200, {}, WAV)
        tts = make_tts('piper_http', player='aplay -q -D {device}', device='plughw:CARD=Array,DEV=0',
                       url='http://127.0.0.1:5000/synthesize', length_scale=1.1)
        tts.transport, tts.popen = transport, FakePlayer
        tts.say('Günaydın, ışıklar açık mı?')
        self.assertEqual(sent[0][0], 'http://127.0.0.1:5000/synthesize')
        self.assertEqual(sent[0][1]['Content-Type'], 'application/json')
        self.assertEqual(sent[0][2], {'text': 'Günaydın, ışıklar açık mı?', 'length_scale': 1.1})
        self.assertEqual(FakePlayer.made[0].argv, ['aplay', '-q', '-D', 'plughw:CARD=Array,DEV=0'])
        self.assertEqual(FakePlayer.made[0].fed, WAV)
        self.assertEqual(FakePlayer.made[0].kw['stdin'], subprocess.PIPE)

    def test_server_down_or_not_a_wav_is_an_error_not_a_crash(self):
        def down(url, headers, body, timeout):
            raise TransportError('connection refused')
        for transport in (down, lambda *a: HttpResult(500, {}, b'oops'), lambda *a: HttpResult(200, {}, b'{}')):
            tts = HttpWavTts('http://127.0.0.1:5000/synthesize', ['aplay'], transport=transport, popen=FakePlayer)
            with self.assertRaises(TtsError):
                tts.say('Merhaba')
        self.assertEqual(FakePlayer.made, [])

    def test_stop_during_synthesis_drops_the_sentence_and_stop_during_playback_kills_it(self):
        tts = HttpWavTts('u', ['aplay'], popen=FakePlayer)

        def slow_server(url, headers, body, timeout):
            tts.stop()                                              # a touch arrives while Piper is still busy
            return HttpResult(200, {}, WAV)
        tts.transport = slow_server
        tts.say('Uzun bir cevap')
        self.assertEqual(FakePlayer.made, [])
        tts.transport = lambda *a: HttpResult(200, {}, WAV)

        class Playing(FakePlayer):
            def communicate(self, data, timeout=None):
                tts.stop()                                          # the touch arrives mid-sentence
        tts.popen = Playing
        tts.say('Bir cevap daha')
        self.assertTrue(FakePlayer.made[0].terminated)

    def test_engine_choice(self):
        self.assertIsInstance(make_tts('none'), NullTts)
        self.assertIsInstance(make_tts('piper_http', player='aplay', url='u'), HttpWavTts)
        with self.assertRaises(ValueError):
            make_tts('piper')


class ServerWrapper(unittest.TestCase):
    """launch_util.STOP_AS_CLEAN_EXIT around whisper-server / Piper: launch's SIGINT is a clean exit, a crash is not."""

    def _argv(self, *cmd):
        import shlex
        from luna_robot.launch_util import STOP_AS_CLEAN_EXIT
        return shlex.split(STOP_AS_CLEAN_EXIT) + list(cmd)

    def test_own_exit_status_comes_through(self):
        self.assertEqual(subprocess.run(self._argv(sys.executable, '-c', 'import sys; sys.exit(3)')).returncode, 3)

    def test_sigint_stops_the_server_and_exits_0(self):
        import signal
        import time
        marker = tempfile.NamedTemporaryFile(delete=False)
        marker.close()
        self.addCleanup(os.remove, marker.name)
        script = ('import signal, sys, time\n'
                  f'signal.signal(signal.SIGTERM, lambda *a: (open({marker.name!r}, "w").write("term"), sys.exit(0)))\n'
                  'time.sleep(30)\n')
        proc = subprocess.Popen(self._argv(sys.executable, '-c', script))
        time.sleep(0.5)
        proc.send_signal(signal.SIGINT)
        self.assertEqual(proc.wait(timeout=10), 0)
        with open(marker.name) as f:
            self.assertEqual(f.read(), 'term')                      # the server got SIGTERM, it was not orphaned


if __name__ == '__main__':
    unittest.main()
