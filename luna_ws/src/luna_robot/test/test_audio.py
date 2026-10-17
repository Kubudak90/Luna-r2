"""Audio side without hardware: XVF3800 USB control against a fake device, PCM levels and utterance cutting,
head-relative DoA → base_link bearing, mute and head-motion gating, DoA → gentle head turn + eye gaze, and
listening/thinking/speaking → face events.
Run from luna_ws/src/luna_robot:  python3 -m unittest discover -s test"""
import math
import os
import struct
import sys
import tempfile
import unittest
import wave
from array import array

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core import xvf3800 as X  # noqa: E402
from luna_robot.core.attention import Attention, AttentionConfig, MockSpeakers  # noqa: E402
from luna_robot.core.audio import MicGate, Segmenter, SegmenterConfig, UtteranceSpool, channel, rms_dbfs  # noqa: E402
from luna_robot.core.head_frame import base_to_head, head_to_base  # noqa: E402
from luna_robot.core.face_mood import FACE_EVENTS, VoiceMood, pick_gaze, voice_state  # noqa: E402


class FakeXvf:
    """pyusb-like device answering vendor reads from a table; the first `busy` reads answer 'retry'."""

    def __init__(self, values, busy=0, status=0):
        self.values, self.busy, self.status, self.calls = values, busy, status, []

    def ctrl_transfer(self, bm_request_type, b_request, w_value, w_index, length, timeout):
        if bm_request_type == X.REQ_OUT:                      # pyusb: the 5th argument is the payload for OUT
            self.calls.append((bm_request_type, b_request, w_value, w_index, bytes(length)))
            return len(length)
        self.calls.append((bm_request_type, b_request, w_value, w_index, length))
        if self.busy:
            self.busy -= 1
            return array('B', [X.STATUS_RETRY] + [0] * (length - 1))
        return array('B', bytes([self.status]) + self.values[(w_index, w_value & 0x7F)])


class Xvf3800Control(unittest.TestCase):
    def test_doa_read_is_seeeds_vendor_request(self):
        dev = FakeXvf({(20, 18): struct.pack('<HH', 270, 1)})
        self.assertEqual(X.XvfControl(dev).doa(), X.DoaReading(270, True))
        self.assertEqual(dev.calls, [(0xC0, 0, 0x80 | 18, 20, 5)])     # IN|vendor|device, read bit, +1 status byte

    def test_busy_device_is_retried(self):
        dev = FakeXvf({(20, 18): struct.pack('<HH', 10, 0)}, busy=2)
        self.assertEqual(X.XvfControl(dev, sleep=lambda s: None).doa(), X.DoaReading(10, False))
        self.assertEqual(len(dev.calls), 3)

    def test_error_status_short_reply_and_endless_busy_raise(self):
        doa = struct.pack('<HH', 1, 1)
        for ctl in (X.XvfControl(FakeXvf({(20, 18): doa}, status=3)),
                    X.XvfControl(FakeXvf({(20, 18): b'\x01'})),
                    X.XvfControl(FakeXvf({(20, 18): doa}, busy=99), retries=3, sleep=lambda s: None)):
            with self.assertRaises(X.XvfError):
                ctl.doa()

    def test_float_params_decode(self):
        dev = FakeXvf({(35, 11): struct.pack('<2f', 1.5, float('nan'))})
        processed, auto = X.XvfControl(dev).read('AUDIO_MGR_SELECTED_AZIMUTHS')
        self.assertAlmostEqual(processed, 1.5, places=6)
        self.assertTrue(math.isnan(auto))                  # XMOS: NaN while no speech is detected

    def test_board_angle_to_head_bearing(self):
        # The board's 0° and counting direction are not documented: both are calibration parameters.
        self.assertEqual(X.board_to_head(0), 0)
        self.assertEqual(X.board_to_head(90), 90)
        self.assertEqual(X.board_to_head(270), -90)
        self.assertEqual(X.board_to_head(180), 180)        # (-180, 180]
        self.assertEqual(X.board_to_head(90, direction=-1), -90)
        self.assertEqual(X.board_to_head(100, zero_deg=90), 10)
        self.assertEqual(X.board_to_head(10, zero_deg=350), 20)

    def test_alsa_card_found_by_name(self):
        cards = (' 0 [tegrahda       ]: tegra-hda - tegra-hda\n'
                 '                      tegra-hda at 0x3518000 irq 63\n'
                 ' 4 [Array          ]: USB-Audio - reSpeaker XVF3800 4-Mic Array\n'
                 '                      Seeed Studio reSpeaker XVF3800 4-Mic Array at usb-3610000.xhci-2.1\n')
        self.assertEqual(X.find_alsa_device(cards), 'plughw:CARD=Array,DEV=0')
        self.assertIsNone(X.find_alsa_device(cards.splitlines()[0]))

    def test_mute_writes_gpo_x0d30_and_reads_it_back(self):
        # Seeed GPIO page: GPO servicer resid 20, GPO_WRITE_VALUE cmd 1 {pin 30, level}; high = mics muted + red LED.
        dev = FakeXvf({(20, 0): bytes([0, 1, 0, 1, 0])})          # X0D11, X0D30, X0D31, X0D33, X0D39
        ctl = X.XvfControl(dev)
        ctl.set_mute(True)
        ctl.set_mute(False)
        self.assertEqual(dev.calls, [(X.REQ_OUT, 0, 1, 20, bytes([30, 1])), (X.REQ_OUT, 0, 1, 20, bytes([30, 0]))])
        self.assertTrue(ctl.mute_pin())

    def test_open_usb_without_pyusb_is_a_clear_error(self):
        with self.assertRaisesRegex(X.XvfError, 'pyusb'):
            X.open_usb(import_usb=lambda: (_ for _ in ()).throw(ImportError('no usb')))


def tone(value, n=100):
    return array('h', [value] * n)


class Pcm(unittest.TestCase):
    def test_channel_pick(self):
        raw = struct.pack('<6h', 100, -1, -100, -2, 100, -3)        # L R interleaved
        self.assertEqual(list(channel(raw, channels=2, index=0)), [100, -100, 100])
        self.assertEqual(list(channel(raw, channels=2, index=1)), [-1, -2, -3])
        self.assertEqual(list(channel(raw[:-1], channels=2, index=0)), [100, -100])   # torn frame dropped

    def test_level(self):
        self.assertAlmostEqual(rms_dbfs(tone(32767)), 0.0, places=2)
        self.assertAlmostEqual(rms_dbfs(tone(3277)), -20.0, places=1)
        self.assertEqual(rms_dbfs(array('h')), -120.0)
        self.assertEqual(rms_dbfs(tone(0)), -120.0)


class Segmenting(unittest.TestCase):
    cfg = SegmenterConfig(rate=1000, pre_roll_s=0.1, hangover_s=0.2, min_s=0.3, max_s=2.0, energy_dbfs=-30.0)

    def feed(self, seg, pattern, flag=True):
        out = []
        for voiced in pattern:
            out += seg.feed(tone(3277 if voiced else 10), speech=bool(voiced) if flag else None)
        return out

    def test_utterance_keeps_pre_roll_and_hangover(self):
        out = self.feed(Segmenter(self.cfg), [0, 0, 1, 1, 1, 1, 0, 0, 0, 0])
        self.assertEqual(len(out), 1)
        self.assertEqual(len(out[0]), 100 + 400 + 200)
        self.assertEqual(out[0][0], 10)                       # starts with the quiet pre-roll

    def test_short_pause_stays_in_one_utterance(self):
        self.assertEqual(len(self.feed(Segmenter(self.cfg), [1, 1, 0, 1, 1, 0, 0, 0])), 1)

    def test_click_shorter_than_min_is_dropped(self):
        self.assertEqual(self.feed(Segmenter(self.cfg), [0, 1, 0, 0, 0, 0]), [])

    def test_long_speech_is_cut_at_max(self):
        out = self.feed(Segmenter(self.cfg), [1] * 30 + [0] * 3)
        self.assertEqual([len(u) for u in out], [2000, 1000 + 200])

    def test_energy_vad_without_a_flag(self):
        self.assertEqual(len(self.feed(Segmenter(self.cfg), [0, 1, 1, 1, 1, 0, 0, 0], flag=False)), 1)

    def test_hold_blocks_new_utterances_but_lets_a_started_one_finish(self):
        seg, out = Segmenter(self.cfg), []
        for _ in range(5):
            out += seg.feed(tone(3277), speech=True, hold=True)      # servo noise flagged as speech while moving
        self.assertEqual(out, [])
        for voiced, hold in [(1, False), (1, False), (1, True), (1, True), (0, True), (0, True)]:
            out += seg.feed(tone(3277 if voiced else 10), speech=bool(voiced), hold=hold)
        self.assertEqual([len(u) for u in out], [100 + 400 + 200])

    def test_reset_drops_the_utterance_in_progress(self):
        seg = Segmenter(self.cfg)
        self.feed(seg, [1, 1, 1])
        seg.reset()
        self.assertEqual(self.feed(seg, [0, 0, 0]), [])

    def test_spool_writes_wav_and_keeps_only_the_last(self):
        with tempfile.TemporaryDirectory() as d:
            open(os.path.join(d, 'notes.txt'), 'w').close()        # not ours: never pruned
            spool = UtteranceSpool(d, rate=16000, keep_last=2)
            paths = [spool.write(tone(i + 1, 1600)) for i in range(3)]
            self.assertEqual(sorted(os.listdir(d)), sorted([os.path.basename(p) for p in paths[1:]] + ['notes.txt']))
            with wave.open(paths[-1]) as w:
                self.assertEqual((w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()), (1, 2, 16000, 1600))


def follow(att, speaker_deg, seconds, t0=0.0, dt=0.05, head=None, speech=True, **kw):
    """Run the attention loop with a speaker at a fixed base_link bearing (audio_node has already turned the
    head-relative DoA into it); the head follows each command at once (head_node slews at 90 °/s, faster than
    attention asks). Returns (outputs, time, [pan, tilt])."""
    head = [0.0, 0.0] if head is None else head
    outs, t = [], t0
    for _ in range(int(round(seconds / dt))):
        t += dt
        out = att.update(t, bearing_deg=speaker_deg, speech=speech and speaker_deg is not None, **kw)
        if out.head_target is not None:
            head[:] = out.head_target
        outs.append(out)
    return outs, t, head


class Attending(unittest.TestCase):
    def test_voice_from_the_left_turns_eyes_first_then_the_head_gently(self):
        att, cfg = Attention(), AttentionConfig()
        outs, t, head = follow(att, 40.0, 3.0)
        self.assertTrue(all(o.gaze is None and o.head_target is None for o in outs[:6]))    # < min_speech_s
        first = next(i for i, o in enumerate(outs) if o.gaze)
        self.assertLessEqual(first, 10)
        self.assertAlmostEqual(outs[first].gaze[0], 40 / 60, places=2)       # eyes jump to the voice
        self.assertLess(outs[first].head_target[0], 5.0)                       # the head starts slowly
        pans = [o.head_target[0] for o in outs if o.head_target]
        self.assertLessEqual(max(abs(b - a) for a, b in zip(pans, pans[1:])), cfg.turn_rate_deg_s * 0.05 + 1e-9)
        self.assertAlmostEqual(head[0], 40.0, places=6)
        self.assertAlmostEqual(head[1], cfg.listen_tilt_deg, places=6)
        self.assertEqual(outs[-1].gaze, (round(40 / 60, 3), round(-cfg.listen_tilt_deg / 12, 3)))  # = head gaze then

    def test_head_limits_are_respected(self):
        att = Attention(AttentionConfig(max_pan_deg=80.0, listen_tilt_deg=30.0))     # asks too much on purpose
        outs, t, head = follow(att, 150.0, 5.0)
        self.assertEqual(head, [60.0, 12.0])
        self.assertTrue(all(abs(o.head_target[0]) <= 60 and abs(o.head_target[1]) <= 12 for o in outs if o.head_target))
        self.assertEqual(outs[-1].gaze[0], 1.0)
        outs, t, head = follow(Attention(), -150.0, 5.0)
        self.assertEqual(head[0], -AttentionConfig().max_pan_deg)

    def test_speaker_inside_the_deadband_moves_only_the_eyes(self):
        outs, t, head = follow(Attention(), 8.0, 3.0)
        self.assertEqual(head[0], 0.0)
        self.assertAlmostEqual(outs[-1].gaze[0], round(8 / 60, 3))

    def test_a_stalled_timer_does_not_make_the_head_jump(self):
        # CPU starvation in Docker delayed the 20 Hz timer; after a stall the head must still move gently.
        att, cfg = Attention(), AttentionConfig()
        outs, t, head = follow(att, 40.0, 0.6)
        before = head[0]
        out = att.update(t + 1.0, bearing_deg=40.0, speech=True)
        self.assertLessEqual(out.head_target[0] - before, cfg.turn_rate_deg_s * 0.1 + 1e-9)

    def test_holds_then_returns_to_centre_and_lets_go(self):
        att, cfg = Attention(), AttentionConfig()
        outs, t, head = follow(att, 40.0, 2.0)
        outs, t, head = follow(att, None, cfg.hold_s - 0.2, t0=t, head=head)
        self.assertEqual(outs[-1].head_target, (40.0, cfg.listen_tilt_deg))
        self.assertIsNotNone(outs[-1].gaze)
        outs, t, head = follow(att, None, 3.0, t0=t, head=head)
        returning = [o.head_target[0] for o in outs if o.head_target]
        self.assertTrue(0 < returning[len(returning) // 2] < 40)
        self.assertEqual(returning[-1], 0.0)
        self.assertEqual((outs[-1].head_target, outs[-1].gaze), (None, None))

    def test_no_reaction_while_asleep_on_the_dock(self):
        outs, t, head = follow(Attention(), 40.0, 3.0, dock_state='CHARGING')
        self.assertTrue(all(o.head_target is None and o.gaze is None for o in outs))

    def test_docking_keeps_the_head_still_but_the_eyes_glance(self):
        outs, t, head = follow(Attention(), 40.0, 3.0, dock_state='APPROACH')
        self.assertTrue(all(o.head_target is None for o in outs))
        self.assertAlmostEqual(outs[-1].gaze[0], round(40 / 60, 3))

    def test_head_turned_when_docking_starts_comes_back_to_centre(self):
        att = Attention()
        outs, t, head = follow(att, 40.0, 3.0)
        outs, t, head = follow(att, 40.0, 3.0, t0=t, head=head, dock_state='SEEK')
        self.assertEqual(head[0], 0.0)
        self.assertIsNone(outs[-1].head_target)

    def test_own_voice_is_ignored_but_keeps_luna_facing_the_person(self):
        outs, t, head = follow(Attention(), 40.0, 3.0, self_speaking=True)
        self.assertTrue(all(o.gaze is None for o in outs))
        att, cfg = Attention(), AttentionConfig()
        outs, t, head = follow(att, 40.0, 2.0)
        outs, t, head = follow(att, None, cfg.hold_s * 2, t0=t, head=head, self_speaking=True)   # a long reply
        self.assertEqual(outs[-1].head_target, (40.0, cfg.listen_tilt_deg))

    def test_scattered_directions_do_not_engage(self):
        att, t, outs = Attention(), 0.0, []
        for i in range(60):
            t += 0.05
            outs.append(att.update(t, bearing_deg=90.0 if i % 2 else -90.0, speech=True))
        self.assertTrue(all(o.gaze is None for o in outs))

    def test_moving_head_freezes_what_attention_heard(self):
        # Tilt servo ~49 mm from a mic (R07): while the head moves, DoA is not trusted and is not a pause either.
        att = Attention()
        outs, t, head = follow(att, 40.0, 2.0)
        outs, t, head = follow(att, -60.0, 1.0, t0=t, head=head, head_moving=True)
        self.assertTrue(all(o.gaze[0] == round(40 / 60, 3) for o in outs))
        out = att.update(t + 0.05, bearing_deg=40.0, speech=True)
        self.assertTrue(out.engaged)                   # no re-warm-up: the frozen second was not a gap

    def test_mock_speakers_answer_like_the_head_mounted_array(self):
        mock = MockSpeakers([(1.0, 2.0, 35.0)], period_s=10.0, elevation_deg=25.0)
        self.assertEqual(mock.sample(0.5), (None, False))
        az, speech = mock.sample(1.5, pan_deg=20.0, tilt_deg=8.0)
        self.assertTrue(speech)
        self.assertAlmostEqual(az, base_to_head(35.0, 25.0, 20.0, 8.0), places=9)
        self.assertAlmostEqual(head_to_base(az, 20.0, 8.0, speaker_elevation_deg=25.0), 35.0, places=6)
        self.assertEqual(mock.sample(11.5)[1], True)


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class HeadFrame(unittest.TestCase):
    """XVF3800 under the head top shell, logo up, turning with pan/tilt (R07): array azimuth → base_link.
    pan + = left (URDF head_pan, +z); tilt + = nose up (head.py), i.e. URDF head_tilt joint = −tilt."""

    def test_level_head_adds_pan_with_wrap_around(self):
        for pan in (-60.0, -25.0, 0.0, 30.0, 60.0):
            for az in (-170.0, -90.0, -10.0, 0.0, 45.0, 135.0, 179.0):
                self.assertAlmostEqual(wrap(head_to_base(az, pan, 0.0, speaker_elevation_deg=0.0) - wrap(pan + az)), 0.0,
                                       places=6)
        self.assertAlmostEqual(head_to_base(170.0, 60.0, 0.0), -130.0, places=6)     # 230° → −130°
        self.assertAlmostEqual(head_to_base(-150.0, -60.0, 0.0), 150.0, places=6)

    def test_result_is_in_minus_180_to_180(self):
        for az in range(-180, 181, 15):
            for pan in (-60, 60):
                for tilt in (-12, 12):
                    b = head_to_base(float(az), float(pan), float(tilt))
                    self.assertTrue(-180.0 < b <= 180.0, b)

    def test_joint_angles_are_held_to_the_head_limits(self):
        self.assertAlmostEqual(head_to_base(10.0, 75.0, 0.0), head_to_base(10.0, 60.0, 0.0), places=9)
        self.assertAlmostEqual(head_to_base(10.0, -80.0, 0.0), head_to_base(10.0, -60.0, 0.0), places=9)
        self.assertAlmostEqual(head_to_base(45.0, 0.0, 30.0), head_to_base(45.0, 0.0, 12.0), places=9)
        self.assertAlmostEqual(head_to_base(45.0, 0.0, -30.0), head_to_base(45.0, 0.0, -12.0), places=9)

    def test_tilt_matters_off_the_tilt_axis_only(self):
        # For a voice in the array plane the tilt axis (head left/right) is unchanged; straight ahead stays ahead
        # at any height. A raised voice heard at exactly 90° on a nose-up head is really a little behind.
        self.assertAlmostEqual(head_to_base(90.0, 0.0, 12.0, speaker_elevation_deg=0.0), 90.0, places=6)
        self.assertAlmostEqual(head_to_base(0.0, 0.0, 12.0), 0.0, places=6)
        self.assertGreater(head_to_base(90.0, 0.0, 12.0, speaker_elevation_deg=25.0), 90.0)
        self.assertNotAlmostEqual(head_to_base(45.0, 0.0, 12.0, speaker_elevation_deg=0.0), 45.0, places=1)

    def test_round_trip_through_the_tilted_array_is_exact(self):
        for pan in (-60.0, 0.0, 35.0, 60.0):
            for tilt in (-12.0, -4.0, 0.0, 4.0, 12.0):
                for az in (-150.0, -60.0, -5.0, 20.0, 89.0, 160.0):
                    measured = base_to_head(az, 25.0, pan, tilt)
                    self.assertAlmostEqual(wrap(head_to_base(measured, pan, tilt, speaker_elevation_deg=25.0) - az), 0.0,
                                           places=6)

    def test_unknown_speaker_height_costs_a_few_degrees(self):
        # A planar array cannot see elevation; 25° (a standing or sitting adult a metre or two away) is assumed.
        worst = 0.0
        for el in range(0, 41, 5):
            for tilt in (-12.0, -6.0, 0.0, 6.0, 12.0):
                for az in range(-180, 180, 5):
                    measured = base_to_head(float(az), float(el), 0.0, tilt)
                    worst = max(worst, abs(wrap(head_to_base(measured, 0.0, tilt, speaker_elevation_deg=25.0) - az)))
        self.assertLess(worst, 6.0)                     # under attention's 12° deadband


class Gating(unittest.TestCase):
    def head(self, gate, t0, seconds, rate_deg_s, start=0.0, dt=0.02):
        pan, t = start, t0
        for _ in range(int(round(seconds / dt))):
            t += dt
            pan += rate_deg_s * dt
            gate.head(t, pan, 0.0)
        return t, pan

    def test_head_motion_pauses_speech_until_it_settles(self):
        gate = MicGate()
        t, pan = self.head(gate, 0.0, 0.5, 0.0)
        self.assertTrue(gate.speech(t, True))
        t, pan = self.head(gate, t, 0.5, 30.0, start=pan)          # attention turning the head
        self.assertTrue(gate.moving(t))
        self.assertFalse(gate.speech(t, True))
        self.assertFalse(gate.may_start_utterance(t))
        t, pan = self.head(gate, t, 0.2, 0.0, start=pan)           # command stopped, servo still settling
        self.assertTrue(gate.moving(t))
        t, pan = self.head(gate, t, 0.3, 0.0, start=pan)
        self.assertFalse(gate.moving(t))
        self.assertTrue(gate.speech(t, True))

    def test_tiny_jitter_is_not_motion(self):
        gate, t = MicGate(), 0.0
        for i in range(100):
            t += 0.02
            gate.head(t, 0.01 * (i % 2), 0.0)
        self.assertFalse(gate.moving(t))

    def test_mute_silences_everything(self):
        gate = MicGate(muted=True)
        self.assertFalse(gate.speech(1.0, True))
        self.assertFalse(gate.may_start_utterance(1.0))
        gate.muted = False
        self.assertTrue(gate.speech(1.0, True))


class VoiceFace(unittest.TestCase):
    def test_voice_state_priority(self):
        self.assertEqual(voice_state(listening=True, thinking=True, speaking=True), 'speaking')
        self.assertEqual(voice_state(listening=True, thinking=True, speaking=False), 'thinking')
        self.assertEqual(voice_state(listening=True, thinking=False, speaking=False), 'listening')
        self.assertEqual(voice_state(listening=False, thinking=False, speaking=False), 'idle')

    def test_listening_thinking_speaking_use_existing_face_events(self):
        vm = VoiceMood()
        seq = [(0.0, 'listening'), (0.5, 'listening'), (1.6, 'listening'), (2.0, 'thinking'), (3.2, 'thinking'),
               (3.5, 'speaking'), (5.9, 'speaking'), (6.1, 'speaking'), (6.5, 'idle'), (9.0, 'idle')]
        got = [(t, [e['type'] for e in vm.update(t, voice=v, dock_state='IDLE')]) for t, v in seq]
        self.assertEqual(got, [(0.0, ['curious']), (0.5, []), (1.6, ['curious']), (2.0, []), (3.2, ['curious']),
                               (3.5, ['stroke']), (5.9, []), (6.1, ['stroke']), (6.5, []), (9.0, [])])
        self.assertTrue({e for _, evs in got for e in evs} <= FACE_EVENTS)

    def test_voice_never_wakes_luna_sleeping_on_the_dock(self):
        vm = VoiceMood()
        self.assertEqual(vm.update(0.0, voice='listening', dock_state='CHARGING'), [])
        self.assertEqual(vm.update(1.0, voice='speaking', dock_state='DOCKED'), [])
        self.assertEqual(vm.update(1.1, voice='speaking', dock_state='UNDOCK'), [{'type': 'stroke'}])

    def test_attention_gaze_wins_while_fresh(self):
        self.assertEqual(pick_gaze((0.1, 0.0), (0.5, -0.2), attention_age_s=0.2), (0.5, -0.2))
        self.assertEqual(pick_gaze((0.1, 0.0), (0.5, -0.2), attention_age_s=0.8), (0.1, 0.0))
        self.assertEqual(pick_gaze((0.1, 0.0), None, attention_age_s=0.0), (0.1, 0.0))


if __name__ == '__main__':
    unittest.main()
