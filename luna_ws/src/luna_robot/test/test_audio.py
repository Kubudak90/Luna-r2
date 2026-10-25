"""Audio side without hardware: ReSpeaker Mic Array v2.0 USB control against a fake device, PCM levels and utterance
cutting, mute and body-turn gating, voice and face → eye gaze + a slow body turn, and listening/thinking/speaking →
face events.
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

from luna_robot.core import respeaker as R  # noqa: E402
from luna_robot.core.attention import Attention, AttentionConfig, MockSpeakers  # noqa: E402
from luna_robot.core.audio import MicGate, Segmenter, SegmenterConfig, UtteranceSpool, channel, rms_dbfs  # noqa: E402
from luna_robot.core.face_mood import FACE_EVENTS, VoiceMood, fresh_gaze, voice_state  # noqa: E402


class FakeArray:
    """pyusb-like device answering vendor reads from a table {(wIndex, wValue): (int32, int32)}."""

    def __init__(self, values, short=False):
        self.values, self.short, self.calls = values, short, []

    def ctrl_transfer(self, bm_request_type, b_request, w_value, w_index, length, timeout):
        self.calls.append((bm_request_type, b_request, w_value, w_index, length))
        reply = struct.pack('<ii', *self.values[(w_index, w_value)])
        return array('B', reply[:4] if self.short else reply)


class ArrayControl(unittest.TestCase):
    def test_doa_read_is_seeeds_tuning_request(self):
        # tuning.py: wValue = 0x80 | offset | 0x40 (int), wIndex = parameter id, 8 bytes back.
        dev = FakeArray({(21, 0xC0): (270, 0), (19, 0xC0 | 22): (1, 0)})
        self.assertEqual(R.ArrayControl(dev).doa(), R.DoaReading(270, True))
        self.assertEqual(dev.calls, [(0xC0, 0, 0xC0, 21, 8), (0xC0, 0, 0xC0 | 22, 19, 8)])

    def test_angle_wraps_and_speech_flag_is_boolean(self):
        dev = FakeArray({(21, 0xC0): (365, 0), (19, 0xC0 | 22): (0, 0)})
        self.assertEqual(R.ArrayControl(dev).doa(), R.DoaReading(5, False))

    def test_short_reply_raises(self):
        with self.assertRaises(R.ArrayError):
            R.ArrayControl(FakeArray({(21, 0xC0): (10, 0)}, short=True)).read('DOAANGLE')

    def test_board_angle_to_base_bearing(self):
        # The board's 0° and counting direction are not documented: both are calibration parameters.
        self.assertEqual(R.board_to_base(0), 0)
        self.assertEqual(R.board_to_base(90), 90)
        self.assertEqual(R.board_to_base(270), -90)
        self.assertEqual(R.board_to_base(180), 180)        # (-180, 180]
        self.assertEqual(R.board_to_base(90, direction=-1), -90)
        self.assertEqual(R.board_to_base(100, zero_deg=90), 10)
        self.assertEqual(R.board_to_base(10, zero_deg=350), 20)

    def test_alsa_card_found_by_name(self):
        cards = (' 0 [tegrahda       ]: tegra-hda - tegra-hda\n'
                 '                      tegra-hda at 0x3518000 irq 63\n'
                 ' 2 [ArrayUAC10     ]: USB-Audio - ReSpeaker 4 Mic Array (UAC1.0)\n'
                 '                      SEEED ReSpeaker 4 Mic Array (UAC1.0) at usb-3610000.xhci-2.3, full speed\n')
        self.assertEqual(R.find_alsa_device(cards), f'plughw:CARD={R.CARD_ID},DEV=0')
        self.assertIsNone(R.find_alsa_device(cards.splitlines()[0]))

    def test_open_usb_errors_say_why(self):
        with self.assertRaisesRegex(R.ArrayError, 'pyusb'):
            R.open_usb(import_usb=lambda: (_ for _ in ()).throw(ImportError('no usb')))

        class Core:
            @staticmethod
            def find(**kw):
                return None
        with self.assertRaisesRegex(R.ArrayError, 'not on USB'):          # audio_node reads this as the privacy slider
            R.open_usb(import_usb=lambda: Core)


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
            out += seg.feed(tone(3277), speech=True, hold=True)      # motor noise flagged as speech while turning
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


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def follow(att, world_deg, seconds, t0=0.0, dt=0.05, yaw=None, speech=True, face=None, **kw):
    """Run the attention loop with a speaker at a fixed world bearing; the body turns as commanded (the array and
    the face camera turn with it, so what they report is the world bearing minus the yaw). face: (world bearing,
    elevation) or None. Returns (outputs, time, [yaw deg])."""
    yaw = [0.0] if yaw is None else yaw
    outs, t = [], t0
    for _ in range(int(round(seconds / dt))):
        t += dt
        bearing = None if world_deg is None else wrap(world_deg - yaw[0])
        f = None if face is None else (wrap(face[0] - yaw[0]), face[1])
        out = att.update(t, yaw_deg=yaw[0], bearing_deg=bearing, speech=speech and bearing is not None, face=f, **kw)
        if out.turn:
            yaw[0] = wrap(yaw[0] + math.degrees(out.turn) * dt)
        outs.append(out)
    return outs, t, yaw


class Attending(unittest.TestCase):
    def test_voice_from_the_left_turns_eyes_first_then_the_body_slowly(self):
        att, cfg = Attention(), AttentionConfig()
        outs, t, yaw = follow(att, 40.0, 5.0)
        self.assertTrue(all(o.gaze is None and o.turn is None for o in outs[:6]))    # < min_speech_s
        first = next(i for i, o in enumerate(outs) if o.gaze)
        self.assertLessEqual(first, 10)
        self.assertAlmostEqual(outs[first].gaze[0], 40 / 60, places=2)       # eyes jump to the voice
        turns = [o.turn for o in outs if o.turn]
        self.assertTrue(turns and all(0 < w <= math.radians(cfg.turn_rate_deg_s) + 1e-9 for w in turns))
        self.assertAlmostEqual(yaw[0], 40.0, delta=cfg.settle_deg)          # the body faces the person
        self.assertIn(0.0, [o.turn for o in outs])                           # the turn ends with an explicit stop
        self.assertIsNone(outs[-1].turn)
        self.assertLess(abs(outs[-1].gaze[0]), 0.06)                         # facing: the eyes look straight on
        self.assertAlmostEqual(outs[-1].gaze[1], round(cfg.listen_elevation_deg / cfg.elevation_range_deg, 3))

    def test_body_turns_all_the_way_round(self):
        att = Attention()
        outs, t, yaw = follow(att, 150.0, 10.0)
        self.assertAlmostEqual(yaw[0], 150.0, delta=AttentionConfig().settle_deg)
        first = next(o for o in outs if o.gaze)
        self.assertEqual(first.gaze[0], 1.0)                                  # eyes at the edge until the body turns

    def test_speaker_inside_the_deadband_moves_only_the_eyes(self):
        outs, t, yaw = follow(Attention(), 8.0, 3.0)
        self.assertEqual(yaw[0], 0.0)
        self.assertTrue(all(o.turn is None for o in outs))
        self.assertAlmostEqual(outs[-1].gaze[0], round(8 / 60, 3))

    def test_a_stalled_timer_does_not_make_the_body_jump(self):
        att, cfg = Attention(), AttentionConfig()
        outs, t, yaw = follow(att, 40.0, 0.6)
        out = att.update(t + 1.0, yaw_deg=yaw[0], bearing_deg=40.0 - yaw[0], speech=True)
        self.assertLessEqual(abs(out.turn), math.radians(cfg.turn_rate_deg_s) + 1e-9)   # a rate, never a jump

    def test_holds_then_lets_go(self):
        att, cfg = Attention(), AttentionConfig()
        outs, t, yaw = follow(att, 40.0, 3.0)
        outs, t, yaw = follow(att, None, cfg.hold_s - 0.2, t0=t, yaw=yaw)
        self.assertIsNotNone(outs[-1].gaze)
        outs, t, yaw = follow(att, None, 1.0, t0=t, yaw=yaw)
        self.assertEqual((outs[-1].turn, outs[-1].gaze), (None, None))       # the face looks around by itself again

    def test_no_reaction_while_asleep_on_the_dock(self):
        outs, t, yaw = follow(Attention(), 40.0, 3.0, dock_state='CHARGING', face=(10.0, 20.0))
        self.assertTrue(all(o.turn is None and o.gaze is None for o in outs))

    def test_docking_keeps_the_body_on_course_but_the_eyes_glance(self):
        outs, t, yaw = follow(Attention(), 40.0, 3.0, dock_state='APPROACH')
        self.assertTrue(all(not o.turn for o in outs))
        self.assertEqual(yaw[0], 0.0)
        self.assertAlmostEqual(outs[-1].gaze[0], round(40 / 60, 3))

    def test_no_turn_while_listening_driving_or_bumped(self):
        for kw in ({'hold': True}, {'driving': True}, {'blocked': True}):
            with self.subTest(**kw):
                outs, t, yaw = follow(Attention(), 40.0, 3.0, **kw)
                self.assertEqual(yaw[0], 0.0)
                self.assertAlmostEqual(outs[-1].gaze[0], round(40 / 60, 3))   # the eyes still go to the voice

    def test_a_turn_cut_short_ends_with_a_stop(self):
        att = Attention()
        outs, t, yaw = follow(att, 60.0, 1.0)
        self.assertTrue(outs[-1].turn and outs[-1].turn > 0)
        out = att.update(t + 0.05, yaw_deg=yaw[0], bearing_deg=60.0 - yaw[0], speech=True, hold=True)
        self.assertEqual(out.turn, 0.0)                                       # a touch on the face: stop at once

    def test_own_voice_is_ignored_but_keeps_luna_facing_the_person(self):
        outs, t, yaw = follow(Attention(), 40.0, 3.0, self_speaking=True)
        self.assertTrue(all(o.gaze is None for o in outs))
        att, cfg = Attention(), AttentionConfig()
        outs, t, yaw = follow(att, 40.0, 3.0)
        outs, t, yaw = follow(att, None, cfg.hold_s * 2, t0=t, yaw=yaw, self_speaking=True)   # a long reply
        self.assertIsNotNone(outs[-1].gaze)

    def test_scattered_directions_do_not_engage(self):
        att, t, outs = Attention(), 0.0, []
        for i in range(60):
            t += 0.05
            outs.append(att.update(t, bearing_deg=90.0 if i % 2 else -90.0, speech=True))
        self.assertTrue(all(o.gaze is None for o in outs))

    def test_turning_body_freezes_what_attention_heard(self):
        # While the body turns, the gearmotors are loud and the bearing moves under the array: not trusted, not a pause.
        att = Attention()
        outs, t, yaw = follow(att, 8.0, 2.0)                                   # engaged, inside the deadband
        outs, t, yaw = follow(att, -60.0, 1.0, t0=t, yaw=yaw, turning=True)
        self.assertTrue(all(o.gaze[0] == round(8 / 60, 3) for o in outs))
        out = att.update(t + 0.05, yaw_deg=yaw[0], bearing_deg=8.0, speech=True)
        self.assertTrue(out.engaged)                   # no re-warm-up: the frozen second was not a gap

    def test_a_face_gets_eye_contact_and_a_turn_before_it_leaves_the_picture(self):
        cfg = AttentionConfig()
        outs, t, yaw = follow(Attention(), None, 2.0, face=(10.0, 20.0))
        self.assertEqual(yaw[0], 0.0)                                         # inside the face deadband
        self.assertEqual(outs[-1].gaze, (round(10 / 60, 3), round(20 / 30, 3)))
        self.assertEqual(outs[-1].target, 'face')
        outs, t, yaw = follow(Attention(), None, 4.0, face=(30.0, 20.0))
        self.assertAlmostEqual(yaw[0], 30.0, delta=cfg.settle_deg)

    def test_the_speakers_face_gives_the_voice_its_height(self):
        outs, t, yaw = follow(Attention(), 5.0, 2.0, face=(9.0, 24.0))
        self.assertEqual(outs[-1].target, 'voice')
        self.assertEqual(outs[-1].gaze, (round(9 / 60, 3), round(24 / 30, 3)))   # the face, not the assumed height
        outs, t, yaw = follow(Attention(), 5.0, 2.0, face=(-40.0, 24.0))
        self.assertEqual(outs[-1].gaze[1], round(AttentionConfig().listen_elevation_deg / 30, 3))   # someone else

    def test_mock_speakers_answer_like_the_body_mounted_array(self):
        mock = MockSpeakers([(1.0, 2.0, 35.0)], period_s=10.0)
        self.assertEqual(mock.sample(0.5), (None, False))
        self.assertEqual(mock.sample(1.5), (35.0, True))
        self.assertEqual(mock.sample(1.5, yaw_deg=20.0), (15.0, True))      # the body turned toward the voice
        self.assertEqual(mock.sample(11.5)[1], True)


class Gating(unittest.TestCase):
    def body(self, gate, t0, seconds, rate, dt=0.02):
        t = t0
        for _ in range(int(round(seconds / dt))):
            t += dt
            gate.body(t, rate)
        return t

    def test_turning_pauses_speech_until_it_settles(self):
        gate = MicGate()
        t = self.body(gate, 0.0, 0.5, 0.0)
        self.assertTrue(gate.speech(t, True))
        t = self.body(gate, t, 0.5, 0.5)                             # attention turning the body
        self.assertTrue(gate.moving(t))
        self.assertFalse(gate.speech(t, True))
        self.assertFalse(gate.may_start_utterance(t))
        t = self.body(gate, t, 0.2, 0.0)                             # wheels stopped, gearbox still settling
        self.assertTrue(gate.moving(t))
        t = self.body(gate, t, 0.3, 0.0)
        self.assertFalse(gate.moving(t))
        self.assertTrue(gate.speech(t, True))

    def test_odometry_noise_is_not_turning(self):
        gate = MicGate()
        t = self.body(gate, 0.0, 2.0, 0.05)
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

    def test_each_voice_state_is_one_face_event(self):
        vm = VoiceMood()
        seq = ['listening', 'listening', 'thinking', 'thinking', 'speaking', 'speaking', 'idle', 'idle']
        got = [[e['type'] for e in vm.update(voice=v, dock_state='IDLE')] for v in seq]
        self.assertEqual(got, [['listening'], [], ['thinking'], [], ['speaking'], [], ['idle'], []])
        self.assertTrue({e for evs in got for e in evs} <= FACE_EVENTS)

    def test_voice_never_wakes_luna_sleeping_on_the_dock(self):
        vm = VoiceMood()
        self.assertEqual(vm.update(voice='listening', dock_state='CHARGING'), [])
        self.assertEqual(vm.update(voice='speaking', dock_state='DOCKED'), [])
        self.assertEqual(vm.update(voice='speaking', dock_state='UNDOCK'), [{'type': 'speaking'}])

    def test_attention_gaze_only_while_fresh(self):
        self.assertEqual(fresh_gaze((0.5, -0.2), age_s=0.2), (0.5, -0.2))
        self.assertIsNone(fresh_gaze((0.5, -0.2), age_s=1.5))
        self.assertIsNone(fresh_gaze(None, age_s=0.0))


if __name__ == '__main__':
    unittest.main()
