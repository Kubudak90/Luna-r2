"""ReSpeaker XVF3800: direction of arrival + speech flag over the USB control interface, and microphone capture
cut into utterance WAVs for speech-to-text (stt_node).

doa_source: usb  (pyusb; the node keeps retrying while the array or pyusb is missing, and publishes nothing)
            mock (scripted voices, for running without the array; capture then feeds silence at their times)
            off
The array turns with the head (R07), so its DoA is converted to base_link with the current pan/tilt
(core/head_frame.py). While the head moves, and settle time after, speech is reported as 0 and no new utterance
starts (tilt servo ~49 mm from a mic). Mute is software (/luna/audio/mute): the board's buttons are inside the
head. Muted: no DoA, no capture, and the board's own mute GPO is set as well when the control interface is open.

Capture uses `arecord` on the array's ALSA card. listen_mode push_to_talk (default): an utterance is kept only if
it starts within listen_window_s of a touch on the face (/luna/voice/talk), one per touch; everything else is
dropped in memory, nothing is written. Not on the dock (Luna sleeps there) and not while Luna speaks.
listen_mode always: every utterance (for a later wake word). /luna/voice/listening tells the face."""
import subprocess
import threading
import time
from array import array

from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32MultiArray, String

from . import topics as T
from .entry import run
from .core import xvf3800 as X
from .core.attention import MockSpeakers
from .core.audio import (ListenConfig, Listener, ListenWindow, MicGate, MicGateConfig, Segmenter, SegmenterConfig,
                         UtteranceSpool, channel)
from .core.head_frame import head_to_base

MOCK_SCRIPT = [(3.0, 2.5, 35.0), (11.0, 2.0, -30.0)]    # (start s, length s, base_link bearing deg) every 20 s
CHUNK_S = 0.02
DOCK_SLEEP = ('CHARGING', 'DOCKED')


def alsa_device(name):
    if name != 'auto':
        return name
    try:
        with open('/proc/asound/cards') as f:
            return X.find_alsa_device(f.read()) or 'default'
    except OSError:
        return 'default'


class AudioNode(Node):
    def __init__(self):
        super().__init__('luna_audio')
        for name, default in [('doa_source', 'usb'), ('doa_rate_hz', 10.0), ('doa_zero_deg', 0.0), ('doa_direction', 1),
                              ('capture', False), ('alsa_device', 'auto'), ('rate', 16000), ('channels', 2),
                              ('capture_channel', 0), ('utterance_dir', '/tmp/luna_utterances'), ('keep_utterances', 5),
                              ('speaker_elevation_deg', 25.0), ('start_muted', False), ('head_moving_deg_s', 2.0),
                              ('head_settle_s', 0.3), ('listen_mode', 'push_to_talk'), ('listen_window_s', 6.0)]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.source, self.zero, self.direction = p('doa_source'), float(p('doa_zero_deg')), int(p('doa_direction'))
        self.pub_doa = self.create_publisher(Float32MultiArray, T.AUDIO_DOA, 10)
        self.pub_utt = self.create_publisher(String, T.AUDIO_UTTERANCE, 10)
        self.elevation = float(p('speaker_elevation_deg'))
        self.gate = MicGate(MicGateConfig(float(p('head_moving_deg_s')), float(p('head_settle_s'))), muted=bool(p('start_muted')))
        self.listen = ListenWindow(ListenConfig(p('listen_mode'), float(p('listen_window_s'))))
        self.lock = threading.Lock()                     # listen window: ROS callbacks and the capture thread
        self.capture_on = bool(p('capture'))
        self.speaking, self.dock_state = False, 'UNKNOWN'
        self.head_pan = self.head_tilt = 0.0
        self.create_subscription(Vector3, T.HEAD_STATE, self._head, 10)
        self.create_subscription(Bool, T.AUDIO_MUTE, lambda m: self._set_mute(bool(m.data)), 10)
        self.create_subscription(Empty, T.VOICE_TALK, self._talk, 10)
        self.create_subscription(Bool, T.VOICE_SPEAKING, lambda m: setattr(self, 'speaking', bool(m.data)), 10)
        self.create_subscription(String, T.DOCK_STATE, lambda m: setattr(self, 'dock_state', m.data), 10)
        self.pub_muted = self.create_publisher(Bool, T.AUDIO_MUTED, 10)
        self.create_timer(1.0, lambda: self.pub_muted.publish(Bool(data=self.gate.muted)))
        self.ctl, self.next_open, self.last_error = None, 0.0, None
        self.speech, self.speech_t = None, 0.0          # last array speech flag, for the capture thread
        self.mock = MockSpeakers(MOCK_SCRIPT) if self.source == 'mock' else None
        if self.source in ('usb', 'mock'):
            self.create_timer(1.0 / p('doa_rate_hz'), self._doa_tick)
        self.get_logger().info(f'DoA source: {self.source}; listening: {self.listen.cfg.mode}'
                               + ('' if self.capture_on else ' (capture off: no questions)'))
        if self.capture_on:
            self.pub_listening = self.create_publisher(Bool, T.VOICE_LISTENING, 10)
            self.create_timer(0.2, self._publish_listening)
            args = (int(p('rate')), p('utterance_dir'), int(p('keep_utterances')))
            if self.mock:
                threading.Thread(target=self._capture_mock, args=args, daemon=True).start()
            else:
                threading.Thread(target=self._capture, args=(alsa_device(p('alsa_device')), int(p('channels')),
                                                             int(p('capture_channel'))) + args, daemon=True).start()

    def _head(self, msg):
        self.head_pan, self.head_tilt = msg.x, msg.y
        self.gate.head(time.monotonic(), msg.x, msg.y)

    def _set_mute(self, muted):
        changed = muted != self.gate.muted
        self.gate.muted = muted
        if muted:
            with self.lock:
                self.listen.cancel()
        if self.ctl is not None:
            try:
                self.ctl.set_mute(muted)
            except Exception as e:
                self._warn_once(f'XVF3800 hardware mute failed (software mute still on): {e}')
        if changed:
            self.get_logger().info('microphones muted' if muted else 'microphones unmuted')
        self.pub_muted.publish(Bool(data=muted))

    def _talk(self, _msg):
        if not self.listen.push_to_talk:
            return                                       # always: every utterance is a question already
        if not self.capture_on:
            self._warn_once('touch to talk, but capture is off: nothing to listen with')
            return
        if self.gate.muted or self.dock_state in DOCK_SLEEP:
            self.get_logger().info('touch to talk ignored: ' + ('muted' if self.gate.muted else 'asleep on the dock'))
            return
        with self.lock:
            opened = self.listen.talk(time.monotonic())
        if opened:
            self.get_logger().info(f'touch: listening for {self.listen.cfg.window_s:.0f} s')
            self._publish_listening()

    def _publish_listening(self):
        with self.lock:
            listening = self.listen.listening(time.monotonic())
        self.pub_listening.publish(Bool(data=listening and not self.gate.muted))

    def _warn_once(self, msg):
        if msg != self.last_error:
            self.get_logger().warning(msg)
            self.last_error = msg

    def _publish(self, head_deg, speech, raw_deg):
        now = time.monotonic()
        moving = self.gate.moving(now)
        speech = self.gate.speech(now, speech)
        base = head_to_base(head_deg, self.head_pan, self.head_tilt, self.elevation)
        self.speech, self.speech_t = speech, now
        self.pub_doa.publish(Float32MultiArray(data=[float(base), 1.0 if speech else 0.0, float(raw_deg),
                                                     1.0 if moving else 0.0, float(head_deg)]))

    def _doa_tick(self):
        if self.gate.muted:
            return                                   # muted: Luna is not listening, not even for a direction
        if self.mock:
            bearing, speech = self.mock.sample(time.monotonic(), self.head_pan, self.head_tilt)
            self._publish(bearing if speech else 0.0, speech, float('nan'))
            return
        now = time.monotonic()
        if self.ctl is None:
            if now < self.next_open:
                return
            self.next_open = now + 5.0
            try:
                self.ctl = X.XvfControl(X.open_usb())
                self.get_logger().info('XVF3800 control interface open')
                self.last_error = None
                self._set_mute(self.gate.muted)          # the board may have rebooted: re-apply the mute state
            except X.XvfError as e:
                self._warn_once(f'{e}; retrying every 5 s')
                return
        try:
            r = self.ctl.doa()
        except Exception as e:          # XvfError, usb.core.USBError (unplugged, permissions)
            self._warn_once(f'XVF3800 read failed: {e}')
            self.ctl = None
            return
        self._publish(X.board_to_head(r.azimuth_deg, self.zero, self.direction), r.speech, r.azimuth_deg)

    def _listener(self, rate, directory, keep):
        return (Listener(Segmenter(SegmenterConfig(rate=rate)), self.gate, self.listen),
                UtteranceSpool(directory, rate=rate, keep_last=keep))

    def _feed(self, listener, spool, samples, now):
        fresh = now - self.speech_t < 0.5
        with self.lock:
            done = listener.feed(samples, now, speech=self.speech if fresh else None, speaking=self.speaking)
        for utt in done:
            self.pub_utt.publish(String(data=spool.write(utt)))

    def _capture(self, device, channels, index, rate, directory, keep):
        listener, spool = self._listener(rate, directory, keep)
        nbytes = int(rate * CHUNK_S) * channels * 2
        argv = ['arecord', '-q', '-D', device, '-f', 'S16_LE', '-r', str(rate), '-c', str(channels), '-t', 'raw']
        while True:
            try:
                proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            except FileNotFoundError:
                self.get_logger().error('arecord not found (alsa-utils): microphone capture disabled')
                return
            self.get_logger().info(f'capturing {device} {rate} Hz {channels} ch, channel {index} → {directory}')
            while True:
                raw = proc.stdout.read(nbytes)
                if not raw:
                    break
                self._feed(listener, spool, channel(raw, channels, index), time.monotonic())
            proc.wait()
            self._warn_once(f'arecord on {device} stopped (code {proc.returncode}); restarting in 5 s')
            time.sleep(5.0)

    def _capture_mock(self, rate, directory, keep):
        """doa:=mock: silence at real-time pace; the scripted voices' speech flag cuts it into utterances."""
        listener, spool = self._listener(rate, directory, keep)
        silence = array('h', bytes(2 * int(rate * CHUNK_S)))
        self.get_logger().info(f'mock capture (silence at the scripted voices) → {directory}')
        t = time.monotonic()
        while True:
            t += CHUNK_S
            time.sleep(max(0.0, t - time.monotonic()))
            self._feed(listener, spool, silence, time.monotonic())


def main():
    run(AudioNode)


if __name__ == '__main__':
    main()
