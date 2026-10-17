"""ReSpeaker XVF3800: direction of arrival + speech flag over the USB control interface, and (optional) microphone
capture cut into utterance WAVs for a speech-to-text plug-in.

doa_source: usb  (pyusb; the node keeps retrying while the array or pyusb is missing, and publishes nothing)
            mock (scripted voices, for running without the array)
            off
Capture uses `arecord` on the array's ALSA card and is off by default: it records people at home."""
import subprocess
import threading
import time

from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from . import topics as T
from .entry import run
from .core import xvf3800 as X
from .core.attention import MockSpeakers
from .core.audio import Segmenter, SegmenterConfig, UtteranceSpool, channel

MOCK_SCRIPT = [(3.0, 2.5, 35.0), (11.0, 2.0, -30.0)]    # (start s, length s, body bearing deg) every 20 s
CHUNK_S = 0.02


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
                              ('capture_channel', 0), ('utterance_dir', '/tmp/luna_utterances'), ('keep_utterances', 5)]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.source, self.zero, self.direction = p('doa_source'), float(p('doa_zero_deg')), int(p('doa_direction'))
        self.pub_doa = self.create_publisher(Float32MultiArray, T.AUDIO_DOA, 10)
        self.pub_utt = self.create_publisher(String, T.AUDIO_UTTERANCE, 10)
        self.head_pan = 0.0
        self.create_subscription(Vector3, T.HEAD_STATE, lambda m: setattr(self, 'head_pan', m.x), 10)
        self.ctl, self.next_open, self.last_error = None, 0.0, None
        self.speech, self.speech_t = None, 0.0          # last array speech flag, for the capture thread
        self.mock = MockSpeakers(MOCK_SCRIPT) if self.source == 'mock' else None
        if self.source in ('usb', 'mock'):
            self.create_timer(1.0 / p('doa_rate_hz'), self._doa_tick)
        self.get_logger().info(f'DoA source: {self.source}')
        if p('capture'):
            device = alsa_device(p('alsa_device'))
            threading.Thread(target=self._capture, args=(device, int(p('rate')), int(p('channels')), int(p('capture_channel')),
                                                         p('utterance_dir'), int(p('keep_utterances'))), daemon=True).start()

    def _warn_once(self, msg):
        if msg != self.last_error:
            self.get_logger().warning(msg)
            self.last_error = msg

    def _publish(self, head_deg, speech, raw_deg):
        self.speech, self.speech_t = speech, time.monotonic()
        self.pub_doa.publish(Float32MultiArray(data=[float(head_deg), 1.0 if speech else 0.0, float(raw_deg)]))

    def _doa_tick(self):
        if self.mock:
            bearing, speech = self.mock.sample(time.monotonic(), self.head_pan)
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

    def _capture(self, device, rate, channels, index, directory, keep):
        seg, spool = Segmenter(SegmenterConfig(rate=rate)), UtteranceSpool(directory, rate=rate, keep_last=keep)
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
                fresh = time.monotonic() - self.speech_t < 0.5
                for utt in seg.feed(channel(raw, channels, index), speech=self.speech if fresh else None):
                    self.pub_utt.publish(String(data=spool.write(utt)))
            proc.wait()
            self._warn_once(f'arecord on {device} stopped (code {proc.returncode}); restarting in 5 s')
            time.sleep(5.0)


def main():
    run(AudioNode)


if __name__ == '__main__':
    main()
