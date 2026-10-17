"""Turn toward a voice: /luna/audio/doa → eye gaze for the face (/luna/face/gaze) and a gentle head turn
(/luna/head/target). Logic in core/attention.py; head targets go out only while attention uses the head."""
import time

from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

from . import topics as T
from .entry import run
from .core.attention import Attention, AttentionConfig

TUNABLE = ['max_pan_deg', 'listen_tilt_deg', 'turn_rate_deg_s', 'deadband_deg', 'min_speech_s', 'hold_s']
DOA_FRESH_S = 0.35


class AttentionNode(Node):
    def __init__(self):
        super().__init__('luna_attention')
        defaults = AttentionConfig()
        for name in TUNABLE:
            self.declare_parameter(name, float(getattr(defaults, name)))
        self.att = Attention(AttentionConfig(**{n: float(self.get_parameter(n).value) for n in TUNABLE}))
        self.doa, self.doa_t = None, 0.0
        self.head_pan, self.dock_state, self.speaking = 0.0, 'UNKNOWN', False   # UNKNOWN: eyes only until the dock reports
        self.create_subscription(Float32MultiArray, T.AUDIO_DOA, self._doa, 10)
        self.create_subscription(Vector3, T.HEAD_STATE, lambda m: setattr(self, 'head_pan', m.x), 10)
        self.create_subscription(String, T.DOCK_STATE, lambda m: setattr(self, 'dock_state', m.data), 10)
        self.create_subscription(Bool, T.VOICE_SPEAKING, lambda m: setattr(self, 'speaking', m.data), 10)
        self.pub_target = self.create_publisher(Vector3, T.HEAD_TARGET, 10)
        self.pub_gaze = self.create_publisher(Vector3, T.FACE_GAZE, 10)
        self.create_timer(0.05, self._tick)

    def _doa(self, msg):
        if len(msg.data) >= 2:
            self.doa, self.doa_t = (msg.data[0], msg.data[1] > 0.5), time.monotonic()

    def _tick(self):
        now = time.monotonic()
        fresh = self.doa is not None and now - self.doa_t < DOA_FRESH_S
        out = self.att.update(now, doa_head_deg=self.doa[0] if fresh else None, speech=fresh and self.doa[1],
                              head_pan_deg=self.head_pan, dock_state=self.dock_state, self_speaking=self.speaking)
        if out.head_target is not None:
            self.pub_target.publish(Vector3(x=out.head_target[0], y=out.head_target[1], z=0.0))
        if out.gaze is not None:
            self.pub_gaze.publish(Vector3(x=out.gaze[0], y=out.gaze[1], z=0.0))


def main():
    run(AttentionNode)


if __name__ == '__main__':
    main()
