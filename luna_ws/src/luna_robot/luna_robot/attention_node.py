"""Turn toward a person: /luna/audio/doa (base_link, speech gated by mute and body turning) and /luna/vision/face →
eye gaze for the face (/luna/face/gaze) and a slow body turn (/luna/attention/cmd_vel, the lowest-priority source in
base_node's mux). Logic in core/attention.py. No turning while Luna takes a question (/luna/voice/listening), while
anything else drives (/cmd_vel fresh) or with a bumper pressed."""
import math
import time

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8

from . import topics as T
from .entry import run
from .core.attention import Attention, AttentionConfig

TUNABLE = ['gaze_range_deg', 'listen_elevation_deg', 'turn_rate_deg_s', 'deadband_deg', 'face_deadband_deg',
           'min_speech_s', 'hold_s', 'face_hold_s']
DOA_FRESH_S = 0.35
FACE_FRESH_S = 0.4
LISTENING_FRESH_S = 1.0     # audio_node publishes /luna/voice/listening at 5 Hz while capture is on
DRIVING_FRESH_S = 1.0       # Nav2's controller publishes at 20 Hz while it drives


class AttentionNode(Node):
    def __init__(self):
        super().__init__('luna_attention')
        defaults = AttentionConfig()
        for name in TUNABLE:
            self.declare_parameter(name, float(getattr(defaults, name)))
        self.att = Attention(AttentionConfig(**{n: float(self.get_parameter(n).value) for n in TUNABLE}))
        self.doa, self.doa_t = None, 0.0
        self.face, self.face_t = None, 0.0
        self.dock_state, self.speaking = 'UNKNOWN', False            # UNKNOWN: eyes only until the dock reports
        self.listening, self.listening_t = False, float('-inf')
        self.driving_t, self.bumpers, self.yaw_deg = float('-inf'), 0, 0.0
        self.create_subscription(Float32MultiArray, T.AUDIO_DOA, self._doa, 10)
        self.create_subscription(Vector3, T.VISION_FACE, self._face, 10)
        self.create_subscription(String, T.DOCK_STATE, lambda m: setattr(self, 'dock_state', m.data), 10)
        self.create_subscription(Bool, T.VOICE_SPEAKING, lambda m: setattr(self, 'speaking', m.data), 10)
        self.create_subscription(Bool, T.VOICE_LISTENING, self._listening, 10)
        self.create_subscription(Twist, T.CMD_VEL, self._driving, 10)
        self.create_subscription(UInt8, T.MCU_BUMPERS, lambda m: setattr(self, 'bumpers', m.data), 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.pub_turn = self.create_publisher(Twist, T.ATTENTION_CMD_VEL, 10)
        self.pub_gaze = self.create_publisher(Vector3, T.FACE_GAZE, 10)
        self.create_timer(0.05, self._tick)

    def _doa(self, msg):
        if len(msg.data) >= 2:
            turning = len(msg.data) >= 4 and msg.data[3] > 0.5
            self.doa, self.doa_t = (msg.data[0], msg.data[1] > 0.5, turning), time.monotonic()

    def _face(self, msg):
        self.face, self.face_t = (msg.x, msg.y), time.monotonic()

    def _listening(self, msg):
        self.listening, self.listening_t = bool(msg.data), time.monotonic()

    def _driving(self, msg):
        if msg.linear.x or msg.angular.z:
            self.driving_t = time.monotonic()

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw_deg = math.degrees(2 * math.atan2(q.z, q.w))

    def _tick(self):
        now = time.monotonic()
        fresh = self.doa is not None and now - self.doa_t < DOA_FRESH_S
        out = self.att.update(now, yaw_deg=self.yaw_deg, bearing_deg=self.doa[0] if fresh else None,
                              speech=fresh and self.doa[1], turning=fresh and self.doa[2],
                              face=self.face if now - self.face_t < FACE_FRESH_S else None,
                              dock_state=self.dock_state, self_speaking=self.speaking,
                              hold=self.listening and now - self.listening_t < LISTENING_FRESH_S,
                              driving=now - self.driving_t < DRIVING_FRESH_S, blocked=self.bumpers != 0)
        if out.turn is not None:
            t = Twist()
            t.angular.z = out.turn
            self.pub_turn.publish(t)
        if out.gaze is not None:
            self.pub_gaze.publish(Vector3(x=out.gaze[0], y=out.gaze[1], z=0.0))


def main():
    run(AttentionNode)


if __name__ == '__main__':
    main()
