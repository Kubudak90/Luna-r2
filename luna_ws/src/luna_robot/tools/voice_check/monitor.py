"""Logs the voice chain's topics with monotonic time and the mock voices' phase (t % 20); states only on change."""
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Empty, String

TOPICS = [('/luna/voice/talk', Empty), ('/luna/voice/listening', Bool), ('/luna/audio/utterance', String),
          ('/luna/stt/status', String), ('/luna/assistant/request', String), ('/luna/assistant/status', String),
          ('/luna/assistant/reply', String), ('/luna/voice/speaking', Bool), ('/luna/dock/state', String),
          ('/luna/head/target', Vector3)]
STATES = {'/luna/voice/listening', '/luna/voice/speaking', '/luna/dock/state', '/luna/head/target'}


class Monitor(Node):
    def __init__(self):
        super().__init__('voice_monitor')
        self.last = {}
        for topic, typ in TOPICS:
            self.create_subscription(typ, topic, lambda m, t=topic: self.log(t, m), 10)

    def log(self, topic, msg):
        value = round(msg.x, 1) if topic == '/luna/head/target' else getattr(msg, 'data', '')
        if topic in STATES and self.last.get(topic) == value:
            return
        self.last[topic] = value
        extra = f' exists={os.path.exists(value)}' if topic == '/luna/audio/utterance' else ''
        now = time.monotonic()
        print(f'{now:9.2f} φ{now % 20:5.2f} {topic} {value!r}{extra}', flush=True)


rclpy.init()
try:
    rclpy.spin(Monitor())
except (KeyboardInterrupt, ExternalShutdownException):
    pass
