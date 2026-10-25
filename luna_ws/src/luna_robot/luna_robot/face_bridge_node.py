"""Face bridge: robot state → Luna face events, streamed to the face page as Server-Sent Events, and the face page
itself.

Standard library HTTP server only (no extra packages in the ROS container). Endpoints:
  GET /            the face (web/face.html, face.js, face-core.js, robot-link.js from the package share), for the kiosk browser
                   on the head's AMOLED: same origin as the events, so allowed_origin is this server itself
  GET /events      text/event-stream, one JSON face event per message
  GET /health      'ok'
  POST /talk       a touch on the face to talk (push-to-talk) → /luna/voice/talk; 204. A browser request from any
                   origin other than allowed_origin is refused (403), so another web page cannot open the microphones.
Default bind is 127.0.0.1: the face runs on the robot itself, no phone needed. Set host:=0.0.0.0 only on a trusted
LAN to watch the face from another screen; events carry no personal data, but anyone on that LAN can then ask Luna
to listen for one question, as if they touched her face."""
import json
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32MultiArray, String, UInt8

from . import topics as T
from .entry import run
from .core.face_mood import MoodMapper, VoiceMood, fresh_gaze, voice_state


LISTENING_FRESH_S = 1.0     # audio_node publishes /luna/voice/listening at 5 Hz while capture is on


class Hub:
    def __init__(self):
        self.clients, self.lock = [], threading.Lock()

    def add(self):
        q = queue.Queue(maxsize=64)
        with self.lock:
            self.clients.append(q)
        return q

    def remove(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)

    def send(self, event):
        data = encode(event)
        with self.lock:
            for q in list(self.clients):
                try:
                    q.put_nowait(data)
                except queue.Full:
                    self.clients.remove(q)     # slow client: drop it, it will reconnect


def encode(event):
    return json.dumps(event, separators=(',', ':'))


MAX_TALK_BODY = 1024
STATIC = {'/': ('face.html', 'text/html; charset=utf-8'), '/face.html': ('face.html', 'text/html; charset=utf-8'),
          '/face.js': ('face.js', 'text/javascript; charset=utf-8'),
          '/face-core.js': ('face-core.js', 'text/javascript; charset=utf-8'),
          '/robot-link.js': ('robot-link.js', 'text/javascript; charset=utf-8')}


def make_handler(hub, origin, snapshot=lambda: [], on_talk=None, static_dir=None):
    """snapshot() returns the face events that bring a newly connected face to the current state; on_talk() is
    called for POST /talk (404 without it); static_dir holds the face page (only the STATIC names are served)."""
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _cors(self):
            self.send_header('Access-Control-Allow-Origin', origin)
            self.send_header('Vary', 'Origin')

        def do_OPTIONS(self):
            if self.path != '/talk' or on_talk is None:
                self.send_error(404)
                return
            self.send_response(204)
            self._cors()
            self.send_header('Access-Control-Allow-Methods', 'POST')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.end_headers()

        def do_POST(self):
            if self.path != '/talk' or on_talk is None:
                self.send_error(404)
                return
            sender = self.headers.get('Origin')      # browsers always send it on a POST; curl and tests do not
            if sender is not None and origin != '*' and sender != origin:
                self.send_error(403)
                return
            try:
                length = int(self.headers.get('Content-Length') or 0)
            except ValueError:
                length = -1
            if not 0 <= length <= MAX_TALK_BODY:
                self.send_error(413)
                return
            self.rfile.read(length)                     # the body carries nothing; read it so the connection stays sane
            on_talk()
            self.send_response(204)
            self._cors()
            self.end_headers()

        def _static(self, name, kind):
            try:
                with open(os.path.join(static_dir, name), 'rb') as f:
                    body = f.read()
            except OSError:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if static_dir and path in STATIC:
                self._static(*STATIC[path])
                return
            if self.path == '/health':
                self.send_response(200)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'ok')
                return
            if self.path != '/events':
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Access-Control-Allow-Origin', origin)
            self.end_headers()
            q = hub.add()             # subscribe before the snapshot: a change in between is sent twice, never lost
            try:
                self.wfile.write(b'retry: 2000\n\n')
                for event in snapshot():               # e.g. a face (re)connecting while Luna is on the dock
                    self.wfile.write(f'data: {encode(event)}\n\n'.encode())
                while True:
                    try:
                        data = q.get(timeout=15)
                        self.wfile.write(f'data: {data}\n\n'.encode())
                    except queue.Empty:
                        self.wfile.write(b': keepalive\n\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                hub.remove(q)
    return Handler


class FaceBridge(Node):
    def __init__(self):
        super().__init__('luna_face_bridge')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8765)
        self.declare_parameter('allowed_origin', 'http://127.0.0.1:8765')
        self.declare_parameter('web_dir', '')                # '' = the package share's web/
        self.hub = Hub()
        self.mood = MoodMapper()
        self.state = {'bumpers': 0, 'dock_state': 'IDLE', 'battery_action': 'run', 'gaze': None, 'muted': False}
        self.create_subscription(UInt8, T.MCU_BUMPERS, lambda m: self._set('bumpers', m.data), 10)
        self.create_subscription(String, T.DOCK_STATE, lambda m: self._set('dock_state', m.data), 10)
        self.create_subscription(String, T.POWER_ACTION, lambda m: self._set('battery_action', m.data), 10)
        self.create_subscription(Bool, T.AUDIO_MUTED, lambda m: self._set('muted', bool(m.data)), 10)
        # Gaze: attention's (a voice, a face) while fresh, otherwise none and the eyes look around by themselves.
        # listening / thinking / speaking / idle become face events through VoiceMood.
        self.gaze, self.gaze_t = None, 0.0
        self.heard_t, self.assistant, self.speaking = 0.0, 'idle', False
        self.stt, self.listening, self.listening_t = 'idle', False, float('-inf')
        self.voice = VoiceMood()
        self.create_subscription(Vector3, T.FACE_GAZE, self._gaze, 10)
        self.create_subscription(Float32MultiArray, T.AUDIO_DOA, self._doa, 10)
        self.create_subscription(String, T.ASSISTANT_STATUS, lambda m: setattr(self, 'assistant', m.data), 10)
        self.create_subscription(Bool, T.VOICE_SPEAKING, lambda m: setattr(self, 'speaking', m.data), 10)
        self.create_subscription(String, T.STT_STATUS, lambda m: setattr(self, 'stt', m.data), 10)
        self.create_subscription(Bool, T.VOICE_LISTENING, self._listening, 10)
        self.pub_talk = self.create_publisher(Empty, T.VOICE_TALK, 10)
        self.create_timer(0.1, self._voice_tick)
        host, port = self.get_parameter('host').value, self.get_parameter('port').value
        web = self.get_parameter('web_dir').value or self._share_web()
        handler = make_handler(self.hub, self.get_parameter('allowed_origin').value,
                               snapshot=lambda: MoodMapper.snapshot(**self.state, voice=self.voice.last),
                               on_talk=lambda: self.pub_talk.publish(Empty()), static_dir=web)
        server = ThreadingHTTPServer((host, port), handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f'face on http://{host}:{port}/ (events /events, page from {web})')

    @staticmethod
    def _share_web():
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory('luna_robot'), 'web')
        except Exception:
            return None

    def _gaze(self, msg):
        self.gaze, self.gaze_t = (msg.x, msg.y), time.monotonic()
        self._set('gaze', self.gaze)

    def _doa(self, msg):
        if len(msg.data) >= 2 and msg.data[1] > 0.5:
            self.heard_t = time.monotonic()

    def _listening(self, msg):
        self.listening, self.listening_t = bool(msg.data), time.monotonic()

    def _voice_tick(self):
        now = time.monotonic()
        if self.state['gaze'] is not None and fresh_gaze(self.gaze, now - self.gaze_t) is None:
            self._set('gaze', None)                     # attention let go: the eyes wander again
        # With capture on, audio_node says when Luna takes a question (push-to-talk: after a touch); without it,
        # any voice the array hears still makes her look attentive.
        listening = self.listening if now - self.listening_t < LISTENING_FRESH_S else now - self.heard_t < 0.5
        voice = voice_state(listening=listening, thinking=self.assistant == 'thinking' or self.stt == 'transcribing',
                            speaking=self.speaking)
        for event in self.voice.update(voice=voice, dock_state=self.state['dock_state']):
            self.hub.send(event)

    def _set(self, key, value):
        self.state[key] = value
        for event in self.mood.update(**self.state):
            self.hub.send(event)


def main():
    run(FaceBridge)


if __name__ == '__main__':
    main()
