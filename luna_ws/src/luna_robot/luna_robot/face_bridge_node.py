"""Face bridge: robot state → Luna face events, streamed to the face PWA as Server-Sent Events.

Standard library HTTP server only (no extra packages in the ROS container). Endpoints:
  GET /events  text/event-stream, one JSON face event per message
  GET /health  'ok'
Default bind is 127.0.0.1 (kiosk browser on the robot). Set host:=0.0.0.0 only on a trusted LAN
if a phone should show the face; events carry no personal data."""
import json
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import String, UInt8

from . import topics as T
from .entry import run
from .core.face_mood import MoodMapper


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


def make_handler(hub, origin, snapshot=lambda: []):
    """snapshot() returns the face events that bring a newly connected face to the current state."""
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
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
        self.declare_parameter('allowed_origin', 'http://127.0.0.1:4173')
        self.hub = Hub()
        self.mood = MoodMapper()
        self.state = {'bumpers': 0, 'dock_state': 'IDLE', 'battery_action': 'run', 'gaze': (0.0, 0.0)}
        self.create_subscription(UInt8, T.MCU_BUMPERS, lambda m: self._set('bumpers', m.data), 10)
        self.create_subscription(String, T.DOCK_STATE, lambda m: self._set('dock_state', m.data), 10)
        self.create_subscription(String, T.POWER_ACTION, lambda m: self._set('battery_action', m.data), 10)
        self.create_subscription(Vector3, T.HEAD_GAZE, lambda m: self._set('gaze', (m.x, m.y)), 10)
        host, port = self.get_parameter('host').value, self.get_parameter('port').value
        handler = make_handler(self.hub, self.get_parameter('allowed_origin').value,
                               snapshot=lambda: MoodMapper.snapshot(**self.state))
        server = ThreadingHTTPServer((host, port), handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f'face events on http://{host}:{port}/events')

    def _set(self, key, value):
        self.state[key] = value
        for event in self.mood.update(**self.state):
            self.hub.send(event)


def main():
    run(FaceBridge)


if __name__ == '__main__':
    main()
