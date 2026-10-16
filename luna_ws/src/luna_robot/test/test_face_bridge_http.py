"""End-to-end check of the face bridge SSE server without ROS: rclpy and message modules are stubbed,
the real Hub/HTTP handler run on a free port and a real HTTP client reads the event stream."""
import json
import os
import socket
import sys
import threading
import types
import unittest
import urllib.request
from http.server import ThreadingHTTPServer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
for name in ['rclpy', 'rclpy.node', 'rclpy.executors', 'geometry_msgs', 'geometry_msgs.msg', 'std_msgs', 'std_msgs.msg']:
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules['rclpy.node'].Node = object
sys.modules['rclpy.executors'].ExternalShutdownException = type('ExternalShutdownException', (Exception,), {})
for mod, names in [('geometry_msgs.msg', ['Vector3']), ('std_msgs.msg', ['String', 'UInt8'])]:
    for n in names:
        setattr(sys.modules[mod], n, type(n, (), {}))

from luna_robot.core.face_mood import MoodMapper  # noqa: E402
from luna_robot.face_bridge_node import Hub, make_handler  # noqa: E402


def free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FaceBridgeHttp(unittest.TestCase):
    def test_stream_delivers_mood_events(self):
        hub, port = Hub(), free_port()
        server = ThreadingHTTPServer(('127.0.0.1', port), make_handler(hub, 'http://127.0.0.1:4173'))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            self.assertEqual(urllib.request.urlopen(f'http://127.0.0.1:{port}/health', timeout=3).read(), b'ok')
            resp = urllib.request.urlopen(f'http://127.0.0.1:{port}/events', timeout=5)
            self.assertEqual(resp.headers['Content-Type'], 'text/event-stream')
            self.assertEqual(resp.headers['Access-Control-Allow-Origin'], 'http://127.0.0.1:4173')
            self.assertEqual(resp.readline(), b'retry: 2000\n')
            resp.readline()
            mood = MoodMapper()
            for ev in mood.update(bumpers=1, dock_state='IDLE', battery_action='run', gaze=(0.3, 0.0)):
                hub.send(ev)
            got = []
            while len(got) < 2:
                line = resp.readline().decode().strip()
                if line.startswith('data: '):
                    got.append(json.loads(line[6:]))
            self.assertEqual(got, [{'type': 'tap'}, {'type': 'gaze', 'x': 0.3, 'y': 0.0}])
        finally:
            server.shutdown()

    def test_client_connecting_while_charging_gets_sleep(self):
        hub, port, mood = Hub(), free_port(), MoodMapper()
        state = {'bumpers': 0, 'dock_state': 'IDLE', 'battery_action': 'run', 'gaze': (0.0, 0.0)}
        server = ThreadingHTTPServer(('127.0.0.1', port),
                                     make_handler(hub, '*', snapshot=lambda: MoodMapper.snapshot(**state)))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            state.update(dock_state='CHARGING', battery_action='charge')
            for ev in mood.update(**state):
                hub.send(ev)                       # the live `sleep` goes out while no face is connected
            resp = urllib.request.urlopen(f'http://127.0.0.1:{port}/events', timeout=5)
            self.assertEqual(resp.readline(), b'retry: 2000\n')
            resp.readline()
            self.assertEqual(resp.readline(), b'data: {"type":"sleep"}\n')
            resp.readline()
            hub.send({'type': 'wake'})             # live events follow the snapshot
            self.assertEqual(resp.readline(), b'data: {"type":"wake"}\n')
        finally:
            server.shutdown()

    def test_unknown_path_is_404(self):
        hub, port = Hub(), free_port()
        server = ThreadingHTTPServer(('127.0.0.1', port), make_handler(hub, '*'))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            with self.assertRaises(urllib.error.HTTPError) as e:
                urllib.request.urlopen(f'http://127.0.0.1:{port}/secret', timeout=3)
            self.assertEqual(e.exception.code, 404)
        finally:
            server.shutdown()


if __name__ == '__main__':
    unittest.main()
