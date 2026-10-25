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
for mod, names in [('geometry_msgs.msg', ['Vector3']), ('std_msgs.msg', ['Bool', 'Empty', 'Float32MultiArray', 'String', 'UInt8'])]:
    for n in names:
        setattr(sys.modules[mod], n, type(n, (), {}))

from luna_robot.core.face_mood import FACE_EVENTS, MoodMapper  # noqa: E402
from luna_robot.face_bridge_node import STATIC, Hub, make_handler  # noqa: E402


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

    def test_face_reconnecting_after_undock_gets_wake(self):
        # The face slept on the dock, lost the stream while Luna undocked and missed the live `wake`.
        hub, port = Hub(), free_port()
        state = {'bumpers': 0, 'dock_state': 'UNDOCK', 'battery_action': 'run', 'gaze': (0.2, 0.0)}
        server = ThreadingHTTPServer(('127.0.0.1', port),
                                     make_handler(hub, '*', snapshot=lambda: MoodMapper.snapshot(**state)))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            resp = urllib.request.urlopen(f'http://127.0.0.1:{port}/events', timeout=5)
            self.assertEqual(resp.readline(), b'retry: 2000\n')
            resp.readline()
            self.assertEqual(resp.readline(), b'data: {"type":"wake"}\n')
            resp.readline()
            self.assertEqual(resp.readline(), b'data: {"type":"gaze","x":0.2,"y":0.0}\n')  # after wake, which clears gaze
        finally:
            server.shutdown()

    def _talk_server(self, origin='http://127.0.0.1:4173', on_talk=True):
        hub, port, talks = Hub(), free_port(), []
        server = ThreadingHTTPServer(('127.0.0.1', port),
                                     make_handler(hub, origin, on_talk=(lambda: talks.append(1)) if on_talk else None))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return f'http://127.0.0.1:{port}/talk', talks

    def _post(self, url, headers=None, data=b'', method='POST'):
        req = urllib.request.Request(url, data=data, headers=headers or {}, method=method)
        try:
            with urllib.request.urlopen(req, timeout=3) as resp:
                return resp.status, dict(resp.headers)
        except urllib.error.HTTPError as e:
            with e:
                return e.code, dict(e.headers)

    def test_touch_from_the_face_page_reaches_ros(self):
        url, talks = self._talk_server()
        status, headers = self._post(url, {'Origin': 'http://127.0.0.1:4173'})
        self.assertEqual((status, talks), (204, [1]))
        self.assertEqual(headers['Access-Control-Allow-Origin'], 'http://127.0.0.1:4173')
        self.assertEqual(self._post(url)[0], 204)                  # curl / ros tools send no Origin
        self.assertEqual(len(talks), 2)

    def test_other_web_pages_cannot_open_the_microphones(self):
        url, talks = self._talk_server()
        self.assertEqual(self._post(url, {'Origin': 'https://evil.example'})[0], 403)
        self.assertEqual(self._post(url, {'Origin': 'null'})[0], 403)
        self.assertEqual(talks, [])

    def test_preflight_big_body_get_and_no_talk_handler(self):
        url, talks = self._talk_server()
        status, headers = self._post(url, {'Origin': 'http://127.0.0.1:4173'}, data=None, method='OPTIONS')
        self.assertEqual(status, 204)
        self.assertEqual(headers['Access-Control-Allow-Methods'], 'POST')
        self.assertEqual(self._post(url, data=b'x' * 4096)[0], 413)
        self.assertEqual(self._post(url, data=None, method='GET')[0], 404)
        self.assertEqual(talks, [])
        url, _ = self._talk_server(on_talk=False)
        self.assertEqual(self._post(url)[0], 404)

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


WEB = os.path.join(os.path.dirname(__file__), '..', 'web')


class FacePage(unittest.TestCase):
    def test_serves_the_face_and_its_modules_only(self):
        hub, port = Hub(), free_port()
        server = ThreadingHTTPServer(('127.0.0.1', port), make_handler(hub, '*', static_dir=WEB))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            for path, kind, needle in (('/', 'text/html', b"import { startFace } from './face.js'"),
                                       ('/face.js', 'text/javascript', b"from './face-core.js'"),
                                       ('/face-core.js', 'text/javascript', b'export function createFaceCore'),
                                       ('/robot-link.js', 'text/javascript', b"from './face-core.js'")):
                with urllib.request.urlopen(f'http://127.0.0.1:{port}{path}', timeout=3) as resp:
                    self.assertTrue(resp.headers['Content-Type'].startswith(kind), path)
                    self.assertIn(needle, resp.read(), path)
            for path in ('/package.json', '/face-core.test.mjs', '/../setup.py', '/web/face.js'):
                with self.assertRaises(urllib.error.HTTPError, msg=path) as e:
                    urllib.request.urlopen(f'http://127.0.0.1:{port}{path}', timeout=3)
                self.assertEqual(e.exception.code, 404)
        finally:
            server.shutdown()

    def test_every_module_the_page_imports_is_served(self):
        import re
        served = {name for name, _ in STATIC.values()}
        for name in served:
            with open(os.path.join(WEB, name)) as f:
                for dep in re.findall(r"from '\./([\w.-]+)'", f.read()):
                    self.assertIn(dep, served, f'{name} imports {dep}')

    def test_face_page_knows_exactly_the_events_face_mood_sends(self):
        import re
        with open(os.path.join(WEB, 'face-core.js')) as f:
            block = re.search(r'FACE_EVENTS = Object\.freeze\(\[(.*?)\]\)', f.read(), re.S).group(1)
        self.assertEqual(set(re.findall(r"'([\w-]+)'", block)), FACE_EVENTS)


if __name__ == '__main__':
    unittest.main()
