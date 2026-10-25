"""Answers like Piper's http_server POST /synthesize: a WAV of silence, 0.06 s per character, 22050 Hz."""
import io, json, wave
from http.server import BaseHTTPRequestHandler, HTTPServer

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_POST(self):
        req = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        buf = io.BytesIO()
        with wave.open(buf, 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(22050)
            w.writeframes(bytes(2 * int(22050 * 0.06 * len(req['text']))))
        print(json.dumps({'path': self.path, 'chars': len(req['text']), 'text': req['text'],
                          'length_scale': req.get('length_scale')}, ensure_ascii=False), flush=True)
        data = buf.getvalue()
        self.send_response(200); self.send_header('Content-Type', 'audio/wav')
        self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)

HTTPServer(('127.0.0.1', 5000), H).serve_forever()
