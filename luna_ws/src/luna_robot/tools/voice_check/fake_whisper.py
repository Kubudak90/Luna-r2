"""Answers like whisper.cpp's whisper-server /inference; logs the fields it got (not a real model)."""
import io, json, sys, wave
from email import policy
from email.parser import BytesParser
from http.server import BaseHTTPRequestHandler, HTTPServer

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_POST(self):
        body = self.rfile.read(int(self.headers['Content-Length']))
        msg = BytesParser(policy=policy.default).parsebytes(f'Content-Type: {self.headers["Content-Type"]}\r\n\r\n'.encode() + body)
        f = {p.get_param('name', header='content-disposition'): p.get_content() for p in msg.iter_parts()}
        with wave.open(io.BytesIO(f['file'])) as w:
            info = (w.getframerate(), w.getnchannels(), round(w.getnframes() / w.getframerate(), 2))
        print(json.dumps({'path': self.path, 'language': f.get('language'), 'response_format': f.get('response_format'),
                          'wav_rate_ch_s': info}), flush=True)
        data = json.dumps({'text': ' Merhaba Luna, bugün hava nasıl?\n'}).encode()
        self.send_response(200); self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)

HTTPServer(('127.0.0.1', 8178), H).serve_forever()
