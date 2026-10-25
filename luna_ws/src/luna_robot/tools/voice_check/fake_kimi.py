"""Answers like Kimi's OpenAI-compatible POST /v1/chat/completions for the router check; logs which fields came.
Quick requests (thinking disabled) answer [DÜŞÜN] for "gökkuşağı" questions; thinking requests answer for real."""
import json
import os
from http.server import BaseHTTPRequestHandler, HTTPServer

OUT = os.environ.get('VOICE_CHECK_OUT', '/tmp/voice_check')


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        quick = body.get('thinking') == {'type': 'disabled'}
        question = body['messages'][-1]['content']
        if quick:
            text = '[DÜŞÜN]' if 'gökkuşağı' in question.lower() else 'Merhaba! Ben Luna, seni gördüğüme sevindim.'
        else:
            text = 'Düşündüm: cevabın burada.'
        with open(os.path.join(OUT, 'kimi.log'), 'a') as f:
            f.write(json.dumps({'path': self.path, 'auth': self.headers.get('Authorization', '')[:13],
                                'model': body.get('model'), 'thinking': body.get('thinking'),
                                'reasoning_effort': body.get('reasoning_effort'),
                                'max_completion_tokens': body.get('max_completion_tokens'),
                                'marker_in_prompt': '[DÜŞÜN]' in body['messages'][0]['content'],
                                'history': len(body['messages']) - 2, 'answer': text}, ensure_ascii=False) + '\n')
        msg = {'role': 'assistant', 'content': text}
        if not quick:
            msg['reasoning_content'] = 'kısa bir düşünme'
        data = json.dumps({'choices': [{'message': msg, 'finish_reason': 'stop'}]}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


HTTPServer(('127.0.0.1', 9100), H).serve_forever()
