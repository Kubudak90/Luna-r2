"""Cloud assistant client and speech output without network or audio: a fake HTTP transport, a fake clock and
a fake process launcher stand in for the real ones. Run from luna_ws/src/luna_robot:
python3 -m unittest discover -s test"""
import json
import os
import stat
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core.assistant import (AssistantClient, ClientPolicy, HttpResult, ProviderConfig,  # noqa: E402
                                       TransportError, load_api_key)
from luna_robot.core.speech import CommandTts, NullTts, SpeechQueue, speakable  # noqa: E402

KEY = 'sk-test-0123456789abcdef'


def ok(text, **extra):
    msg = {'role': 'assistant', 'content': text, **extra}
    return HttpResult(200, {}, json.dumps({'choices': [{'message': msg, 'finish_reason': 'stop'}],
                                           'usage': {'total_tokens': 42}}).encode())


def err(status, kind, headers=None, message='nope'):
    return HttpResult(status, headers or {}, json.dumps({'error': {'type': kind, 'message': message}}).encode())


class FakeTransport:
    def __init__(self, *answers):
        self.answers, self.calls = list(answers), []

    def __call__(self, url, headers, body, timeout):
        self.calls.append({'url': url, 'headers': headers, 'body': json.loads(body), 'timeout': timeout})
        a = self.answers.pop(0)
        if isinstance(a, Exception):
            raise a
        return a


class Clock:
    def __init__(self):
        self.t, self.sleeps = 1000.0, []

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.sleeps.append(s)
        self.t += s


def client(*answers, key=KEY, enabled=True, provider=ProviderConfig(), **policy):
    transport, clock = FakeTransport(*answers), Clock()
    c = AssistantClient(provider, ClientPolicy(**policy), api_key=key, transport=transport, enabled=enabled,
                        clock=clock, sleep=clock.sleep)
    return c, transport, clock


class ApiKey(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def key_file(self, *parts, mode=0o600):
        path = os.path.join(self.tmp.name, *parts)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            f.write(f'  {KEY}\n')
        os.chmod(path, mode)
        return path

    def test_environment_wins_over_the_file(self):
        key, notes = load_api_key('MOONSHOT_API_KEY', self.key_file('k'), environ={'MOONSHOT_API_KEY': 'sk-env'})
        self.assertEqual((key, notes), ('sk-env', []))

    def test_key_file_outside_the_repo_is_read_and_stripped(self):
        self.assertEqual(load_api_key('MOONSHOT_API_KEY', self.key_file('cfg', 'k'), environ={}), (KEY, []))

    def test_key_file_inside_a_git_checkout_is_refused(self):
        os.makedirs(os.path.join(self.tmp.name, 'repo', '.git'))
        key, notes = load_api_key('MOONSHOT_API_KEY', self.key_file('repo', 'config', 'k'), environ={})
        self.assertIsNone(key)
        self.assertIn('git', notes[0])

    def test_world_readable_key_file_is_used_with_a_warning(self):
        key, notes = load_api_key('MOONSHOT_API_KEY', self.key_file('k', mode=0o644), environ={})
        self.assertEqual(key, KEY)
        self.assertIn('chmod 600', notes[0])

    def test_missing_key(self):
        self.assertEqual(load_api_key('MOONSHOT_API_KEY', os.path.join(self.tmp.name, 'none'), environ={})[0], None)
        self.assertEqual(load_api_key('MOONSHOT_API_KEY', '', environ={'MOONSHOT_API_KEY': '  '})[0], None)


class Client(unittest.TestCase):
    def test_no_network_without_consent(self):
        c, transport, _ = client(ok('hi'), enabled=False)
        reply = c.ask('merhaba')
        self.assertEqual((reply.source, reply.reason), ('offline', 'disabled'))
        self.assertTrue(reply.text)
        self.assertEqual(transport.calls, [])

    def test_no_network_without_a_key(self):
        c, transport, _ = client(ok('hi'), key=None)
        self.assertEqual(c.ask('merhaba').reason, 'no_key')
        self.assertEqual(transport.calls, [])

    def test_request_matches_kimi_code_chat_completions(self):
        c, transport, _ = client(ok('Merhaba!'), timeout_s=12.0)
        reply = c.ask('merhaba Luna')
        self.assertEqual((reply.text, reply.source, reply.reason), ('Merhaba!', 'cloud', None))
        call = transport.calls[0]
        self.assertEqual(call['url'], 'https://api.kimi.com/coding/v1/chat/completions')
        self.assertEqual(call['headers']['Authorization'], f'Bearer {KEY}')
        self.assertEqual(call['headers']['Content-Type'], 'application/json')
        body = call['body']
        self.assertEqual(body['model'], 'kimi-for-coding')            # K2.8 Preview
        self.assertEqual(body['thinking'], {'type': 'disabled'})          # spoken replies: no slow reasoning pass
        self.assertEqual(body['messages'][0]['role'], 'system')
        self.assertEqual(body['messages'][-1], {'role': 'user', 'content': 'merhaba Luna'})
        self.assertIn('max_completion_tokens', body)
        self.assertNotIn('temperature', body)                             # fixed by the model; other values are errors
        self.assertEqual(call['timeout'], 12.0)

    def test_other_openai_compatible_provider(self):
        p = ProviderConfig(name='other', base_url='https://llm.example/v1/', model='m', extra_body={})
        c, transport, _ = client(ok('x'), provider=p)
        c.ask('hi')
        self.assertEqual(transport.calls[0]['url'], 'https://llm.example/v1/chat/completions')
        self.assertNotIn('thinking', transport.calls[0]['body'])

    def test_history_is_kept_and_trimmed(self):
        c, transport, _ = client(ok('a1'), ok('a2'), ok('a3'), history_turns=1)
        c.ask('q1')
        c.ask('q2')
        self.assertEqual([m['content'] for m in transport.calls[1]['body']['messages'][1:]], ['q1', 'a1', 'q2'])
        c.ask('q3')
        self.assertEqual([m['content'] for m in transport.calls[2]['body']['messages'][1:]], ['q2', 'a2', 'q3'])

    def test_reasoning_is_not_spoken(self):
        c, _, _ = client(ok('Cevap.', reasoning_content='uzun düşünce'))
        self.assertEqual(c.ask('?').text, 'Cevap.')

    def test_server_errors_are_retried_with_backoff(self):
        c, transport, clock = client(err(503, 'server_unavailable'), err(500, 'server_error'), ok('tamam'))
        self.assertEqual(c.ask('?').text, 'tamam')
        self.assertEqual(len(transport.calls), 3)
        self.assertEqual(clock.sleeps, [1.0, 2.0])

    def test_retry_after_is_honoured(self):
        c, transport, clock = client(err(429, 'engine_overloaded_error', {'Retry-After': '3'}), ok('tamam'))
        self.assertEqual(c.ask('?').source, 'cloud')
        self.assertEqual(clock.sleeps, [3.0])

    def test_quota_auth_model_and_filter_errors_are_not_retried(self):
        for answer, reason in [(err(429, 'exceeded_current_quota_error'), 'quota'),
                               (err(401, 'invalid_authentication_error'), 'auth'),
                               (err(404, 'resource_not_found_error'), 'model'),
                               (err(400, 'content_filter'), 'content_filter'),
                               (err(400, 'invalid_request_error'), 'bad_request')]:
            c, transport, _ = client(answer)
            reply = c.ask('?')
            self.assertEqual((reply.source, reply.reason, len(transport.calls)), ('offline', reason, 1))
            self.assertTrue(reply.text)

    def test_timeouts_use_up_the_retries_then_fall_back(self):
        c, transport, _ = client(*[TransportError('timed out')] * 3, retries=2)
        reply = c.ask('?')
        self.assertEqual((reply.source, reply.reason, len(transport.calls)), ('offline', 'network', 3))

    def test_total_deadline_bounds_the_wait(self):
        c, transport, clock = client(err(429, 'engine_overloaded_error', {'Retry-After': '100'}), ok('late'),
                                     deadline_s=30.0)
        self.assertEqual(c.ask('?').reason, 'rate_limit')
        self.assertEqual((clock.sleeps, len(transport.calls)), ([], 1))

    def test_breaker_skips_the_network_after_repeated_failures_then_recovers(self):
        c, transport, clock = client(*[TransportError('down')] * 2, ok('geri geldim'), retries=0,
                                     breaker_failures=2, breaker_cooldown_s=60.0)
        c.ask('1')
        c.ask('2')
        self.assertEqual(c.ask('3').reason, 'breaker_open')
        self.assertEqual(len(transport.calls), 2)
        clock.t += 61
        self.assertEqual(c.ask('4').text, 'geri geldim')

    def test_bad_response_is_an_offline_reply_not_a_crash(self):
        for body in [b'not json', b'{"choices": []}', json.dumps({'choices': [{'message': {'content': ' '}}]}).encode()]:
            c, _, _ = client(HttpResult(200, {}, body))
            self.assertEqual(c.ask('?').reason, 'bad_response')

    def test_input_is_bounded_and_empty_input_is_ignored(self):
        c, transport, _ = client(ok('x'), max_input_chars=10)
        self.assertIsNone(c.ask('   '))
        c.ask('a' * 50)
        self.assertEqual(transport.calls[0]['body']['messages'][-1]['content'], 'a' * 10)

    def test_key_never_leaks_into_replies(self):
        c, _, _ = client(err(401, 'invalid_authentication_error', message=f'bad key {KEY}'))
        reply = c.ask('?')
        self.assertNotIn(KEY, reply.text + (reply.detail or ''))
        self.assertNotIn(KEY, repr(c))


class FakePopen:
    launched = []

    def __init__(self, argv, stdin=None, stdout=None, stderr=None):
        FakePopen.launched.append(argv)
        self.stdout, self.returncode = 'pipe', 0

    def wait(self, timeout=None):
        return 0

    def poll(self):
        return 0

    def terminate(self):
        pass


class Speech(unittest.TestCase):
    def test_speakable_strips_markdown_links_and_urls(self):
        self.assertEqual(speakable('**Merhaba** _dünya_! [Kaynak](https://x.y) bak: https://a.b/c `kod` # Başlık'),
                         'Merhaba dünya! Kaynak bak: kod Başlık')

    def test_long_text_is_cut_at_a_sentence(self):
        self.assertEqual(speakable('Bir iki. Üç dört beş. Altı yedi sekiz dokuz.', max_chars=25), 'Bir iki. Üç dört beş.')
        self.assertEqual(speakable('a' * 30, max_chars=10), 'a' * 10)

    def test_queue_splits_sentences_and_drops_oldest_when_full(self):
        q = SpeechQueue(max_items=3)
        q.push('Bir. İki? Üç! Dört.')
        self.assertEqual([q.pop() for _ in range(4)], ['İki?', 'Üç!', 'Dört.', None])
        q.push('Yeni.')
        q.clear()
        self.assertIsNone(q.pop())

    def test_command_tts_gives_text_as_one_argument_never_a_shell(self):
        FakePopen.launched = []
        tts = CommandTts(['espeak-ng', '-v', 'tr', '--stdout', '{text}'], player=['aplay', '-q', '-D', '{device}'],
                         device='plughw:CARD=Array,DEV=0', popen=FakePopen)
        tts.say('Merhaba; rm -rf ~')
        self.assertEqual(FakePopen.launched, [['espeak-ng', '-v', 'tr', '--stdout', 'Merhaba; rm -rf ~'],
                                              ['aplay', '-q', '-D', 'plughw:CARD=Array,DEV=0']])
        FakePopen.launched = []
        tts.say('-5 derece')
        self.assertEqual(FakePopen.launched[0][-1], ' -5 derece')          # not an option to the synth

    def test_null_tts_takes_about_as_long_as_speaking(self):
        slept = []
        NullTts(chars_per_s=15.0, sleep=slept.append).say('a' * 30)
        self.assertEqual(slept, [2.0])


if __name__ == '__main__':
    unittest.main()
