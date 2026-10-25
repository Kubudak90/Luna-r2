"""Router without network: the free pre-filter, the [DÜŞÜN] escalation, the spoken filler, one shared history.
Run from luna_ws/src/luna_robot:  python3 -m unittest discover -s test"""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot.core.assistant import AssistantClient, ClientPolicy, HttpResult, ProviderConfig, TransportError  # noqa: E402
from luna_robot.core.router import (FILLER_TR, MARKER, ROUTE_PROMPT_TR, Router, RouterConfig,  # noqa: E402
                                    is_escalation, needs_thinking)

KEY = 'sk-test-router'
FAST = ProviderConfig(extra_body={'thinking': {'type': 'disabled'}}, max_completion_tokens=300)
THINK = ProviderConfig(extra_body={'reasoning_effort': 'low'}, max_completion_tokens=4000)


def ok(text):
    return HttpResult(200, {}, json.dumps({'choices': [{'message': {'role': 'assistant', 'content': text}}]}).encode())


class Transport:
    def __init__(self, *answers):
        self.answers, self.calls = list(answers), []

    def __call__(self, url, headers, body, timeout):
        self.calls.append({'body': json.loads(body), 'timeout': timeout})
        a = self.answers.pop(0)
        if isinstance(a, Exception):
            raise a
        return a


def router(fast_answers=(), think_answers=(), key=KEY, enabled=True):
    said, ft, tt = [], Transport(*fast_answers), Transport(*think_answers)
    fast = AssistantClient(FAST, ClientPolicy(retries=0), api_key=key, transport=ft, enabled=enabled,
                           system_prompt='Sen Luna\'sın.' + ROUTE_PROMPT_TR, sleep=lambda s: None)
    think = AssistantClient(THINK, ClientPolicy(retries=0, timeout_s=30.0), api_key=key, transport=tt, enabled=enabled,
                            system_prompt='Sen Luna\'sın.', sleep=lambda s: None)
    return Router(fast, think, RouterConfig(), say=said.append), ft, tt, said


class PreFilter(unittest.TestCase):
    def test_hard_questions_go_straight_to_thinking(self):
        for text, why in [('12 çarpı 7 kaç eder?', 'hesap'), ('5+3 nedir', 'hesap'), ('48 / 6', 'hesap'),
                          ('Üç artı beş ne yapar?', 'hesap'), ('Bunu HESAPLA: yedi kere sekiz', 'hesap'),
                          ('15 - 9 kaç?', 'hesap'), ('Ödevime yardım eder misin?', 'ödev'),
                          ('Bu denklemi çözer misin', 'ödev'), ('Buzdolabı nasıl çalışır?', 'açıklama'),
                          ('Kurbağa ile kara kurbağası arasındaki fark ne?', 'açıklama'),
                          ('İYİ ama bunu bana ADIM ADIM anlat', 'açıklama'),
                          (' '.join(['kelime'] * 26), 'uzun')]:
            self.assertEqual(needs_thinking(text), why, text)

    def test_small_talk_stays_quick(self):
        for text in ('Merhaba Luna', 'Kediler neden mırlar?', 'Bana bir masal anlatır mısın?', 'Saat 7:30 oldu mu?',
                     '2-3 gün sonra doğum günüm', 'Bugün hava nasıl?', 'Eksik parçayı buldum', 'Işıkları kapat',
                     ' '.join(['kelime'] * 25), '', None):
            self.assertIsNone(needs_thinking(text), text)

    def test_escalation_marker_variants(self):
        for reply in (MARKER, '[düşün]', '[DUSUN]', ' [ Düşün ] ', 'DÜŞÜN', 'Düşün.', f'{MARKER} Bir saniye.'):
            self.assertTrue(is_escalation(reply), reply)
        for reply in ('Biraz düşün, sonra söyle.', 'Düşünmek güzeldir.', 'Merhaba!', '', None):
            self.assertFalse(is_escalation(reply), reply)


class Routing(unittest.TestCase):
    def test_small_talk_is_one_quick_call_without_thinking(self):
        r, ft, tt, said = router([ok('Merhaba! Nasılsın?')])
        routed = r.ask('Merhaba Luna')
        self.assertEqual((routed.route, routed.why, routed.reply.text), ('fast', None, 'Merhaba! Nasılsın?'))
        self.assertEqual(len(ft.calls), 1)
        self.assertEqual(tt.calls, [])
        self.assertEqual(said, [])
        body = ft.calls[0]['body']
        self.assertEqual(body['thinking'], {'type': 'disabled'})
        self.assertNotIn('reasoning_effort', body)
        self.assertIn(MARKER, body['messages'][0]['content'])         # the quick model knows how to ask for help

    def test_rule_sends_a_sum_to_thinking_after_the_filler(self):
        r, ft, tt, said = router(think_answers=[ok('84 eder.')])
        routed = r.ask('12 çarpı 7 kaç eder?')
        self.assertEqual((routed.route, routed.why, routed.reply.text), ('think', 'hesap', '84 eder.'))
        self.assertEqual(ft.calls, [])
        self.assertEqual(said, [FILLER_TR])
        body = tt.calls[0]['body']
        self.assertEqual(body['reasoning_effort'], 'low')
        self.assertNotIn('thinking', body)                              # thinking stays on
        self.assertEqual(body['max_completion_tokens'], 4000)            # reasoning counts against it
        self.assertNotIn(MARKER, body['messages'][0]['content'])
        self.assertAlmostEqual(tt.calls[0]['timeout'], 30.0, places=2)

    def test_quick_model_asks_for_thinking(self):
        r, ft, tt, said = router([ok(MARKER)], [ok('Gökkuşağı, güneş ışığı yağmur damlasında kırılınca oluşur.')])
        routed = r.ask('Gökkuşağı neden oluşur?')
        self.assertEqual((routed.route, routed.why), ('think', 'model'))
        self.assertEqual(said, [FILLER_TR])
        self.assertEqual(len(ft.calls), 1)
        self.assertEqual(len(tt.calls), 1)
        # one shared history, and the marker is in neither
        expect = ['Gökkuşağı neden oluşur?', 'Gökkuşağı, güneş ışığı yağmur damlasında kırılınca oluşur.']
        self.assertEqual([m['content'] for m in r.fast.history], expect)
        self.assertEqual([m['content'] for m in r.think.history], expect)

    def test_the_next_quick_question_sees_the_thinking_answer(self):
        r, ft, tt, said = router([ok('Rica ederim!')], [ok('84 eder.')])
        r.ask('12 çarpı 7 kaç eder?')
        r.ask('Teşekkürler')
        contents = [m['content'] for m in ft.calls[0]['body']['messages'][1:]]
        self.assertEqual(contents, ['12 çarpı 7 kaç eder?', '84 eder.', 'Teşekkürler'])
        self.assertEqual(len(r.think.history), 4)

    def test_no_filler_when_thinking_cannot_be_asked(self):
        for kw in ({'key': None}, {'enabled': False}):
            r, ft, tt, said = router(**kw)
            routed = r.ask('12 çarpı 7 kaç eder?')
            self.assertEqual(said, [], kw)                               # no "let me think" before "I'm offline"
            self.assertEqual(routed.reply.source, 'offline')
            self.assertEqual(tt.calls, [])

    def test_offline_quick_answer_is_not_escalated(self):
        r, ft, tt, said = router([TransportError('down')])
        routed = r.ask('Merhaba')
        self.assertEqual((routed.route, routed.reply.source, routed.reply.reason), ('fast', 'offline', 'network'))
        self.assertEqual((tt.calls, said), ([], []))

    def test_marker_never_stays_in_the_context_even_if_thinking_fails(self):
        r, ft, tt, said = router([ok(MARKER)], [TransportError('down')])
        routed = r.ask('Gökkuşağı neden oluşur?')
        self.assertEqual((routed.route, routed.reply.source), ('think', 'offline'))
        self.assertEqual(r.fast.history, [])
        self.assertEqual(r.think.history, [])

    def test_dropped_answer_leaves_both_histories(self):
        r, ft, tt, said = router([ok('Merhaba!'), ok(MARKER)], [ok('Uzun cevap.')])
        r.ask('Merhaba')
        routed = r.ask('Gökkuşağı neden oluşur?')
        r.forget_last(routed)                                           # a touch cut it off
        self.assertEqual([m['content'] for m in r.fast.history], ['Merhaba', 'Merhaba!'])
        self.assertEqual(r.think.history, r.fast.history)


if __name__ == '__main__':
    unittest.main()
