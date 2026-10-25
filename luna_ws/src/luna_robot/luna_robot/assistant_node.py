"""Cloud assistant client: /luna/assistant/request (text) → /luna/assistant/reply (text to speak).

Provider-agnostic (OpenAI-compatible chat completions; default the owner's Kimi Code membership, K2.8 Preview). Nothing is sent anywhere unless
cloud_enabled is true (the owner's consent) and a key is found in the environment (api_key_env) or in
api_key_file outside any git checkout. Otherwise, and on every network or API failure, a short offline reply is
spoken instead. Questions and answers are not logged, only their length, the route and the outcome.
Router (core/router.py, router_enabled): the quick model (thinking off) answers first; hard questions go to the
same provider with thinking on (think_*), after a spoken "let me think".
A touch on the face (/luna/voice/talk) means a new question is coming: a waiting question is dropped, and so is
the answer to the one in flight when it arrives."""
import json
import queue
import threading

from rclpy.node import Node
from std_msgs.msg import Empty, String

from . import topics as T
from .entry import run
from .core.assistant import SYSTEM_PROMPT_TR, AssistantClient, ClientPolicy, ProviderConfig, load_api_key
from .core.router import FILLER_TR, ROUTE_PROMPT_TR, Routed, Router, RouterConfig


class AssistantNode(Node):
    def __init__(self):
        super().__init__('luna_assistant')
        prov, pol, rcfg = ProviderConfig(), ClientPolicy(), RouterConfig()
        params = [('cloud_enabled', False), ('provider', prov.name), ('base_url', prov.base_url), ('model', prov.model),
                  ('api_key_env', prov.api_key_env), ('api_key_file', prov.api_key_file),
                  ('extra_body_json', json.dumps(prov.extra_body)), ('max_completion_tokens', prov.max_completion_tokens),
                  ('timeout_s', pol.timeout_s), ('deadline_s', pol.deadline_s), ('retries', pol.retries),
                  ('history_turns', pol.history_turns), ('system_prompt', ''), ('speak_offline_replies', True),
                  ('router_enabled', True), ('router_long_words', rcfg.long_words), ('think_filler', FILLER_TR),
                  ('think_model', ''), ('think_extra_body_json', '{"reasoning_effort": "low"}'),
                  ('think_max_completion_tokens', 4000), ('think_timeout_s', 30.0), ('think_deadline_s', 45.0)]
        for name, default in params:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        provider = ProviderConfig(p('provider'), p('base_url'), p('model'), p('api_key_env'), p('api_key_file'),
                                  self._json('extra_body_json'), int(p('max_completion_tokens')))
        policy = ClientPolicy(timeout_s=float(p('timeout_s')), deadline_s=float(p('deadline_s')), retries=int(p('retries')),
                              history_turns=int(p('history_turns')))
        enabled = bool(p('cloud_enabled'))
        key, notes = load_api_key(provider.api_key_env, provider.api_key_file) if enabled else (None, [])
        for note in notes:
            self.get_logger().warning(note)
        prompt = p('system_prompt') or SYSTEM_PROMPT_TR
        self.router = None
        if p('router_enabled'):
            think = ProviderConfig(provider.name, provider.base_url, p('think_model') or provider.model,
                                   provider.api_key_env, provider.api_key_file, self._json('think_extra_body_json'),
                                   int(p('think_max_completion_tokens')))
            think_policy = ClientPolicy(timeout_s=float(p('think_timeout_s')), deadline_s=float(p('think_deadline_s')),
                                        retries=policy.retries, history_turns=policy.history_turns)
            self.client = AssistantClient(provider, policy, api_key=key, enabled=enabled,
                                          system_prompt=prompt + ROUTE_PROMPT_TR)
            self.router = Router(self.client, AssistantClient(think, think_policy, api_key=key, enabled=enabled,
                                                              system_prompt=prompt),
                                 RouterConfig(int(p('router_long_words')), p('think_filler')), say=self._say)
        else:
            self.client = AssistantClient(provider, policy, api_key=key, enabled=enabled, system_prompt=prompt)
        self.speak_offline = bool(p('speak_offline_replies'))
        self.pub_reply = self.create_publisher(String, T.ASSISTANT_REPLY, 10)
        self.pub_status = self.create_publisher(String, T.ASSISTANT_STATUS, 10)
        self.pending = queue.Queue(maxsize=1)
        self.talks = 0                         # touches so far; an answer is spoken only if none came while it was asked
        self.create_subscription(String, T.ASSISTANT_REQUEST, self._request, 10)
        self.create_subscription(Empty, T.VOICE_TALK, self._talk, 10)
        threading.Thread(target=self._worker, daemon=True).start()
        state = 'cloud ON' if enabled and key else ('cloud on, but no key' if enabled else 'cloud OFF (no consent)')
        route = f', router → {self.router.think.provider.model} for hard questions' if self.router else ''
        self.get_logger().info(f'assistant {provider.name}/{provider.model}: {state}{route}')
        self.pub_status.publish(String(data='idle'))

    def _json(self, name):
        try:
            return json.loads(self.get_parameter(name).value or '{}')
        except ValueError:
            self.get_logger().error(f'{name} is not JSON; sending no extra fields')
            return {}

    def _say(self, text):
        self.pub_reply.publish(String(data=text))

    def _request(self, msg):
        try:                                   # one question at a time; a newer question replaces a waiting one
            self.pending.get_nowait()
        except queue.Empty:
            pass
        self.pending.put_nowait(msg.data)

    def _talk(self, _msg):
        self.talks += 1
        try:
            self.pending.get_nowait()
        except queue.Empty:
            pass

    def _worker(self):
        while True:
            text = self.pending.get()
            talks = self.talks
            self.pub_status.publish(String(data='thinking'))
            routed = self.router.ask(text) if self.router else Routed(self.client.ask(text), 'fast')
            reply = routed.reply
            if reply is None:
                self.pub_status.publish(String(data='idle'))
                continue
            if self.talks != talks:
                if self.router:
                    self.router.forget_last(routed)
                elif reply.source == 'cloud':
                    self.client.forget_last()
                self.get_logger().info('answer dropped: the face was touched for a new question meanwhile')
                self.pub_status.publish(String(data='idle'))
                continue
            status = 'ok' if reply.source == 'cloud' else f'offline:{reply.reason}'
            route = routed.route + (f' ({routed.why})' if routed.why else '')
            self.get_logger().info(f'{route} {status}: asked {len(text)} chars, reply {len(reply.text)} chars'
                                   + (f' ({reply.detail})' if reply.detail else ''))
            if reply.source == 'cloud' or self.speak_offline:
                self.pub_reply.publish(String(data=reply.text))
            self.pub_status.publish(String(data=status))


def main():
    run(AssistantNode)


if __name__ == '__main__':
    main()
