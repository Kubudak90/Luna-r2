"""Provider-agnostic cloud assistant client for heavy questions (OpenAI-compatible chat completions).

Default provider: Kimi (Moonshot AI). Official docs read 2026-10-17 (platform.moonshot.ai → platform.kimi.ai):
POST https://api.moonshot.ai/v1/chat/completions, `Authorization: Bearer <key>`, reply in
choices[0].message.content. Models listed then: kimi-k3, kimi-k2.7-code(-highspeed), kimi-k2.6. There is no
"Kimi 2.8" in the official list, so the model is a parameter. kimi-k2.6 can switch thinking off
({"thinking": {"type": "disabled"}}), which keeps a spoken reply quick; its temperature/top_p are fixed by the API
(other values are errors), so none are sent. Errors (docs "Common Error Codes"): 429 engine_overloaded /
rate_limit_reached and 500/503 are worth retrying, 429 exceeded_current_quota, 400, 401, 403, 404 are not.

Safety rules kept here:
- No request leaves the robot unless `enabled` (the owner's consent) is set AND a key is present.
- The key comes only from an environment variable or a file outside any git checkout; it is never logged.
- Every failure ends in a spoken offline reply instead of silence; a breaker stops hammering a dead link."""
import json
import os
import socket
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field

SYSTEM_PROMPT_TR = (
    "Sen Luna'sın: bir evde yaşayan, meraklı ve sevecen küçük bir arkadaş robot. Türkçe konuş. Cevapların sesli "
    "okunacak: en fazla üç kısa cümle kur; markdown, liste, emoji ya da bağlantı kullanma. Bilmediğin şeyi uydurma, "
    "emin değilsen söyle. Çocuklarla da konuşabilirsin; yaşlarına uygun ve nazik ol.")

FALLBACK_TR = {
    'disabled': 'Şu an bulut beynime bağlı değilim. Basit şeylerde yine buradayım.',
    'no_key': 'Şu an bulut beynime bağlı değilim. Basit şeylerde yine buradayım.',
    'network': 'İnternete ulaşamıyorum. Birazdan tekrar sorar mısın?',
    'breaker_open': 'İnternete ulaşamıyorum. Birazdan tekrar sorar mısın?',
    'rate_limit': 'Biraz yoğunum. Birkaç saniye sonra tekrar sorar mısın?',
    'server': 'Biraz yoğunum. Birkaç saniye sonra tekrar sorar mısın?',
    'content_filter': 'Bu konuda yardımcı olamıyorum.',
}
FALLBACK_OTHER_TR = 'Bulut tarafında bir sorun var, şimdilik cevap veremiyorum.'
BREAKER_REASONS = {'network', 'server', 'rate_limit', 'auth', 'quota'}


@dataclass(frozen=True)
class ProviderConfig:
    name: str = 'kimi'
    base_url: str = 'https://api.moonshot.ai/v1'
    model: str = 'kimi-k2.6'
    api_key_env: str = 'MOONSHOT_API_KEY'
    api_key_file: str = '~/.config/luna/moonshot_api_key'
    extra_body: dict = field(default_factory=lambda: {'thinking': {'type': 'disabled'}})
    max_completion_tokens: int = 300


@dataclass(frozen=True)
class ClientPolicy:
    timeout_s: float = 15.0             # one HTTP attempt
    deadline_s: float = 30.0            # everything, retries included: someone is waiting for Luna's answer
    retries: int = 2
    backoff_s: float = 1.0
    backoff_max_s: float = 8.0
    breaker_failures: int = 3           # failed questions in a row before the network is skipped...
    breaker_cooldown_s: float = 60.0    # ...for this long
    history_turns: int = 6              # question/answer pairs kept as context
    max_input_chars: int = 2000


@dataclass(frozen=True)
class HttpResult:
    status: int
    headers: dict
    body: bytes


class TransportError(Exception):
    """No HTTP answer at all: DNS, connection, TLS or timeout."""


@dataclass(frozen=True)
class Reply:
    text: str
    source: str                         # 'cloud' | 'offline'
    reason: str = None                  # why offline: disabled, no_key, network, rate_limit, auth, ...
    detail: str = None                  # for the log; never contains the key
    usage: dict = None


def urllib_transport(url, headers, body, timeout):
    req = urllib.request.Request(url, data=body, headers=headers, method='POST')
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return HttpResult(resp.status, dict(resp.headers), resp.read())
    except urllib.error.HTTPError as e:
        return HttpResult(e.code, dict(e.headers or {}), e.read() or b'')
    except (urllib.error.URLError, socket.timeout, OSError) as e:
        raise TransportError(str(getattr(e, 'reason', e))) from e


def _inside_git_checkout(path):
    d = os.path.dirname(os.path.realpath(path))
    while True:
        if os.path.exists(os.path.join(d, '.git')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return None
        d = parent


def load_api_key(env_var, key_file, environ=os.environ):
    """(key or None, notes for the log). The environment wins; a key file inside a git checkout is refused so a
    key can never be committed by accident; a file other users can read is used with a warning."""
    key = environ.get(env_var, '').strip() if env_var else ''
    if key:
        return key, []
    if not key_file:
        return None, [f'no API key: set {env_var}']
    path = os.path.expanduser(key_file)
    if not os.path.isfile(path):
        return None, [f'no API key: set {env_var} or create {path} (chmod 600)']
    repo = _inside_git_checkout(path)
    if repo:
        return None, [f'refusing key file {path}: it is inside a git checkout ({repo}); keep keys outside the repo']
    with open(path) as f:
        key = f.read().strip()
    if not key:
        return None, [f'key file {path} is empty']
    notes = []
    if os.stat(path).st_mode & 0o077:
        notes.append(f'key file {path} is readable by other users: chmod 600 {path}')
    return key, notes


def _header(headers, name):
    for k, v in (headers or {}).items():
        if k.lower() == name.lower():
            return v
    return None


def _classify(res):
    """→ (reason, retryable, retry_after_s, error text)."""
    try:
        err = json.loads(res.body or b'{}').get('error') or {}
    except (ValueError, AttributeError):
        err = {}
    kind, message = str(err.get('type', '')), str(err.get('message', ''))
    retry_after = None
    try:
        retry_after = float(_header(res.headers, 'Retry-After'))
    except (TypeError, ValueError):
        pass
    s = res.status
    if s == 429:
        reason, retry = ('quota', False) if kind == 'exceeded_current_quota_error' else ('rate_limit', True)
    elif s in (500, 502, 503, 504):
        reason, retry = 'server', True
    elif s in (401, 403):
        reason, retry = 'auth', False
    elif s == 404:
        reason, retry = 'model', False
    elif s == 400:
        reason, retry = ('content_filter' if kind == 'content_filter' else 'bad_request'), False
    else:
        reason, retry = f'http_{s}', False
    return reason, retry, retry_after, f'HTTP {s} {kind}: {message}'.strip()


class AssistantClient:
    def __init__(self, provider=ProviderConfig(), policy=ClientPolicy(), *, api_key=None, transport=urllib_transport,
                 enabled=False, clock=time.monotonic, sleep=time.sleep, system_prompt=SYSTEM_PROMPT_TR):
        self.provider, self.policy, self.enabled = provider, policy, enabled
        self._key, self._transport, self._clock, self._sleep = api_key, transport, clock, sleep
        self.system_prompt = system_prompt
        self.history = []                      # [user, assistant, user, assistant, ...] message dicts
        self._failures, self._open_until = 0, None

    def __repr__(self):
        return (f'AssistantClient(provider={self.provider.name!r}, model={self.provider.model!r}, '
                f'enabled={self.enabled}, key={"set" if self._key else "missing"})')

    def _redact(self, text):
        return text.replace(self._key, '***') if self._key and text else text

    def _offline(self, reason, detail=None):
        return Reply(FALLBACK_TR.get(reason, FALLBACK_OTHER_TR), 'offline', reason, self._redact(detail))

    def forget_last(self):
        """The last cloud answer was never heard (dropped for a newer question): keep that pair out of the context."""
        del self.history[-2:]

    def ask(self, text):
        """Blocking; call from a worker thread. Returns None for empty input, else a Reply (never raises for
        network or API trouble)."""
        text = (text or '').strip()[:self.policy.max_input_chars]
        if not text:
            return None
        if not self.enabled:
            return self._offline('disabled')
        if not self._key:
            return self._offline('no_key')
        pol, now = self.policy, self._clock()
        if self._open_until is not None and now < self._open_until:
            return self._offline('breaker_open')
        self._open_until = None
        user = {'role': 'user', 'content': text}
        body = {'model': self.provider.model,
                'messages': [{'role': 'system', 'content': self.system_prompt}] + self.history + [user],
                'max_completion_tokens': self.provider.max_completion_tokens, **(self.provider.extra_body or {})}
        data = json.dumps(body, ensure_ascii=False).encode()
        url = self.provider.base_url.rstrip('/') + '/chat/completions'
        headers = {'Authorization': f'Bearer {self._key}', 'Content-Type': 'application/json'}
        start, attempt = now, 0
        while True:
            remaining = pol.deadline_s - (self._clock() - start)
            try:
                res = self._transport(url, headers, data, max(1.0, min(pol.timeout_s, remaining)))
            except TransportError as e:
                reason, retry, retry_after, detail = 'network', True, None, str(e)
            else:
                if res.status == 200:
                    reply = self._parse(res, user)
                    if reply:
                        return reply
                    reason, retry, retry_after, detail = 'bad_response', False, None, 'HTTP 200 without a usable reply'
                else:
                    reason, retry, retry_after, detail = _classify(res)
            if not retry or attempt >= pol.retries:
                break
            wait = retry_after if retry_after is not None else min(pol.backoff_s * 2 ** attempt, pol.backoff_max_s)
            if self._clock() - start + wait >= pol.deadline_s:
                break
            self._sleep(wait)
            attempt += 1
        if reason in BREAKER_REASONS:
            self._failures += 1
            if self._failures >= pol.breaker_failures:
                self._failures, self._open_until = 0, self._clock() + pol.breaker_cooldown_s
        return self._offline(reason, detail)

    def _parse(self, res, user):
        try:
            payload = json.loads(res.body)
            content = payload['choices'][0]['message']['content']
        except (ValueError, KeyError, IndexError, TypeError):
            return None
        if not isinstance(content, str) or not content.strip():
            return None
        content = content.strip()
        # reasoning_content (thinking models) is neither spoken nor kept: kimi-k2.6 drops old reasoning by default.
        self.history += [user, {'role': 'assistant', 'content': content}]
        self.history = self.history[-2 * self.policy.history_turns:] if self.policy.history_turns > 0 else []
        self._failures = 0
        return Reply(content, 'cloud', usage=payload.get('usage'))
