"""Which model answers: the quick one first, the thinking one for hard questions.

- fast: K2.8 Preview without thinking (the Kimi Code membership's `kimi-for-coding`, thinking disabled). Most of a
  child's talk ends here, and the first word comes soonest.
- think: the same model with thinking on at low effort (reasoning_effort "low"); slower, more careful.
A question goes to think straight away when free rules say it is hard (sums, homework, "how does it work",
long questions); otherwise the fast model answers, and its system prompt tells it to reply only [DÜŞÜN] when the
question needs careful thought. Either way Luna first says a short "let me think" so the wait is not silent, but
only if the thinking model can be asked at all (consent, key, breaker). Both clients keep one shared history.
Standard library only."""
import re
from dataclasses import dataclass

MARKER = '[DÜŞÜN]'
ROUTE_PROMPT_TR = (
    ' Soru dikkatli düşünmeyi gerektiriyorsa (bir hesap, ödev, birkaç adımlı bir açıklama ya da emin olmadığın bir '
    f'bilgi) cevap verme, yalnız {MARKER} yaz.')
FILLER_TR = 'Hmm, bir düşüneyim.'

_ESCALATE = re.compile(r'\[\s*d[üu][şs][üu]n\s*\]|^\W*d[üu][şs][üu]n\W*$', re.IGNORECASE)
# A sum between two numbers; "-" and ":" only with spaces, so "2-3 gün" and "saat 7:30" stay small talk.
_NUMBER_OP = re.compile(r'\d+([.,]\d+)?\s*([+*/×÷x]|\s-\s|\s:\s|artı|eksi|çarpı|bölü|kere)\s*\d+')
_OP_WORD = re.compile(r'\b(artı|çarpı|bölü)\b')        # "üç artı beş" said in words
RULES = (
    ('hesap', ('kaç eder', 'kaç yapar', 'hesapla', 'karekök', 'yüzde kaç', 'işlemi')),
    ('ödev', ('ödev', 'denklem', 'kesir', 'problemi çöz', 'soruyu çöz', 'problem çöz')),
    ('açıklama', ('nasıl çalışır', 'açıklar mısın', 'açıkla', 'farkı ne', 'arasındaki fark', 'adım adım')),
)


def _lower_tr(text):
    return text.replace('İ', 'i').replace('I', 'ı').lower()


@dataclass(frozen=True)
class RouterConfig:
    long_words: int = 25            # a question this long is not small talk
    filler: str = FILLER_TR


def needs_thinking(text, cfg=RouterConfig()):
    """The free pre-filter: why this question goes to the thinking model ('hesap', 'ödev', 'açıklama', 'uzun'),
    or None."""
    t = _lower_tr(text or '')
    if _NUMBER_OP.search(t) or _OP_WORD.search(t):
        return 'hesap'
    for why, phrases in RULES:
        if any(p in t for p in phrases):
            return why
    return 'uzun' if len(t.split()) > cfg.long_words else None


def is_escalation(reply_text):
    return bool(_ESCALATE.search(reply_text or ''))


@dataclass(frozen=True)
class Routed:
    reply: object                   # core.assistant.Reply, or None for an empty question
    route: str                      # 'fast' | 'think'
    why: str = None                 # think: a pre-filter rule, or 'model' (the fast model asked for it)


class Router:
    """fast and think are AssistantClients; say(text) speaks a line at once (the filler)."""

    def __init__(self, fast, think, cfg=RouterConfig(), say=lambda text: None):
        self.fast, self.think, self.cfg, self.say = fast, think, cfg, say

    def _sync(self, source):
        other = self.think if source is self.fast else self.fast
        other.history = list(source.history)

    def _ask_think(self, text, why):
        if self.think.can_try():
            self.say(self.cfg.filler)
        reply = self.think.ask(text)
        if reply is not None and reply.source == 'cloud':
            self._sync(self.think)
        return Routed(reply, 'think', why)

    def ask(self, text):
        why = needs_thinking(text, self.cfg)
        if why:
            return self._ask_think(text, why)
        reply = self.fast.ask(text)
        if reply is None or reply.source != 'cloud':
            return Routed(reply, 'fast')
        if is_escalation(reply.text):
            self.fast.forget_last()                   # the marker is no answer: keep it out of the shared context
            return self._ask_think(text, 'model')
        self._sync(self.fast)
        return Routed(reply, 'fast')

    def forget_last(self, routed):
        """The answer was dropped unheard: keep that pair out of both histories."""
        if routed.reply is not None and routed.reply.source == 'cloud':
            source = self.think if routed.route == 'think' else self.fast
            source.forget_last()
            self._sync(source)
