"""Speech-to-text: /luna/audio/utterance (WAV path) → transcript → /luna/assistant/request. On the robot only:
the recording never leaves it, and the WAV is deleted once read (delete_wav). Transcripts are not logged, only
their length and the outcome (/luna/stt/status).

stt_engine: whisper_server (whisper.cpp's whisper-server at stt_url; the model stays loaded)
            command        (stt_command, split into argv without a shell; {wav} and {lang} filled in)
            none
A question that cannot be understood gets a short spoken answer instead of silence (reply_on_failure): with
push-to-talk someone touched the face and is waiting."""
import os
import queue
import threading

from rclpy.node import Node
from std_msgs.msg import String

from . import topics as T
from .entry import run
from .core.stt import make_stt

NOT_UNDERSTOOD_TR = 'Seni tam anlayamadım. Bir daha söyler misin?'
NO_EARS_TR = 'Şu an konuşmanı anlayamıyorum. Biraz sonra tekrar dener misin?'


class SttNode(Node):
    def __init__(self):
        super().__init__('luna_stt')
        for name, default in [('stt_engine', 'none'), ('stt_url', 'http://127.0.0.1:8178/inference'),
                              ('stt_command', 'whisper-cli -m /models/whisper/ggml-large-v3-turbo-q5_0.bin -l {lang} '
                                              '-nt -np -f {wav}'),
                              ('language', 'tr'), ('timeout_s', 20.0), ('prompt', ''), ('min_chars', 2), ('audio_ctx', 0),
                              ('delete_wav', True), ('reply_on_failure', True)]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.engine = p('stt_engine')
        self.stt = make_stt(self.engine, url=p('stt_url'), command=p('stt_command'), language=p('language'),
                            timeout_s=float(p('timeout_s')), prompt=p('prompt'), min_chars=int(p('min_chars')),
                            audio_ctx=int(p('audio_ctx')))
        self.delete_wav, self.reply_on_failure = bool(p('delete_wav')), bool(p('reply_on_failure'))
        self.pub_request = self.create_publisher(String, T.ASSISTANT_REQUEST, 10)
        self.pub_reply = self.create_publisher(String, T.ASSISTANT_REPLY, 10)
        self.pub_status = self.create_publisher(String, T.STT_STATUS, 10)
        self.pending = queue.Queue(maxsize=1)
        self.last_error = None
        self.create_subscription(String, T.AUDIO_UTTERANCE, self._utterance, 10)
        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(f'STT: {type(self.stt).__name__}' + (f' {p("stt_url")}' if self.engine == 'whisper_server' else ''))
        self.pub_status.publish(String(data='idle'))

    def _utterance(self, msg):
        try:                                    # one at a time; a newer utterance replaces a waiting one
            self._discard(self.pending.get_nowait())
        except queue.Empty:
            pass
        self.pending.put_nowait(msg.data)

    def _discard(self, path):
        if self.delete_wav:
            try:
                os.remove(path)
            except OSError:
                pass

    def _worker(self):
        while True:
            path = self.pending.get()
            self.pub_status.publish(String(data='transcribing'))
            result = self.stt.transcribe(path)
            self._discard(path)
            if result.ok:
                self.last_error = None
                self.get_logger().info(f'ok: {len(result.text)} chars in {result.seconds:.2f} s')
                self.pub_status.publish(String(data='ok'))
                self.pub_request.publish(String(data=result.text))
                continue
            status = 'empty' if result.reason == 'empty' else f'error:{result.reason}'
            line = f'{status}' + (f' ({result.detail})' if result.detail else '')
            if result.reason == 'empty':
                self.get_logger().info(f'{line} after {result.seconds:.2f} s')
            elif line != self.last_error:
                self.get_logger().warning(line)
                self.last_error = line
            self.pub_status.publish(String(data=status))
            if self.reply_on_failure:
                self.pub_reply.publish(String(data=NOT_UNDERSTOOD_TR if result.reason == 'empty' else NO_EARS_TR))


def main():
    run(SttNode)


if __name__ == '__main__':
    main()
