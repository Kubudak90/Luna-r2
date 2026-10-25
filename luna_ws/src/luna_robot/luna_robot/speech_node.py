"""Speaker output: /luna/assistant/reply → text-to-speech → the XVF3800's speaker output; /luna/voice/speaking
tells attention (own voice is not a speaker), the microphones (no question while Luna talks) and the face.
A touch on the face (/luna/voice/talk) cuts Luna off: the queue is dropped and the sentence stops.

tts_engine: none       (no sound, but speaking lasts about as long as the sentence would)
            piper_http (Piper's HTTP server at tts_url, Turkish voice tr_TR-dfki-medium; played with tts_player)
            command    (tts_command / tts_player, split into argv without a shell; {text} and {device} filled in)"""
import threading

from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from . import topics as T
from .audio_node import alsa_device
from .entry import run
from .core.speech import SpeechQueue, make_tts, speakable


class SpeechNode(Node):
    def __init__(self):
        super().__init__('luna_speech')
        for name, default in [('tts_engine', 'none'), ('tts_command', 'espeak-ng -v tr --stdout {text}'),
                              ('tts_player', 'aplay -q -D {device}'), ('tts_url', 'http://127.0.0.1:5000/synthesize'),
                              ('tts_length_scale', 1.0), ('alsa_device', 'auto'), ('max_chars', 400)]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        device = alsa_device(p('alsa_device'))
        self.tts = make_tts(p('tts_engine'), p('tts_command'), p('tts_player'), device, p('tts_url'),
                            float(p('tts_length_scale')))
        self.max_chars = int(p('max_chars'))
        self.queue = SpeechQueue()
        self.speaking, self.last_error = False, None
        self.pub_speaking = self.create_publisher(Bool, T.VOICE_SPEAKING, 10)
        self.create_subscription(String, T.ASSISTANT_REPLY, self._reply, 10)
        self.create_subscription(Empty, T.VOICE_TALK, self._talk, 10)
        self.create_timer(0.2, lambda: self.pub_speaking.publish(Bool(data=self.speaking)))
        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(f'TTS: {type(self.tts).__name__} on {device}')

    def _reply(self, msg):
        text = speakable(msg.data, self.max_chars)
        if text:
            self.queue.push(text)

    def _talk(self, _msg):
        if self.speaking:
            self.get_logger().info('touched while speaking: stopping')
        self.queue.clear()
        self.tts.stop()

    def _set_speaking(self, value):
        if value != self.speaking:
            self.speaking = value
            self.pub_speaking.publish(Bool(data=value))

    def _worker(self):
        while True:
            sentence = self.queue.pop(timeout=0.2)
            if sentence is None:
                self._set_speaking(False)          # only once the queue is empty: no flicker between sentences
                continue
            self._set_speaking(True)
            try:
                self.tts.say(sentence)
                self.last_error = None
            except Exception as e:                  # a missing synth binary or server must not kill the node
                if str(e) != self.last_error:
                    self.get_logger().error(f'TTS failed: {e}')
                    self.last_error = str(e)


def main():
    run(SpeechNode)


if __name__ == '__main__':
    main()
