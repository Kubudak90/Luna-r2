"""Stands in for aplay: reads the WAV on stdin and 'plays' it in real time; logs start, end or kill to
$VOICE_CHECK_OUT/aplay.log."""
import io, os, signal, sys, time, wave
log = open(os.path.join(os.environ.get('VOICE_CHECK_OUT', '/tmp/voice_check'), 'aplay.log'), 'a')
def killed(*a):
    log.write(f'{time.monotonic():.2f} killed\n'); log.flush(); sys.exit(0)
signal.signal(signal.SIGTERM, killed)
data = sys.stdin.buffer.read()
with wave.open(io.BytesIO(data)) as w:
    secs = w.getnframes() / w.getframerate()
log.write(f'{time.monotonic():.2f} play {secs:.2f}s device={sys.argv[1:]}\n'); log.flush()
time.sleep(secs)
log.write(f'{time.monotonic():.2f} done\n'); log.flush()
