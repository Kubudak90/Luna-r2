#!/bin/bash
# Live voice chain in the ROS container (README "Ses zinciri"): mock MCU + mock DoA voices, fake whisper-server /
# Piper / aplay over real HTTP. Patches the INSTALLED luna.yaml (start_soc 0.60, fake player); colcon build restores it.
# Usage: bash /ws/src/luna_robot/tools/voice_check/live.sh [short]   → logs in $VOICE_CHECK_OUT (/tmp/voice_check)
HERE=$(cd "$(dirname "$0")" && pwd)
export VOICE_CHECK_OUT=${VOICE_CHECK_OUT:-/tmp/voice_check}
OUT=$VOICE_CHECK_OUT
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
CFG=/ws/install/luna_robot/share/luna_robot/config/luna.yaml
sed -i 's/^    start_soc: 0.24 /    start_soc: 0.60 /' "$CFG"
sed -i "s#^    tts_player: aplay -q -D {device} #    tts_player: python3 $HERE/fake_aplay.py {device} #" "$CFG"
grep -n "start_soc\|tts_player\|stt_engine\|tts_engine\|capture:\|listen_mode" "$CFG"
rm -rf $OUT && mkdir -p $OUT /tmp/luna_utterances
phase() { python3 -c "import time; time.sleep(($1 - time.monotonic() % 20) % 20)"; }
post() {  # post ORIGIN → HTTP status
  python3 - "$1" <<'PY'
import sys, urllib.request, urllib.error
req = urllib.request.Request('http://127.0.0.1:8765/talk', data=b'', method='POST', headers={'Origin': sys.argv[1]} if sys.argv[1] else {})
try:
    print(urllib.request.urlopen(req, timeout=3).status)
except urllib.error.HTTPError as e:
    print(e.code)
PY
}
note() { echo "$(python3 -c 'import time; t=time.monotonic(); print(f"{t:9.2f} φ{t % 20:5.2f}")') ### $*" | tee -a $OUT/monitor.log; }
python3 $HERE/fake_whisper.py > $OUT/whisper.log 2>&1 & W=$!
python3 $HERE/fake_piper.py > $OUT/piper.log 2>&1 & P=$!
set -m
ros2 launch luna_robot luna.launch.py doa:=mock > $OUT/launch.log 2>&1 &
LPID=$!
python3 $HERE/monitor.py >> $OUT/monitor.log 2>&1 &
MPID=$!
ros2 topic echo /luna/head/target > $OUT/head_target.log 2>&1 &
HPID=$!
sleep 10
curl -sN http://127.0.0.1:8765/events > $OUT/face_events.log 2>&1 &
CPID=$!
phase 1.0; note "A: dokunma (yüz sayfasının Origin'i) → $(post http://127.0.0.1:4173)"
phase 8.0; note "B: Luna konuşurken dokunma → $(post http://127.0.0.1:4173)"
if [ "${1:-}" = short ]; then phase 16.0; note "kısa koşu bitti"; ls -la /tmp/luna_utterances >> $OUT/monitor.log; kill -INT -"$LPID"; wait "$LPID"; echo "launch exit $?" >> $OUT/monitor.log; kill "$MPID" "$CPID" "$W" "$P" "$HPID" 2>/dev/null; exit 0; fi
phase 1.0; note "C: başka sitenin Origin'i → $(post https://evil.example)"
phase 9.0; note "D: dokunma yok, sahte ses 11.0'da konuşuyor"
phase 19.0; note "E: dock isteği"; ros2 topic pub --once /luna/dock/request std_msgs/msg/String "{data: dock}" >/dev/null
for i in $(seq 60); do grep -q "'CHARGING'" $OUT/monitor.log && break; sleep 1; done
phase 1.0; note "F: dock'ta dokunma → $(post '')"
phase 7.0; note "kapanış: SIGINT süreç grubuna"
ls -la /tmp/luna_utterances >> $OUT/monitor.log
kill -INT -"$LPID"; wait "$LPID"; echo "launch exit $?" >> $OUT/monitor.log
kill "$MPID" "$CPID" "$W" "$P" "$HPID" 2>/dev/null
