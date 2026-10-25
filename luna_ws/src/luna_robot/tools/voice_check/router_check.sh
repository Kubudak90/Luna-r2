#!/bin/bash
# Router in the live stack: cloud:=true against fake_kimi.py (127.0.0.1:9100), mock MCU, fake Piper / aplay.
# Usage: bash /ws/src/luna_robot/tools/voice_check/router_check.sh   → logs in $VOICE_CHECK_OUT (/tmp/voice_check)
HERE=$(cd "$(dirname "$0")" && pwd)
export VOICE_CHECK_OUT=${VOICE_CHECK_OUT:-/tmp/voice_check}
OUT=$VOICE_CHECK_OUT
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
CFG=/ws/install/luna_robot/share/luna_robot/config/luna.yaml
cp /ws/src/luna_robot/config/luna.yaml "$CFG"
sed -i 's/^    start_soc: 0.24 /    start_soc: 0.60 /' "$CFG"
sed -i "s#^    tts_player: aplay -q -D {device} #    tts_player: python3 $HERE/fake_aplay.py {device} #" "$CFG"
sed -i 's#^    base_url: https://api.kimi.com/coding/v1 #    base_url: http://127.0.0.1:9100/v1 #' "$CFG"
grep -n "base_url\|router_enabled\|think_extra" "$CFG"
rm -rf $OUT && mkdir -p $OUT
python3 $HERE/fake_kimi.py > $OUT/fake_kimi.out 2>&1 & K=$!
python3 $HERE/fake_piper.py > $OUT/piper.log 2>&1 & P=$!
export KIMI_API_KEY=sk-fake-router-check
set -m
ros2 launch luna_robot luna.launch.py doa:=mock cloud:=true voice_servers:=false > $OUT/launch.log 2>&1 &
LPID=$!
python3 $HERE/monitor.py >> $OUT/monitor.log 2>&1 &
MPID=$!
sleep 10
for q in "Merhaba Luna" "12 çarpı 7 kaç eder?" "Gökkuşağı neden oluşur?"; do
  echo "### soru: $q" >> $OUT/monitor.log
  ros2 topic pub -w 2 --once /luna/assistant/request std_msgs/msg/String "{data: '$q'}" > /dev/null
  python3 -c "import time; time.sleep(8)"
done
kill -INT -"$LPID"; wait "$LPID"; echo "launch exit $?" >> $OUT/monitor.log
kill "$MPID" "$K" "$P" 2>/dev/null
