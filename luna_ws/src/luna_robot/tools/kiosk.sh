#!/bin/sh
# Luna's face on the head's 5.5" AMOLED, outside ROS: a full-screen browser on the face bridge's page
# (face_bridge_node, http://127.0.0.1:8765/). Run it from the Orin's desktop session autostart (JetPack 6, X11):
#   ~/.config/autostart/luna-face.desktop → Exec=/path/to/luna_robot/tools/kiosk.sh
# The panel is 1080 × 1920 portrait and lies landscape in the head, so the output and its touch are turned.
# Overrides: FACE_URL, FACE_OUTPUT (xrandr name, default: the first connected HDMI/DP output),
#            FACE_ROTATE (left | right | normal | inverted, default left), FACE_TOUCH (xinput device name).
set -eu

url=${FACE_URL:-http://127.0.0.1:8765/}
rotate=${FACE_ROTATE:-left}
output=${FACE_OUTPUT:-$(xrandr --query | awk '/ connected/ && /^(HDMI|DP)/ {print $1; exit}')}

xset s off -dpms s noblank 2>/dev/null || true          # the face must never blank
if [ -n "$output" ]; then
  xrandr --output "$output" --rotate "$rotate" || echo "kiosk: could not rotate $output" >&2
fi

# Touch follows the rotation (libinput coordinate transformation matrix).
case "$rotate" in
  left) matrix="0 -1 1 1 0 0 0 0 1" ;;
  right) matrix="0 1 0 -1 0 1 0 0 1" ;;
  inverted) matrix="-1 0 1 0 -1 1 0 0 1" ;;
  *) matrix="1 0 0 0 1 0 0 0 1" ;;
esac
touch=${FACE_TOUCH:-$(xinput list --name-only 2>/dev/null | grep -i -m1 -E 'touch|waveshare|wch' || true)}
if [ -n "$touch" ]; then
  # shellcheck disable=SC2086  # the matrix is nine separate arguments
  xinput set-prop "$touch" "Coordinate Transformation Matrix" $matrix || echo "kiosk: could not turn touch '$touch'" >&2
else
  echo "kiosk: no touch device found; set FACE_TOUCH" >&2
fi

# Wait for the bridge (the ROS container may still be starting); the page also reconnects on its own.
i=0
while [ "$i" -lt 120 ] && ! curl -fsS "${url%/}/health" >/dev/null 2>&1; do i=$((i + 1)); sleep 1; done

browser=$(command -v chromium-browser || command -v chromium || command -v google-chrome || true)
if [ -z "$browser" ]; then
  echo "kiosk: no Chromium found (sudo snap install chromium)" >&2
  exit 1
fi
exec "$browser" --kiosk --app="$url" --noerrdialogs --disable-infobars --no-first-run --disable-translate \
  --disable-features=TranslateUI --overscroll-history-navigation=0 --disable-pinch --check-for-update-interval=31536000 \
  --password-store=basic --incognito
