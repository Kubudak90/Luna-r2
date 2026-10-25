#!/bin/sh
# Builds and runs the host unit tests for lib/luna_core with the system C++ compiler (or $CXX).
# Same language rules as the Teensy build (no exceptions/RTTI), plus ASan/UBSan.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
core="$here/../lib/luna_core/src"
out="${TMPDIR:-/tmp}/luna_core_host_test"
${CXX:-c++} -std=gnu++17 -O1 -g -Wall -Wextra -Wpedantic -Wshadow -Werror \
  -fno-exceptions -fno-rtti -fsanitize=address,undefined -fno-sanitize-recover=all \
  -I"$core" "$here/test_core.cpp" "$core/daly_bms.cpp" "$core/cliff.cpp" "$core/debounce.cpp" "$core/drive.cpp" -o "$out"
"$out"
