#!/usr/bin/env bash
# Host unit tests of the portable firmware modules (needs gcc, e.g. MSYS2 UCRT64 at C:\msys64\ucrt64\bin).
#   bash deploy/firmware/build_host_tests.sh [export_dir]
# Default export_dir: the newest outputs/robot_v2/deploy/export_* folder.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
EXPORT="${1:-$(ls -d "$ROOT"/outputs/robot_v2/deploy/export_* | sort | tail -1)}"
EXPORT="$(cd "$EXPORT" && pwd)"  # cmake resolves relative paths against the build folder
OUT="$ROOT/outputs/robot_v2/deploy/host_tests"
mkdir -p "$OUT"
echo "export: $EXPORT"
SRC=("$HERE"/src/policy.c "$HERE"/src/obs.c "$HERE"/src/attitude.c "$HERE"/src/joints.c "$HERE"/src/feetech.c
     "$HERE"/src/controller.c "$HERE"/test/test_host.c)
FLAGS=(-std=c11 -O2 -Wall -Wextra -Werror -I"$HERE/src" -I"$EXPORT" -DALLOW_UNVERIFIED_CALIB)
status=0
for kind in fp32 int8; do
  extra=()
  [ "$kind" = int8 ] && extra=(-DPOLICY_INT8)
  gcc "${FLAGS[@]}" "${extra[@]}" "${SRC[@]}" -lm -o "$OUT/test_host_$kind.exe"
  echo "== $kind =="
  "$OUT/test_host_$kind.exe" "$EXPORT" || status=1
done
exit $status
