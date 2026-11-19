#!/usr/bin/env bash
# Build the RP2350 firmware (no flashing).
#   bash deploy/firmware/build_firmware.sh [export_dir] [extra cmake args, e.g. -DALLOW_UNVERIFIED_CALIB=ON]
# Tools: CMake + Ninja from the project venv .venv_fw (uv pip install cmake ninja); ARM GCC 14.2, Pico SDK 2.1 and
# picotool from the teammate's handoff (references/mango_x4_handoff_20261117/.../mango_feetech/.tools, read only).
# Output: outputs/robot_v2/deploy/firmware_build/<timestamp>/biped_runtime.uf2 (+ .elf, .map, build info).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
TOOLS="$ROOT/references/mango_x4_handoff_20261117/MANGO_X4_HANDOFF/mango_feetech/.tools"
EXPORT="${1:-$(ls -d "$ROOT"/outputs/robot_v2/deploy/export_* | sort | tail -1)}"
shift || true
OUT="$ROOT/outputs/robot_v2/deploy/firmware_build/$(date +%Y%m%d_%H%M%S)"
BUILD="$OUT/build"
mkdir -p "$BUILD"
export PATH="$ROOT/.venv_fw/Scripts:$TOOLS/bin:$PATH"
export PICO_TOOLCHAIN_PATH="$TOOLS/bin"
win() { cygpath -m "$1"; }
cmake -S "$(win "$HERE")" -B "$(win "$BUILD")" -G Ninja \
  -DPICO_SDK_PATH="$(win "$TOOLS/pico-sdk")" -DEXPORT_DIR="$(win "$EXPORT")" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 "$@" > "$OUT/cmake_configure.log" 2>&1 || { tail -30 "$OUT/cmake_configure.log"; exit 1; }
cmake --build "$(win "$BUILD")" --parallel 8 > "$OUT/cmake_build.log" 2>&1 || { grep -E "error|warning" "$OUT/cmake_build.log" | head -40; exit 1; }
echo "compiler warnings: $(grep -c "warning" "$OUT/cmake_build.log" || true)"
"$TOOLS/picotool-build/Release/picotool.exe" uf2 convert "$(win "$BUILD/biped_runtime.elf")" "$(win "$OUT/biped_runtime.uf2")" \
  --family rp2350-arm-s --abs-block 0x101fff00 > /dev/null
cp "$BUILD/biped_runtime.elf" "$BUILD/biped_runtime.elf.map" "$OUT/" 2>/dev/null || cp "$BUILD/biped_runtime.elf" "$OUT/"
"$TOOLS/bin/arm-none-eabi-size.exe" "$OUT/biped_runtime.elf" | tee "$OUT/size.txt"
{ echo "export: $EXPORT"; echo "cmake args: $*"; echo "git: $(git -C "$ROOT" rev-parse --short HEAD)$(git -C "$ROOT" diff --quiet || echo -dirty)"; } > "$OUT/build_info.txt"
echo "[firmware] $OUT/biped_runtime.uf2"
