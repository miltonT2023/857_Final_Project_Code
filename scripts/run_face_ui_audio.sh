#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIBGOMP="/lib/aarch64-linux-gnu/libgomp.so.1"

cd "$PROJECT_ROOT"

if [[ -f "$LIBGOMP" ]]; then
  if [[ -n "${LD_PRELOAD:-}" ]]; then
    export LD_PRELOAD="$LIBGOMP:$LD_PRELOAD"
  else
    export LD_PRELOAD="$LIBGOMP"
  fi
else
  echo "Warning: $LIBGOMP was not found; continuing without LD_PRELOAD." >&2
fi

source install/setup.bash
exec ros2 launch milton_final_project face_ui_audio_launch.py "$@"
