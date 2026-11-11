#!/usr/bin/env bash
# Provision the headless EGL loader without depending on Ubuntu package mirrors.
set -euo pipefail
if python3 - <<'PY'
import ctypes,sys
try:
    for library in ['libEGL.so.1','libGL.so.1','libgomp.so.1']:ctypes.CDLL(library)
except OSError:sys.exit(1)
PY
then
    echo 'Open3D runtime libraries already available.'
    exit 0
fi
runtime_directory=$(python3 tools/prepare_open3d_runtime.py --output-dir "${RUNNER_TEMP:-/tmp}/greenspray-runtime")
export LD_LIBRARY_PATH="$runtime_directory:${LD_LIBRARY_PATH:-}"
python3 - <<'PY'
import ctypes
for library in ['libEGL.so.1','libGL.so.1','libgomp.so.1']:ctypes.CDLL(library)
print('Verified job-local Open3D runtime libraries.')
PY
if [[ -n "${GITHUB_ENV:-}" ]]; then
    printf 'LD_LIBRARY_PATH=%s\n' "$LD_LIBRARY_PATH" >> "$GITHUB_ENV"
fi
