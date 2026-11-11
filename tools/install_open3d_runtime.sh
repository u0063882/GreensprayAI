#!/usr/bin/env bash
# Use Ubuntu's signed HTTPS repositories rather than the runner's Azure mirror.
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
. /etc/os-release
if [[ "${ID:-}" != ubuntu || -z "${VERSION_CODENAME:-}" ]]; then
    echo 'This runtime installer requires Ubuntu.' >&2
    exit 1
fi
apt_work=$(mktemp -d "${RUNNER_TEMP:-/tmp}/greenspray-apt.XXXXXX")
trap 'rm -rf "$apt_work"' EXIT
chmod 755 "$apt_work"
cat > "$apt_work/ubuntu.sources" <<EOF
Types: deb
URIs: https://archive.ubuntu.com/ubuntu
Suites: ${VERSION_CODENAME} ${VERSION_CODENAME}-updates
Components: main universe
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

Types: deb
URIs: https://security.ubuntu.com/ubuntu
Suites: ${VERSION_CODENAME}-security
Components: main universe
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
apt_options=(-o "Dir::Etc::sourcelist=$apt_work/ubuntu.sources" -o 'Dir::Etc::sourceparts=-'
             -o 'Acquire::Retries=2' -o 'Acquire::http::Timeout=15' -o 'Acquire::https::Timeout=15')
sudo apt-get "${apt_options[@]}" update
sudo apt-get "${apt_options[@]}" install --no-install-recommends -y libegl1 libgl1 libgomp1
