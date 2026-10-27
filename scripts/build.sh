#!/usr/bin/env bash
set -o pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"

if [ ! -d /opt/ros/humble ]; then
  echo "[build] 没找到 /opt/ros/humble，请在 WSL2 的 Ubuntu 22.04 里执行本脚本" >&2
  exit 1
fi

source "$WS_ROOT/scripts/env.sh"

find "$WS_ROOT/scripts" "$WS_ROOT/src" -type f \
  \( -name '*.sh' -o -name '*.py' -o -name '*.yaml' -o -name '*.xacro' -o -name '*.urdf' \
     -o -name '*.world' -o -name '*.rviz' -o -name '*.xml' -o -name '*.txt' -o -name '*.cfg' \) \
  -print0 2>/dev/null | xargs -0 -r sed -i 's/\r$//'

chmod +x "$WS_ROOT"/scripts/*.sh "$WS_ROOT"/scripts/*.py 2>/dev/null || true
chmod +x "$WS_ROOT"/src/*/tools/* 2>/dev/null || true

echo "[build] colcon build --symlink-install"
colcon build --symlink-install

echo
echo "[build] 完成。每个新终端都要先 source："
echo "        source $WS_ROOT/install/setup.bash"
