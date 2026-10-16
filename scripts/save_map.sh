#!/usr/bin/env bash
set -o pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_BASE="${1:-$WS_ROOT/src/navbot_gazebo/maps/lab}"

if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
  echo "[save_map] 先执行 ./scripts/build.sh" >&2
  exit 1
fi
source "$WS_ROOT/scripts/env.sh"

mkdir -p "$(dirname "$MAP_BASE")"

echo "[save_map] 订阅 /map 并写出 $MAP_BASE.pgm / $MAP_BASE.yaml"
timeout 60 ros2 run nav2_map_server map_saver_cli -f "$MAP_BASE" --fmt pgm

echo
echo "[save_map] 结果："
ls -l "$MAP_BASE".* 
echo "[save_map] 注意：install 目录里也有一份（构建时拷贝）。若用过 colcon build --symlink-install，"
echo "           src 与 install 指向同一份，改一次即可；否则请重新 build。"
