#!/usr/bin/env bash
set -o pipefail
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source ./scripts/env.sh
./scripts/kill_sim.sh

setsid ros2 launch navbot_gazebo navigate.launch.py \
  headless:=true use_rviz:=false enable_camera:=false \
  > .logs/nav.log 2>&1 &
PG=$!
trap 'kill -INT -"$PG" 2>/dev/null; sleep 4; kill -TERM -"$PG" 2>/dev/null' EXIT

for i in $(seq 1 70); do
  grep -qE "Managed nodes are active" .logs/nav.log 2>/dev/null && break
  sleep 3
done
sleep 15
echo "=== Nav2 进程数 / 激活情况 ==="
echo "  process started: $(grep -c 'process started' .logs/nav.log) 个"
grep -oE "Managed nodes are active" .logs/nav.log | sort | uniq -c | sed 's/^/  /'
grep -iE "Failed to bring up|exception|error while|Invalid parameter" .logs/nav.log | head -5 | sed 's/^/  ERR /'

echo "=== 关键话题 ==="
timeout 20 ros2 topic list 2>/dev/null | grep -E "^/map$|^/plan$|^/odom$|^/scan$|^/imu/data$|^/amcl_pose$|^/initialpose$" | tr '\n' ' ' | sed 's/^/  /'; echo

echo "=== TF（仿真时钟）==="
timeout 40 python3 scripts/tf_probe.py 'map->odom' 'odom->base_footprint' \
  'base_footprint->base_link' 'scanner_link->scanner_frame' 2>&1 | sed 's/^/  /'

echo "=== 动作服务端 ==="
timeout 20 ros2 action list 2>/dev/null | grep -E "navigate_to_pose|follow_waypoints" | tr '\n' ' ' | sed 's/^/  /'; echo

echo "=== 起巡检节点 ==="
setsid ros2 launch navbot_patrol patrol.launch.py > .logs/patrol.log 2>&1 &
PP=$!
sleep 70
grep -E "使用航点配置|航点 [0-9]+ 个|Nav2 is ready|Following .* goals|巡检进度" .logs/patrol.log | head -6 | sed 's/^/  /'
echo "=== /plan 是否已有路径（规划器在工作）==="
timeout 40 ros2 topic echo /plan --field poses --no-arr 2>&1 | tail -2 | sed 's/^/  /'
kill -INT -"$PP" 2>/dev/null; sleep 4; kill -TERM -"$PP" 2>/dev/null
echo "--- patrol 日志尾部 ---"; tail -5 .logs/patrol.log | sed 's/^/  /'
