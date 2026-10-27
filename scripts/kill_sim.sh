#!/usr/bin/env bash
set -o pipefail

PATTERNS=(
  '/opt/ros/humble/bin/ros2 launch robot_'
  '/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'
  '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py'
  '/opt/ros/humble/lib/robot_localization/ekf_node'
  '/opt/ros/humble/lib/nav2_'
  '/opt/ros/humble/lib/slam_toolbox/'
  '/opt/ros/humble/lib/rclcpp_components/component_container'
  '/opt/ros/humble/lib/rviz2/rviz2'
  '/opt/ros/humble/lib/teleop_twist_keyboard/'
  '/opt/ros/humble/bin/ros2 launch navbot_'
  'lib/navbot_patrol/patrol'
  '/homebot2_ws/src/robot_'
)

ancestors() {
  local pid=$$ out=""
  while [ -n "$pid" ] && [ "$pid" != "0" ] && [ "$pid" != "1" ]; do
    out="$out $pid"
    pid=$(awk '{print $4}' "/proc/$pid/stat" 2>/dev/null)
  done
  echo "$out"
}

PROTECT=" $(ancestors) "

kill_pattern() {
  local pat="$1" pid
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    case "$PROTECT" in *" $pid "*) continue ;; esac
    kill -TERM "$pid" 2>/dev/null || true
  done
}

for p in "${PATTERNS[@]}"; do kill_pattern "$p"; done
pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true

for i in $(seq 1 15); do
  pgrep -x gzserver >/dev/null 2>&1 || break
  sleep 1
done
if pgrep -x gzserver >/dev/null 2>&1; then
  echo "[kill_sim] gzserver 未响应 SIGTERM，强制清理"
  pkill -9 -x gzserver 2>/dev/null || true
  sleep 2
fi

if ! pgrep -x gzserver >/dev/null 2>&1 \
   && ! pgrep -f '/opt/ros/humble/bin/ros2 launch robot_' >/dev/null 2>&1; then
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  rm -f /tmp/gzmaster-* 2>/dev/null
fi
ros2 daemon stop --force >/dev/null 2>&1 || true

LEFT=$(pgrep -f '/opt/ros/humble/bin/ros2 launch (navbot|robot)_|/opt/ros/humble/lib/nav2_|lib/navbot_patrol/patrol' 2>/dev/null | wc -l)
echo "[kill_sim] 完成，本工程残留启动进程 $LEFT 个"
exit 0
