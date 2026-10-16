#!/usr/bin/env bash
set -o pipefail

STRICT=0
[ "${1:-}" = "--strict" ] && STRICT=1

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$WS_ROOT/scripts/env.sh"

fail=0
rule() { printf '%s\n' "------------------------------------------------------------------------"; }

topic_count() {  # 发布者为 0/1/多
  timeout 12 ros2 topic info "$1" --verbose 2>/dev/null | awk '/Publisher count/{print $3; exit}'
}

rule
echo "[1] 关键传感器与里程计话题"
for t in /odom /scan /imu/data; do
  n="$(topic_count "$t")"
  if [ -z "$n" ]; then
    echo "  MISSING  $t （话题不存在 —— 仿真没起来？）"; fail=1
  elif [ "$n" = "0" ]; then
    echo "  DANGER   $t 存在但没有发布者"; fail=1
  elif [ "$n" = "1" ]; then
    echo "  OK       $t  发布者 1 个"
  else
    echo "  WARN     $t  发布者 $n 个（多于 1 个通常意味着重复发布）"; fail=1
  fi
done
sub=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | awk '/Subscription count/{print $3}')
if [ "${sub:-0}" -ge 1 ] 2>/dev/null; then
  echo "  OK       /cmd_vel  订阅者 $sub 个（差速插件在听）"
else
  echo "  MISSING  /cmd_vel 没有订阅者 —— 差速插件没起来"; fail=1
fi

rule
echo "[2] EKF 的两个输入必须真实存在（原项目缺这两个，EKF 空转）"
if [ "$(topic_count /odom)" = "1" ] && [ "$(topic_count /imu/data)" = "1" ]; then
  echo "  OK       /odom + /imu/data 都有数据，EKF 会有输入"
else
  echo "  DANGER   EKF 输入不完整：/odom=$(topic_count /odom) /imu/data=$(topic_count /imu/data)"
  echo "           若装了 robot_localization 但用 use_ekf:=false，可忽略本项"
  fail=1
fi

rule
echo "[3] TF 发布者（odom->base_footprint 只能有一个主人：EKF 或差速插件）"
timeout 12 ros2 topic info /tf --verbose 2>/dev/null | grep -E "Node name" | sed 's/^/  /'
echo "  若同时出现 ekf_filter_node 与 gzserver/差速插件，就是 TF 双主人，必须二选一"

rule
echo "[4] TF 链路连通性（用 scripts/tf_probe.py，按仿真时钟查询）"
timeout 40 python3 "$WS_ROOT/scripts/tf_probe.py" \
  'odom->base_footprint' 'base_footprint->base_link' 'base_link->scanner_frame' \
  2>&1 | sed 's/^/  /' || fail=1
if timeout 40 python3 "$WS_ROOT/scripts/tf_probe.py" 'map->base_footprint' 2>&1 | grep -q "OK"; then
  echo "  map -> base_footprint 已就绪（定位已激活）"
else
  echo "  n/a  map -> base_footprint（没起 AMCL/slam_toolbox 时正常）"
fi

rule
echo "[5] Nav2 服务端"
for a in navigate_to_pose follow_waypoints; do
  if timeout 12 ros2 action list 2>/dev/null | grep -q "^/$a$"; then
    echo "  OK       /$a"
  else
    echo "  MISSING  /$a（Nav2 导航栈没起来）"; fail=1
  fi
done
timeout 12 ros2 topic list 2>/dev/null | grep -q "^/map$" \
  && echo "  OK       /map 已发布" || { echo "  MISSING  /map"; fail=1; }

rule
if [ "$fail" = "0" ]; then
  echo "结论：全部关键项正常。"
else
  echo "结论：有项目异常，见上面 DANGER/MISSING 行。"
fi
[ "$STRICT" = "1" ] && exit $fail
exit 0
