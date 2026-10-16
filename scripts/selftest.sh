#!/usr/bin/env bash
set -o pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"
mkdir -p .logs

if [ ! -f install/setup.bash ]; then
  echo "[selftest] 还没编译，先执行 ./scripts/build.sh" >&2
  exit 1
fi

source "$WS_ROOT/scripts/env.sh"
"$WS_ROOT/scripts/kill_sim.sh"

wait_for_odom() {  # 等 gzserver 把插件起起来（冷启动通常 15~40 s）
  local log="$1"
  for i in $(seq 1 40); do
    grep -q "Advertise odometry on" "$log" 2>/dev/null && return 0
    sleep 2
  done
  return 1
}

echo "[selftest] 阶段 1：headless 仿真 + 健康检查（日志 .logs/selftest_sim.log）"
setsid ros2 launch navbot_gazebo sim.launch.py \
  headless:=true use_rviz:=false > .logs/selftest_sim.log 2>&1 &
SIM_PG=$!
if wait_for_odom .logs/selftest_sim.log; then
  sleep 15
  ./scripts/check_system.sh
else
  echo "[selftest] 阶段 1 仿真没起来，日志尾部："
  tail -25 .logs/selftest_sim.log
fi
kill -INT -"$SIM_PG" 2>/dev/null || true
sleep 4
"$WS_ROOT/scripts/kill_sim.sh"

echo
echo "[selftest] 阶段 2：里程计标定（标定节点与仿真同会话；日志 .logs/verify.log）"
timeout 600 ros2 launch navbot_gazebo verify.launch.py \
  duration:=2.0 wall_limit:=420 w:=0.9 > .logs/verify.log 2>&1
RC=$?
grep -E "verify\]|separation set" .logs/verify.log | tail -8

echo
if grep -q "PASS" .logs/verify.log; then
  echo "[selftest] 里程计标定 PASS（轮距声明与几何一致）"
  RC=0
else
  echo "[selftest] 里程计标定未取得通过结果，返回码 $RC，详见 .logs/verify.log"
fi
"$WS_ROOT/scripts/kill_sim.sh"
exit $RC
