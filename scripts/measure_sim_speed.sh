#!/usr/bin/env bash
set -o pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"
LIDAR="${1:-true}"
CAMERA="${2:-true}"
WINDOW="${3:-20}"

source ./scripts/env.sh
./scripts/kill_sim.sh

setsid ros2 launch navbot_gazebo sim.launch.py \
  headless:=true use_rviz:=false enable_lidar:="$LIDAR" enable_camera:="$CAMERA" \
  > .logs/speed.log 2>&1 &
PG=$!
trap 'kill -INT -"$PG" 2>/dev/null; sleep 3; kill -TERM -"$PG" 2>/dev/null' EXIT

for i in $(seq 1 30); do
  grep -q "Advertise odometry on" .logs/speed.log 2>/dev/null && break
  sleep 2
done
sleep 10

cat > /tmp/_speed.py <<PYEOF
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
rclpy.init(); n = Node('speed'); st = []
n.create_subscription(Odometry, '/odom',
                      lambda m: st.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
                      QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
t0 = time.monotonic()
while time.monotonic() - t0 < $WINDOW:
    rclpy.spin_once(n, timeout_sec=0.05)
w = time.monotonic() - t0
if len(st) > 3:
    print('lidar=$LIDAR camera=$CAMERA  帧数=%d  仿真推进 %.2f s / 墙上 %.1f s  =>  %.3fx 实时'
          % (len(st), st[-1] - st[0], w, (st[-1] - st[0]) / w))
else:
    print('lidar=$LIDAR camera=$CAMERA  没收到 /odom（帧数=%d）—— 看 .logs/speed.log' % len(st))
PYEOF
timeout $((WINDOW + 60)) python3 /tmp/_speed.py
