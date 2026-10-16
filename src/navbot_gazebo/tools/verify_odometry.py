#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

def unwrap(series):
    out, acc = [], 0.0
    for i, value in enumerate(series):
        if i > 0:
            delta = value - series[i - 1]
            while delta > math.pi:
                acc += 2.0 * math.pi
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                acc -= 2.0 * math.pi
                delta += 2.0 * math.pi
        out.append(value - acc)
    return out

class OdomChecker(Node):
    def __init__(self):
        super().__init__('verify_odometry')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.imu = None
        self.create_subscription(Odometry, '/odom', self._odom_cb, BEST_EFFORT)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, BEST_EFFORT)

    def _odom_cb(self, msg):
        self.odom = msg

    def _imu_cb(self, msg):
        self.imu = msg

def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=6.0)
    ap.add_argument('--wall-limit', type=float, default=300.0)
    ap.add_argument('--v', type=float, default=0.10)
    ap.add_argument('--w', type=float, default=0.80)
    ap.add_argument('--tol', type=float, default=0.05)
    args, _unknown = ap.parse_known_args(sys.argv[1:] if argv is None else argv)

    rclpy.init()
    node = OdomChecker()

    deadline = time.monotonic() + 150.0
    while (node.odom is None or node.imu is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.odom is None or node.imu is None:
        print('[verify] 未凑齐 /odom 与 /imu/data（odom=%s imu=%s）；'
              '请用 ros2 launch navbot_gazebo verify.launch.py'
              % (node.odom is not None, node.imu is not None), file=sys.stderr)
        return 2

    cmd = Twist()
    cmd.linear.x = args.v
    cmd.angular.z = args.w
    print('[verify] v=%.2f w=%.2f 目标仿真 %.1f s 墙上上限 %.0f s'
          % (args.v, args.w, args.duration, args.wall_limit))

    odom_yaws, true_yaws = [], []
    sim_t0 = None
    wall_start = time.monotonic()
    last_sample = -1.0
    sim_elapsed = 0.0
    while rclpy.ok():
        node.cmd.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.odom is None or node.imu is None:
            continue
        sim_now = stamp_sec(node.odom)
        if sim_t0 is None:
            sim_t0 = sim_now
        sim_elapsed = sim_now - sim_t0
        wall_elapsed = time.monotonic() - wall_start
        if wall_elapsed - last_sample >= 0.3:
            odom_yaws.append(quat_to_yaw(node.odom.pose.pose.orientation))
            true_yaws.append(quat_to_yaw(node.imu.orientation))
            last_sample = wall_elapsed
        if sim_elapsed >= args.duration or wall_elapsed >= args.wall_limit:
            break

    node.cmd.publish(Twist())
    time.sleep(0.5)

    if len(true_yaws) < 5:
        print('[verify] 采样不足：%d 组' % len(true_yaws), file=sys.stderr)
        return 2

    oy, ty = unwrap(odom_yaws), unwrap(true_yaws)
    denom = sum(t * t for t in ty)
    k = sum(a * b for a, b in zip(oy, ty)) / denom if denom > 1e-6 else float('nan')
    rmse = math.sqrt(sum((a - k * b) ** 2 for a, b in zip(oy, ty)) / len(oy))
    total_turn = ty[-1] - ty[0]
    wall_total = time.monotonic() - wall_start

    print('[verify] 采样 %d 组；仿真 %.2f s / 墙上 %.1f s（%.4fx 实时）'
          % (len(ty), sim_elapsed, wall_total, sim_elapsed / max(wall_total, 1e-6)))
    print('[verify] 真值累计转角 %.3f rad' % total_turn)
    print('[verify] k = yaw_odom / yaw_true = %.4f  RMSE = %.4f rad' % (k, rmse))
    if math.isnan(k) or abs(total_turn) < 0.5:
        print('[verify] 转角不足，请加大 --duration 或 --w')
        return 1
    if abs(k - 1.0) <= args.tol:
        print('[verify] PASS：插件声明轮距与 URDF 关节几何一致')
        return 0
    print('[verify] FAIL：k=%.3f，核对 wheel_separation 与轮心 y 坐标' % k)
    return 3

if __name__ == '__main__':
    sys.exit(main())
