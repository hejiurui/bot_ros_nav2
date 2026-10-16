#!/usr/bin/env python3
import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def package_config_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('navbot_patrol'),
                            'config', 'waypoints.yaml')
    except Exception:
        return None


def resolve_config(cli_path):
    for candidate in (cli_path, os.environ.get('NAVBOT_WAYPOINTS'), package_config_path()):
        if candidate and os.path.exists(candidate):
            return candidate
    raise FileNotFoundError(
        '找不到航点配置：用 --waypoints 指定，或设置 NAVBOT_WAYPOINTS，'
        '或先 colcon build 让包内 config/ 生效')


def load_config(path):
    with open(path, 'r', encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle) or {}
    waypoints = cfg.get('waypoints') or []
    if not waypoints:
        raise ValueError('%s 中没有 waypoints 条目' % path)
    for i, wp in enumerate(waypoints):
        for key in ('x', 'y'):
            if key not in wp:
                raise ValueError('%s 第 %d 个航点缺少 %s' % (path, i, key))
        wp.setdefault('yaw_deg', 0.0)
        wp.setdefault('name', 'wp%d' % i)
    return cfg


def yaw_to_quaternion(yaw_rad):
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class PatrolRunner(BasicNavigator):

    def __init__(self, cfg, loop_override=None):
        super().__init__('navbot_patrol')
        self.cfg = cfg
        task = cfg.get('task') or {}
        self.ready_timeout = float(task.get('ready_timeout_sec', 30.0))
        self.task_timeout = float(task.get('timeout_sec', 420.0))
        self.log_period = float(task.get('log_period_sec', 3.0))
        self.loop = int(loop_override if loop_override is not None
                        else task.get('loop', 1))
        self.started = time.monotonic()

    def make_pose(self, x, y, yaw_deg, frame='map'):
        q_x, q_y, q_z, q_w = yaw_to_quaternion(math.radians(yaw_deg))
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def poses(self):
        return [self.make_pose(wp['x'], wp['y'], wp['yaw_deg'])
                for wp in self.cfg['waypoints']]

    @staticmethod
    def route_length(waypoints_cfg):
        pts = [(float(w['x']), float(w['y'])) for w in waypoints_cfg]
        return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))

    def wait_ready(self):
        deadline = time.monotonic() + self.ready_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                if self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
                    return True
            except Exception:
                return False
            self.info('等待 FollowWaypoints 服务端（先启动 navigate.launch.py）')
        return False

    def apply_initial_pose(self):
        ip = self.cfg.get('initial_pose') or {}
        if not ip.get('set', False):
            self.info('配置未要求注入初始位姿')
            return
        self.setInitialPose(self.make_pose(ip.get('x', 0.0), ip.get('y', 0.0),
                                           ip.get('yaw_deg', 0.0)))

    def await_task(self, total):
        last_index = -1
        last_log = 0.0
        deadline = time.monotonic() + self.task_timeout
        while rclpy.ok():
            try:
                done = self.isTaskComplete()
            except Exception:
                return TaskResult.CANCELED
            if done:
                return self.getResult()
            if time.monotonic() > deadline:
                self.error('任务超时 %.0f s，发送 cancelTask' % self.task_timeout)
                try:
                    self.cancelTask()
                except Exception:
                    return TaskResult.CANCELED
                for _ in range(20):
                    if self.isTaskComplete():
                        return self.getResult()
                    time.sleep(0.1)
                return TaskResult.CANCELED
            try:
                feedback = self.getFeedback()
            except Exception:
                feedback = None
            index = int(feedback.current_waypoint) if feedback is not None else -1
            now = time.monotonic()
            if index != last_index or (now - last_log) >= self.log_period:
                names = self.cfg['waypoints']
                label = names[index]['name'] if 0 <= index < len(names) else '-'
                self.info('进度 %d/%d 当前目标 %s' % (index + 1, total, label))
                last_index, last_log = index, now
            time.sleep(0.1)
        return TaskResult.CANCELED

    def run(self):
        names = [wp['name'] for wp in self.cfg['waypoints']]
        self.info('航点 %d 个，航线 %.2f m：%s'
                  % (len(names), self.route_length(self.cfg['waypoints']), ' -> '.join(names)))

        if not self.wait_ready():
            self.error('%.0f s 内 Nav2 未就绪' % self.ready_timeout)
            return 2

        self.apply_initial_pose()
        self.waitUntilNav2Active()

        poses = self.poses()
        for lap in range(1, max(1, self.loop) + 1):
            self.info('===== 第 %d/%d 轮 =====' % (lap, self.loop))
            if not self.followWaypoints(poses):
                return 4
            result = self.await_task(len(poses))
            elapsed = time.monotonic() - self.started
            if result == TaskResult.SUCCEEDED:
                self.info('本轮完成，累计 %.1f s' % elapsed)
            elif result == TaskResult.CANCELED:
                self.error('本轮被取消，累计 %.1f s' % elapsed)
                return 3
            else:
                self.error('本轮失败 result=%r，累计 %.1f s' % (result, elapsed))
                return 4
        return 0


def parse_args(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('--waypoints', default=None)
    ap.add_argument('--loop', type=int, default=None)
    ap.add_argument('--timeout', type=float, default=None)
    args, _unknown = ap.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        path = resolve_config(args.waypoints)
        cfg = load_config(path)
    except (FileNotFoundError, ValueError) as exc:
        print('[navbot_patrol] 配置错误：%s' % exc, file=sys.stderr)
        return 1
    print('[navbot_patrol] 航点配置：%s' % path)

    if args.timeout is not None:
        cfg.setdefault('task', {})['timeout_sec'] = args.timeout

    rclpy.init()
    node = PatrolRunner(cfg, loop_override=args.loop)
    try:
        code = node.run()
    except KeyboardInterrupt:
        try:
            node.cancelTask()
        except Exception:
            pass
        print('[navbot_patrol] 已请求取消任务')
        code = 130
    except Exception as exc:
        if rclpy.ok():
            raise
        print('[navbot_patrol] 收到关闭信号，退出（%s）' % type(exc).__name__)
        code = 130
    finally:
        try:
            node.destroyNode()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
