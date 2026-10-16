"""巡检任务启动：注入 use_sim_time，避免时间戳跑在 TF 前面。

    ros2 launch navbot_patrol patrol.launch.py
    ros2 launch navbot_patrol patrol.launch.py waypoints_file:=/绝对路径/xxx.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_waypoints = os.path.join(
        get_package_share_directory('navbot_patrol'), 'config', 'waypoints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='navbot_patrol',
            executable='patrol',
            name='navbot_patrol',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['--waypoints', LaunchConfiguration('waypoints_file')],
        ),
    ])
