"""模型自检：只在 RViz 里看几何与 TF，不起仿真。

    ros2 launch navbot_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('navbot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'navbot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' publish_odom_tf:=true']), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(pkg, 'rviz', 'model.rviz')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': False}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])
