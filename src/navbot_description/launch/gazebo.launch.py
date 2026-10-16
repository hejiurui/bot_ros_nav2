"""模型进 Gazebo 空世界，验证驱动与传感器（不含 SLAM/导航）。

    ros2 launch navbot_description gazebo.launch.py
    ros2 launch navbot_description gazebo.launch.py headless:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('navbot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'navbot.urdf.xacro')
    gazebo_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    headless = LaunchConfiguration('headless')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' publish_odom_tf:=', PythonExpression(
                     ["'true' if '", headless, "'.lower() in ('true','1') else 'true'"]),
                 ' enable_camera:=true']),
        value_type=str)

    return LaunchDescription([
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1',
                               condition=UnlessCondition(headless)),

        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'gzserver.launch.py')),
            launch_arguments={'verbose': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'gzclient.launch.py')),
            condition=UnlessCondition(headless)),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
             output='screen'),

        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description', '-entity', 'navbot',
                        '-x', '0.0', '-y', '0.0', '-z', '0.02',
                        '-timeout', '240'],
             output='screen'),

        Node(package='rviz2', executable='rviz2', output='screen',
             parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(pkg, 'rviz', 'model.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
