import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sim = get_package_share_directory('navbot_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='6.0'),
        DeclareLaunchArgument('wall_limit', default_value='300.0'),
        DeclareLaunchArgument('v', default_value='0.10'),
        DeclareLaunchArgument('w', default_value='0.80'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'sim.launch.py')),
            launch_arguments={
                'headless': 'true',
                'use_rviz': 'false',
                'enable_camera': 'false',
            }.items()),

        Node(
            package='navbot_gazebo',
            executable='verify_odometry.py',
            name='verify_odometry',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--duration', LaunchConfiguration('duration'),
                '--wall-limit', LaunchConfiguration('wall_limit'),
                '--v', LaunchConfiguration('v'),
                '--w', LaunchConfiguration('w'),
            ],
        ),
    ])
