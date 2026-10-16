"""建图 = 仿真底座 + slam_toolbox（复用 nav2_bringup 的 slam_launch）。

    ros2 launch navbot_gazebo slam.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ./scripts/save_map.sh
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _has_package(name):
    try:
        get_package_share_directory(name)
        return True
    except Exception:
        return False


def generate_launch_description():
    pkg_sim = get_package_share_directory('navbot_gazebo')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    slam_params = os.path.join(pkg_sim, 'config', 'slam.yaml')
    rviz_config = os.path.join(pkg_sim, 'config', 'nav.rviz')
    maps_dir = os.path.join(pkg_sim, 'maps')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_ekf',
                              default_value='true' if _has_package('robot_localization')
                              else 'false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'sim.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_ekf': LaunchConfiguration('use_ekf'),
                'headless': LaunchConfiguration('headless'),
                'enable_camera': LaunchConfiguration('enable_camera'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'slam_launch.py')),
            launch_arguments={
                'namespace': '',
                'params_file': slam_params,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items()),

        ExecuteProcess(cmd=['mkdir', '-p', maps_dir], shell=True, output='ignored'),
        ExecuteProcess(
            cmd=['echo',
                 '[建图] 遥控走完全屋后执行 ./scripts/save_map.sh ；'
                 '不想跑建图可用 ./scripts/gen_map_from_world.py 直接按场景坐标出图'],
            shell=True, output='screen'),
    ])
