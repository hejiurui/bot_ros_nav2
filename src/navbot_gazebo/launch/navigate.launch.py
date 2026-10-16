"""自主导航 = 仿真底座 + AMCL 定位 + Nav2 导航栈。

    ros2 launch navbot_gazebo navigate.launch.py
    ros2 launch navbot_gazebo navigate.launch.py use_composition:=True
"""
import os
import sys

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


def _nav_bool(name, default='False'):
    """nav2 的 launch 里有 PythonExpression(['not ', use_composition])，
    值会被 eval()，小写 true 会变成变量名报 NameError，所以在生成期归一。"""
    for arg in sys.argv[1:]:
        if arg.startswith(name + ':='):
            value = arg.split(':=', 1)[1].strip().lower()
            return 'True' if value in ('true', '1', 'yes') else 'False'
    return default


def generate_launch_description():
    pkg_sim = get_package_share_directory('navbot_gazebo')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_sim, 'params', 'nav2_params.yaml')
    default_map = os.path.join(pkg_sim, 'maps', 'lab_map.yaml')
    has_map = os.path.exists(default_map)
    rviz_config = os.path.join(pkg_sim, 'config',
                               'nav.rviz' if has_map else 'sim.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_file_path')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_ekf',
                                        default_value='true' if _has_package('robot_localization')
                                        else 'false'))
    ld.add_action(DeclareLaunchArgument('headless', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_camera', default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_composition', default_value='False'))
    ld.add_action(DeclareLaunchArgument('rviz_config', default_value=rviz_config))
    ld.add_action(DeclareLaunchArgument('map_file_path', default_value=default_map))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ekf': LaunchConfiguration('use_ekf'),
            'headless': LaunchConfiguration('headless'),
            'enable_camera': LaunchConfiguration('enable_camera'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items()))

    if not has_map:
        ld.add_action(ExecuteProcess(
            cmd=['echo',
                 '[导航] 未找到 %s：本次只启动 Nav2，不启动 AMCL。'
                 ' 可先跑 slam.launch.py 建图，或 ./scripts/gen_map_from_world.py 直接出图'
                 % default_map],
            shell=True, output='screen'))
    else:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'localization_launch.py')),
            launch_arguments={
                'namespace': '',
                'map': map_yaml,
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'use_composition': _nav_bool('use_composition'),
                'container_name': 'nav2_loc_container',
            }.items()))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': '',
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': _nav_bool('use_composition'),
            'container_name': 'nav2_nav_container',
        }.items()))

    return ld
