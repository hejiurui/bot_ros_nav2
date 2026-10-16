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

def _has_package(name):
    try:
        get_package_share_directory(name)
        return True
    except Exception:
        return False

def generate_launch_description():
    pkg_sim = get_package_share_directory('navbot_gazebo')
    pkg_desc = get_package_share_directory('navbot_description')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'navbot.urdf.xacro')
    world_file = os.path.join(pkg_sim, 'worlds', 'lab.world')
    ekf_file = os.path.join(pkg_sim, 'config', 'ekf.yaml')
    rviz_file = os.path.join(pkg_sim, 'config', 'sim.rviz')
    gazebo_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    has_ekf = _has_package('robot_localization')

    lc_use_ekf = LaunchConfiguration('use_ekf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    odom_tf_from_plugin = PythonExpression(
        ["'true' if '", lc_use_ekf, "'.lower() in ('false','0') else 'false'"])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' publish_odom_tf:=', odom_tf_from_plugin,
                 ' lidar_visualize:=', LaunchConfiguration('lidar_visualize'),
                 ' enable_camera:=', LaunchConfiguration('enable_camera'),
                 ' enable_lidar:=', LaunchConfiguration('enable_lidar')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_simulator', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('lidar_visualize', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('rviz_config', default_value=rviz_file),
        DeclareLaunchArgument('ekf_params_file', default_value=ekf_file),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.02'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('spawn_timeout', default_value='240'),
        DeclareLaunchArgument('use_ekf', default_value='true' if has_ekf else 'false'),

        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1',
                               condition=UnlessCondition(headless)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'verbose': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('use_simulator'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'gzclient.launch.py')),
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('use_simulator'), "'.lower() not in ('false','0')",
                 " and '", headless, "'.lower() in ('false','0')"]))),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
             output='screen'),

        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description', '-entity', 'navbot',
                        '-x', LaunchConfiguration('spawn_x'),
                        '-y', LaunchConfiguration('spawn_y'),
                        '-z', LaunchConfiguration('spawn_z'),
                        '-Y', LaunchConfiguration('spawn_yaw'),
                        '-timeout', LaunchConfiguration('spawn_timeout')],
             output='screen'),

        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             parameters=[LaunchConfiguration('ekf_params_file'),
                         {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
             condition=IfCondition(lc_use_ekf)),

        Node(package='rviz2', executable='rviz2', output='screen',
             parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
             arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
