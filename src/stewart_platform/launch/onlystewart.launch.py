import os
from typing import List

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _append_env_path(current: str, *new_paths: str) -> str:
    paths: List[str] = [p for p in [*new_paths, current] if p]
    return os.pathsep.join(paths)


def _is_true(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def _gazebo(context, world_file):
    """gz sim with the empty Stewart world; -s (server only) when gui:=false."""
    gui = _is_true(LaunchConfiguration('gui').perform(context))
    verbose = _is_true(LaunchConfiguration('verbose').perform(context))
    gz_args = f"-r -v {4 if verbose else 1} {'' if gui else '-s '}{world_file}"
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )]


def generate_launch_description():
    package_share = get_package_share_directory('stewart_platform')
    package_prefix = get_package_prefix('stewart_platform')

    sdf_file = os.path.join(package_share, 'stewart_sdf_model', 'stewart_sdf_gz.sdf')
    world_file = os.path.join(package_share, 'world', 'stewart_gz.sdf')
    plugin_dir = os.path.join(package_prefix, 'lib')

    use_sim_time = LaunchConfiguration('use_sim_time')

    set_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        _append_env_path(os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''), plugin_dir))

    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        _append_env_path(os.environ.get('GZ_SIM_RESOURCE_PATH', ''), package_share))

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'stewart',
            '-name', 'stewart',
            '-file', sdf_file,
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    ik_node = Node(
        package='stewart_platform',
        executable='ik',
        name='inverse_kinematic',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        set_plugin_path,
        set_resource_path,

        OpaqueFunction(function=_gazebo, args=[world_file]),
        spawn_entity,
        clock_bridge,
        ik_node
    ])
