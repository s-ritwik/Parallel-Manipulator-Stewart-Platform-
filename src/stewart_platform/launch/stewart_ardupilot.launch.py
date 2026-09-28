import os
from typing import List

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _append_env_path(current: str, *new_paths: str) -> str:
    paths: List[str] = [p for p in [*new_paths, current] if p]
    return os.pathsep.join(paths)


def _is_true(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def _gazebo(context):
    """gz sim with the runway world; -s (server only) when gui:=false."""
    world_file = LaunchConfiguration('world').perform(context)
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
    # Stewart spawn in the gz world (ENU). Classic spawned at gazebo (3, 0), which
    # its (x, y) -> (-y, x) mapping put at MAVROS/ENU (0, 3); keep that geometry.
    spawn_x = 0.0
    spawn_y = 3.0
    spawn_z = 0.0

    # ArduPilot Gazebo Harmonic plugin (github.com/ArduPilot/ardupilot_gazebo, built in-tree)
    ardupilot_gazebo_dir = os.environ.get(
        'ARDUPILOT_GAZEBO_DIR',
        os.path.join(os.environ.get('HOME', ''), 'projects', 'ardupilot_gazebo_gz'))

    # Runway world with the iris (camera variant); the Stewart model is spawned below.
    world_file = os.path.join(package_share, 'world', 'iris_stewart_runway.sdf')

    # Stewart SDF and plugin path as before
    sdf_file = os.path.join(package_share, 'stewart_sdf_model', 'stewart_fractal_gz.sdf')
    plugin_dir = os.path.join(package_prefix, 'lib')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_xy_motion = LaunchConfiguration('enable_xy_motion')
    xy_command_rate = LaunchConfiguration('xy_command_rate')
    xy_x_amplitude = LaunchConfiguration('xy_x_amplitude')
    xy_y_amplitude = LaunchConfiguration('xy_y_amplitude')
    xy_frequency = LaunchConfiguration('xy_frequency')
    xy_phase_y = LaunchConfiguration('xy_phase_y')

    # Environment setup for Gazebo
    set_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        _append_env_path(
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            plugin_dir, os.path.join(ardupilot_gazebo_dir, 'build')))

    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        _append_env_path(
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            package_share,
            os.path.join(ardupilot_gazebo_dir, 'models'),
            os.path.join(ardupilot_gazebo_dir, 'worlds')))

    # Explicitly spawn the Stewart platform model from SDF
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'iris_runway',
            '-name', 'stewart',
            '-file', sdf_file,
            '-x', f'{spawn_x}', '-y', f'{spawn_y}', '-z', f'{spawn_z}'
        ],
        output='screen',
    )

    # Gazebo Classic published /clock and the camera through gazebo_ros; bridge them now.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/iris/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
    )

    # Images through image_bridge: parameter_bridge only sustains ~20 Hz at 1280x720.
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='gz_image_bridge',
        arguments=['/iris/front_camera/image_raw'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
    )

    # Inverse Kinematics Node
    ik_node = Node(
        package='stewart_platform',
        executable='ik',
        name='inverse_kinematic',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Platfor pose controller Node
    platform_pose_pub_node = Node(
        package='stewart_platform',
        executable='controller_platform_ros2.py',
        name='platform_pose_controller',
        output='screen',
        parameters=[
            {'use_csv': True},
            {'csv_path': '/home/rycker/projects/ros2_ws/src/stewart_platform_learning/src/stewart_platform/D1H3_heave.csv'},
            {'command_rate': 20.0},
        ],
    )

    # Optional Gazebo-only XY motion for the full Stewart base model.
    # This path bypasses IK and publishes directly to /stewart/base_pose_cmd.
    base_xy_motion_node = Node(
        package='stewart_platform',
        executable='base_xy_motion_ros2.py',
        name='base_xy_motion',
        output='screen',
        condition=IfCondition(enable_xy_motion),
        parameters=[
            {'enabled': True},
            {'command_rate': xy_command_rate},
            {'x_center': spawn_x},
            {'y_center': spawn_y},
            {'z': spawn_z},
            {'x_amplitude': xy_x_amplitude},
            {'y_amplitude': xy_y_amplitude},
            {'frequency': xy_frequency},
            {'phase_y': xy_phase_y},
            {'roll': 0.0},
            {'pitch': 0.0},
            {'yaw': 0.0},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('enable_xy_motion', default_value='false'),
        DeclareLaunchArgument('xy_command_rate', default_value='50.0'),
        DeclareLaunchArgument('xy_x_amplitude', default_value='0.5'),
        DeclareLaunchArgument('xy_y_amplitude', default_value='0.5'),
        DeclareLaunchArgument('xy_frequency', default_value='0.1'),
        DeclareLaunchArgument('xy_phase_y', default_value='1.57079632679'),

        set_plugin_path,
        set_resource_path,

        OpaqueFunction(function=_gazebo),
        spawn_entity,
        bridge,
        image_bridge,
        ik_node,
        platform_pose_pub_node,
        base_xy_motion_node,
    ])
