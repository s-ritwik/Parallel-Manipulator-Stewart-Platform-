# Kept for backwards compatibility: the old file was an unported ROS 1 launch
# (gazebo_ros spawn_model). Same as onlystewart.launch.py.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('stewart_platform'), 'launch', 'onlystewart.launch.py')))
    ])
