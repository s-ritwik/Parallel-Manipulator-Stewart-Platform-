import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_model',
            output='screen',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        ),
        launch_ros.actions.Node(
            package='stewart_platform',
            executable='ik',
            name='inverse_kinematic',
            output='screen',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
