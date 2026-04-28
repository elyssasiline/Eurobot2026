"""
strategy/launch/strategy.launch.py

Lance uniquement la state machine (utile pour tests sans le reste).
En production, elle est lancée par robot_bringup/robot.launch.py.

Usage standalone :
  ros2 launch strategy strategy.launch.py
  ros2 launch strategy strategy.launch.py team:=yellow
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    default_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'robot_params.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Chemin vers robot_params.yaml'
    )

    team_arg = DeclareLaunchArgument(
        'team',
        default_value='blue',
        description="Couleur équipe : 'blue' ou 'yellow'"
    )

    config = LaunchConfiguration('config')
    team   = LaunchConfiguration('team')

    return LaunchDescription([
        config_arg,
        team_arg,

        Node(
            package='strategy',
            executable='mission_node',
            name='mission',
            parameters=[
                config,
                {'robot.team': team},
            ],
            output='screen'
        ),
    ])