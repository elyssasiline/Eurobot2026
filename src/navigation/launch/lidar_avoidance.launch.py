"""
navigation/launch/lidar.launch.py

Lance le RPLidar + le node d'évitement d'obstacles.
Les paramètres viennent du robot_params.yaml de robot_bringup.

Usage standalone :
  ros2 launch navigation lidar.launch.py
Usage depuis robot_bringup (config injecté automatiquement) :
  ros2 launch robot_bringup robot.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ----------------------------------------------------------
    # Config : priorité à l'argument injecté par robot_bringup,
    # fallback sur le YAML local de navigation si lancé seul
    # ----------------------------------------------------------
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

        # RPLidar A1 — paramètres lus depuis le YAML (section navigation)
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[
                config,
                {'robot.team': team},
            ],
            output='screen'
        ),

        # Obstacle avoidance
        Node(
            package='navigation',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance',
            parameters=[
                config,
                {'robot.team': team},
            ],
            output='screen'
        ),
    ])