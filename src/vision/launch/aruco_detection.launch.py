"""
vision/launch/aruco_detection.launch.py

Lance la caméra + le détecteur ArUco.
Les paramètres viennent du robot_params.yaml de robot_bringup.

Usage standalone :
  ros2 launch vision aruco_detection.launch.py
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

        # Node caméra
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[
                config,
                {'robot.team': team},
            ],
            output='screen'
        ),

        # Node détecteur ArUco
        Node(
            package='vision',
            executable='aruco_detector_node',
            name='aruco_detector',
            parameters=[
                config,
                {'robot.team': team},
            ],
            output='screen'
        ),
    ])