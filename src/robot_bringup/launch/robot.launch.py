"""
robot_bringup/launch/robot.launch.py

Lance l'ensemble du robot :
  - RPLidar + obstacle avoidance
  - Caméra + détection ArUco
  - State machine (strategy)
  - Communication WIFI PAMIs

Usage :
  ros2 launch robot_bringup robot.launch.py
  ros2 launch robot_bringup robot.launch.py team:=yellow
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ----------------------------------------------------------
    # Chemins
    # ----------------------------------------------------------
    bringup_dir = get_package_share_directory('robot_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'robot_params.yaml')

    nav_dir = get_package_share_directory('navigation')
    vision_dir = get_package_share_directory('vision')

    # ----------------------------------------------------------
    # Argument team — peut être surchargé en ligne de commande
    # La valeur par défaut vient du YAML mais on garde l'arg
    # pour pouvoir faire : ros2 launch ... team:=yellow
    # ----------------------------------------------------------
    team_arg = DeclareLaunchArgument(
        'team',
        default_value='blue',
        description="Couleur équipe du jour : 'blue' ou 'yellow'"
    )

    # ----------------------------------------------------------
    # Sous-launch navigation (lidar + obstacle avoidance)
    # ----------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={
            'config': config_file,
            'team': LaunchConfiguration('team'),
        }.items()
    )

    # ----------------------------------------------------------
    # Sous-launch vision (caméra + ArUco)
    # ----------------------------------------------------------
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_dir, 'launch', 'aruco_detection.launch.py')
        ),
        launch_arguments={
            'config': config_file,
            'team': LaunchConfiguration('team'),
        }.items()
    )

    # ----------------------------------------------------------
    # Node stratégie / state machine
    # ----------------------------------------------------------
    strategy_node = Node(
        package='strategy',
        executable='mission_node',
        name='mission',
        parameters=[
            config_file,
            {'robot.team': LaunchConfiguration('team')},  # surcharge si arg passé
        ],
        output='screen'
    )

    # ----------------------------------------------------------
    # Node communication WIFI PAMIs
    # ----------------------------------------------------------
    communication_node = Node(
        package='communication',
        executable='wifi_bridge_node',
        name='wifi_bridge',
        parameters=[
            config_file,
            {'robot.team': LaunchConfiguration('team')},
        ],
        output='screen'
    )

    return LaunchDescription([
        team_arg,
        navigation_launch,
        vision_launch,
        strategy_node,
        communication_node,
    ])