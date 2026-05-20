"""
robot_bringup/launch/robot.launch.py

Lance l'ensemble du robot :
  - Micro-ROS agent Teensy moteurs    (/dev/ttyACM0) ✅ testée
  - Micro-ROS agent Teensy IR + pince (/dev/ttyACM1) ⚠️  pas encore testée
  - RPLidar + obstacle avoidance      (/dev/ttyUSB0)
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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ----------------------------------------------------------
    # Chemins
    # ----------------------------------------------------------
    bringup_dir = get_package_share_directory('robot_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'robot_params.yaml')

    nav_dir    = get_package_share_directory('navigation')
    vision_dir = get_package_share_directory('vision')

    # ----------------------------------------------------------
    # Argument team
    # ----------------------------------------------------------
    team_arg = DeclareLaunchArgument(
        'team',
        default_value='blue',
        description="Couleur équipe du jour : 'blue' ou 'yellow'"
    )

    # ----------------------------------------------------------
    # Micro-ROS agent — Teensy moteurs (/dev/ttyACM0) ✅
    # ----------------------------------------------------------
    micro_ros_agent_motors = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_motors',
        arguments=['serial', '--dev', '/dev/ttyACM1', '--baudrate', '115200'],
        output='screen'
    )

    # ----------------------------------------------------------
    # Micro-ROS agent — Teensy IR + pince (/dev/ttyACM1) ⚠️
    # ----------------------------------------------------------
    micro_ros_agent_ir_gripper = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_ir_gripper',
        arguments=['serial', '--dev', '/dev/ttyACM0', '--baudrate', '115200'],
        output='screen'
    )

    # ----------------------------------------------------------
    # Sous-launch navigation (RPLidar /dev/ttyUSB0 + obstacle avoidance)
    # ----------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'lidar_avoidance.launch.py')
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
    # Démarre 3s après les agents micro-ROS pour laisser le temps
    # aux Teensys de s'enregistrer
    # ----------------------------------------------------------
    strategy_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='strategy',
                executable='mission_node',
                name='mission',
                parameters=[
                    config_file,
                    {'robot.team': LaunchConfiguration('team')},
                ],
                output='screen'
            )
        ]
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
        micro_ros_agent_motors,      # Teensy moteurs   — ACM0 ✅
        micro_ros_agent_ir_gripper,  # Teensy IR+pince  — ACM1 ⚠️
        navigation_launch,           # RPLidar + obstacle avoidance
        vision_launch,               # Caméra + ArUco
        # strategy_node,               # State machine (délai 3s), commenté pour démo
        # communication_node,          # WiFi PAMIs
    ])