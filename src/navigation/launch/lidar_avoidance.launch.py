from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Port série du RPLidar'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame ID du lidar'
    )
    
    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        
        # Node RPLidar
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'angle_compensate': True,
                'scan_mode': 'Standard',  # ou 'Express' pour A1M8
                'serial_baudrate': 115200,
            }],
            output='screen'
        ),
        
        # Node Obstacle Avoidance
        Node(
            package='navigation',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance',
            parameters=[{
                'min_obstacle_distance': 0.3,      # 30cm
                'critical_distance': 0.15,          # 15cm
                'front_angle_range': 45.0,          # 45° devant
                'side_angle_range': 90.0,           # 90° sur les côtés
                'max_linear_speed': 0.3,            # m/s
                'max_angular_speed': 1.0,           # rad/s
                'enable_avoidance': True,
            }],
            output='screen'
        ),
    ])