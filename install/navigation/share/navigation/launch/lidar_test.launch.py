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
                'scan_mode': 'Standard',
                'serial_baudrate': 115200,
            }],
            output='screen'
        ),
        
        # Node de TEST (au lieu de obstacle_avoidance)
        Node(
            package='navigation',
            executable='lidar_test',
            name='lidar_test',
            output='screen'
        ),
    ])