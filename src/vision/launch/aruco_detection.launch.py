from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node caméra avec haute résolution
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[
                {'width': 1920},
                {'height': 1080},
                {'format': 'RGB888'}
            ],
            output='screen'
        ),
        
        # Node détecteur ArUco
        Node(
            package='vision',
            executable='aruco_detector_node',
            name='aruco_detector',
            parameters=[
                {'confidence_threshold': 0.85},
                {'publish_annotated_image': True},
                {'camera_topic': '/camera/image_raw'}
            ],
            output='screen'
        ),
    ])