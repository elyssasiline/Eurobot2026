#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test')
        self.bridge = CvBridge()
        self.photo_taken = False
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('En attente d\'une image...')
    
    def image_callback(self, msg):
        if not self.photo_taken:
            try:
                # Convertir en image OpenCV
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
                # Sauvegarder
                filename = '/workspace/test_photo.jpg'
                cv2.imwrite(filename, frame)
                
                self.get_logger().info(f'Photo sauvegardée: {filename}')
                self.get_logger().info(f'Taille: {frame.shape}')
                self.photo_taken = True
                
            except Exception as e:
                self.get_logger().error(f'Erreur: {e}')

def main():
    rclpy.init()
    node = CameraTest()
    
    # Attendre 1 photo puis quitter
    while not node.photo_taken and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()
    print('\n✓ Photo capturée! Fichier: /workspace/test_photo.jpg')

if __name__ == '__main__':
    main()
