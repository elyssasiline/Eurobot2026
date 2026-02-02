#!/usr/bin/env python3
"""
Node ROS2 pour la détection de marqueurs ArUco personnalisés
Publie les détections sur des topics ROS2
Version utilisant OpenCV/v4l2 pour la capture (compatible Docker)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

# DÉFINITION DES PATTERNS PERSONNALISÉS (grille 6x6)
CUSTOM_PATTERNS = {
    "JAUNE": np.array([
        [1, 1, 1, 1, 1, 1],
        [1, 1, 0, 1, 1, 1],
        [1, 0, 1, 0, 0, 1],
        [1, 1, 0, 0, 1, 1],
        [1, 1, 0, 1, 1, 1],
        [1, 1, 1, 1, 1, 1]
    ], dtype=np.uint8),
    
    "BLEU": np.array([
        [1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1],
        [1, 0, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 1],
        [1, 1, 0, 1, 0, 1],
        [1, 1, 1, 1, 1, 1]
    ], dtype=np.uint8),
    
    "NOIR": np.array([
        [1, 1, 1, 1, 1, 1],
        [1, 1, 1, 0, 1, 1],
        [1, 0, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1],
        [1, 0, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1]
    ], dtype=np.uint8)
}

PATTERN_COLORS = {
    "JAUNE": (0, 215, 255),
    "BLEU": (255, 100, 0),
    "NOIR": (128, 128, 128)
}


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Paramètres
        self.declare_parameter('camera_index', -1)  # -1 = auto-détection
        self.declare_parameter('camera_width', 1920)
        self.declare_parameter('camera_height', 1080)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('confidence_threshold', 0.85)
        self.declare_parameter('publish_annotated_image', True)
        
        # Récupérer les paramètres
        camera_index = self.get_parameter('camera_index').value
        camera_width = self.get_parameter('camera_width').value
        camera_height = self.get_parameter('camera_height').value
        camera_fps = self.get_parameter('camera_fps').value
        publish_rate = self.get_parameter('publish_rate').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, 
            'aruco/detections', 
            10
        )
        
        self.raw_image_pub = self.create_publisher(
            Image,
            'aruco/image_raw',
            10
        )
        
        if self.publish_annotated:
            self.annotated_image_pub = self.create_publisher(
                Image,
                'aruco/image_annotated',
                10
            )
        
        # Bridge OpenCV <-> ROS2
        self.bridge = CvBridge()
        
        # Initialiser ArUco
        self.get_logger().info('Initialisation du détecteur ArUco...')
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Initialiser la caméra avec OpenCV/v4l2
        self.get_logger().info(f'Initialisation de la caméra /dev/video{camera_index}...')
        
        # Si camera_index = -1, auto-détection
        if camera_index == -1:
            self.get_logger().info('Mode auto-détection activé, recherche du bon device...')
            camera_index = self._find_working_camera()
            self.get_logger().info(f'Caméra trouvée: /dev/video{camera_index}')
        
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Impossible d\'ouvrir /dev/video{camera_index}!')
            raise RuntimeError(f'Camera /dev/video{camera_index} not available')
        
        # Configurer la caméra
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
        self.cap.set(cv2.CAP_PROP_FPS, camera_fps)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        # Vérifier la résolution effective
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.get_logger().info(f'Résolution caméra: {actual_width}x{actual_height} @ {actual_fps} FPS')
        
        # Laisser la caméra s'initialiser
        time.sleep(1)
        
        # Capturer quelques frames de warmup
        for _ in range(5):
            self.cap.read()
        
        # Timer pour la capture et détection
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.detect_and_publish)
        
        self.frame_count = 0
        self.detection_count = 0
        
        self.get_logger().info('✓ Node ArUco démarré!')
        self.get_logger().info(f'  • Device: /dev/video{camera_index}')
        self.get_logger().info(f'  • Résolution: {actual_width}x{actual_height}')
        self.get_logger().info(f'  • Fréquence: {publish_rate} Hz')
        self.get_logger().info(f'  • Seuil confiance: {self.confidence_threshold}')
    
    def _find_working_camera(self):
        """Trouve automatiquement un device caméra fonctionnel"""
        # Sur RPi5, essayer d'abord les devices courants
        test_devices = [0, 1, 2, 3, 4, 5, 6, 7, 10, 11]
        
        for device_id in test_devices:
            try:
                test_cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
                if test_cap.isOpened():
                    # Essayer de capturer une frame
                    ret, frame = test_cap.read()
                    test_cap.release()
                    
                    if ret and frame is not None and frame.size > 0:
                        self.get_logger().info(f'  ✓ /dev/video{device_id} fonctionne!')
                        return device_id
                    else:
                        self.get_logger().debug(f'  ✗ /dev/video{device_id} s\'ouvre mais ne capture pas')
                else:
                    test_cap.release()
            except Exception as e:
                self.get_logger().debug(f'  ✗ /dev/video{device_id}: {e}')
        
        raise RuntimeError('Aucune caméra fonctionnelle trouvée!')
    
    def extract_marker_pattern(self, gray_img, corners):
        """Extrait le pattern d'un marqueur ArUco"""
        pts = corners.reshape(4, 2)
        dst_size = 60
        dst_pts = np.array([
            [0, 0],
            [dst_size, 0],
            [dst_size, dst_size],
            [0, dst_size]
        ], dtype=np.float32)
        
        matrix = cv2.getPerspectiveTransform(pts.astype(np.float32), dst_pts)
        warped = cv2.warpPerspective(gray_img, matrix, (dst_size, dst_size))
        _, binary = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        cell_size = dst_size // 6
        pattern = np.zeros((6, 6), dtype=np.uint8)
        
        for i in range(6):
            for j in range(6):
                y = i * cell_size + cell_size // 2
                x = j * cell_size + cell_size // 2
                sample_region = binary[
                    max(0, y-2):min(dst_size, y+3),
                    max(0, x-2):min(dst_size, x+3)
                ]
                pattern[i, j] = 1 if np.mean(sample_region) < 128 else 0
        
        return pattern
    
    def match_pattern(self, pattern):
        """Compare le pattern avec les patterns personnalisés"""
        best_match = None
        best_score = 0
        best_color = None
        
        for color_name, ref_pattern in CUSTOM_PATTERNS.items():
            for rotation in range(4):
                rotated = np.rot90(pattern, rotation)
                matches = np.sum(rotated == ref_pattern)
                score = matches / 36.0
                
                if score > best_score:
                    best_score = score
                    best_match = color_name
                    best_color = PATTERN_COLORS[color_name]
        
        if best_score >= self.confidence_threshold:
            return best_match, best_color, best_score
        
        return None, None, best_score
    
    def detect_custom_aruco(self, frame):
        """Détecte les marqueurs ArUco personnalisés"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        
        detected_markers = []
        
        if corners is not None and len(corners) > 0:
            for i, corner in enumerate(corners):
                pattern = self.extract_marker_pattern(gray, corner[0])
                color_name, color_bgr, confidence = self.match_pattern(pattern)
                
                if color_name:
                    detected_markers.append({
                        'corners': corner[0],
                        'color_name': color_name,
                        'color_bgr': color_bgr,
                        'confidence': confidence
                    })
        
        return detected_markers
    
    def draw_markers_info(self, frame, detected_markers):
        """Dessine les marqueurs détectés"""
        frame_annotated = frame.copy()
        
        for marker in detected_markers:
            corner = marker['corners']
            color_name = marker['color_name']
            color_bgr = marker['color_bgr']
            confidence = marker['confidence']
            
            center_x = int(np.mean(corner[:, 0]))
            center_y = int(np.mean(corner[:, 1]))
            
            # Contour
            pts = corner.astype(np.int32)
            cv2.polylines(frame_annotated, [pts], True, color_bgr, 3)
            
            # Coins
            for point in pts:
                cv2.circle(frame_annotated, tuple(point), 8, color_bgr, -1)
            
            # Centre
            cv2.circle(frame_annotated, (center_x, center_y), 10, color_bgr, -1)
            cv2.circle(frame_annotated, (center_x, center_y), 12, (255, 255, 255), 2)
            
            # Texte
            text = f"{color_name} {confidence*100:.0f}%"
            cv2.putText(frame_annotated, text,
                       (center_x - 80, center_y - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
        
        return frame_annotated
    
    def create_detection_msg(self, detected_markers):
        """Crée un message Detection2DArray pour ROS2"""
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        
        for marker in detected_markers:
            detection = Detection2D()
            
            # Centre du marqueur
            corner = marker['corners']
            center_x = float(np.mean(corner[:, 0]))
            center_y = float(np.mean(corner[:, 1]))
            
            detection.bbox.center.position.x = center_x
            detection.bbox.center.position.y = center_y
            
            # Taille approximative
            width = float(np.linalg.norm(corner[0] - corner[1]))
            height = float(np.linalg.norm(corner[1] - corner[2]))
            detection.bbox.size_x = width
            detection.bbox.size_y = height
            
            # Hypothèse (couleur détectée)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = marker['color_name']
            hypothesis.hypothesis.score = marker['confidence']
            detection.results.append(hypothesis)
            
            msg.detections.append(detection)
        
        return msg
    
    def detect_and_publish(self):
        """Callback du timer - capture, détecte et publie"""
        try:
            # Capturer l'image
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warning('Échec de capture d\'image')
                return
            
            self.frame_count += 1
            
            # Détecter les marqueurs
            detected_markers = self.detect_custom_aruco(frame)
            
            # Convertir BGR (OpenCV) vers RGB pour ROS
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Publier l'image brute
            ros_image = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            self.raw_image_pub.publish(ros_image)
            
            # Publier les détections
            if detected_markers:
                detection_msg = self.create_detection_msg(detected_markers)
                self.detection_pub.publish(detection_msg)
                
                self.detection_count += 1
                
                # Log des détections
                markers_str = ', '.join([f"{m['color_name']}({m['confidence']*100:.0f}%)" 
                                        for m in detected_markers])
                self.get_logger().info(f"Détection #{self.detection_count}: {markers_str}")
                
                # Publier l'image annotée
                if self.publish_annotated:
                    frame_annotated = self.draw_markers_info(frame, detected_markers)
                    annotated_msg = self.bridge.cv2_to_imgmsg(
                        frame_annotated, 
                        encoding='bgr8'
                    )
                    annotated_msg.header.stamp = ros_image.header.stamp
                    annotated_msg.header.frame_id = 'camera_frame'
                    self.annotated_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Erreur lors de la détection: {e}')
    
    def destroy_node(self):
        """Nettoyage à la destruction du node"""
        self.get_logger().info('Arrêt du node ArUco...')
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()