#!/usr/bin/env python3
"""
vision/vision/aruco_detector_node.py

Node de détection ArUco personnalisés (patterns 6x6).
Tous les paramètres viennent de robot_params.yaml.

La logique couleur équipe est appliquée ici :
  team=blue   → les caisses JAUNE sont à retourner
  team=yellow → les caisses BLEUE sont à retourner

Topics abonnés :
  /camera/image_raw  (sensor_msgs/Image)

Topics publiés :
  /aruco/detections       (vision_msgs/Detection2DArray)
  /aruco/corners          (std_msgs/Float32MultiArray)
  /aruco/image_annotated  (sensor_msgs/Image)  si publish_annotated_image=true
  /aruco/box_to_flip      (std_msgs/String)    — 'FLIP' ou 'KEEP' selon équipe
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


# ----------------------------------------------------------
# Patterns ArUco personnalisés (grille 6x6)
# ----------------------------------------------------------
CUSTOM_PATTERNS = {
    'JAUNE': np.array([
        [1, 1, 1, 1, 1, 1],
        [1, 1, 0, 1, 1, 1],
        [1, 0, 1, 0, 0, 1],
        [1, 1, 0, 0, 1, 1],
        [1, 1, 0, 1, 1, 1],
        [1, 1, 1, 1, 1, 1]
    ], dtype=np.uint8),

    'BLEU': np.array([
        [1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1],
        [1, 0, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 1],
        [1, 1, 0, 1, 0, 1],
        [1, 1, 1, 1, 1, 1]
    ], dtype=np.uint8),

    'NOIR': np.array([
        [1, 1, 1, 1, 1, 1],
        [1, 1, 1, 0, 1, 1],
        [1, 0, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1],
        [1, 0, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1]
    ], dtype=np.uint8),
}

PATTERN_COLORS = {
    'JAUNE': (0, 215, 255),
    'BLEU':  (255, 100,   0),
    'NOIR':  (128, 128, 128),
}

FLIP_COLOR = {
    'blue':   'JAUNE',
    'yellow': 'BLEU',
}


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('robot.team', 'blue')
        self.declare_parameter('vision.confidence_threshold', 0.85)
        self.declare_parameter('vision.publish_annotated_image', True)
        self.declare_parameter('vision.camera_topic', '/camera/image_raw')

        self.team                 = self.get_parameter('robot.team').value
        self.confidence_threshold = self.get_parameter('vision.confidence_threshold').value
        self.publish_annotated    = self.get_parameter('vision.publish_annotated_image').value
        camera_topic              = self.get_parameter('vision.camera_topic').value

        self.flip_color = FLIP_COLOR.get(self.team, 'JAUNE')

        self.detection_pub = self.create_publisher(Detection2DArray, '/aruco/detections', 10)
        self.corners_pub   = self.create_publisher(Float32MultiArray, '/aruco/corners', 10)
        self.flip_pub      = self.create_publisher(String, '/aruco/box_to_flip', 10)

        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, '/aruco/image_annotated', 10)

        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10
        )

        self.bridge = CvBridge()

        self.aruco_dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.aruco_params.adaptiveThreshWinSizeMin              = 3
        self.aruco_params.adaptiveThreshWinSizeMax              = 23
        self.aruco_params.adaptiveThreshWinSizeStep             = 10
        self.aruco_params.minMarkerPerimeterRate                = 0.02
        self.aruco_params.perspectiveRemovePixelPerCell         = 8
        self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.13
        self.frame_count     = 0
        self.detection_count = 0

        self.get_logger().info(f'✓ ArUcoDetector démarré — équipe: {self.team}')
        self.get_logger().info(f'  • Couleur à retourner: {self.flip_color}')
        self.get_logger().info(f'  • Seuil confiance: {self.confidence_threshold}')
        self.get_logger().info(f'  • Topic caméra: {camera_topic}')

    def image_callback(self, msg: Image):
        try:
            self.frame_count += 1
            frame    = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detected = self._detect(frame)

            if detected:
                self.detection_pub.publish(self._build_detection_msg(detected))

                corners_msg = Float32MultiArray()
                for marker in detected:
                    corners_msg.data.extend(marker['corners'].flatten().tolist())
                self.corners_pub.publish(corners_msg)

                for marker in detected:
                    flip_msg = String()
                    if marker['color_name'] == self.flip_color:
                        flip_msg.data = f"FLIP:{marker['color_name']}:{marker['confidence']:.2f}"
                    else:
                        flip_msg.data = f"KEEP:{marker['color_name']}:{marker['confidence']:.2f}"
                    self.flip_pub.publish(flip_msg)

                self.detection_count += 1
                markers_str = ', '.join(
                    [f"{m['color_name']}({m['confidence']*100:.0f}%)" for m in detected]
                )
                self.get_logger().info(f'Détection #{self.detection_count}: {markers_str}')

                if self.publish_annotated:
                    annotated     = self._draw_markers(frame, detected)
                    annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                    annotated_msg.header = msg.header
                    self.annotated_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f'Erreur détection: {e}')

    def _detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        detected = []
        if corners:
            for corner in corners:
                pattern    = self._extract_pattern(gray, corner[0])
                color_name, color_bgr, confidence = self._match_pattern(pattern)
                if color_name:
                    detected.append({
                        'corners':    corner[0],
                        'color_name': color_name,
                        'color_bgr':  color_bgr,
                        'confidence': confidence,
                    })
        return detected

    def _extract_pattern(self, gray, corners):
        pts     = corners.reshape(4, 2)
        dst_size = 60
        dst_pts = np.array([[0,0],[dst_size,0],[dst_size,dst_size],[0,dst_size]], dtype=np.float32)
        M       = cv2.getPerspectiveTransform(pts.astype(np.float32), dst_pts)
        warped  = cv2.warpPerspective(gray, M, (dst_size, dst_size))
        _, binary = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        cell    = dst_size // 6
        pattern = np.zeros((6, 6), dtype=np.uint8)
        for i in range(6):
            for j in range(6):
                y, x   = i * cell + cell // 2, j * cell + cell // 2
                region = binary[max(0,y-2):min(dst_size,y+3), max(0,x-2):min(dst_size,x+3)]
                pattern[i, j] = 1 if np.mean(region) < 128 else 0
        return pattern

    def _match_pattern(self, pattern):
        best_match, best_score, best_color = None, 0, None
        for color_name, ref in CUSTOM_PATTERNS.items():
            for r in range(4):
                score = np.sum(np.rot90(pattern, r) == ref) / 36.0
                if score > best_score:
                    best_score, best_match, best_color = score, color_name, PATTERN_COLORS[color_name]
        if best_score >= self.confidence_threshold:
            return best_match, best_color, best_score
        return None, None, best_score

    def _build_detection_msg(self, detected):
        msg = Detection2DArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        for marker in detected:
            det = Detection2D()
            c   = marker['corners']
            det.bbox.center.position.x = float(np.mean(c[:, 0]))
            det.bbox.center.position.y = float(np.mean(c[:, 1]))
            det.bbox.size_x            = float(np.linalg.norm(c[0] - c[1]))
            det.bbox.size_y            = float(np.linalg.norm(c[1] - c[2]))
            hyp                        = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id    = marker['color_name']
            hyp.hypothesis.score       = marker['confidence']
            det.results.append(hyp)
            msg.detections.append(det)
        return msg

    def _draw_markers(self, frame, detected):
        out = frame.copy()
        for m in detected:
            c      = m['corners'].astype(np.int32)
            cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))
            col    = m['color_bgr']
            cv2.polylines(out, [c], True, col, 3)
            for pt in c:
                cv2.circle(out, tuple(pt), 8, col, -1)
            cv2.circle(out, (cx, cy), 10, col, -1)
            cv2.circle(out, (cx, cy), 12, (255,255,255), 2)
            action = 'FLIP' if m['color_name'] == self.flip_color else 'KEEP'
            label  = f"{m['color_name']} {m['confidence']*100:.0f}% [{action}]"
            cv2.putText(out, label, (cx - 90, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        return out


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