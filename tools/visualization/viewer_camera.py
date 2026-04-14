"""
tools/viewer.py

Viewer temps réel — souscrit aux topics ROS2 et affiche une fenêtre OpenCV.
Lance APRÈS aruco_detector_node et box_detector_node.

Usage :
    python3 tools/viewer.py

Topics attendus :
  /camera/image_raw              (sensor_msgs/Image)
  /aruco/detections              (vision_msgs/Detection2DArray)
  /aruco/corners                 (std_msgs/Float32MultiArray)
  /vision/box_poses              (geometry_msgs/PoseArray)
  /vision/nearest_box_distance   (std_msgs/Float32)

Quitter : touche Q ou Echap
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import os
import sys

CALIBRATION_FILE = 'src/vision/config/calibration.npz'

# Tag ArUco : 40mm
MARKER_SIZE_M = 0.04

# Dimensions réelles caisse : 150x50mm
BOX_W_M = 0.15
BOX_H_M = 0.05

COLORS = {
    'JAUNE': (0,   215, 255),
    'BLEU':  (255, 100,   0),
}
COLOR_DEFAULT = (200, 200, 200)


class ViewerNode(Node):

    def __init__(self, camera_matrix=None, dist_coeffs=None):
        super().__init__('vision_viewer')

        self.camera_matrix = camera_matrix
        self.dist_coeffs   = dist_coeffs
        self.has_calib     = camera_matrix is not None
        self.bridge        = CvBridge()

        self._lock             = threading.Lock()
        self.latest_frame      = None
        self.latest_detections = []
        self.latest_corners    = []
        self.latest_poses      = []
        self.nearest_distance  = None

        self.create_subscription(Image,             '/camera/image_raw',            self._image_cb,     10)
        self.create_subscription(Detection2DArray,  '/aruco/detections',            self._detection_cb, 10)
        self.create_subscription(Float32MultiArray, '/aruco/corners',               self._corners_cb,   10)
        self.create_subscription(PoseArray,         '/vision/box_poses',            self._poses_cb,     10)
        self.create_subscription(Float32,           '/vision/nearest_box_distance', self._dist_cb,      10)

        self.get_logger().info('✓ Viewer démarré — Q ou Echap pour quitter')
        if not self.has_calib:
            self.get_logger().warn('Pas de calibration.npz — overlay 3D désactivé')

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------

    def _image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        with self._lock:
            self.latest_frame = frame

    def _detection_cb(self, msg):
        detections = []
        for det in msg.detections:
            detections.append({
                'color':      det.results[0].hypothesis.class_id,
                'confidence': det.results[0].hypothesis.score,
                'cx':         det.bbox.center.position.x,
                'cy':         det.bbox.center.position.y,
            })
        with self._lock:
            self.latest_detections = detections

    def _corners_cb(self, msg):
        corners = []
        data = msg.data
        for i in range(0, len(data), 8):
            c = np.array(data[i:i+8], dtype=np.float32).reshape(4, 2)
            corners.append(c)
        with self._lock:
            self.latest_corners = corners

    def _poses_cb(self, msg):
        poses = []
        for p in msg.poses:
            tvec = np.array([p.position.x, p.position.y, p.position.z], dtype=np.float32)
            poses.append({'tvec': tvec})
        with self._lock:
            self.latest_poses = poses

    def _dist_cb(self, msg):
        with self._lock:
            self.nearest_distance = msg.data

    # ----------------------------------------------------------------
    # Rendu
    # ----------------------------------------------------------------

    def render(self):
        with self._lock:
            if self.latest_frame is None:
                return None
            frame      = self.latest_frame.copy()
            detections = list(self.latest_detections)
            corners    = list(self.latest_corners)
            poses      = list(self.latest_poses)
            nearest    = self.nearest_distance

        h, w = frame.shape[:2]

        for i, det in enumerate(detections):
            color_name = det['color']
            col        = COLORS.get(color_name, COLOR_DEFAULT)
            cx, cy     = int(det['cx']), int(det['cy'])

            # ---- Contour du tag ----
            if i < len(corners):
                pts = corners[i].astype(np.int32)
                cv2.polylines(frame, [pts], True, col, 3)
                for pt in pts:
                    cv2.circle(frame, tuple(pt), 6, col, -1)

            # ---- Centre ----
            cv2.circle(frame, (cx, cy), 10, col, -1)
            cv2.circle(frame, (cx, cy), 12, (255, 255, 255), 2)

            # ---- Overlay 3D ----
            tvec, rvec = None, None
            if self.has_calib and i < len(corners):
                try:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], MARKER_SIZE_M,
                        self.camera_matrix, self.dist_coeffs
                    )
                    rvec = rvecs[0][0]
                    tvec = tvecs[0][0]

                    # Axes XYZ
                    cv2.drawFrameAxes(
                        frame, self.camera_matrix, self.dist_coeffs,
                        rvec, tvec, 0.03
                    )

                    # Contour 3D boîte 150x50mm
                    hw = BOX_W_M / 2
                    hh = BOX_H_M / 2
                    box_3d = np.float32([
                        [-hw, -hh, 0], [ hw, -hh, 0],
                        [ hw,  hh, 0], [-hw,  hh, 0],
                    ])
                    pts_2d, _ = cv2.projectPoints(
                        box_3d, rvec, tvec,
                        self.camera_matrix, self.dist_coeffs
                    )
                    pts_2d = pts_2d.reshape(-1, 2).astype(int)
                    cv2.polylines(frame, [pts_2d], True, col, 2)

                except Exception:
                    pass

            # ---- Infos texte ----
            lines = [f"{color_name}  {det['confidence']*100:.0f}%"]

            if tvec is not None:
                distance_m = float(np.linalg.norm(tvec))
                angle_deg  = float(np.degrees(np.arctan2(tvec[0], tvec[2])))
                lines.append(f"dist  {distance_m * 100:.1f} cm")
                lines.append(f"angle {angle_deg:+.1f} deg")
            elif i < len(poses):
                # Fallback si pas de calibration mais poses dispo
                distance_m = float(np.linalg.norm(poses[i]['tvec']))
                lines.append(f"dist  {distance_m * 100:.1f} cm")

            text_x = cx + 16
            text_y = cy - 10
            for j, line in enumerate(lines):
                y = text_y + j * 26
                cv2.putText(frame, line, (text_x + 1, y + 1),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3)
                cv2.putText(frame, line, (text_x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)

        # ---- Bandeau bas ----
        bar_h   = 36
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, h - bar_h), (w, h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        info = f"Tags detectes : {len(detections)}"
        if nearest is not None:
            info += f"   |   Plus proche : {nearest * 100:.1f} cm"
        if not self.has_calib:
            info += "   |   [overlay 3D desactive — pas de calibration]"

        cv2.putText(frame, info, (12, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (220, 220, 220), 2)

        return frame


def main():
    rclpy.init()

    camera_matrix, dist_coeffs = None, None
    if os.path.exists(CALIBRATION_FILE):
        data          = np.load(CALIBRATION_FILE)
        camera_matrix = data['camera_matrix']
        dist_coeffs   = data['dist_coeffs']
        print(f'Calibration chargée : {CALIBRATION_FILE}')
    else:
        print(f'[WARN] Calibration absente ({CALIBRATION_FILE}) — overlay 3D désactivé')

    node     = ViewerNode(camera_matrix, dist_coeffs)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print('En attente du flux caméra...')
    cv2.namedWindow('Vision Robot', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Vision Robot', 1280, 720)

    try:
        while rclpy.ok():
            frame = node.render()

            if frame is None:
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, 'En attente du flux camera...',
                            (80, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (150, 150, 150), 2)
                cv2.imshow('Vision Robot', placeholder)
            else:
                cv2.imshow('Vision Robot', frame)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()