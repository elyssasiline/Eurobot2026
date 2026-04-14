"""
vision/vision/box_detector_node.py

Node ROS2 — estimation de pose 3D des caisses.
Tourne en parallèle de aruco_detector_node.

Tag ArUco    : 40mm  (MARKER_SIZE_M = 0.04)
Caisse       : 150x50x30mm — face portant le tag = 150x50mm

Souscrit :
  /aruco/corners      (std_msgs/Float32MultiArray)
  /aruco/detections   (vision_msgs/Detection2DArray)
  /camera/image_raw   (sensor_msgs/Image)

Publie :
  /vision/box_poses              (geometry_msgs/PoseArray)
  /vision/nearest_box_distance   (std_msgs/Float32)
  /vision/image_poses            (sensor_msgs/Image) si publish_annotated_image=true
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

from vision.pose_estimator import PoseEstimator

# Dimensions réelles
MARKER_SIZE_M = 0.04   # tag ArUco 40mm
BOX_W_M       = 0.15   # caisse 150mm
BOX_H_M       = 0.05   # caisse 50mm


class BoxDetectorNode(Node):

    def __init__(self):
        super().__init__('box_detector')

        # ----------------------------------------------------------------
        # Paramètres
        # ----------------------------------------------------------------
        self.declare_parameter(
            'vision.calibration_file',
            '/workspace/src/vision/config/calibration.npz',
        )
        self.declare_parameter('vision.publish_annotated_image', True)

        calib_file         = self.get_parameter('vision.calibration_file').value
        self.pub_annotated = self.get_parameter('vision.publish_annotated_image').value

        if not os.path.exists(calib_file):
            self.get_logger().error(f'Calibration introuvable : {calib_file}')
            self.get_logger().error("Lance tools/calibrate.py d'abord !")
            raise FileNotFoundError(calib_file)

        self.estimator = PoseEstimator(calib_file, marker_size_m=MARKER_SIZE_M)
        self.bridge    = CvBridge()

        # État partagé entre callbacks
        self.latest_frame   = None
        self.latest_corners = []
        self.latest_colors  = []

        # ----------------------------------------------------------------
        # Subscribers
        # ----------------------------------------------------------------
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10
        )
        self.create_subscription(
            Float32MultiArray, '/aruco/corners', self._corners_cb, 10
        )
        self.create_subscription(
            Detection2DArray, '/aruco/detections', self._detection_cb, 10
        )

        # ----------------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------------
        self.pose_pub = self.create_publisher(PoseArray, '/vision/box_poses', 10)
        self.dist_pub = self.create_publisher(Float32,   '/vision/nearest_box_distance', 10)
        if self.pub_annotated:
            self.annotated_pub = self.create_publisher(Image, '/vision/image_poses', 10)

        self.get_logger().info('✓ BoxDetector démarré')
        self.get_logger().info(f'  • Tag size  : {MARKER_SIZE_M*1000:.0f} mm')
        self.get_logger().info(f'  • Box size  : {BOX_W_M*1000:.0f}x{BOX_H_M*1000:.0f} mm')

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------

    def _image_cb(self, msg: Image):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _corners_cb(self, msg: Float32MultiArray):
        corners = []
        data = msg.data
        for i in range(0, len(data), 8):
            c = np.array(data[i:i+8], dtype=np.float32).reshape(4, 2)
            corners.append(c)
        self.latest_corners = corners

    def _detection_cb(self, msg: Detection2DArray):
        if not self.latest_corners:
            return

        pose_array = PoseArray()
        pose_array.header = msg.header
        distances  = []
        annotated  = self.latest_frame.copy() if self.latest_frame is not None else None

        for i, det in enumerate(msg.detections):
            if i >= len(self.latest_corners):
                break

            corners    = self.latest_corners[i]
            color_name = det.results[0].hypothesis.class_id
            result     = self.estimator.estimate(corners)

            # Pose
            p = Pose()
            p.position.x = float(result['tvec'][0])
            p.position.y = float(result['tvec'][1])
            p.position.z = float(result['tvec'][2])
            pose_array.poses.append(p)
            distances.append(result['distance_m'])

            self.get_logger().info(
                f"{color_name} — "
                f"dist={result['distance_m']*100:.1f}cm  "
                f"angle={result['angle_deg']:+.1f}°  "
                f"tvec=({result['tvec'][0]*100:.1f}, "
                f"{result['tvec'][1]*100:.1f}, "
                f"{result['tvec'][2]*100:.1f}) cm"
            )

            # Overlay
            if annotated is not None:
                cx = int(det.bbox.center.position.x)
                cy = int(det.bbox.center.position.y)

                # Axes XYZ
                cv2.drawFrameAxes(
                    annotated,
                    self.estimator.camera_matrix,
                    self.estimator.dist_coeffs,
                    result['rvec'], result['tvec'], 0.03
                )

                # Contour boîte 150x50mm
                box_pts = self.estimator.project_box_corners(
                    result['rvec'], result['tvec'],
                    box_w_m=BOX_W_M, box_h_m=BOX_H_M
                )
                cv2.polylines(annotated, [box_pts], True, (0, 255, 0), 2)

                # Texte
                label = f"{color_name} {result['distance_m']*100:.1f}cm {result['angle_deg']:+.1f}deg"
                cv2.putText(annotated, label, (cx, cy - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        self.pose_pub.publish(pose_array)

        if distances:
            d_msg      = Float32()
            d_msg.data = float(min(distances))
            self.dist_pub.publish(d_msg)

        if annotated is not None and self.pub_annotated:
            self.annotated_pub.publish(
                self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            )


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()