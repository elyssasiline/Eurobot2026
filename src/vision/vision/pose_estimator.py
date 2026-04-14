"""
vision/vision/pose_estimator.py

Logique pure de calcul de pose 3D depuis corners ArUco.
Aucune dépendance ROS — testable avec un simple python3.
"""
import numpy as np
import cv2


class PoseEstimator:

    def __init__(self, calibration_file: str, marker_size_m: float = 0.04):
        data = np.load(calibration_file)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs   = data['dist_coeffs']
        self.marker_size   = marker_size_m

    def estimate(self, corners: np.ndarray) -> dict:
        """
        corners : np.array (4, 2) — coins du tag en pixels

        Retourne :
          distance_m  — distance caméra ↔ tag en mètres
          tvec        — (x, y, z) en mètres
          rvec        — vecteur rotation Rodrigues
          angle_deg   — angle horizontal par rapport à l'axe optique
        """
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corners], self.marker_size,
            self.camera_matrix, self.dist_coeffs
        )
        tvec = tvecs[0][0]
        rvec = rvecs[0][0]

        return {
            'distance_m': float(np.linalg.norm(tvec)),
            'tvec':       tvec,
            'rvec':       rvec,
            'angle_deg':  float(np.degrees(np.arctan2(tvec[0], tvec[2]))),
        }

    def project_box_corners(
        self,
        rvec: np.ndarray,
        tvec: np.ndarray,
        box_w_m: float = 0.15,
        box_h_m: float = 0.05,
    ) -> np.ndarray:
        """
        Projette le contour 3D de la face de la caisse dans l'image.
        box_w_m : 150mm, box_h_m : 50mm
        Retourne np.array (4, 2) en pixels.
        """
        hw = box_w_m / 2
        hh = box_h_m / 2
        box_3d = np.float32([
            [-hw, -hh, 0], [ hw, -hh, 0],
            [ hw,  hh, 0], [-hw,  hh, 0],
        ])
        pts_2d, _ = cv2.projectPoints(
            box_3d, rvec, tvec,
            self.camera_matrix, self.dist_coeffs
        )
        return pts_2d.reshape(-1, 2).astype(int)