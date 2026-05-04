#!/usr/bin/env python3
"""
navigation/navigation/obstacle_avoidance_node.py

Node d'évitement d'obstacles basé sur le RPLidar A1.

Comportement :
  - Obstacle stable ou qui s'éloigne → rien, state machine garde la main
  - Obstacle qui SE RAPPROCHE → ralentissement progressif (scalaire sur cmd_vel)
  - Obstacle à < critical_distance → arrêt complet jusqu'à ce qu'il disparaisse

Logique de ralentissement :
  - Ne se déclenche que si vitesse d'approche > approach_speed_threshold (m/s)
  - Facteur linéaire entre slow_start_distance (scale=1.0) et critical_distance (scale=0.0)
  - Pondéré par la vitesse d'approche (approche rapide = ralentissement plus agressif)

Topics abonnés :
  /scan           (sensor_msgs/LaserScan)
  /cmd_vel_raw    (geometry_msgs/Twist)   — commande brute de la state machine

Topics publiés :
  /cmd_vel        (geometry_msgs/Twist)   — commande finale (scalée ou stop)
  /obstacle_alert (std_msgs/String)       — OK / SLOWING / DANGER
  /cmd_vel_scale  (std_msgs/Float32)      — facteur [0.0, 1.0]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
import math
import time


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # ----------------------------------------------------------
        # Paramètres
        # ----------------------------------------------------------
        self.declare_parameter('robot.team',                         'blue')
        self.declare_parameter('navigation.critical_distance',        0.20)
        self.declare_parameter('navigation.slow_start_distance',      0.40)
        self.declare_parameter('navigation.front_angle_range',       45.0)
        self.declare_parameter('navigation.side_angle_range',        90.0)
        self.declare_parameter('navigation.max_linear_speed',         0.30)
        self.declare_parameter('navigation.max_angular_speed',        1.00)
        self.declare_parameter('navigation.enable_avoidance',         True)
        self.declare_parameter('navigation.approach_speed_threshold', 0.02)

        self.team               = self.get_parameter('robot.team').value
        self.crit_dist          = self.get_parameter('navigation.critical_distance').value
        self.slow_start         = self.get_parameter('navigation.slow_start_distance').value
        self.front_range        = self.get_parameter('navigation.front_angle_range').value
        self.side_range         = self.get_parameter('navigation.side_angle_range').value
        self.max_lin            = self.get_parameter('navigation.max_linear_speed').value
        self.max_ang            = self.get_parameter('navigation.max_angular_speed').value
        self.enabled            = self.get_parameter('navigation.enable_avoidance').value
        self.approach_threshold = self.get_parameter('navigation.approach_speed_threshold').value

        # ----------------------------------------------------------
        # État interne
        # ----------------------------------------------------------
        self.prev_front_dist = None
        self.prev_scan_time  = None
        self.approach_speed  = 0.0
        self.in_danger       = False
        self.latest_raw_cmd  = Twist()

        # ----------------------------------------------------------
        # Topics
        # ----------------------------------------------------------
        self.scan_sub    = self.create_subscription(LaserScan, '/scan',        self._cb_scan,    10)
        self.raw_cmd_sub = self.create_subscription(Twist,     '/cmd_vel_raw', self._cb_raw_cmd, 10)

        self.cmd_pub   = self.create_publisher(Twist,   '/cmd_vel',        10)
        self.alert_pub = self.create_publisher(String,  '/obstacle_alert', 10)
        self.scale_pub = self.create_publisher(Float32, '/cmd_vel_scale',  10)

        self.get_logger().info(f'✓ ObstacleAvoidance démarré — équipe: {self.team}')
        self.get_logger().info(f'  • DANGER  : < {self.crit_dist}m')
        self.get_logger().info(f'  • SLOWING : rapprochement détecté entre {self.slow_start}m et {self.crit_dist}m')
        self.get_logger().info(f'  • Seuil vitesse approche : {self.approach_threshold}m/s')

    # ----------------------------------------------------------
    # Callback commande brute (depuis state machine)
    # ----------------------------------------------------------
    def _cb_raw_cmd(self, msg: Twist):
        self.latest_raw_cmd = msg

    # ----------------------------------------------------------
    # Callback scan principal
    # ----------------------------------------------------------
    def _cb_scan(self, msg: LaserScan):
        if not self.enabled:
            return

        now = time.time()
        front_dist, _, _ = self._analyze_sectors(msg)

        # ── Calcul vitesse d'approche ─────────────────────────
        # Positif = obstacle qui se rapproche
        if (self.prev_front_dist is not None
                and self.prev_scan_time is not None
                and front_dist != float('inf')
                and self.prev_front_dist != float('inf')):
            dt = now - self.prev_scan_time
            if dt > 0:
                self.approach_speed = (self.prev_front_dist - front_dist) / dt
            else:
                self.approach_speed = 0.0
        else:
            self.approach_speed = 0.0

        self.prev_front_dist = front_dist
        self.prev_scan_time  = now

        # ── Décision ─────────────────────────────────────────
        status, scale = self._decide(front_dist)

        # ── Publier alerte ────────────────────────────────────
        alert_msg      = String()
        alert_msg.data = status
        self.alert_pub.publish(alert_msg)

        # ── Publier facteur de vitesse ────────────────────────
        scale_msg      = Float32()
        scale_msg.data = float(scale)
        self.scale_pub.publish(scale_msg)

        # ── Appliquer sur /cmd_vel ────────────────────────────
        if status == 'DANGER':
            self.cmd_pub.publish(Twist())
            if not self.in_danger:
                self.get_logger().warn(
                    f'🚨 DANGER — obstacle à {front_dist:.2f}m — arrêt forcé'
                )
            self.in_danger = True

        elif status == 'SLOWING':
            scaled = self._scale_cmd(self.latest_raw_cmd, scale)
            self.cmd_pub.publish(scaled)
            self.in_danger = False
            self.get_logger().warn(
                f'⚠️  SLOWING — dist={front_dist:.2f}m  '
                f'approche={self.approach_speed:.3f}m/s  '
                f'scale={scale:.2f}'
            )

        else:  # OK
            if self.in_danger:
                self.cmd_pub.publish(Twist())
                self.get_logger().info('✅ Obstacle dégagé — reprise')
            else:
                self.cmd_pub.publish(self.latest_raw_cmd)  # ← ajouter cette ligne
            self.in_danger = False

    # ----------------------------------------------------------
    # Décision
    # ----------------------------------------------------------
    def _decide(self, front: float):
        """
        Retourne (status, scale) :
          'DANGER'  , 0.0      → arrêt complet
          'SLOWING' , 0.0-1.0  → ralentissement progressif
          'OK'      , 1.0      → pas d'intervention
        """
        # DANGER
        if front <= self.crit_dist:
            return 'DANGER', 0.0

        # SLOWING : obstacle dans la zone ET qui se rapproche
        if (front < self.slow_start
                and self.approach_speed > self.approach_threshold):

            # Facteur linéaire distance
            rang  = self.slow_start - self.crit_dist
            reste = front - self.crit_dist
            scale = max(0.0, min(1.0, reste / rang))

            # Pondération vitesse d'approche (max 30% de malus supplémentaire)
            speed_factor = min(1.0, self.approach_speed / 0.3)
            scale = scale * (1.0 - 0.3 * speed_factor)
            scale = max(0.0, min(1.0, scale))

            return 'SLOWING', scale

        return 'OK', 1.0

    # ----------------------------------------------------------
    # Application du facteur de vitesse
    # ----------------------------------------------------------
    def _scale_cmd(self, cmd: Twist, scale: float) -> Twist:
        out = Twist()
        out.linear.x  = cmd.linear.x  * scale
        out.linear.y  = cmd.linear.y  * scale
        out.angular.z = cmd.angular.z  # rotation inchangée
        return out

    # ----------------------------------------------------------
    # Analyse des secteurs angulaires
    # ----------------------------------------------------------
    def _analyze_sectors(self, msg: LaserScan):
        front_min = float('inf')
        left_min  = float('inf')
        right_min = float('inf')

        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max:
                continue
            if math.isnan(dist) or math.isinf(dist):
                continue

            angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            while angle_deg >  180: angle_deg -= 360
            while angle_deg < -180: angle_deg += 360

            half_front = self.front_range / 2.0

            if -half_front <= angle_deg <= half_front:
                front_min = min(front_min, dist)
            elif 0 < angle_deg <= self.side_range:
                left_min  = min(left_min,  dist)
            elif -self.side_range <= angle_deg < 0:
                right_min = min(right_min, dist)

        return front_min, left_min, right_min


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()