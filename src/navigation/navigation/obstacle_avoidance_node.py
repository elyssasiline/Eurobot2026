#!/usr/bin/env python3
"""
navigation/navigation/obstacle_avoidance_node.py

Node d'évitement d'obstacles basé sur le RPLidar A1.
Tous les paramètres sont lus depuis robot_params.yaml (section navigation + robot).

Arbitrage /cmd_vel :
  - État OK      → NE publie PAS sur /cmd_vel (la state machine garde la main)
  - État WARNING → publie une rotation sur /cmd_vel (prise de main)
  - État DANGER  → publie stop sur /cmd_vel (prise de main)

La state machine écoute /obstacle_alert et stoppe d'elle-même en DANGER.
Le lidar agit en backup direct sur /cmd_vel uniquement quand nécessaire.

Topics abonnés :
  /scan  (sensor_msgs/LaserScan)

Topics publiés :
  /cmd_vel        (geometry_msgs/Twist)  — UNIQUEMENT en WARNING ou DANGER
  /obstacle_alert (std_msgs/String)      — état : OK / WARNING / DANGER
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # ----------------------------------------------------------
        # Paramètres
        # ----------------------------------------------------------
        self.declare_parameter('robot.team', 'blue')
        self.declare_parameter('navigation.min_obstacle_distance', 0.30)
        self.declare_parameter('navigation.critical_distance', 0.15)
        self.declare_parameter('navigation.front_angle_range', 45.0)
        self.declare_parameter('navigation.side_angle_range', 90.0)
        self.declare_parameter('navigation.max_linear_speed', 0.30)
        self.declare_parameter('navigation.max_angular_speed', 1.00)
        self.declare_parameter('navigation.enable_avoidance', True)

        self.team        = self.get_parameter('robot.team').value
        self.min_dist    = self.get_parameter('navigation.min_obstacle_distance').value
        self.crit_dist   = self.get_parameter('navigation.critical_distance').value
        self.front_range = self.get_parameter('navigation.front_angle_range').value
        self.side_range  = self.get_parameter('navigation.side_angle_range').value
        self.max_lin     = self.get_parameter('navigation.max_linear_speed').value
        self.max_ang     = self.get_parameter('navigation.max_angular_speed').value
        self.enabled     = self.get_parameter('navigation.enable_avoidance').value

        # ----------------------------------------------------------
        # État interne
        # ----------------------------------------------------------
        self.current_status  = 'OK'   # dernier statut publié
        self.holding_control = False  # True = le lidar a la main sur /cmd_vel

        # ----------------------------------------------------------
        # Topics
        # ----------------------------------------------------------
        self.scan_sub  = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',        10)
        self.alert_pub = self.create_publisher(String, '/obstacle_alert', 10)

        self.get_logger().info(f'✓ ObstacleAvoidance démarré — équipe: {self.team}')
        self.get_logger().info(f'  • Évitement: {"activé" if self.enabled else "désactivé"}')
        self.get_logger().info(f'  • Distances: warning={self.min_dist}m / critical={self.crit_dist}m')
        self.get_logger().info(f'  • Arbitrage: publie /cmd_vel UNIQUEMENT en WARNING/DANGER')

    # ----------------------------------------------------------
    # Callback principal
    # ----------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        if not self.enabled:
            return

        front_min, left_min, right_min = self._analyze_sectors(msg)
        new_status, cmd = self._decide(front_min, left_min, right_min)

        # --- Publier l'alerte (toujours) ---
        alert_msg       = String()
        alert_msg.data  = new_status
        self.alert_pub.publish(alert_msg)

        # --- Arbitrage /cmd_vel ---
        # Prise de main : WARNING ou DANGER → on publie la commande lidar
        if new_status in ('WARNING', 'DANGER'):
            self.holding_control = True
            self.cmd_pub.publish(cmd)
            self.get_logger().warn(
                f'[{new_status}] avant={front_min:.2f}m G={left_min:.2f}m D={right_min:.2f}m'
            )

        # Relâche de main : on vient de passer en OK
        elif new_status == 'OK' and self.holding_control:
            self.holding_control = False
            # On publie UN SEUL stop propre pour annuler la dernière commande lidar,
            # puis la state machine reprend immédiatement via /obstacle_alert→OK
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.get_logger().info('✅ Obstacle dégagé — relâche /cmd_vel à la state machine')

        # OK stable : on ne publie RIEN sur /cmd_vel → state machine garde la main
        self.current_status = new_status

    # ----------------------------------------------------------
    # Analyse des secteurs angulaires
    # ----------------------------------------------------------
    def _analyze_sectors(self, msg: LaserScan):
        """Extrait la distance minimale dans chaque secteur angulaire."""
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

    # ----------------------------------------------------------
    # Décision
    # ----------------------------------------------------------
    def _decide(self, front: float, left: float, right: float):
        """
        Détermine le statut et la commande moteur.

        Retourne (status, cmd) :
          - cmd n'est utilisée QUE si status != 'OK'
          - En OK, cmd est ignorée (non publiée)
        """
        cmd = Twist()

        # Arrêt d'urgence
        if front < self.crit_dist:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            return 'DANGER', cmd

        # Évitement
        if front < self.min_dist:
            cmd.linear.x = 0.0
            # Tourner du côté qui a le plus d'espace
            if right > left:
                cmd.angular.z = -self.max_ang   # tourner à droite
            else:
                cmd.angular.z =  self.max_ang   # tourner à gauche
            return 'WARNING', cmd

        # Voie libre — cmd non publiée
        return 'OK', cmd


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