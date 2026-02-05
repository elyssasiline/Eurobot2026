#!/usr/bin/env python3
"""
Node ROS2 pour l'évitement d'obstacles avec RPLidar A1
Analyse les scans du lidar et publie des commandes d'évitement
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import numpy as np
import math


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Paramètres
        self.declare_parameter('min_obstacle_distance', 0.3)  # 30cm
        self.declare_parameter('critical_distance', 0.15)     # 15cm - arrêt d'urgence
        self.declare_parameter('front_angle_range', 45.0)     # Angle devant le robot (degrés)
        self.declare_parameter('side_angle_range', 90.0)      # Angle latéral
        self.declare_parameter('max_linear_speed', 0.3)       # m/s
        self.declare_parameter('max_angular_speed', 1.0)      # rad/s
        self.declare_parameter('enable_avoidance', True)      # Activer/désactiver l'évitement
        
        self.min_distance = self.get_parameter('min_obstacle_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.front_angle = self.get_parameter('front_angle_range').value
        self.side_angle = self.get_parameter('side_angle_range').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.avoidance_enabled = self.get_parameter('enable_avoidance').value
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.cmd_vel_input_sub = self.create_subscription(
            Twist,
            '/cmd_vel_input',  # Commandes depuis la navigation
            self.cmd_vel_input_callback,
            10
        )
        
        self.enable_sub = self.create_subscription(
            Bool,
            '/obstacle_avoidance/enable',
            self.enable_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',  # Commandes vers les moteurs
            10
        )
        
        self.obstacle_detected_pub = self.create_publisher(
            Bool,
            '/obstacle_avoidance/obstacle_detected',
            10
        )
        
        self.avoidance_state_pub = self.create_publisher(
            String,
            '/obstacle_avoidance/state',
            10
        )
        
        # Variables d'état
        self.last_scan = None
        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.min_front_distance = float('inf')
        self.last_cmd_vel_input = Twist()
        
        # Timer pour publier l'état
        self.create_timer(0.1, self.publish_state)
        
        self.get_logger().info('✓ Obstacle Avoidance Node démarré!')
        self.get_logger().info(f'  • Distance min: {self.min_distance}m')
        self.get_logger().info(f'  • Distance critique: {self.critical_distance}m')
        self.get_logger().info(f'  • Évitement: {"ACTIVÉ" if self.avoidance_enabled else "DÉSACTIVÉ"}')
    
    def enable_callback(self, msg):
        """Active/désactive l'évitement d'obstacles"""
        self.avoidance_enabled = msg.data
        state = "ACTIVÉ" if self.avoidance_enabled else "DÉSACTIVÉ"
        self.get_logger().info(f'Évitement d\'obstacles: {state}')
    
    def cmd_vel_input_callback(self, msg):
        """Reçoit les commandes de navigation à filtrer"""
        self.last_cmd_vel_input = msg
    
    def angle_to_index(self, angle_deg, scan):
        """Convertit un angle en degrés vers un index du scan"""
        angle_rad = math.radians(angle_deg)
        
        # Normaliser l'angle dans [angle_min, angle_max]
        if angle_rad < scan.angle_min:
            angle_rad += 2 * math.pi
        
        index = int((angle_rad - scan.angle_min) / scan.angle_increment)
        return max(0, min(index, len(scan.ranges) - 1))
    
    def analyze_scan(self, scan):
        """Analyse le scan lidar pour détecter les obstacles"""
        if len(scan.ranges) == 0:
            return
        
        # Réinitialiser les détections
        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.min_front_distance = float('inf')
        
        # Zones à analyser (en degrés)
        # Front: -45° à +45°
        # Left: 45° à 135°
        # Right: -135° à -45°
        
        front_ranges = []
        left_ranges = []
        right_ranges = []
        
        for i, distance in enumerate(scan.ranges):
            # Ignorer les valeurs invalides
            if distance < scan.range_min or distance > scan.range_max or math.isnan(distance) or math.isinf(distance):
                continue
            
            # Calculer l'angle de ce point
            angle_rad = scan.angle_min + i * scan.angle_increment
            angle_deg = math.degrees(angle_rad)
            
            # Normaliser l'angle entre -180 et 180
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360
            
            # Classifier par zone
            if -self.front_angle <= angle_deg <= self.front_angle:
                front_ranges.append(distance)
            elif self.front_angle < angle_deg <= self.side_angle:
                left_ranges.append(distance)
            elif -self.side_angle <= angle_deg < -self.front_angle:
                right_ranges.append(distance)
        
        # Analyser les zones
        if front_ranges:
            self.min_front_distance = min(front_ranges)
            if self.min_front_distance < self.min_distance:
                self.obstacle_front = True
        
        if left_ranges and min(left_ranges) < self.min_distance:
            self.obstacle_left = True
        
        if right_ranges and min(right_ranges) < self.min_distance:
            self.obstacle_right = True
        
        # Log périodique
        if hasattr(self, 'scan_count'):
            self.scan_count += 1
        else:
            self.scan_count = 0
        
        if self.scan_count % 50 == 0:  # Log toutes les 50 scans (~1 sec à 50Hz)
            obstacles = []
            if self.obstacle_front:
                obstacles.append(f"FRONT({self.min_front_distance:.2f}m)")
            if self.obstacle_left:
                obstacles.append("LEFT")
            if self.obstacle_right:
                obstacles.append("RIGHT")
            
            if obstacles:
                self.get_logger().info(f"Obstacles détectés: {', '.join(obstacles)}")
    
    def compute_avoidance_command(self):
        """Calcule la commande d'évitement basée sur les obstacles détectés"""
        cmd = Twist()
        
        if not self.avoidance_enabled:
            # Pas d'évitement, transmettre la commande telle quelle
            return self.last_cmd_vel_input
        
        # Arrêt d'urgence si obstacle critique devant
        if self.obstacle_front and self.min_front_distance < self.critical_distance:
            self.get_logger().warn(f'⚠️  ARRÊT D\'URGENCE - Obstacle à {self.min_front_distance:.2f}m')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd
        
        # Commande de base depuis la navigation
        cmd.linear.x = self.last_cmd_vel_input.linear.x
        cmd.angular.z = self.last_cmd_vel_input.angular.z
        
        # Si obstacle devant et on avance
        if self.obstacle_front and cmd.linear.x > 0:
            # Réduire la vitesse linéaire proportionnellement à la distance
            speed_factor = max(0.0, (self.min_front_distance - self.critical_distance) / 
                             (self.min_distance - self.critical_distance))
            cmd.linear.x *= speed_factor
            
            # Ajouter une rotation pour éviter
            if self.obstacle_left and not self.obstacle_right:
                # Tourner à droite
                cmd.angular.z = -self.max_angular * 0.5
                self.get_logger().debug('Évitement: rotation droite')
            elif self.obstacle_right and not self.obstacle_left:
                # Tourner à gauche
                cmd.angular.z = self.max_angular * 0.5
                self.get_logger().debug('Évitement: rotation gauche')
            elif not self.obstacle_left and not self.obstacle_right:
                # Les deux côtés libres, tourner dans le sens de la commande initiale
                # ou choisir un côté par défaut
                if abs(cmd.angular.z) < 0.1:
                    cmd.angular.z = self.max_angular * 0.3  # Gauche par défaut
                self.get_logger().debug('Évitement: rotation légère')
        
        # Limiter les vitesses
        cmd.linear.x = max(-self.max_linear, min(self.max_linear, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))
        
        return cmd
    
    def scan_callback(self, msg):
        """Callback pour les scans lidar"""
        self.last_scan = msg
        
        # Analyser le scan
        self.analyze_scan(msg)
        
        # Calculer et publier la commande d'évitement
        cmd_vel_output = self.compute_avoidance_command()
        self.cmd_vel_pub.publish(cmd_vel_output)
        
        # Publier l'état de détection d'obstacle
        obstacle_msg = Bool()
        obstacle_msg.data = self.obstacle_front or self.obstacle_left or self.obstacle_right
        self.obstacle_detected_pub.publish(obstacle_msg)
    
    def publish_state(self):
        """Publie l'état actuel de l'évitement"""
        state_msg = String()
        
        if not self.avoidance_enabled:
            state_msg.data = "DISABLED"
        elif self.obstacle_front and self.min_front_distance < self.critical_distance:
            state_msg.data = "EMERGENCY_STOP"
        elif self.obstacle_front or self.obstacle_left or self.obstacle_right:
            state_msg.data = "AVOIDING"
        else:
            state_msg.data = "CLEAR"
        
        self.avoidance_state_pub.publish(state_msg)


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