#!/usr/bin/env python3
"""
Script simple pour tester le lidar et afficher les obstacles détectés
Affiche la distance et la position des obstacles en temps réel
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class LidarTestNode(Node):
    def __init__(self):
        super().__init__('lidar_test')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.scan_count = 0
        
        print("=" * 80)
        print("TEST LIDAR - DÉTECTION D'OBSTACLES")
        print("=" * 80)
        print("Attente des scans du lidar...")
        print()
    
    def scan_callback(self, msg):
        """Analyse chaque scan du lidar"""
        self.scan_count += 1
        
        # Ne traiter qu'un scan sur 5 pour ne pas spammer
        if self.scan_count % 5 != 0:
            return
        
        print(f"\n{'=' * 80}")
        print(f"SCAN #{self.scan_count // 5}")
        print(f"{'=' * 80}")
        
        # Informations générales
        print(f"• Nombre de points: {len(msg.ranges)}")
        print(f"• Portée min: {msg.range_min:.2f}m - max: {msg.range_max:.2f}m")
        print(f"• Angle: {math.degrees(msg.angle_min):.1f}° à {math.degrees(msg.angle_max):.1f}°")
        print()
        
        # Analyser par secteurs
        obstacles_front = []
        obstacles_left = []
        obstacles_right = []
        obstacles_back = []
        
        for i, distance in enumerate(msg.ranges):
            # Ignorer les valeurs invalides
            if distance < msg.range_min or distance > msg.range_max or \
               math.isnan(distance) or math.isinf(distance):
                continue
            
            # Calculer l'angle de ce point
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)
            
            # Normaliser entre -180 et 180
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360
            
            # Classifier par secteur (45° chacun)
            if -45 <= angle_deg <= 45:
                obstacles_front.append((angle_deg, distance))
            elif 45 < angle_deg <= 135:
                obstacles_left.append((angle_deg, distance))
            elif -135 <= angle_deg < -45:
                obstacles_right.append((angle_deg, distance))
            else:
                obstacles_back.append((angle_deg, distance))
        
        # Afficher les obstacles par secteur
        print("OBSTACLES DÉTECTÉS PAR SECTEUR:")
        print("-" * 80)
        
        # FRONT (devant)
        if obstacles_front:
            min_front = min(obstacles_front, key=lambda x: x[1])
            print(f"🔴 DEVANT     : {len(obstacles_front):3d} points | "
                  f"Plus proche: {min_front[1]:.2f}m à {min_front[0]:+6.1f}°")
            
            # Détails des obstacles proches (< 1m)
            close_front = [o for o in obstacles_front if o[1] < 1.0]
            if close_front:
                print(f"   ⚠️  ALERTE: {len(close_front)} obstacle(s) < 1m:")
                for angle, dist in sorted(close_front, key=lambda x: x[1])[:5]:
                    print(f"      • {dist:.2f}m à {angle:+6.1f}°")
        else:
            print(f"⚪ DEVANT     : Aucun obstacle")
        
        # LEFT (gauche)
        if obstacles_left:
            min_left = min(obstacles_left, key=lambda x: x[1])
            print(f"🔵 GAUCHE    : {len(obstacles_left):3d} points | "
                  f"Plus proche: {min_left[1]:.2f}m à {min_left[0]:+6.1f}°")
        else:
            print(f"⚪ GAUCHE    : Aucun obstacle")
        
        # RIGHT (droite)
        if obstacles_right:
            min_right = min(obstacles_right, key=lambda x: x[1])
            print(f"🟡 DROITE    : {len(obstacles_right):3d} points | "
                  f"Plus proche: {min_right[1]:.2f}m à {min_right[0]:+6.1f}°")
        else:
            print(f"⚪ DROITE    : Aucun obstacle")
        
        # BACK (arrière)
        if obstacles_back:
            min_back = min(obstacles_back, key=lambda x: x[1])
            print(f"🟣 ARRIÈRE   : {len(obstacles_back):3d} points | "
                  f"Plus proche: {min_back[1]:.2f}m à {min_back[0]:+6.1f}°")
        else:
            print(f"⚪ ARRIÈRE   : Aucun obstacle")
        
        print()
        
        # Carte ASCII simple (vue de dessus)
        print("CARTE RADAR (vue de dessus):")
        print("-" * 80)
        self.print_ascii_radar(obstacles_front, obstacles_left, obstacles_right, obstacles_back)
        
        # Statistiques globales
        all_obstacles = obstacles_front + obstacles_left + obstacles_right + obstacles_back
        if all_obstacles:
            closest = min(all_obstacles, key=lambda x: x[1])
            print(f"\n📍 OBSTACLE LE PLUS PROCHE: {closest[1]:.2f}m à {closest[0]:+6.1f}°")
            
            # Alerte si très proche
            if closest[1] < 0.3:
                print(f"   ⚠️⚠️⚠️  DANGER - OBSTACLE CRITIQUE < 30cm ⚠️⚠️⚠️")
            elif closest[1] < 0.5:
                print(f"   ⚠️  ATTENTION - Obstacle proche < 50cm")
    
    def print_ascii_radar(self, front, left, right, back):
        """Affiche une carte radar ASCII"""
        # Grille 21x21 (robot au centre)
        grid = [[' ' for _ in range(21)] for _ in range(21)]
        center = 10
        
        # Robot au centre
        grid[center][center] = 'R'
        
        # Fonction pour placer un obstacle
        def place_obstacle(angle_deg, distance, symbol):
            # Limiter à 2m pour l'affichage
            if distance > 2.0:
                distance = 2.0
            
            # Convertir en coordonnées grille
            angle_rad = math.radians(angle_deg)
            x = int(center + distance * 5 * math.sin(angle_rad))
            y = int(center - distance * 5 * math.cos(angle_rad))
            
            if 0 <= x < 21 and 0 <= y < 21:
                grid[y][x] = symbol
        
        # Placer les obstacles
        for angle, dist in front:
            place_obstacle(angle, dist, '█')
        for angle, dist in left:
            place_obstacle(angle, dist, '▓')
        for angle, dist in right:
            place_obstacle(angle, dist, '▒')
        for angle, dist in back:
            place_obstacle(angle, dist, '░')
        
        # Afficher la grille
        print("     N")
        print("     ↑")
        for i, row in enumerate(grid):
            if i == center:
                print("O ← " + ''.join(row) + " → E")
            else:
                print("    " + ''.join(row))
        print("     ↓")
        print("     S")
        print("\nLégende: R=Robot  █=Devant  ▓=Gauche  ▒=Droite  ░=Arrière")


def main(args=None):
    rclpy.init(args=args)
    node = LidarTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n" + "=" * 80)
        print("ARRÊT DU TEST")
        print("=" * 80)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
