#!/usr/bin/env python3
"""
Visualisation radar en temps réel pour RPLidar A1
Affichage propre sans scroll avec carte radar interactive
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import sys
import os


class RadarDisplayNode(Node):
    def __init__(self):
        super().__init__('radar_display')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Paramètres de la carte radar
        self.radar_height = 21  # Hauteur de la grille (réduite)
        self.radar_width = 41   # Largeur = 2x hauteur pour compenser le ratio des caractères
        self.max_display_distance = 2.0  # Afficher jusqu'à 2m (réduit)
        self.danger_distance = 0.3  # Distance de danger (30cm)
        self.warning_distance = 0.5  # Distance d'avertissement (50cm)
        
        # Statistiques
        self.scan_count = 0
        self.obstacles_close = []
        self.closest_obstacle = None
        
        print("\033[2J\033[H")  # Clear screen
        print("╔════════════════════════════════════════════════════════════════════════════════╗")
        print("║                      🎯 RADAR LIDAR - DÉTECTION D'OBSTACLES                     ║")
        print("╚════════════════════════════════════════════════════════════════════════════════╝")
        print()
        print("⏳ Initialisation... En attente des données du lidar...")
        print()
    
    def clear_screen(self):
        """Efface l'écran et repositionne le curseur"""
        print("\033[2J\033[H", end='')
    
    def scan_callback(self, msg):
        """Callback pour traiter les scans du lidar"""
        self.scan_count += 1
        
        # Traiter un scan sur 2 pour fluidité
        if self.scan_count % 2 != 0:
            return
        
        # Analyser les obstacles
        obstacles = self.analyze_scan(msg)
        
        # Afficher l'interface
        self.display_radar(obstacles, msg)
    
    def analyze_scan(self, scan):
        """Analyse le scan et extrait les obstacles"""
        obstacles = []
        self.obstacles_close = []
        self.closest_obstacle = {'distance': float('inf'), 'angle': 0}
        
        for i, distance in enumerate(scan.ranges):
            # Filtrer les valeurs invalides
            if distance < scan.range_min or distance > scan.range_max or \
               math.isnan(distance) or math.isinf(distance):
                continue
            
            # Limiter à la distance d'affichage
            if distance > self.max_display_distance:
                continue
            
            # Calculer l'angle
            angle_rad = scan.angle_min + i * scan.angle_increment
            angle_deg = math.degrees(angle_rad)
            
            # Normaliser l'angle [-180, 180]
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360
            
            obstacles.append({
                'distance': distance,
                'angle_rad': angle_rad,
                'angle_deg': angle_deg
            })
            
            # Obstacles proches
            if distance < self.warning_distance:
                self.obstacles_close.append({
                    'distance': distance,
                    'angle': angle_deg
                })
            
            # Obstacle le plus proche
            if distance < self.closest_obstacle['distance']:
                self.closest_obstacle = {
                    'distance': distance,
                    'angle': angle_deg
                }
        
        return obstacles
    
    def display_radar(self, obstacles, scan_msg):
        """Affiche la carte radar et les informations"""
        self.clear_screen()
        
        # En-tête
        print("╔════════════════════════════════════════════════════════════════════════════════╗")
        print("║                      🎯 RADAR LIDAR - DÉTECTION D'OBSTACLES                     ║")
        print("╚════════════════════════════════════════════════════════════════════════════════╝")
        print()
        
        # Informations générales
        total_points = len(obstacles)
        close_count = len(self.obstacles_close)
        
        print(f"📊 STATISTIQUES")
        print(f"├─ Scan #{self.scan_count // 2}")
        print(f"├─ Points détectés: {total_points}")
        print(f"├─ Obstacles proches (<{self.warning_distance}m): {close_count}")
        
        if self.closest_obstacle['distance'] != float('inf'):
            dist = self.closest_obstacle['distance']
            angle = self.closest_obstacle['angle']
            
            if dist < self.danger_distance:
                status = "🔴 DANGER"
            elif dist < self.warning_distance:
                status = "🟠 ATTENTION"
            else:
                status = "🟢 OK"
            
            print(f"└─ Plus proche: {dist:.2f}m à {angle:+6.1f}° {status}")
        else:
            print(f"└─ Aucun obstacle détecté")
        
        print()
        print("─" * 82)
        print()
        
        # Alertes si obstacles proches (EN HAUT)
        if self.obstacles_close:
            print("⚠️  ALERTES OBSTACLES PROCHES:")
            # Trier par distance
            sorted_close = sorted(self.obstacles_close, key=lambda x: x['distance'])
            for i, obs in enumerate(sorted_close[:5]):  # Top 5
                dist = obs['distance']
                angle = obs['angle']
                
                if dist < self.danger_distance:
                    icon = "🔴"
                else:
                    icon = "🟠"
                
                print(f"    {icon} {dist:.2f}m à {angle:+6.1f}°")
            print()
        else:
            print("✅ Aucun obstacle proche")
            print()
        
        print("─" * 82)
        print()
        
        # Créer la grille radar (EN BAS)
        grid = self.create_radar_grid(obstacles)
        
        # Afficher la grille
        print("                         CARTE RADAR")
        print("                    (Portée: 2m maximum)")
        print()
        print("                              N")
        print("                              ↑")
        
        for i, row in enumerate(grid):
            if i == self.radar_height // 2:
                print(f"O ← {''.join(row)} → E")
            else:
                print(f"    {''.join(row)}")
        
        print("                              ↓")
        print("                              S")
        print()
        
        # Légende
        print("    Légende:  R=Robot   █=Obstacle   ·=Cercles (0.5m, 1m, 1.5m, 2m)   │─=Axes")
        print()
        print("─" * 82)
        print("Press Ctrl+C to stop")
    
    def create_radar_grid(self, obstacles):
        """Crée la grille radar 2D (rectangulaire pour compenser le ratio des caractères)"""
        height = self.radar_height
        width = self.radar_width
        center_x = width // 2
        center_y = height // 2
        
        # Initialiser la grille avec des espaces
        grid = [[' ' for _ in range(width)] for _ in range(height)]
        
        # Dessiner les cercles de distance (tous les 0.5m)
        for radius_m in [0.5, 1.0, 1.5, 2.0]:
            for angle in range(0, 360, 3):  # Tous les 3 degrés pour plus de points
                angle_rad = math.radians(angle)
                
                # Calculer position en mètres
                x_m = radius_m * math.sin(angle_rad)
                y_m = radius_m * math.cos(angle_rad)
                
                # Convertir en pixels avec échelles différentes pour X et Y
                scale_x = center_x / self.max_display_distance
                scale_y = center_y / self.max_display_distance
                
                x = int(center_x - x_m * scale_x)  # INVERSER X (- au lieu de +)
                y = int(center_y - y_m * scale_y)
                
                if 0 <= x < width and 0 <= y < height and grid[y][x] == ' ':
                    grid[y][x] = '·'
        
        # Dessiner les axes cardinaux
        # Axe vertical (N-S)
        for i in range(height):
            if grid[i][center_x] == ' ':
                grid[i][center_x] = '│'
        
        # Axe horizontal (O-E)
        for i in range(width):
            if grid[center_y][i] == ' ':
                grid[center_y][i] = '─'
        
        # Placer les obstacles
        obstacle_positions = set()
        for obs in obstacles:
            distance = obs['distance']
            angle_rad = obs['angle_rad']
            
            # Convertir en coordonnées cartésiennes (mètres)
            x_m = distance * math.sin(angle_rad)
            y_m = distance * math.cos(angle_rad)
            
            # Convertir en coordonnées de grille avec échelles séparées
            scale_x = center_x / self.max_display_distance
            scale_y = center_y / self.max_display_distance
            
            grid_x = int(center_x - x_m * scale_x)  # INVERSER X (- au lieu de +)
            grid_y = int(center_y - y_m * scale_y)
            
            # Vérifier les limites
            if 0 <= grid_x < width and 0 <= grid_y < height:
                pos = (grid_y, grid_x)
                if pos not in obstacle_positions:
                    obstacle_positions.add(pos)
                    grid[grid_y][grid_x] = '█'
        
        # Placer le robot au centre (en dernier pour pas être écrasé)
        grid[center_y][center_x] = 'R'
        
        return grid


def main(args=None):
    rclpy.init(args=args)
    node = RadarDisplayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n" + "=" * 82)
        print("ARRÊT DU RADAR")
        print("=" * 82)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()