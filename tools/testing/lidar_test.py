#!/usr/bin/env python3
"""
tools/lidar_test.py

Test interactif du lidar + obstacle_avoidance_node.
Affiche en temps réel :
  - Les distances par secteur (avant / gauche / droite)
  - Le statut courant publié par obstacle_avoidance
  - Les commandes /cmd_vel générées
  - Un mini-radar ASCII

Usage :
  python3 lidar_test.py

Prérequis :
  - Le RPLidar est branché et le node rplidar tourne :
      ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0
  - L'obstacle_avoidance_node tourne :
      ros2 run navigation obstacle_avoidance_node
  
  Ou tout en une fois :
      ros2 launch navigation lidar.launch.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time
import sys


R    = '\033[91m'
G    = '\033[92m'
Y    = '\033[93m'
C    = '\033[96m'
W    = '\033[97m'
DIM  = '\033[2m'
RST  = '\033[0m'
BOLD = '\033[1m'


class LidarTestNode(Node):

    FRONT_RANGE = 45.0   # degrés — cône avant surveillé
    SIDE_RANGE  = 90.0   # degrés — côtés
    WARN_DIST   = 0.30   # m
    CRIT_DIST   = 0.15   # m

    # Radar ASCII
    RADAR_H = 15
    RADAR_W = 31
    MAX_DIST = 1.5       # m — portée affichée

    def __init__(self):
        super().__init__('lidar_test')

        # ── Données ──────────────────────────────────────────
        self.scan_count   = 0
        self.latest_scan  = None
        self.alert_status = '—'
        self.cmd_vel      = None
        self.cmd_count    = 0
        self.alert_count  = 0
        self.start_time   = time.time()

        # Historique pour calculer le taux de détection
        self.alert_history = []   # 'OK' / 'WARNING' / 'DANGER'

        # ── Subscribers ──────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',           self._cb_scan,    10)
        self.create_subscription(String,    '/obstacle_alert', self._cb_alert,   10)
        self.create_subscription(Twist,     '/cmd_vel',        self._cb_cmd_vel, 10)

        # ── Timer affichage 10 Hz ─────────────────────────────
        self.create_timer(0.1, self._display)

        print('\033[2J\033[H', end='')

    # ─────────────────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────────────────
    def _cb_scan(self, msg: LaserScan):
        self.latest_scan = msg
        self.scan_count += 1

    def _cb_alert(self, msg: String):
        self.alert_status = msg.data
        self.alert_count += 1
        self.alert_history.append(msg.data)
        if len(self.alert_history) > 50:
            self.alert_history.pop(0)

    def _cb_cmd_vel(self, msg: Twist):
        self.cmd_vel   = msg
        self.cmd_count += 1

    # ─────────────────────────────────────────────────────────
    #  Analyse des secteurs
    # ─────────────────────────────────────────────────────────
    def _analyze(self, msg: LaserScan):
        sectors = {
            'front': float('inf'),
            'left':  float('inf'),
            'right': float('inf'),
            'rear':  float('inf'),
        }
        points = []  # (angle_deg, distance)

        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max:
                continue
            if math.isnan(dist) or math.isinf(dist):
                continue

            angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            while angle_deg >  180: angle_deg -= 360
            while angle_deg < -180: angle_deg += 360

            points.append((angle_deg, dist))

            half_f = self.FRONT_RANGE / 2.0
            if -half_f <= angle_deg <= half_f:
                sectors['front'] = min(sectors['front'], dist)
            elif 0 < angle_deg <= self.SIDE_RANGE:
                sectors['left']  = min(sectors['left'],  dist)
            elif -self.SIDE_RANGE <= angle_deg < 0:
                sectors['right'] = min(sectors['right'], dist)
            elif abs(angle_deg) > 135:
                sectors['rear']  = min(sectors['rear'],  dist)

        return sectors, points

    # ─────────────────────────────────────────────────────────
    #  Radar ASCII
    # ─────────────────────────────────────────────────────────
    def _make_radar(self, points):
        H = self.RADAR_H
        W = self.RADAR_W
        cx = W // 2
        cy = H // 2

        grid = [[' '] * W for _ in range(H)]

        # Cercles de distance (0.5m, 1.0m, 1.5m)
        for r_m in [0.5, 1.0, 1.5]:
            for deg in range(0, 360, 5):
                rad = math.radians(deg)
                x_m = r_m * math.sin(rad)
                y_m = r_m * math.cos(rad)
                gx  = int(cx - x_m * (cx / self.MAX_DIST))
                gy  = int(cy - y_m * (cy / self.MAX_DIST))
                if 0 <= gx < W and 0 <= gy < H:
                    if grid[gy][gx] == ' ':
                        grid[gy][gx] = '·'

        # Axe avant (haut)
        for i in range(cy):
            if grid[i][cx] == ' ':
                grid[i][cx] = '│'

        # Obstacles
        for (angle_deg, dist) in points:
            if dist > self.MAX_DIST:
                continue
            rad = math.radians(angle_deg)
            x_m = dist * math.sin(rad)
            y_m = dist * math.cos(rad)
            gx = int(cx - x_m * (cx / self.MAX_DIST))
            gy  = int(cy - y_m * (cy / self.MAX_DIST))
            if 0 <= gx < W and 0 <= gy < H:
                grid[gy][gx] = '█'

        # Cône de surveillance avant
        for deg in range(-int(self.FRONT_RANGE/2), int(self.FRONT_RANGE/2)+1, 3):
            rad = math.radians(deg)
            for step in range(1, cy):
                gx = int(cx - math.sin(rad) * step)
                gy = int(cy - math.cos(rad) * step)
                if 0 <= gx < W and 0 <= gy < H and grid[gy][gx] == ' ':
                    grid[gy][gx] = '░'

        # Robot
        grid[cy][cx] = 'R'

        return grid

    # ─────────────────────────────────────────────────────────
    #  Affichage
    # ─────────────────────────────────────────────────────────
    def _display(self):
        print('\033[H', end='')
        t = time.time() - self.start_time
        hz = self.scan_count / max(t, 0.1)

        print(f'{BOLD}╔══════════════════════════════════════════════════════╗{RST}')
        print(f'{BOLD}║         🔭  LIDAR TEST  —  t={t:6.1f}s  Hz≈{hz:4.1f}           ║{RST}')
        print(f'{BOLD}╚══════════════════════════════════════════════════════╝{RST}')
        print()

        if self.latest_scan is None:
            print(f'  {Y}⏳ En attente du topic /scan ...{RST}')
            print()
            print(f'  Vérifier que le lidar tourne :')
            print(f'  {DIM}ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0{RST}')
            print(f'  {DIM}ros2 launch navigation lidar.launch.py{RST}')
            return

        scan    = self.latest_scan
        sectors, points = self._analyze(scan)

        # ── Statut global ─────────────────────────────────────
        status_col = {
            'OK': G, 'WARNING': Y, 'DANGER': R, '—': DIM
        }.get(self.alert_status, W)
        status_icon = {
            'OK': '✅', 'WARNING': '⚠️ ', 'DANGER': '🚨', '—': '❓'
        }.get(self.alert_status, '?')

        print(f'  Statut : {status_col}{BOLD}{status_icon} {self.alert_status}{RST}   '
              f'{DIM}(alertes reçues: {self.alert_count}){RST}')
        print()

        # ── Distances par secteur ─────────────────────────────
        def dist_bar(d, label, width=20):
            if d == float('inf'):
                bar_str = '─' * width
                col = G
                val = '>1.5m'
            else:
                filled = int(min(d / self.MAX_DIST, 1.0) * width)
                col    = R if d < self.CRIT_DIST else (Y if d < self.WARN_DIST else G)
                bar_str = '█' * filled + '░' * (width - filled)
                val = f'{d:.2f}m'
                if d < self.CRIT_DIST:
                    val += ' 🚨'
                elif d < self.WARN_DIST:
                    val += ' ⚠️'
            return f'  {label:<8} {col}[{bar_str}]{RST} {val}'

        print(dist_bar(sectors['front'], 'AVANT'))
        print(dist_bar(sectors['left'],  'GAUCHE'))
        print(dist_bar(sectors['right'], 'DROITE'))
        print(dist_bar(sectors['rear'],  'ARRIÈRE'))
        print()

        # ── Commande /cmd_vel ─────────────────────────────────
        print(f'  {C}cmd_vel{RST}  (reçues: {self.cmd_count})')
        if self.cmd_vel:
            vx = self.cmd_vel.linear.x
            vy = self.cmd_vel.linear.y
            wz = self.cmd_vel.angular.z
            if vx == 0 and vy == 0 and wz == 0:
                print(f'    {DIM}STOP (0, 0, 0){RST}')
            else:
                col = R if (vx == 0 and wz != 0) else G
                print(f'    {col}vx={vx:+.3f}  vy={vy:+.3f}  wz={wz:+.3f}{RST}')
        else:
            print(f'    {DIM}— aucune commande reçue (obstacle_avoidance ne tourne pas ?){RST}')
        print()

        # ── Radar ─────────────────────────────────────────────
        print(f'  Radar (portée {self.MAX_DIST}m)    N=avant robot')
        print(f'                 N')
        radar = self._make_radar(points)
        for i, row in enumerate(radar):
            prefix = 'O ' if i == self.RADAR_H // 2 else '  '
            suffix = ' E' if i == self.RADAR_H // 2 else ''
            print(f'  {prefix}{"".join(row)}{suffix}')
        print(f'                 S')
        print()

        # ── Stats ─────────────────────────────────────────────
        if self.alert_history:
            nb_ok      = self.alert_history.count('OK')
            nb_warn    = self.alert_history.count('WARNING')
            nb_danger  = self.alert_history.count('DANGER')
            total      = len(self.alert_history)
            print(f'  Dernières {total} alertes : '
                  f'{G}OK={nb_ok}{RST}  '
                  f'{Y}WARN={nb_warn}{RST}  '
                  f'{R}DANGER={nb_danger}{RST}')

        valid_pts = [d for (_, d) in points if d < self.MAX_DIST]
        print(f'  Points valides dans scan : {len(valid_pts)} / {len(scan.ranges)}')
        print()
        print(f'{DIM}  Ctrl+C pour quitter{RST}')


def main():
    rclpy.init()
    node = LidarTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()