#!/usr/bin/env python3
"""
tools/topic_monitor.py

Monitor interactif de tous les topics du robot.
Affiche en temps réel l'état de chaque topic avec timestamp du dernier message.

Usage :
  python3 topic_monitor.py
  python3 topic_monitor.py --test-start   (simule le top départ)
  python3 topic_monitor.py --test-aruco   (simule une détection ArUco)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32, UInt8, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import sys
import time
import argparse
import math


# ─────────────────────────────────────────────────────────────
#  Couleurs ANSI
# ─────────────────────────────────────────────────────────────
R  = '\033[91m'
G  = '\033[92m'
Y  = '\033[93m'
B  = '\033[94m'
C  = '\033[96m'
W  = '\033[97m'
DIM = '\033[2m'
RST = '\033[0m'
BOLD = '\033[1m'


class TopicMonitor(Node):

    def __init__(self, inject_start=False, inject_aruco=False):
        super().__init__('topic_monitor')

        self.inject_start = inject_start
        self.inject_aruco = inject_aruco

        # ── État reçu par topic ──────────────────────────────
        self.data = {
            '/start_signal':                {'val': None, 't': None, 'count': 0},
            '/ir_line':                     {'val': None, 't': None, 'count': 0},
            '/aruco/box_to_flip':           {'val': None, 't': None, 'count': 0},
            '/vision/nearest_box_distance': {'val': None, 't': None, 'count': 0},
            '/obstacle_alert':              {'val': None, 't': None, 'count': 0},
            '/wheel_odom':                  {'val': None, 't': None, 'count': 0},
            '/gripper/status':              {'val': None, 't': None, 'count': 0},
            '/cmd_vel':                     {'val': None, 't': None, 'count': 0},
            '/strategy/state':              {'val': None, 't': None, 'count': 0},
            '/strategy/score':              {'val': None, 't': None, 'count': 0},
            '/scan':                        {'val': None, 't': None, 'count': 0},
        }

        # ── Subscribers ──────────────────────────────────────
        self.create_subscription(Bool,     '/start_signal',                self._mk(Bool,     '/start_signal',                lambda m: f'{"🟢 TRUE" if m.data else "⭕ FALSE"}'), 10)
        self.create_subscription(UInt8,    '/ir_line',                     self._mk(UInt8,    '/ir_line',                     lambda m: f'0b{m.data:05b}  ({m.data})'), 10)
        self.create_subscription(String,   '/aruco/box_to_flip',           self._mk(String,   '/aruco/box_to_flip',           lambda m: m.data), 10)
        self.create_subscription(Float32,  '/vision/nearest_box_distance', self._mk(Float32,  '/vision/nearest_box_distance', lambda m: f'{m.data:.3f} m'), 10)
        self.create_subscription(String,   '/obstacle_alert',              self._mk(String,   '/obstacle_alert',              lambda m: self._fmt_alert(m.data)), 10)
        self.create_subscription(Odometry, '/wheel_odom',                  self._mk(Odometry, '/wheel_odom',                  lambda m: f'x={m.pose.pose.position.x:.3f} y={m.pose.pose.position.y:.3f}'), 10)
        self.create_subscription(String,   '/gripper/status',              self._mk(String,   '/gripper/status',              lambda m: m.data), 10)
        self.create_subscription(Twist,    '/cmd_vel',                     self._mk(Twist,    '/cmd_vel',                     lambda m: self._fmt_twist(m)), 10)
        self.create_subscription(String,   '/strategy/state',              self._mk(String,   '/strategy/state',              lambda m: self._fmt_state(m.data)), 10)
        self.create_subscription(Int32,    '/strategy/score',              self._mk(Int32,    '/strategy/score',              lambda m: f'🏆 {m.data}'), 10)
        self.create_subscription(LaserScan,'/scan',                        self._mk(LaserScan,'/scan',                        lambda m: self._fmt_scan(m)), 10)

        # ── Publishers (injection de test) ───────────────────
        self.start_pub  = self.create_publisher(Bool,   '/start_signal',      10)
        self.aruco_pub  = self.create_publisher(String, '/aruco/box_to_flip', 10)
        self.ir_pub     = self.create_publisher(UInt8,  '/ir_line',           10)

        # ── Timer affichage 4 Hz ─────────────────────────────
        self.create_timer(0.25, self._display)
        self.start_time = time.time()

        # ── Injections différées ─────────────────────────────
        if self.inject_start:
            self.create_timer(2.0, self._inject_start_once)
        if self.inject_aruco:
            self.create_timer(4.0, self._inject_aruco_once)

        print('\033[2J\033[H', end='')  # clear screen

    # ─────────────────────────────────────────────────────────
    #  Factory callback
    # ─────────────────────────────────────────────────────────
    def _mk(self, msg_type, topic, formatter):
        def cb(msg):
            self.data[topic]['val']   = formatter(msg)
            self.data[topic]['t']     = time.time()
            self.data[topic]['count'] += 1
        return cb

    # ─────────────────────────────────────────────────────────
    #  Formateurs
    # ─────────────────────────────────────────────────────────
    def _fmt_twist(self, m: Twist) -> str:
        vx  = m.linear.x
        vy  = m.linear.y
        wz  = m.angular.z
        col = R if (vx == 0 and vy == 0 and wz == 0) else G
        return f'{col}vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}{RST}'

    def _fmt_alert(self, s: str) -> str:
        colors = {'OK': G, 'WARNING': Y, 'DANGER': R}
        icons  = {'OK': '✅', 'WARNING': '⚠️ ', 'DANGER': '🚨'}
        c = colors.get(s, W)
        i = icons.get(s, '?')
        return f'{c}{i} {s}{RST}'

    def _fmt_state(self, s: str) -> str:
        colors = {
            'INIT': DIM, 'FOLLOW_LINE': G, 'APPROACH_BOX': C,
            'BLIND_ADVANCE': Y, 'GRAB': B, 'PLACE': B,
            'AVOID': R, 'STOP': DIM,
        }
        icons = {
            'INIT': '⏳', 'FOLLOW_LINE': '➡️ ', 'APPROACH_BOX': '📦',
            'BLIND_ADVANCE': '🔲', 'GRAB': '🦾', 'PLACE': '🎯',
            'AVOID': '🚨', 'STOP': '🛑',
        }
        c = colors.get(s, W)
        i = icons.get(s, '?')
        return f'{c}{BOLD}{i} {s}{RST}'

    def _fmt_scan(self, m: LaserScan) -> str:
        valid = [d for d in m.ranges
                 if m.range_min < d < m.range_max and not math.isnan(d) and not math.isinf(d)]
        if not valid:
            return f'{DIM}aucune mesure valide{RST}'
        min_d = min(valid)
        n     = len(valid)
        col   = R if min_d < 0.15 else (Y if min_d < 0.30 else G)
        return f'{col}min={min_d:.2f}m{RST}  pts={n}'

    # ─────────────────────────────────────────────────────────
    #  Affichage
    # ─────────────────────────────────────────────────────────
    def _display(self):
        now = time.time()
        elapsed = now - self.start_time
        print('\033[H', end='')  # cursor home (pas de clear → pas de flash)

        print(f'{BOLD}╔══════════════════════════════════════════════════════════════╗{RST}')
        print(f'{BOLD}║         🤖  ROBOT TOPIC MONITOR  —  t={elapsed:6.1f}s              ║{RST}')
        print(f'{BOLD}╚══════════════════════════════════════════════════════════════╝{RST}')
        print()

        groups = [
            ('📡 STRATEGY', ['/start_signal', '/strategy/state', '/strategy/score']),
            ('🚗 MOUVEMENT', ['/cmd_vel', '/wheel_odom', '/ir_line']),
            ('👁️  VISION',   ['/aruco/box_to_flip', '/vision/nearest_box_distance']),
            ('🛡️  SÉCURITÉ', ['/obstacle_alert', '/scan']),
            ('🦾 PINCE',    ['/gripper/status']),
        ]

        for group_name, topics in groups:
            print(f'{C}{BOLD}  {group_name}{RST}')
            for topic in topics:
                d    = self.data[topic]
                age  = (now - d['t']) if d['t'] else None
                cnt  = d['count']
                val  = d['val'] if d['val'] is not None else f'{DIM}— pas de données —{RST}'

                # Indicateur de fraîcheur
                if age is None:
                    freshness = f'{DIM}[offline]{RST}'
                elif age < 0.5:
                    freshness = f'{G}[live]{RST}  '
                elif age < 2.0:
                    freshness = f'{Y}[{age:.1f}s]{RST}'
                else:
                    freshness = f'{R}[{age:.0f}s ago]{RST}'

                # Aligner le nom du topic
                tname = f'{DIM}{topic}{RST}'
                print(f'    {freshness} {tname:<42} {val}  {DIM}#{cnt}{RST}')
            print()

        # ── Aide injection ────────────────────────────────────
        print(f'{DIM}─────────────────────────────────────────────────────────────{RST}')
        print(f'  Injections disponibles (entrée interactive) :')
        print(f'  {Y}s{RST} = publier /start_signal    '
              f'{Y}f{RST} = simuler FLIP ArUco    '
              f'{Y}k{RST} = simuler KEEP ArUco')
        print(f'  {Y}i{RST} = simuler IR ligne centrée  '
              f'{Y}q{RST} = quitter')
        print(f'{DIM}─────────────────────────────────────────────────────────────{RST}')

    # ─────────────────────────────────────────────────────────
    #  Injections
    # ─────────────────────────────────────────────────────────
    def _inject_start_once(self):
        msg = Bool(); msg.data = True
        self.start_pub.publish(msg)
        self.get_logger().info('💉 Injection /start_signal = True')

    def _inject_aruco_once(self):
        msg = String(); msg.data = 'FLIP:JAUNE:0.92'
        self.aruco_pub.publish(msg)
        self.get_logger().info('💉 Injection /aruco/box_to_flip = FLIP:JAUNE:0.92')

    def inject_start(self):
        msg = Bool(); msg.data = True
        self.start_pub.publish(msg)
        print(f'{G}💉 /start_signal publié{RST}')

    def inject_flip(self):
        msg = String(); msg.data = 'FLIP:JAUNE:0.95'
        self.aruco_pub.publish(msg)
        print(f'{G}💉 /aruco/box_to_flip = FLIP:JAUNE:0.95{RST}')

    def inject_keep(self):
        msg = String(); msg.data = 'KEEP:BLEU:0.88'
        self.aruco_pub.publish(msg)
        print(f'{G}💉 /aruco/box_to_flip = KEEP:BLEU:0.88{RST}')

    def inject_ir_center(self):
        msg = UInt8(); msg.data = 0b00100  # capteur central actif
        self.ir_pub.publish(msg)
        print(f'{G}💉 /ir_line = 0b00100 (centre){RST}')


# ─────────────────────────────────────────────────────────────
#  Main avec input non-bloquant
# ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test-start', action='store_true',
                        help='Injecte /start_signal=True après 2s')
    parser.add_argument('--test-aruco', action='store_true',
                        help='Injecte ArUco FLIP après 4s')
    args = parser.parse_args()

    rclpy.init()
    node = TopicMonitor(inject_start=args.test_start, inject_aruco=args.test_aruco)

    import threading
    import select

    # Thread ROS en arrière-plan
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    print('\033[2J\033[H', end='')

    try:
        while rclpy.ok():
            # Input non-bloquant
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip().lower()
                if key == 'q':
                    break
                elif key == 's':
                    node.inject_start()
                elif key == 'f':
                    node.inject_flip()
                elif key == 'k':
                    node.inject_keep()
                elif key == 'i':
                    node.inject_ir_center()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()