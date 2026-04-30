#!/usr/bin/env python3
"""
tools/simulate_mission.py

Simule un match complet pour tester la state machine sans matériel.
Rejoue le scénario suivant :

  t=0s   → top départ
  t=2s   → IR ligne centrée (FOLLOW_LINE actif)
  t=4s   → ArUco détecté (FLIP:JAUNE) → APPROACH_BOX
  t=6s   → tag perdu → BLIND_ADVANCE
  t=7s   → capteur distance confirme position → GRAB
  t=10s  → PLACE (pince ouverte)
  t=13s  → retour FOLLOW_LINE
  t=15s  → obstacle DANGER → AVOID
  t=17s  → obstacle dégagé → reprise
  t=20s  → 2ème ArUco (KEEP:BLEU)
  t=30s  → fin simulée

En parallèle, affiche tous les topics en live.

Usage :
  python3 simulate_mission.py
  python3 simulate_mission.py --team yellow
  python3 simulate_mission.py --fast   (x3 vitesse)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32, UInt8, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time
import sys
import threading
import argparse


# ── Couleurs ANSI ────────────────────────────────────────────
R    = '\033[91m'
G    = '\033[92m'
Y    = '\033[93m'
B    = '\033[94m'
C    = '\033[96m'
M    = '\033[95m'
W    = '\033[97m'
DIM  = '\033[2m'
RST  = '\033[0m'
BOLD = '\033[1m'


# ── Scénario ─────────────────────────────────────────────────
# (délai_s, description, fonction_injection)
SCENARIO = [
    ( 1.0, 'Top départ envoyé',                    'start'),
    ( 2.0, 'IR ligne centrée (0b00100)',            'ir_center'),
    ( 4.0, 'ArUco FLIP:JAUNE détecté',             'aruco_flip'),
    ( 4.5, 'ArUco FLIP:JAUNE (2ème frame)',        'aruco_flip'),
    ( 5.0, 'ArUco FLIP:JAUNE (3ème frame)',        'aruco_flip'),
    ( 6.0, 'Tag perdu (plus d\'ArUco publié)',     'aruco_lost'),
    ( 7.2, 'Capteur distance : 0.07m (caisse OK)', 'distance_ok'),
    ( 9.0, 'Pince fermée (gripper CLOSED)',        'gripper_closed'),
    (11.0, 'Zone dépôt détectée (IR bande)',       'ir_deposit'),
    (12.5, 'Pince ouverte (gripper OPEN)',         'gripper_open'),
    (14.0, 'IR ligne centrée (retour ligne)',       'ir_center'),
    (16.0, 'OBSTACLE DANGER simulé',               'obstacle_danger'),
    (18.5, 'Obstacle dégagé',                      'obstacle_ok'),
    (19.0, 'IR ligne centrée (reprend)',            'ir_center'),
    (21.0, 'ArUco KEEP:BLEU détecté',              'aruco_keep'),
    (21.5, 'ArUco KEEP:BLEU (2ème frame)',         'aruco_keep'),
    (22.0, 'Tag perdu',                            'aruco_lost'),
    (23.5, 'Capteur distance : 0.07m',             'distance_ok'),
    (25.0, 'Pince fermée',                         'gripper_closed'),
    (27.0, 'Zone dépôt détectée',                  'ir_deposit'),
    (28.5, 'Pince ouverte',                        'gripper_open'),
    (30.0, 'IR ligne centrée (retour ligne)',       'ir_center'),
]


class SimulatorNode(Node):

    def __init__(self, team='blue', speed_factor=1.0):
        super().__init__('mission_simulator')

        self.team         = team
        self.speed_factor = speed_factor   # >1 = plus rapide
        self.sim_start    = None
        self.events_done  = set()
        self.log          = []             # journal des événements

        # ── Topics reçus (monitoring) ────────────────────────
        self.rx = {
            '/strategy/state':  {'val': '—', 't': None, 'prev': '—', 'changes': []},
            '/strategy/score':  {'val': '—', 't': None},
            '/cmd_vel':         {'val': '—', 't': None, 'count': 0},
            '/obstacle_alert':  {'val': '—', 't': None},
            '/gripper/command': {'val': '—', 't': None, 'history': []},
        }

        # ── Publishers (simulation capteurs) ─────────────────
        self.pub_start    = self.create_publisher(Bool,    '/start_signal',                10)
        self.pub_ir       = self.create_publisher(UInt8,   '/ir_line',                     10)
        self.pub_aruco    = self.create_publisher(String,  '/aruco/box_to_flip',           10)
        self.pub_dist     = self.create_publisher(Float32, '/vision/nearest_box_distance', 10)
        self.pub_alert    = self.create_publisher(String,  '/obstacle_alert',              10)
        self.pub_odom     = self.create_publisher(Odometry,'/wheel_odom',                  10)
        self.pub_gripper  = self.create_publisher(String,  '/gripper/status',              10)

        # ── Subscribers (monitoring state machine) ───────────
        self.create_subscription(String, '/strategy/state',  self._cb_state,   10)
        self.create_subscription(Int32,  '/strategy/score',  self._cb_score,   10)
        self.create_subscription(Twist,  '/cmd_vel',         self._cb_cmd,     10)
        self.create_subscription(String, '/obstacle_alert',  self._cb_alert,   10)
        self.create_subscription(String, '/gripper/command', self._cb_gripper, 10)

        # ── Odométrie simulée (avance linéaire) ──────────────
        self.sim_odom_x   = 0.0
        self.odom_running = False

        # ── Timers ───────────────────────────────────────────
        self.create_timer(0.05,  self._run_scenario)   # scénario 20 Hz
        self.create_timer(0.02,  self._publish_odom)   # odom 50 Hz
        self.create_timer(0.15,  self._display)        # affichage ~7 Hz

        self.sim_start = time.time()
        print('\033[2J\033[H', end='')
        self._log(f'Simulateur démarré — équipe={team}  vitesse x{speed_factor}')

    # ── Callbacks monitoring ─────────────────────────────────

    def _cb_state(self, msg: String):
        prev = self.rx['/strategy/state']['val']
        if msg.data != prev:
            self.rx['/strategy/state']['prev'] = prev
            self.rx['/strategy/state']['changes'].append(
                (self._t(), prev, msg.data)
            )
            self._log(f'STATE  {prev} → {msg.data}')
        self.rx['/strategy/state']['val'] = msg.data
        self.rx['/strategy/state']['t']   = time.time()

    def _cb_score(self, msg: Int32):
        self.rx['/strategy/score']['val'] = str(msg.data)
        self.rx['/strategy/score']['t']   = time.time()

    def _cb_cmd(self, msg: Twist):
        vx = msg.linear.x; wz = msg.angular.z
        self.rx['/cmd_vel']['val']   = f'vx={vx:+.2f} wz={wz:+.2f}'
        self.rx['/cmd_vel']['t']     = time.time()
        self.rx['/cmd_vel']['count'] += 1
        # Simuler mouvement si cmd active
        self.odom_running = (vx != 0.0)

    def _cb_alert(self, msg: String):
        self.rx['/obstacle_alert']['val'] = msg.data
        self.rx['/obstacle_alert']['t']   = time.time()

    def _cb_gripper(self, msg: String):
        self.rx['/gripper/command']['val'] = msg.data
        self.rx['/gripper/command']['t']   = time.time()
        self.rx['/gripper/command']['history'].append((self._t(), msg.data))
        self._log(f'GRIPPER commande reçue: {msg.data}')

    # ── Odométrie simulée ────────────────────────────────────

    def _publish_odom(self):
        if self.odom_running:
            self.sim_odom_x += 0.20 * 0.02  # 20cm/s * 20ms

        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = self.sim_odom_x
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(msg)

    # ── Scénario ─────────────────────────────────────────────

    def _t(self) -> float:
        """Temps simulé écoulé (en tenant compte du speed_factor)."""
        return (time.time() - self.sim_start) * self.speed_factor

    def _run_scenario(self):
        t = self._t()

        for i, (delay, desc, action) in enumerate(SCENARIO):
            if i in self.events_done:
                continue
            if t >= delay:
                self.events_done.add(i)
                self._log(f't={delay:.1f}s → {desc}')
                self._inject(action)

    def _inject(self, action: str):
        """Publie le message correspondant à l'action."""

        if action == 'start':
            m = Bool(); m.data = True
            self.pub_start.publish(m)

        elif action == 'ir_center':
            m = UInt8(); m.data = 0b00100
            self.pub_ir.publish(m)

        elif action == 'aruco_flip':
            m = String()
            m.data = f'FLIP:{"JAUNE" if self.team == "blue" else "BLEU"}:0.93'
            self.pub_aruco.publish(m)

        elif action == 'aruco_keep':
            m = String()
            m.data = f'KEEP:{"BLEU" if self.team == "blue" else "JAUNE"}:0.88'
            self.pub_aruco.publish(m)

        elif action == 'aruco_lost':
            # On ne publie rien — la state machine détecte l'absence de message
            pass

        elif action == 'distance_ok':
            m = Float32(); m.data = 0.07   # 7cm — caisse détectée sous le robot
            self.pub_dist.publish(m)

        elif action == 'gripper_closed':
            m = String(); m.data = 'CLOSED'
            self.pub_gripper.publish(m)

        elif action == 'gripper_open':
            m = String(); m.data = 'OPEN'
            self.pub_gripper.publish(m)

        elif action == 'ir_deposit':
            m = UInt8(); m.data = 0b11111   # tous capteurs → bande blanche dépôt
            self.pub_ir.publish(m)

        elif action == 'obstacle_danger':
            m = String(); m.data = 'DANGER'
            self.pub_alert.publish(m)

        elif action == 'obstacle_ok':
            m = String(); m.data = 'OK'
            self.pub_alert.publish(m)

    # ── Log ──────────────────────────────────────────────────

    def _log(self, msg: str):
        t = self._t()
        self.log.append((t, msg))
        if len(self.log) > 40:
            self.log.pop(0)

    # ── Affichage ────────────────────────────────────────────

    def _display(self):
        print('\033[H', end='')
        t = self._t()

        print(f'{BOLD}╔══════════════════════════════════════════════════════════════╗{RST}')
        print(f'{BOLD}║    🤖  SIMULATION MISSION  —  t={t:6.1f}s  équipe={self.team:<6}      ║{RST}')
        print(f'{BOLD}╚══════════════════════════════════════════════════════════════╝{RST}')
        print()

        # ── État courant ─────────────────────────────────────
        state = self.rx['/strategy/state']['val']
        score = self.rx['/strategy/score']['val']
        alert = self.rx['/obstacle_alert']['val']
        cmd   = self.rx['/cmd_vel']['val']
        grip  = self.rx['/gripper/command']['val']

        state_col = {
            'INIT': DIM, 'FOLLOW_LINE': G, 'APPROACH_BOX': C,
            'BLIND_ADVANCE': Y, 'GRAB': B, 'PLACE': M,
            'AVOID': R, 'STOP': DIM, '—': DIM,
        }.get(state, W)

        state_icon = {
            'INIT': '⏳', 'FOLLOW_LINE': '➡️ ', 'APPROACH_BOX': '📦',
            'BLIND_ADVANCE': '🔲', 'GRAB': '🦾', 'PLACE': '🎯',
            'AVOID': '🚨', 'STOP': '🛑', '—': '❓',
        }.get(state, '?')

        alert_col  = {'OK': G, 'WARNING': Y, 'DANGER': R, '—': DIM}.get(alert, W)
        cmd_col    = G if 'vx=+' in str(cmd) else (R if cmd == '—' else DIM)
        grip_col   = {
            'CLOSE': R, 'OPEN': G, 'FLIP': Y, 'CLOSED': R, 'OPEN': G, '—': DIM
        }.get(grip, W)

        print(f'  {BOLD}État machine  :{RST} {state_col}{BOLD}{state_icon} {state:<16}{RST}   Score: 🏆 {score}')
        print(f'  {BOLD}Obstacle      :{RST} {alert_col}{alert}{RST}')
        print(f'  {BOLD}cmd_vel       :{RST} {cmd_col}{cmd}{RST}   {DIM}({self.rx["/cmd_vel"]["count"]} msgs){RST}')
        print(f'  {BOLD}Pince cmd     :{RST} {grip_col}{grip}{RST}')
        print(f'  {BOLD}Odom simulée  :{RST} x={self.sim_odom_x:.3f}m')
        print()

        # ── Transitions d'état observées ─────────────────────
        changes = self.rx['/strategy/state']['changes']
        print(f'  {C}{BOLD}Transitions d\'état observées :{RST}')
        if not changes:
            print(f'    {DIM}aucune — state machine lancée ?{RST}')
        else:
            for (ct, prev, nxt) in changes[-8:]:
                ok = G if nxt not in ('AVOID', 'STOP') else R
                print(f'    {DIM}t={ct:5.1f}s{RST}  {prev:<16} → {ok}{nxt}{RST}')
        print()

        # ── Prochain événement ───────────────────────────────
        next_events = [
            (delay, desc) for i, (delay, desc, _) in enumerate(SCENARIO)
            if i not in self.events_done
        ]
        if next_events:
            nd, ndesc = next_events[0]
            remaining = (nd / self.speed_factor) - (time.time() - self.sim_start)
            remaining = max(0.0, remaining)
            print(f'  {Y}⏭  Prochain événement dans {remaining:.1f}s : {ndesc}{RST}')
        else:
            print(f'  {G}✅ Scénario complet !{RST}')
        print()

        # ── Journal ──────────────────────────────────────────
        print(f'  {BOLD}Journal :{RST}')
        for (lt, lmsg) in self.log[-15:]:
            col = (R if 'DANGER' in lmsg or 'AVOID' in lmsg
                   else G if ('STATE' in lmsg or 'GRIPPER' in lmsg)
                   else DIM)
            print(f'    {DIM}t={lt:5.1f}s{RST}  {col}{lmsg}{RST}')

        # ── Diagnostic si rien ne se passe ──────────────────
        state_t = self.rx['/strategy/state']['t']
        if state_t is None and t > 3.0:
            print()
            print(f'  {R}{BOLD}⚠  /strategy/state muet depuis {t:.0f}s{RST}')
            print(f'  {Y}  → La state machine tourne-t-elle ?{RST}')
            print(f'  {DIM}  ros2 run strategy mission_node{RST}')
            print(f'  {DIM}  ou : ros2 launch robot_bringup robot.launch.py{RST}')

        print()
        print(f'{DIM}  Ctrl+C pour quitter — speed_factor x{self.speed_factor}{RST}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--team',  default='blue', choices=['blue', 'yellow'])
    parser.add_argument('--fast',  action='store_true', help='Scénario x3 vitesse')
    args = parser.parse_args()

    speed = 3.0 if args.fast else 1.0

    rclpy.init()
    node = SimulatorNode(team=args.team, speed_factor=speed)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Résumé final
        print('\n\n')
        print(f'{BOLD}══ RÉSUMÉ SIMULATION ══{RST}')
        changes = node.rx['/strategy/state']['changes']
        print(f'  Transitions observées : {len(changes)}')
        for (ct, prev, nxt) in changes:
            print(f'    t={ct:.1f}s  {prev} → {nxt}')
        grip_hist = node.rx['/gripper/command']['history']
        print(f'  Commandes pince : {[g for (_, g) in grip_hist]}')
        print(f'  Score final     : {node.rx["/strategy/score"]["val"]}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()