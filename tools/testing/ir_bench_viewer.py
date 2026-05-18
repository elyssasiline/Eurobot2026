#!/usr/bin/env python3
"""
tools/ir_bench_viewer.py

Viewer live pour le banc IR — à lancer EN PARALLÈLE de robot.launch.py.

Écoute les vrais topics publiés par la Teensy IR + la state machine :
  /ir_position       (std_msgs/Int32)    — position 0–14000 / -1 perdu / -2 intersection
  /ir_line           (std_msgs/UInt16)   — masque brut 15 bits
  /cmd_vel_raw       (geometry_msgs/Twist) — instructions envoyées par la state machine
  /strategy/state    (std_msgs/String)   — état courant de la machine
  /ir_intersection   (std_msgs/Bool)     — flag intersection Teensy

N'envoie RIEN, ne publie RIEN. Pure lecture.

Usage (sur la Raspberry, dans un terminal séparé) :
  source ~/ros2_ws/install/setup.bash
  python3 tools/ir_bench_viewer.py
  python3 tools/ir_bench_viewer.py --team yellow
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
import sys
import threading
import time
import argparse

# ── Constantes IR (identiques à ir_config.h) ─────────────────
IR_COUNT           = 15
IR_VALID_COUNT     = 13
IR_IGNORE_MASK     = (1 << 10) | (1 << 14)  # C11, C15 ignorés
LINE_LOST          = -1
LINE_INTERSECTION  = -2
IR_CENTER          = 7000

# ── Couleurs ANSI ─────────────────────────────────────────────
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


def position_to_correction(ir_position: int, team: str) -> tuple:
    """
    Reproduit exactement _compute_line_correction() + logique intersection
    de state_machine_node.py — pour vérifier la cohérence avec /cmd_vel_raw.
    """
    if ir_position == LINE_LOST:
        return 0.0, f'{DIM}ligne perdue — maintien dernière correction{RST}'
    if ir_position == LINE_INTERSECTION:
        sign = +1.0 if team == 'blue' else -1.0
        direction = 'gauche' if sign > 0 else 'droite'
        return sign * 0.6, f'{M}INTERSECTION → virage {direction}{RST}'

    error = (ir_position - IR_CENTER) / IR_CENTER
    Kp    = 1.2
    correction = -Kp * error

    if abs(error) < 0.05:
        desc = f'{G}centré — tout droit{RST}'
    elif correction > 0:
        desc = f'{B}décalé droite ({error:+.2f}) → correction gauche{RST}'
    else:
        desc = f'{R}décalé gauche ({error:+.2f}) → correction droite{RST}'
    return correction, desc


class IRBenchViewer(Node):

    def __init__(self, team: str):
        super().__init__('ir_bench_viewer')
        self.team = team

        # ── État reçu ────────────────────────────────────────
        self.ir_position       = None   # Int32
        self.ir_mask           = None   # UInt16
        self.cmd_vx            = None   # float
        self.cmd_wz            = None   # float
        self.strategy_state    = '—'
        self.ir_intersection   = None   # Bool depuis Teensy

        self.t_ir_pos          = None   # timestamps dernière réception
        self.t_ir_mask         = None
        self.t_cmd             = None

        self.cmd_count         = 0
        self.ir_count          = 0

        # Historique angular.z pour mini-graphe (80 valeurs)
        self.wz_history        = []
        # Historique cohérence (pour taux de divergence)
        self.coherence_history = []

        # ── Subscribers ──────────────────────────────────────
        self.create_subscription(Int32,  '/ir_position',    self._cb_ir_pos,    10)
        self.create_subscription(UInt16, '/ir_line',        self._cb_ir_mask,   10)
        self.create_subscription(Twist,  '/cmd_vel_raw',    self._cb_cmd,       10)
        self.create_subscription(String, '/strategy/state', self._cb_state,     10)
        self.create_subscription(Bool,   '/ir_intersection',self._cb_intersect, 10)

        # ── Timer affichage 10 Hz ─────────────────────────────
        self.create_timer(0.10, self._display)

        self.get_logger().info('ir_bench_viewer démarré — en écoute…')

    # ── Callbacks ────────────────────────────────────────────

    def _cb_ir_pos(self, msg: Int32):
        self.ir_position = msg.data
        self.t_ir_pos    = time.time()
        self.ir_count   += 1

    def _cb_ir_mask(self, msg: UInt16):
        self.ir_mask  = msg.data
        self.t_ir_mask = time.time()

    def _cb_cmd(self, msg: Twist):
        self.cmd_vx   = msg.linear.x
        self.cmd_wz   = msg.angular.z
        self.t_cmd    = time.time()
        self.cmd_count += 1

    def _cb_state(self, msg: String):
        self.strategy_state = msg.data

    def _cb_intersect(self, msg: Bool):
        self.ir_intersection = msg.data

    # ── Helpers affichage ────────────────────────────────────

    def _age_str(self, t) -> str:
        if t is None:
            return f'{R}jamais reçu{RST}'
        age = time.time() - t
        if age < 0.2:
            return f'{G}OK ({age*1000:.0f}ms){RST}'
        if age < 1.0:
            return f'{Y}{age*1000:.0f}ms{RST}'
        return f'{R}TIMEOUT ({age:.1f}s){RST}'

    def _sensor_strip(self, mask: int) -> str:
        chars = []
        for i in range(IR_COUNT):
            if IR_IGNORE_MASK & (1 << i):
                chars.append(f'{DIM}×{RST}')
            elif mask & (1 << i):
                chars.append(f'{Y}█{RST}')
            else:
                chars.append(f'{DIM}░{RST}')
        return ' '.join(chars)

    def _bar(self, value: float, min_v: float, max_v: float,
             width: int = 24, fill='█', empty='░') -> str:
        ratio  = max(0.0, min(1.0, (value - min_v) / (max_v - min_v)))
        filled = round(ratio * width)
        return fill * filled + empty * (width - filled)

    def _mini_graph(self, history: list, width: int = 44) -> str:
        h = history[-width:]
        if not h:
            return ' ' * width
        line = []
        for v in h:
            if abs(v) < 0.08:
                line.append(f'{G}│{RST}')
            elif v > 0:
                line.append(f'{B}▲{RST}')
            else:
                line.append(f'{R}▼{RST}')
        return ' ' * (width - len(line)) + ''.join(line)

    # ── Affichage principal ───────────────────────────────────

    def _display(self):
        now = time.time()

        # Calcul correction attendue
        if self.ir_position is not None:
            expected_wz, corr_desc = position_to_correction(self.ir_position, self.team)
        else:
            expected_wz, corr_desc = 0.0, f'{DIM}en attente de /ir_position…{RST}'

        # Historique wz reçu
        if self.cmd_wz is not None:
            self.wz_history.append(self.cmd_wz)
            if len(self.wz_history) > 80:
                self.wz_history.pop(0)

        # Cohérence
        coherent = None
        if self.cmd_wz is not None and self.t_cmd and (now - self.t_cmd) < 0.4:
            diff     = abs(self.cmd_wz - expected_wz)
            coherent = diff < 0.08
            self.coherence_history.append(coherent)
            if len(self.coherence_history) > 50:
                self.coherence_history.pop(0)

        # ── Rendu ─────────────────────────────────────────────
        print('\033[H', end='')  # home sans clear (évite le flash)

        print(f'{BOLD}╔══════════════════════════════════════════════════════════════╗{RST}')
        print(f'{BOLD}║    🔬  IR BENCH VIEWER — lecture capteurs réels              ║{RST}')
        print(f'{BOLD}║    équipe={self.team:<8}  topics ROS2 live                       ║{RST}')
        print(f'{BOLD}╚══════════════════════════════════════════════════════════════╝{RST}')
        print()

        # ── Santé des topics ─────────────────────────────────
        print(f'  {BOLD}Santé des topics :{RST}')
        print(f'    /ir_position    {self._age_str(self.t_ir_pos)}'
              f'   ({self.ir_count} msgs reçus)')
        print(f'    /ir_line        {self._age_str(self.t_ir_mask)}')
        print(f'    /cmd_vel_raw    {self._age_str(self.t_cmd)}'
              f'   ({self.cmd_count} msgs reçus)')
        print()

        # ── Capteurs IR (masque brut) ─────────────────────────
        print(f'  {BOLD}Capteurs IR (C01→C15) :{RST}')
        if self.ir_mask is not None:
            print(f'  {self._sensor_strip(self.ir_mask)}')
            label_line = '  C01' + ' ' * 24 + 'C08' + ' ' * 22 + 'C15'
            print(f'{DIM}{label_line}{RST}')

            # Flag intersection depuis Teensy
            if self.ir_intersection:
                print(f'  {M}⚡ INTERSECTION confirmée par la Teensy{RST}')
            else:
                print(f'  {DIM}   (pas d\'intersection){RST}')
        else:
            print(f'  {DIM}  en attente de /ir_line…{RST}')
            print()
        print()

        # ── Position IR ──────────────────────────────────────
        print(f'  {BOLD}Position IR :{RST}', end='  ')
        if self.ir_position is None:
            print(f'{DIM}en attente…{RST}')
        elif self.ir_position == LINE_LOST:
            print(f'{R}LIGNE PERDUE (-1){RST}')
        elif self.ir_position == LINE_INTERSECTION:
            print(f'{M}INTERSECTION (-2){RST}')
        else:
            pos_col = (G if abs(self.ir_position - IR_CENTER) < 1000
                       else Y if abs(self.ir_position - IR_CENTER) < 3000
                       else R)
            print(f'{pos_col}{self.ir_position}{RST} / 14000')
            bar = self._bar(self.ir_position, 0, 14000)
            ptr_pos = round((self.ir_position / 14000) * 24)
            print(f'  G [{bar}] D')
            print(f'     {" " * ptr_pos}↑')

        print()

        # ── Correction attendue (calculée ici) ───────────────
        print(f'  {BOLD}Correction attendue  :{RST}  {expected_wz:+.3f} rad/s')
        print(f'  {corr_desc}')
        print()

        # ── Commande reçue de la state machine ───────────────
        print(f'  {BOLD}Commande reçue /cmd_vel_raw :{RST}')
        if self.cmd_vx is None:
            print(f'    {DIM}en attente…{RST}')
            if self.strategy_state == 'INIT':
                print(f'    {DIM}→ envoie le top départ : ros2 topic pub --once /start_signal std_msgs/msg/Bool "data: true"{RST}')
        else:
            cmd_age = now - self.t_cmd if self.t_cmd else 99
            fresh   = cmd_age < 0.3
            col     = G if fresh and self.cmd_vx > 0 else (Y if fresh else DIM)

            print(f'    vx  = {col}{self.cmd_vx:+.3f} m/s{RST}   '
                  f'{"→ robot avance" if self.cmd_vx > 0.01 else "→ robot stoppé" if self.cmd_vx == 0.0 else "→ recule"}')
            print(f'    wz  = {col}{self.cmd_wz:+.3f} rad/s{RST}  '
                  f'{"→ tourne gauche" if self.cmd_wz > 0.05 else "→ tourne droite" if self.cmd_wz < -0.05 else "→ tout droit"}')

            # Cohérence
            if coherent is not None:
                if coherent:
                    pct = (sum(self.coherence_history) / len(self.coherence_history) * 100
                           if self.coherence_history else 100)
                    print(f'    {G}✓ cohérent avec correction attendue  '
                          f'(taux OK : {pct:.0f}%){RST}')
                else:
                    diff = abs(self.cmd_wz - expected_wz)
                    print(f'    {Y}⚠ écart {diff:.3f} rad/s vs correction attendue{RST}')
                    print(f'    {DIM}  Normal si : état AVOID, GRAB, PLACE, ou délai 3s post-démarrage{RST}')
        print()

        # ── Mini graphe wz ───────────────────────────────────
        print(f'  {BOLD}Historique angular.z reçu :{RST}  '
              f'{DIM}▲=gauche  ▼=droite  │=tout droit{RST}')
        print(f'  [{self._mini_graph(self.wz_history, 44)}]')
        print()

        # ── État machine à états ─────────────────────────────
        state_col = {
            'FOLLOW_LINE':   G,
            'APPROACH_BOX':  C,
            'BLIND_ADVANCE': Y,
            'GRAB':          B,
            'PLACE':         M,
            'AVOID':         R,
            'STOP':          DIM,
            'INIT':          DIM,
            '—':             DIM,
        }.get(self.strategy_state, W)

        print(f'  {BOLD}État machine à états :{RST}  {state_col}{self.strategy_state}{RST}')
        print()

        # ── Guide rapide ─────────────────────────────────────
        print(f'{DIM}  ─────────────────────────────────────────────────────────────{RST}')
        print(f'{DIM}  Ce que tu dois voir lors du bench :{RST}')
        print(f'{DIM}  • Ligne centrée    → wz ≈ 0.000   vx > 0{RST}')
        print(f'{DIM}  • Ligne à droite   → wz < 0       correction vers droite{RST}')
        print(f'{DIM}  • Ligne à gauche   → wz > 0       correction vers gauche{RST}')
        print(f'{DIM}  • Intersection     → wz ≈ ±0.600  virage {"gauche" if self.team=="blue" else "droite"} ({self.team}){RST}')
        print(f'{DIM}  • Ligne perdue     → vx > 0       wz = dernière valeur{RST}')
        print(f'{DIM}  ─────────────────────────────────────────────────────────────{RST}')
        print(f'{DIM}  Ctrl+C pour quitter{RST}')


def main():
    parser = argparse.ArgumentParser(description='Viewer live banc IR')
    parser.add_argument('--team', default='blue', choices=['blue', 'yellow'],
                        help='Équipe (pour calcul direction intersection)')
    args = parser.parse_args()

    rclpy.init()
    node = IRBenchViewer(team=args.team)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Effacer l'écran une seule fois au départ
    print('\033[2J\033[H', end='')
    print('\033[?25l', end='')  # cacher le curseur

    try:
        while rclpy.ok():
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        print('\033[?25h', end='')  # réafficher le curseur
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()