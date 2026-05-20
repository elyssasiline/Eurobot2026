#!/usr/bin/env python3
"""
testing/demo_parcours.py — v2

Lit /obstacle_alert ET /scan directement pour détecter les obstacles
même quand le robot est à l'arrêt. Pas de time.sleep() dans les callbacks.

Usage :
  python3 tools/testing/demo_parcours.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math


# ============================================================
#  PARAMÈTRES
# ============================================================

LINEAR_SPEED     = 0.18   # m/s
TURN_SPEED       = 0.70   # rad/s

T_AVANCE_1       = 4.0    # s — avance initiale
T_WAIT_OBS       = 5.0    # s — attente avant contournement
T_PIVOT_90       = 1.35   # s — ~90° (à calibrer !)
T_AVANCE_CONT    = 2.5    # s — avance latérale contournement
T_DEMO_PIVOT     = 1.2    # s — pivot démo
T_STRAFFE        = 1.5    # s — straffe démo
T_RECUL          = 3.0    # s — recul final
T_PAUSE          = 0.4    # s — pause entre mouvements

OBSTACLE_DIST    = 0.25   # m — seuil détection obstacle direct sur /scan
FRONT_ANGLE      = 45.0   # degrés — cône avant surveillé


# ============================================================
#  ÉTATS
# ============================================================

class S:
    PAUSE_BEFORE     = "PAUSE_BEFORE"
    AVANCE_1         = "AVANCE_1"
    WAIT_OBSTACLE    = "WAIT_OBSTACLE"
    BYPASS_PIVOT_G   = "BYPASS_PIVOT_G"
    BYPASS_AVANCE    = "BYPASS_AVANCE"
    BYPASS_PIVOT_D   = "BYPASS_PIVOT_D"
    BYPASS_REALIGN   = "BYPASS_REALIGN"
    PAUSE            = "PAUSE"          # pause générique entre mouvements
    DEMO_PIVOT_G     = "DEMO_PIVOT_G"
    DEMO_PIVOT_D     = "DEMO_PIVOT_D"
    DEMO_STRAFFE_G   = "DEMO_STRAFFE_G"
    DEMO_STRAFFE_D   = "DEMO_STRAFFE_D"
    RECUL            = "RECUL"
    FIN              = "FIN"


class DemoParcours(Node):

    def __init__(self):
        super().__init__('demo_parcours')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        # Deux sources d'info obstacle : alert (quand robot bouge) + scan direct
        self.obstacle_alert  = 'OK'
        self.front_dist      = float('inf')

        self.create_subscription(String,    '/obstacle_alert', self._cb_alert, 10)
        self.create_subscription(LaserScan, '/scan',           self._cb_scan,  10)

        self.state       = S.PAUSE_BEFORE
        self.state_t0    = self.get_clock().now()
        self.next_state  = S.AVANCE_1   # état après la pause initiale
        self.obs_wait_t  = None

        self.create_timer(1.0 / 20, self._spin)

        self.get_logger().info("=== DEMO PARCOURS — démarrage dans 2s ===")

    # ----------------------------------------------------------
    def _cb_alert(self, msg: String):
        self.obstacle_alert = msg.data

    def _cb_scan(self, msg: LaserScan):
        """Calcule la distance minimale dans le cône avant."""
        half = FRONT_ANGLE / 2.0
        min_dist = float('inf')
        for i, d in enumerate(msg.ranges):
            if d < msg.range_min or d > msg.range_max or math.isnan(d) or math.isinf(d):
                continue
            angle = math.degrees(msg.angle_min + i * msg.angle_increment)
            while angle >  180: angle -= 360
            while angle < -180: angle += 360
            if -half <= angle <= half:
                min_dist = min(min_dist, d)
        self.front_dist = min_dist

    def _obstacle(self) -> bool:
        """Retourne True si obstacle détecté par l'une ou l'autre source."""
        return self.obstacle_alert == 'DANGER' or self.front_dist < OBSTACLE_DIST

    # ----------------------------------------------------------
    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.state_t0).nanoseconds / 1e9

    def _transition(self, new_state: str):
        self.get_logger().info(f"  → {new_state}")
        self.state    = new_state
        self.state_t0 = self.get_clock().now()

    def _transition_after_pause(self, next_state: str):
        """Pause de T_PAUSE secondes puis transition vers next_state."""
        self.next_state = next_state
        self._stop()
        self._transition(S.PAUSE)

    def _pub(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x  = vx
        msg.linear.y  = vy
        msg.angular.z = wz
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._pub()

    # ----------------------------------------------------------
    def _spin(self):
        e = self._elapsed()

        # ── PAUSE INITIALE ────────────────────────────────────
        if self.state == S.PAUSE_BEFORE:
            if e >= 2.0:
                self.get_logger().info("[AVANCE_1] Avance vers l'obstacle...")
                self._transition(S.AVANCE_1)

        # ── AVANCE_1 ──────────────────────────────────────────
        elif self.state == S.AVANCE_1:
            if self._obstacle():
                self._stop()
                self.get_logger().warn(
                    f"🚧 Obstacle à {self.front_dist:.2f}m — attente {T_WAIT_OBS}s"
                )
                self.obs_wait_t = self.get_clock().now()
                self._transition(S.WAIT_OBSTACLE)
            elif e >= T_AVANCE_1:
                self._stop()
                self.get_logger().info("Pas d'obstacle — démo mouvements")
                self._transition_after_pause(S.DEMO_PIVOT_G)
            else:
                self._pub(vx=LINEAR_SPEED)

        # ── WAIT_OBSTACLE ─────────────────────────────────────
        elif self.state == S.WAIT_OBSTACLE:
            self._stop()
            waited = (self.get_clock().now() - self.obs_wait_t).nanoseconds / 1e9
            if not self._obstacle():
                self.get_logger().info("✅ Obstacle dégagé — reprise")
                self._transition(S.AVANCE_1)
            elif waited >= T_WAIT_OBS:
                self.get_logger().warn("⏰ Obstacle persistant — contournement")
                self._transition(S.BYPASS_PIVOT_G)

        # ── BYPASS ────────────────────────────────────────────
        elif self.state == S.BYPASS_PIVOT_G:
            if e >= T_PIVOT_90:
                self._transition_after_pause(S.BYPASS_AVANCE)
            else:
                self._pub(wz=TURN_SPEED)

        elif self.state == S.BYPASS_AVANCE:
            if e >= T_AVANCE_CONT:
                self._transition_after_pause(S.BYPASS_PIVOT_D)
            else:
                self._pub(vx=LINEAR_SPEED)

        elif self.state == S.BYPASS_PIVOT_D:
            if e >= T_PIVOT_90:
                self._transition_after_pause(S.BYPASS_REALIGN)
            else:
                self._pub(wz=-TURN_SPEED)

        elif self.state == S.BYPASS_REALIGN:
            if e >= T_AVANCE_CONT:
                self._stop()
                self.get_logger().info("Contournement terminé → démo mouvements")
                self._transition_after_pause(S.DEMO_PIVOT_G)
            else:
                self._pub(vx=LINEAR_SPEED)

        # ── PAUSE générique ───────────────────────────────────
        elif self.state == S.PAUSE:
            if e >= T_PAUSE:
                self._transition(self.next_state)

        # ── DÉMO PIVOT GAUCHE ─────────────────────────────────
        elif self.state == S.DEMO_PIVOT_G:
            if e >= T_DEMO_PIVOT:
                self.get_logger().info("Pivot G OK")
                self._transition_after_pause(S.DEMO_PIVOT_D)
            else:
                self._pub(wz=TURN_SPEED)

        # ── DÉMO PIVOT DROITE (recentrage) ────────────────────
        elif self.state == S.DEMO_PIVOT_D:
            if e >= T_DEMO_PIVOT * 2:
                self.get_logger().info("Pivot D OK")
                self._transition_after_pause(S.DEMO_STRAFFE_G)
            else:
                self._pub(wz=-TURN_SPEED)

        # ── DÉMO STRAFFE GAUCHE ───────────────────────────────
        elif self.state == S.DEMO_STRAFFE_G:
            if e >= T_STRAFFE:
                self.get_logger().info("Straffe G OK")
                self._transition_after_pause(S.DEMO_STRAFFE_D)
            else:
                self._pub(vy=-LINEAR_SPEED)

        # ── DÉMO STRAFFE DROITE ───────────────────────────────
        elif self.state == S.DEMO_STRAFFE_D:
            if e >= T_STRAFFE:
                self.get_logger().info("Straffe D OK")
                self._transition_after_pause(S.RECUL)
            else:
                self._pub(vy=LINEAR_SPEED)

        # ── RECUL ─────────────────────────────────────────────
        elif self.state == S.RECUL:
            if e >= T_RECUL:
                self._stop()
                self._transition(S.FIN)
            else:
                self._pub(vx=-LINEAR_SPEED)

        # ── FIN ───────────────────────────────────────────────
        elif self.state == S.FIN:
            self._stop()
            self.get_logger().info("=== DEMO TERMINÉE ===")
            raise SystemExit


# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = DemoParcours()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()