#!/usr/bin/env python3
"""
strategy/strategy/state_machine_node.py

Machine à états principale du robot Eurobot.

Trajectoire : INIT → FOLLOW_LINE → APPROACH_BOX → GRAB → PLACE → (boucle)
              AVOID peut interrompre n'importe quel état sauf INIT/STOP

États :
  INIT          — attente top départ (corde tirée → /start_signal)
  FOLLOW_LINE   — suivi de ligne noire IR, avance vers prochaine cible
  APPROACH_BOX  — ArUco détecté, avance jusqu'à perte de vue du tag
  BLIND_ADVANCE — tag perdu, avance calculée pour se positionner au-dessus
  GRAB          — commande pince + lecture FLIP/KEEP
  PLACE         — dépôt caisse à la zone, avance jusqu'à marqueur de dépôt
  AVOID         — obstacle détecté, robot stoppé, reprend état précédent
  STOP          — fin de partie (100s) ou erreur fatale

Topics abonnés :
  /start_signal              (std_msgs/Bool)         — top départ
  /ir_line                   (std_msgs/UInt8)         — bitmask capteurs IR [STUB]
  /aruco/box_to_flip         (std_msgs/String)        — 'FLIP:...' ou 'KEEP:...'
  /vision/nearest_box_distance (std_msgs/Float32)     — distance à la caisse (capteur bas)
  /obstacle_alert            (std_msgs/String)        — OK / WARNING / DANGER
  /wheel_odom                (nav_msgs/Odometry)      — odométrie pour distance aveugle
  /gripper/status            (std_msgs/String)        — état pince [STUB camarade]

Topics publiés :
  /cmd_vel_raw               (geometry_msgs/Twist)    — commandes moteurs
  /gripper/command           (std_msgs/String)        — OPEN / CLOSE / FLIP [STUB]
  /strategy/state            (std_msgs/String)        — état courant (debug)
  /strategy/score            (std_msgs/Int32)         — score estimé
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Bool, String, Float32, UInt8, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from enum import Enum, auto
import math
import time


# =============================================================================
#  États
# =============================================================================
class State(Enum):
    INIT          = auto()
    FOLLOW_LINE   = auto()
    APPROACH_BOX  = auto()
    BLIND_ADVANCE = auto()
    GRAB          = auto()
    PLACE         = auto()
    AVOID         = auto()
    STOP          = auto()


# =============================================================================
#  Machine à états
# =============================================================================
class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine')

        # ------------------------------------------------------------------
        # Paramètres
        # ------------------------------------------------------------------
        self.declare_parameter('robot.team', 'blue')
        self.declare_parameter('mission.line_follow_speed', 0.20)
        self.declare_parameter('mission.approach_speed', 0.05)
        self.declare_parameter('mission.grab_distance', 0.08)
        self.declare_parameter('mission.grab_duration', 1.5)
        self.declare_parameter('mission.release_duration', 1.0)
        self.declare_parameter('mission.deposit_timeout', 5.0)
        self.declare_parameter('mission.max_boxes_per_run', 1)
        # Distance à avancer "à l'aveugle" après perte du tag (en mètres)
        self.declare_parameter('mission.blind_advance_distance', 0.15)
        # Durée max du match (secondes)
        self.declare_parameter('mission.match_duration', 100.0)
        # Durée de la rotation à l'intersection (secondes)
        self.declare_parameter('mission.intersection_turn_duration', 0.8)
        # Vitesse angulaire de la rotation à l'intersection (rad/s)
        self.declare_parameter('mission.intersection_turn_speed', 0.6)

        self.team                    = self.get_parameter('robot.team').value
        self.line_speed              = self.get_parameter('mission.line_follow_speed').value
        self.approach_speed          = self.get_parameter('mission.approach_speed').value
        self.grab_distance           = self.get_parameter('mission.grab_distance').value
        self.grab_duration           = self.get_parameter('mission.grab_duration').value
        self.release_duration        = self.get_parameter('mission.release_duration').value
        self.deposit_timeout         = self.get_parameter('mission.deposit_timeout').value
        self.blind_advance_dist      = self.get_parameter('mission.blind_advance_distance').value
        self.match_duration          = self.get_parameter('mission.match_duration').value
        self.intersection_turn_dur   = self.get_parameter('mission.intersection_turn_duration').value
        self.intersection_turn_speed = self.get_parameter('mission.intersection_turn_speed').value

        # ------------------------------------------------------------------
        #  Direction de virage à l'intersection selon l'équipe
        #    blue   → tourne à gauche  → angular.z positif
        #    yellow → tourne à droite  → angular.z négatif
        # ------------------------------------------------------------------
        if self.team == 'blue':
            self._intersection_turn_sign = +1.0
        else:
            self._intersection_turn_sign = -1.0

        self.get_logger().info(
            f'  • Intersections : virage {"gauche" if self.team == "blue" else "droite"}'
            f' (équipe {self.team})'
        )

        # ------------------------------------------------------------------
        # État interne
        # ------------------------------------------------------------------
        self.state          = State.INIT
        self.prev_state     = State.INIT   # pour reprendre après AVOID
        self.match_started  = False
        self.match_start_t  = None
        self.score          = 0
        self.boxes_grabbed  = 0

        # Données capteurs
        self.ir_mask            = 0          # bitmask IR (0 = pas de données)
        self.ir_position        = -1         # -1 = ligne perdue, -2 = intersection
        self.box_to_flip        = None       # 'FLIP' ou 'KEEP'
        self.box_distance       = None       # distance capteur bas (m)
        self.obstacle_status    = 'OK'       # OK / WARNING / DANGER
        self.odom_x             = 0.0        # position odométrique cumulée (m)
        self.odom_y             = 0.0
        self.odom_theta         = 0.0
        self.gripper_status     = 'UNKNOWN'  # état pince

        # Pour BLIND_ADVANCE
        self.blind_start_odom    = None      # snapshot odom au début avance aveugle
        self.blind_distance_done = 0.0

        # Pour GRAB
        self.grab_start_time = None
        self.grab_action     = None          # 'FLIP' ou 'KEEP'

        # Pour PLACE
        self.place_start_time = None

        # ------------------------------------------------------------------
        #  Gestion intersection
        #    _intersection_active  : True pendant la phase de rotation
        #    _intersection_start_t : instant de début de rotation
        # ------------------------------------------------------------------
        self._intersection_active  = False
        self._intersection_start_t = None
        self._last_correction      = 0.0     # dernière correction angulaire connue

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(Bool,    '/start_signal',                self._cb_start,    10)
        self.create_subscription(UInt8,   '/ir_line',                     self._cb_ir,       10)
        self.create_subscription(Int32,   '/ir_position',                 self._cb_ir_pos,   10)
        self.create_subscription(String,  '/aruco/box_to_flip',           self._cb_aruco,    10)
        self.create_subscription(Float32, '/vision/nearest_box_distance', self._cb_distance, 10)
        self.create_subscription(String,  '/obstacle_alert',              self._cb_obstacle, 10)
        self.create_subscription(Odometry,'/wheel_odom',                  self._cb_odom,     10)
        self.create_subscription(String,  '/gripper/status',              self._cb_gripper,  10)

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.cmd_pub     = self.create_publisher(Twist,  '/cmd_vel_raw',     10)
        self.gripper_pub = self.create_publisher(String, '/gripper/command', 10)
        self.state_pub   = self.create_publisher(String, '/strategy/state',  10)
        self.score_pub   = self.create_publisher(Int32,  '/strategy/score',  10)

        # ------------------------------------------------------------------
        # Timer principal : boucle d'exécution à 20 Hz
        # ------------------------------------------------------------------
        self.create_timer(0.05, self._spin_once)

        self.get_logger().info(f'✓ StateMachine démarrée — équipe: {self.team}')
        self.get_logger().info(f'  • En attente du top départ (/start_signal)')

    # ======================================================================
    #  CALLBACKS CAPTEURS
    # ======================================================================

    def _cb_start(self, msg: Bool):
        if msg.data and self.state == State.INIT:
            self.get_logger().info('🚦 TOP DÉPART reçu !')
            self.match_started = True
            self.match_start_t = self.get_clock().now()
            self._transition(State.FOLLOW_LINE)

    def _cb_ir(self, msg: UInt8):
        self.ir_mask = msg.data

    def _cb_ir_pos(self, msg: Int32):
        self.ir_position = msg.data

    def _cb_aruco(self, msg: String):
        """Reçoit 'FLIP:JAUNE:0.92' ou 'KEEP:BLEU:0.87'"""
        parts = msg.data.split(':')
        if parts:
            self.box_to_flip = parts[0]  # 'FLIP' ou 'KEEP'

    def _cb_distance(self, msg: Float32):
        self.box_distance = msg.data

    def _cb_obstacle(self, msg: String):
        """
        Gestion de l'interruption obstacle.
        AVOID prend la main sur n'importe quel état actif (sauf INIT/STOP/AVOID lui-même).
        """
        prev = self.obstacle_status
        self.obstacle_status = msg.data
        if msg.data == 'DANGER':
            self.get_logger().warn('🚨 DANGER signalé')
        elif msg.data == 'OK' and prev == 'DANGER':
            self.get_logger().info('✅ Obstacle dégagé')

    def _cb_odom(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        # Extraction yaw depuis quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_theta = math.atan2(siny, cosy)

    def _cb_gripper(self, msg: String):
        self.gripper_status = msg.data

    # ======================================================================
    #  BOUCLE PRINCIPALE
    # ======================================================================

    def _spin_once(self):
        # Fin de match
        if self.match_started and self.state not in (State.STOP,):
            elapsed = (self.get_clock().now() - self.match_start_t).nanoseconds / 1e9
            if elapsed >= self.match_duration:
                self.get_logger().info('⏱️  Fin de match !')
                self._stop_motors()
                self._transition(State.STOP)
                return

        # Gestion obstacle : AVOID prend la main sauf en INIT/STOP/AVOID
        if (self.obstacle_status == 'DANGER'
                and self.state not in (State.INIT, State.STOP, State.AVOID)):
            self.prev_state = self.state
            self._transition(State.AVOID)

        elif self.obstacle_status == 'OK' and self.state == State.AVOID:
            self._transition(self.prev_state)

        # Dispatch selon état courant
        {
            State.INIT:          self._run_init,
            State.FOLLOW_LINE:   self._run_follow_line,
            State.APPROACH_BOX:  self._run_approach_box,
            State.BLIND_ADVANCE: self._run_blind_advance,
            State.GRAB:          self._run_grab,
            State.PLACE:         self._run_place,
            State.AVOID:         self._run_avoid,
            State.STOP:          self._run_stop,
        }[self.state]()

        # Publier état courant
        s = String(); s.data = self.state.name
        self.state_pub.publish(s)
        sc = Int32(); sc.data = self.score
        self.score_pub.publish(sc)

    # ======================================================================
    #  HANDLERS D'ÉTATS
    # ======================================================================

    def _run_init(self):
        """Attente passive du top départ."""
        pass

    def _run_follow_line(self):
        """
        Suivi de ligne IR.
        Logique :
          - ir_position dans [0, 14000] → suivi normal avec correction P
          - ir_position == -1 (LINE_LOST)         → maintenir dernière correction
          - ir_position == -2 (LINE_INTERSECTION)  → virage selon équipe
        
        Transition → APPROACH_BOX si ArUco détecté.
        """
        # --- Transition : ArUco détecté ---
        if self.box_to_flip is not None:
            self.get_logger().info(f'📦 ArUco détecté ({self.box_to_flip}) → APPROACH_BOX')
            self._intersection_active = False   # annuler rotation en cours si besoin
            self._transition(State.APPROACH_BOX)
            return

        # --- Gestion intersection en cours ---
        if self._intersection_active:
            self._run_intersection_turn()
            return

        # --- Déclenchement d'une nouvelle intersection ---
        if self.ir_position == -2:
            self.get_logger().info(
                f'✖️  Intersection détectée → virage '
                f'{"gauche" if self._intersection_turn_sign > 0 else "droite"}'
            )
            self._intersection_active  = True
            self._intersection_start_t = self.get_clock().now()
            self._run_intersection_turn()
            return

        # --- Suivi de ligne normal ---
        cmd = Twist()
        cmd.linear.x  = self.line_speed
        cmd.angular.z = self._compute_line_correction()
        self.cmd_pub.publish(cmd)

    def _run_intersection_turn(self):
        """
        Exécute la rotation à l'intersection pendant intersection_turn_duration secondes,
        puis reprend le suivi de ligne.

        Sens de rotation :
          blue   → angular.z > 0  (gauche)
          yellow → angular.z < 0  (droite)
        """
        elapsed = (self.get_clock().now() - self._intersection_start_t).nanoseconds / 1e9

        if elapsed < self.intersection_turn_dur:
            cmd = Twist()
            cmd.linear.x  = self.line_speed * 0.5   # avance réduite pendant le virage
            cmd.angular.z = self._intersection_turn_sign * self.intersection_turn_speed
            self.cmd_pub.publish(cmd)
        else:
            # Rotation terminée → reprendre le suivi normal
            self.get_logger().info('↩️  Virage intersection terminé — suivi de ligne')
            self._intersection_active  = False
            self._intersection_start_t = None
            # Publier un stop bref pour stabiliser avant de reprendre
            self._stop_motors()

    def _compute_line_correction(self) -> float:
        """
        Utilise la position centre de masse publiée par la Teensy IR.
        Position : 0 (extrême gauche) à 14000 (extrême droite), -1 = perdu, -2 = intersection.
        Centre = 7000. Erreur positive = robot décalé à droite → corriger à gauche.

        Les valeurs -1 et -2 sont gérées en amont dans _run_follow_line ;
        cette fonction ne reçoit que des valeurs dans [0, 14000].
        """
        if self.ir_position < 0:
            # Sécurité : ne devrait pas arriver ici, maintenir dernière correction
            return self._last_correction

        # Erreur normalisée : -1.0 (tout à gauche) → +1.0 (tout à droite)
        error = (self.ir_position - 7000) / 7000.0

        Kp = 1.2   # à calibrer — commencer à 0.6 et monter progressivement
        correction = -Kp * error
        self._last_correction = correction
        return correction

    def _run_approach_box(self):
        """
        Tag ArUco visible : avancer doucement jusqu'à perdre le tag de vue.
        Une fois le tag perdu → BLIND_ADVANCE.
        """
        if self.box_to_flip is None:
            # Tag perdu → passer en avance aveugle
            self.get_logger().info('👁️  Tag perdu → BLIND_ADVANCE')
            self.blind_start_odom = (self.odom_x, self.odom_y)
            self._transition(State.BLIND_ADVANCE)
            return

        # Avancer doucement vers la caisse
        cmd = Twist()
        cmd.linear.x = self.approach_speed
        self.cmd_pub.publish(cmd)

        # Reset le box_to_flip pour détecter la disparition au prochain cycle
        # (il sera re-rempli par le callback si toujours visible)
        self.box_to_flip = None

    def _run_blind_advance(self):
        """
        Tag perdu : avancer d'une distance calculée pour se placer au-dessus.
        Backup : capteur de distance bas confirme la position.

        Transition → GRAB quand :
          - distance parcourue >= blind_advance_dist, OU
          - capteur bas détecte la caisse à distance grab_distance
        """
        # --- Backup capteur de distance ---
        if self.box_distance is not None and self.box_distance <= self.grab_distance:
            self.get_logger().info(
                f'📏 Capteur distance confirme position ({self.box_distance:.3f}m) → GRAB'
            )
            self._stop_motors()
            self._transition(State.GRAB)
            return

        # --- Calcul distance parcourue depuis début avance aveugle ---
        if self.blind_start_odom is not None:
            dx = self.odom_x - self.blind_start_odom[0]
            dy = self.odom_y - self.blind_start_odom[1]
            dist_done = math.sqrt(dx * dx + dy * dy)

            if dist_done >= self.blind_advance_dist:
                self.get_logger().info(
                    f'📐 Distance aveugle atteinte ({dist_done:.3f}m) → GRAB'
                )
                self._stop_motors()
                self._transition(State.GRAB)
                return

        # Continuer l'avance lente
        cmd = Twist()
        cmd.linear.x = self.approach_speed
        self.cmd_pub.publish(cmd)

    def _run_grab(self):
        """
        Fermer la pince, éventuellement flipper la caisse.
        [STUB pince] — en attente du code camarade.

        Transitions :
          - pince fermée + action FLIP → commande flip
          - timeout → transition PLACE (sécurité)
        """
        now = self.get_clock().now()

        # Initialisation de l'état GRAB
        if self.grab_start_time is None:
            self.grab_start_time = now
            self.grab_action = self.box_to_flip  # mémoriser l'action
            grip_msg = String()
            grip_msg.data = 'CLOSE'
            self.gripper_pub.publish(grip_msg)
            self.get_logger().info(f'🦾 GRAB — action: {self.grab_action}')
            return

        elapsed = (now - self.grab_start_time).nanoseconds / 1e9

        # Attendre fermeture pince
        if elapsed < self.grab_duration:
            return

        # Pince fermée → flip si nécessaire
        if self.grab_action == 'FLIP' and elapsed < self.grab_duration + 0.5:
            flip_msg = String()
            flip_msg.data = 'FLIP'
            self.gripper_pub.publish(flip_msg)
            return

        # Fin GRAB → PLACE
        self.get_logger().info('✅ GRAB terminé → PLACE')
        self.boxes_grabbed  += 1
        self.grab_start_time = None
        self._transition(State.PLACE)

    def _run_place(self):
        """
        Avancer jusqu'à la zone de dépôt (marqueur IR ou timeout),
        puis ouvrir la pince et déposer la caisse.

        Transition → FOLLOW_LINE une fois la caisse déposée.
        """
        now = self.get_clock().now()

        if self.place_start_time is None:
            self.place_start_time = now
            self.get_logger().info('🎯 PLACE — avance vers zone dépôt')

        elapsed = (now - self.place_start_time).nanoseconds / 1e9

        # --- Détection zone de dépôt ---
        # TODO : utiliser marqueur IR spécifique zone dépôt (bit pattern particulier)
        deposit_zone_detected = self._detect_deposit_zone()

        if not deposit_zone_detected and elapsed < self.deposit_timeout:
            # Continuer le suivi de ligne vers la zone de dépôt
            cmd = Twist()
            cmd.linear.x  = self.line_speed
            cmd.angular.z = self._compute_line_correction()
            self.cmd_pub.publish(cmd)
            return

        # Zone détectée (ou timeout) → déposer
        if elapsed >= self.deposit_timeout:
            self.get_logger().warn('⏰ Timeout dépôt — dépôt forcé')

        self._stop_motors()

        # Ouvrir la pince
        grip_msg = String()
        grip_msg.data = 'OPEN'
        self.gripper_pub.publish(grip_msg)

        # Mise à jour score
        self.score += 1
        self.get_logger().info(f'✅ Caisse déposée — score: {self.score}')

        # Reset pour prochain cycle
        self.place_start_time = None
        self.box_to_flip      = None
        self.box_distance     = None

        # Reprendre le suivi de ligne
        self._transition(State.FOLLOW_LINE)

    def _detect_deposit_zone(self) -> bool:
        """
        Détecte la zone de dépôt.
        [STUB] — à implémenter avec le pattern IR de la zone de dépôt.

        Exemple : tous les capteurs IR actifs = bande blanche de dépôt.
        """
        # TODO : définir le pattern IR de la zone de dépôt avec les capteurs réels
        # Exemple possible : if self.ir_mask == 0b11111: return True
        return False

    def _run_avoid(self):
        """
        Obstacle détecté : robot stoppé.
        Le retour à prev_state est géré dans _spin_once quand status revient à 'OK'.
        """
        self._stop_motors()

    def _run_stop(self):
        """Fin de partie ou erreur fatale."""
        self._stop_motors()

    # ======================================================================
    #  UTILITAIRES
    # ======================================================================

    def _transition(self, new_state: State):
        self.get_logger().info(f'  ↳ {self.state.name} → {new_state.name}')
        self.state = new_state

    def _stop_motors(self):
        cmd = Twist()  # tous les champs à 0.0 par défaut
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()