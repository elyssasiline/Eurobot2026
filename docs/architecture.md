# Architecture technique — Robot Principal Eurobot 2025

## Table des matières

- [Vue d'ensemble](#vue-densemble)
- [Environnement et infrastructure](#environnement-et-infrastructure)
- [Nodes ROS2 — Raspberry Pi](#nodes-ros2--raspberry-pi)
- [Firmware Teensy — micro-ROS](#firmware-teensy--micro-ros)
- [Paramètre équipe](#paramètre-équipe)
- [Outils de diagnostic](#outils-de-diagnostic)
- [Topics ROS2](#topics-ros2)
- [Difficultés rencontrées](#difficultés-rencontrées)
- [Perspectives](#perspectives)

---

## Vue d'ensemble

L'architecture repose sur une séparation nette entre deux couches :

- **Couche décisionnelle** (Raspberry Pi 5) : stratégie, perception, navigation — sous ROS2 Humble dans un container Docker.
- **Couche d'exécution** (2× Teensy 4.1) : moteurs, encodeurs, capteurs IR, pince — via micro-ROS.

Les deux couches communiquent par le bridge micro-ROS en USB/Serial.

---

## Environnement et infrastructure

### Container Docker (`docker/`)

L'environnement ROS2 tourne dans un container basé sur `arm64v8/ros:humble`.

| Fichier | Rôle |
|---|---|
| `Dockerfile.raspberry` | Image principale ROS2 pour le Raspberry Pi 5 |
| `Dockerfile.teensy` | Image pour le build firmware Teensy |
| `docker-compose.yaml` | Orchestration des containers |
| `entrypoint.sh` | Point d'entrée du container |
| `build.sh` / `start.sh` / `stop.sh` | Scripts de gestion du cycle de vie |

Couches ajoutées à l'image de base :

| Composant | Mode d'installation |
|---|---|
| libcamera | Compilé depuis les sources (fork Raspberry Pi), pipelines `rpi/vc4` et `rpi/pisp` |
| RPLidar | `ros-humble-rplidar-ros` via apt |
| micro-ROS Agent | Compilé depuis les sources (`micro_ros_msgs` puis `micro-ROS-Agent`) |
| camera_ros | Compilé dans le workspace `/app` |

Le container est configuré avec `privileged: true` et `network_mode: host` pour l'accès aux devices physiques :

| Device | Rôle |
|---|---|
| `/dev/ttyUSB0` | LiDAR |
| `/dev/ttyACM0` | Teensy 1 |
| `/dev/ttyACM1` | Teensy 2 |

Un utilisateur non-root `rosuser` (groupes `dialout` et `video`) est créé pour l'accès aux périphériques.

---

### Organisation du workspace (`src/`)

| Package | Nodes | Rôle |
|---|---|---|
| `communication` | `wifi_bridge_node`, `fleet_manager_node` | Pont WiFi vers les PAMIs |
| `navigation` | `obstacle_avoidance_node`, `navigation_node`, `lidar_node` | Évitement d'obstacles |
| `robot_bringup` | — | Config YAML centrale + launch global |
| `strategy` | `state_machine_node`, `mission_planner_node` | Machine à états principale |
| `vision` | `aruco_detector_node`, `box_detector_node`, `camera_node`, `pose_estimator` | Détection ArUco et caisses |

### Configuration centrale (`robot_params.yaml`)

Tous les paramètres sont centralisés dans `src/robot_bringup/config/robot_params.yaml`, chargé par chaque node au démarrage via `parameters=[config_file]`.

Le paramètre `team` (`blue` ou `yellow`) est surchargeable en ligne de commande :

```bash
ros2 launch robot_bringup robot.launch.py team:=yellow
```

Il est propagé à l'ensemble des nodes par le launch file, sans édition manuelle du YAML.

Les configs spécifiques à la vision sont dans `src/vision/config/` :
- `aruco_params.yaml` — paramètres de détection ArUco
- `box_detection_params.yaml` — paramètres de détection des caisses
- `camera_params.yaml` — calibration caméra

---

## Nodes ROS2 — Raspberry Pi

### `aruco_detector_node.py` (vision)

Gère la chaîne complète de vision par ordinateur.

**Souscriptions :** `/camera/image_raw`

**Publications :**
- `/aruco/detections` (`vision_msgs/Detection2DArray`)
- `/aruco/box_to_flip` (`std_msgs/String`) — format : `FLIP:JAUNE:0.92` ou `KEEP:BLEU:0.87`
- `/aruco/image_annotated` (`sensor_msgs/Image`) — activable via `publish_annotated_image`

**Pipeline de détection :**
1. Extraction du patch par transformation de perspective (`warpPerspective`) → image 60×60 px
2. Binarisation Otsu, lecture de chaque cellule de la grille 6×6
3. Comparaison par rotation (4 orientations) aux patterns de référence
4. Seuil de confiance à 85 % (paramétrable via `confidence_threshold`)

Les patterns ArUco sont définis manuellement en grille 6×6 pour chaque couleur (JAUNE, BLEU, NOIR), adaptés aux éléments de jeu de la compétition.

**Logique équipe :** `team=blue` → caisses JAUNE à retourner ; `team=yellow` → caisses BLEUE.

---

### `obstacle_avoidance_node.py` (navigation)

S'intercale entre la state machine et les moteurs.

**Souscriptions :** `/cmd_vel_raw`, `/scan`

**Publications :** `/cmd_vel`, `/obstacle_alert`, `/cmd_vel_scale`

| Statut | Condition | Comportement |
|---|---|---|
| `OK` | Aucun obstacle | Commande brute transmise telle quelle |
| `SLOWING` | Obstacle entre 0,40 m et 0,20 m, qui se rapproche (> 0,02 m/s) | Ralentissement |
| `DANGER` | Obstacle à moins de 0,20 m | Arrêt complet jusqu'à disparition |

La condition `SLOWING` ne se déclenche que si l'obstacle se rapproche, pour éviter les freinages sur obstacles stationnaires.

---

### `state_machine_node.py` (strategy)

Cœur décisionnel du robot. Tourne à 20 Hz. Implémente une machine à **8 états**.

| État | Description |
|---|---|
| `INIT` | Attente du top départ sur `/start_signal`. Enregistre le timestamp de début. |
| `FOLLOW_LINE` | Suivi de la ligne noire par les capteurs IR. Correction angulaire par somme pondérée (positions `[-2,-1,0,1,2]`, gain `Kp=0.5`). Détection d'intersection à 85 % de capteurs actifs → tourne selon `team`. |
| `APPROACH_BOX` | Avance lentement (0,05 m/s) tant que le tag ArUco est visible. |
| `BLIND_ADVANCE` | Tag disparu (robot trop proche). Avance encore 0,15 m par odométrie. |
| `GRAB` | Fermeture de la pince, attente 1,5 s, FLIP si couleur adverse. |
| `PLACE` | Suivi de ligne vers la zone de dépôt. Timeout 5,0 s. Ouverture pince, score incrémenté. |
| `AVOID` | Interruption sur `obstacle_status == 'DANGER'`. Mémorise l'état courant, reprend sur `OK`. |
| `STOP` | Atteint à la 100e seconde ou sur erreur fatale. Arrêt définitif. |

L'état courant est publié sur `/strategy/state` et le score sur `/strategy/score` à 20 Hz.

---

## Firmware Teensy — micro-ROS

### Architecture générale

Développé sous PlatformIO (framework Arduino). Chaque Teensy gère son cycle de vie micro-ROS via une machine à 4 états :

```
WAITING_AGENT → AGENT_AVAILABLE → AGENT_CONNECTED → AGENT_DISCONNECTED
```

Reconnexion automatique sans reboot si l'agent sur le Raspberry redémarre.

Chaque Teensy dispose de deux environnements PlatformIO :

| Environnement | Rôle |
|---|---|
| `teensy_motors` / `teensy_ir` | Match, avec micro-ROS |
| `motors_diag` / `ir_diag` | Calibration, sans micro-ROS |

---

### Teensy 1 — Moteurs et encodeurs (`teensy_firmware/`)

```
teensy_firmware/
├── src/
│   ├── main.cpp           # Cycle de vie micro-ROS + exécuteur
│   ├── motor_node.cpp     # Cinématique Mecanum + driver VNH5019
│   ├── encoder_node.cpp   # Lecture encodeurs + odométrie
│   └── sensor_node.cpp    # Ultrasons + batterie (partiel)
├── lib/
│   ├── motor_driver/      # Abstraction driver moteur
│   └── encoders/          # Abstraction encodeurs quadrature
└── test/
    └── diagnostic.cpp     # Interface série interactive sans micro-ROS
```

**Cinématique Mecanum 45° :**

```
AVL =  vx - vy - wz
AVR =  vx + vy + wz
ARL =  vx + vy - wz
ARR =  vx - vy + wz
```

- Composantes normalisées vers `[-1.0, 1.0]`
- Watchdog 500 ms : arrêt moteurs si plus de `/cmd_vel`
- Frein actif à vitesse nulle (`INA=L, INB=L, PWM=MAX`)
- PWM à 20 kHz, résolution 12 bits

**Odométrie Mecanum** (publiée sur `/wheel_odom`) :

```
dx     = (AVL + AVR + ARL + ARR) / 4
dy     = (-AVL + AVR + ARL - ARR) / 4
dtheta = (-AVL + AVR - ARL + ARR) / (4 * (Lx + Ly))
```

Encodeurs quadrature lus par interruptions (mode `CHANGE` sur 8 broches), ISR marquées `FASTRUN` (exécution en RAM).

---

### Teensy 2 — Capteurs IR et pince (`teensy_firmware2/`)

```
teensy_firmware2/
├── src/
│   └── main.cpp           # Cycle de vie micro-ROS + ir_node + gripper_node
├── include/
│   └── ir_config.h        # Pinout et seuils capteurs IR
└── test/
    └── ir_diag.cpp        # Validation capteurs IR sans micro-ROS
```

**`ir_node`** : lit les capteurs IR (actifs LOW, ligne noire = LOW), publie un bitmask `std_msgs/UInt8` sur `/ir_line` toutes les 100 ms.

- Ligne centrée → avance droit
- Ligne décentrée → correction angulaire
- ≥ 85 % capteurs actifs → intersection détectée, décision de sens par la state machine

**`gripper_node`** : souscrit à `/gripper/command`, exécute les séquences `CLOSE`, `FLIP`, `OPEN` via actionneurs linéaires. Partage le Teensy 2 avec `ir_node` dans le même exécuteur micro-ROS.

---

## Paramètre équipe

Le paramètre `team` conditionne deux comportements :

| Comportement | `team=blue` | `team=yellow` |
|---|---|---|
| Couleur à retourner | JAUNE | BLEU |
| Sens aux intersections | Droite | Gauche |

Le top départ est géré par la corde de lancement qui publie `std_msgs/Bool = True` sur `/start_signal`, démarrant simultanément le timer de match et le signal vers les PAMIs.

---

## Outils de diagnostic

### `tools/testing/`

| Fichier | Rôle |
|---|---|
| `topic_monitor.py` | Surveillance de tous les topics à 4 Hz + injection de messages de test clavier |
| `lidar_test.py` | Monitor LiDAR temps réel : distances par secteur, radar ASCII 15×31, statistiques alertes |
| `simulate_mission.py` | Simulation de mission complète sans hardware |
| `ir_bench_viewer.py` | Visualisation en temps réel des capteurs IR |
| `test_camera.py` | Test et validation du flux caméra |

**Injections disponibles dans `topic_monitor.py` :**

| Touche | Action |
|---|---|
| `s` | Top départ |
| `f` | ArUco FLIP |
| `k` | ArUco KEEP |
| `i` | IR centré |

### `tools/calibration/`

| Fichier | Rôle |
|---|---|
| `calibrate_camera.py` | Calibration caméra (matrice intrinsèque + distorsion) |
| `chessboard.png` | Mire de calibration |

### `tools/visualization/`

| Fichier | Rôle |
|---|---|
| `viewer_camera.py` | Visualisation du flux caméra en temps réel |

### Diagnostic Teensy (`test/diagnostic.cpp` / `test/ir_diag.cpp`)

Interface série interactive sans micro-ROS.

Commandes disponibles (Teensy 1) :

| Commande | Action |
|---|---|
| `t1`–`t4` | Test individuel moteur (2 s + lecture encodeur) |
| `sy` | Synchronisation : compare les ticks des 4 roues |
| `vn` | Rampe PWM : mesure vitesse en mm/s, identifie la zone morte |
| `sq` | Séquence automatique : avance, recul, pivots, straffs, diagonales |
| `inv1`–`inv4` | Inversion à chaud d'un moteur |
| `didc` | Diagnostic VNH5019 (lecture pins EN/DIAG) |

---

## Topics ROS2

| Topic | Type | Publisher | Subscriber(s) | Fréquence |
|---|---|---|---|---|
| `/start_signal` | `std_msgs/Bool` | corde (GPIO) | `state_machine_node` | 1× |
| `/camera/image_raw` | `sensor_msgs/Image` | `camera_node` | `aruco_detector_node` | 30 Hz |
| `/aruco/box_to_flip` | `std_msgs/String` | `aruco_detector_node` | `state_machine_node` | sur détection |
| `/scan` | `LaserScan` | `lidar_node` | `obstacle_avoidance_node` | ~10 Hz |
| `/obstacle_alert` | `std_msgs/String` | `obstacle_avoidance_node` | `state_machine_node` | ~10 Hz |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | `state_machine_node` | `obstacle_avoidance_node` | 20 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | `obstacle_avoidance_node` | `motor_node` (T1) | 20 Hz |
| `/ir_line` | `std_msgs/UInt8` | `ir_node` (T2) | `state_machine_node` | 10 Hz |
| `/wheel_odom` | `nav_msgs/Odometry` | `encoder_node` (T1) | `state_machine_node` | 20 Hz |
| `/gripper/command` | `std_msgs/String` | `state_machine_node` | `gripper_node` (T2) | sur action |
| `/gripper/status` | `std_msgs/String` | `gripper_node` (T2) | `state_machine_node` | sur événement |
| `/strategy/state` | `std_msgs/String` | `state_machine_node` | `topic_monitor` | 20 Hz |
| `/strategy/score` | `std_msgs/Int32` | `state_machine_node` | `topic_monitor` | 20 Hz |

---

## Difficultés rencontrées

### Compilation libcamera et micro-ROS Agent

Aucun package précompilé n'existe pour arm64/Humble. La version de `meson` disponible via apt étant trop ancienne, elle a été installée via pip (conflit à résoudre manuellement). Pour micro-ROS Agent, la compilation de `micro_ros_msgs` **avant** `micro-ROS-Agent` est critique et non documentée explicitement.

### Transition APPROACH_BOX → BLIND_ADVANCE

La disparition du tag ArUco est détectée par reset de `box_to_flip` à `None` à chaque itération : si le callback ne le remplit pas avant la suivante, la disparition est actée. Cette logique est sensible au taux de rafraîchissement de la caméra. Un capteur de distance bas est prévu en backup.

### Saturation du bridge micro-ROS

Une publication trop fréquente des encodeurs saturait le bridge et provoquait des pertes sur `/cmd_vel`. Résolu par l'introduction d'une période minimale `PERIOD_ODOM_MS`, tout en conservant le décodage des ticks à chaque ISR.

### Sous-systèmes en attente d'intégration

| Sous-système | État | Ce qu'il manque |
|---|---|---|
| Capteurs ultrasons | Code prêt dans `sensor_node.cpp`, `sensorNodeSpin()` commenté | Branchement + test |
| Capteur distance bas | Topic `/vision/nearest_box_distance` souscrit, logique écrite | Branchement + test |
| Supervision batterie | Code complet dans `sensor_node.cpp` (diviseur résistif + ACS712) | Branchement + test |
| Communication WiFi PAMIs | Architecture dans `wifi_bridge_node`, params IP/port dans YAML | Configuration ESP32 |

---

## Perspectives

| Évolution | Description |
|---|---|
| **Détection IA** | Remplacer ou compléter ArUco par un node YOLO/MobileNet publiant sur `/aruco/box_to_flip` — aucune autre couche touchée |
| **Nav2** | `/wheel_odom` et `/scan` sont directement compatibles avec le stack Nav2 pour une navigation autonome complète |
| **Multi-robot** | `ROS_DOMAIN_ID` permet d'isoler ou faire communiquer plusieurs instances ROS2 ; la coordination PAMIs pourrait migrer vers le graphe ROS2 |
| **Simulation** | Architecture 100 % topics + paramètres → compatible Gazebo pour valider la stratégie sans hardware |

L'ensemble de la logique métier est isolé dans `state_machine_node.py` et `robot_params.yaml`. Une équipe future peut réutiliser toute la base en ne réécrivant que la machine à états et les paramètres de mission.