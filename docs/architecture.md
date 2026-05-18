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

### Organisation du workspace (`src/`)

| Package | Node principal | Rôle |
|---|---|---|
| `navigation` | `obstacle_avoidance_node` | Évitement d'obstacles |
| `vision` | `aruco_detector_node`, `box_detector_node` | Détection ArUco et caisses |
| `strategy` | `mission_node` | Machine à états principale |
| `robot_bringup` | — | Config YAML centrale + launch global |
| `communication` | `wifi_bridge_node` | Pont WiFi vers les PAMIs |

### Configuration centrale (`robot_params.yaml`)

Tous les paramètres sont centralisés dans `src/robot_bringup/config/robot_params.yaml`, chargé par chaque node au démarrage via `parameters=[config_file]`.

Le paramètre `team` (`blue` ou `yellow`) est surchargeable en ligne de commande :

```bash
ros2 launch robot_bringup robot.launch.py team:=yellow
```

Il est propagé à l'ensemble des nodes par le launch file, sans édition manuelle du YAML.

---

## Nodes ROS2 — Raspberry Pi

### `aruco_detector_node` (vision)

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

### `obstacle_avoidance_node` (navigation)

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

### `mission_node` (strategy)

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

**`ir_node`** : lit les capteurs IR (actifs LOW, ligne noire = LOW), publie un bitmask `std_msgs/UInt8` sur `/ir_line` à 100 ms.

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

### `tools/lidar_test.py`

Monitor temps réel à 10 Hz :
- Distances minimales par secteur avec barres colorées
- Statut `OK` / `SLOWING` / `DANGER`
- Commandes `/cmd_vel` générées
- Radar ASCII 15×31 en temps réel
- Statistiques sur les 50 dernières alertes

### `tools/topic_monitor.py`

Surveillance de l'ensemble des topics à 4 Hz (fraîcheur, dernière valeur, compteur de messages). Injection de messages de test depuis le clavier :

| Touche | Action |
|---|---|
| `s` | Top départ |
| `f` | ArUco FLIP |
| `k` | ArUco KEEP |
| `i` | IR centré |

### Diagnostic Teensy (`motors_diag` / `ir_diag`)

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
| `/start_signal` | `std_msgs/Bool` | corde (GPIO) | `strategy_node` | 1× |
| `/camera/image_raw` | `sensor_msgs/Image` | `camera_node` | `aruco_detector_node` | 30 Hz |
| `/aruco/box_to_flip` | `std_msgs/String` | `aruco_detector_node` | `strategy_node` | sur détection |
| `/scan` | `LaserScan` | `lidar_node` | `avoidance_node` | ~10 Hz |
| `/obstacle_alert` | `std_msgs/String` | `avoidance_node` | `strategy_node` | ~10 Hz |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | `strategy_node` | `avoidance_node` | 20 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | `avoidance_node` | `motor_node` (T1) | 20 Hz |
| `/ir_line` | `std_msgs/UInt8` | `ir_node` (T2) | `strategy_node` | 10 Hz |
| `/wheel_odom` | `nav_msgs/Odometry` | `encoder_node` (T1) | `strategy_node` | 20 Hz |
| `/gripper/command` | `std_msgs/String` | `strategy_node` | `gripper_node` (T2) | sur action |
| `/gripper/status` | `std_msgs/String` | `gripper_node` (T2) | `strategy_node` | sur événement |
| `/strategy/state` | `std_msgs/String` | `strategy_node` | `topic_monitor` | 20 Hz |
| `/strategy/score` | `std_msgs/Int32` | `strategy_node` | `topic_monitor` | 20 Hz |

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
| Capteurs ultrasons | Code prêt, `sensorNodeSpin()` commenté | Branchement + test |
| Capteur distance bas | Topic `/vision/nearest_box_distance` souscrit, logique écrite | Branchement + test |
| Supervision batterie | Code complet dans `sensor_node.cpp` | Branchement + test |
| Communication WiFi PAMIs | Architecture dans `wifi_bridge_node`, params dans YAML | Configuration ESP32 |

---

## Perspectives

| Évolution | Description |
|---|---|
| **Détection IA** | Remplacer ou compléter ArUco par un node YOLO/MobileNet publiant sur `/aruco/box_to_flip` — aucune autre couche touchée |
| **Nav2** | `/wheel_odom` et `/scan` sont directement compatibles avec le stack Nav2 pour une navigation autonome complète |
| **Multi-robot** | `ROS_DOMAIN_ID` permet d'isoler ou faire communiquer plusieurs instances ROS2 ; la coordination PAMIs pourrait migrer vers le graphe ROS2 |
| **Simulation** | Architecture 100 % topics + paramètres → compatible Gazebo pour valider la stratégie sans hardware |

L'ensemble de la logique métier est isolé dans `mission_node` et `robot_params.yaml`. Une équipe future peut réutiliser toute la base en ne réécrivant que la machine à états et les paramètres de mission.