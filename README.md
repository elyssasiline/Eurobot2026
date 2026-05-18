# Robot Principal — Eurobot 2025

Robot autonome conçu pour la compétition **Eurobot 2025**, basé sur ROS2 Humble, un Raspberry Pi 5 et deux Teensy 4.1 via micro-ROS.

---

## Stack technique

| Couche | Technologie |
|---|---|
| Middleware | ROS2 Humble |
| Compute | Raspberry Pi 5 (arm64) |
| Environnement | Docker (`arm64v8/ros:humble`) |
| Bas niveau | 2× Teensy 4.1 + micro-ROS |
| Firmware | PlatformIO / Arduino |
| Vision | OpenCV ArUco (patterns custom) |
| LiDAR | RPLidar (rplidar-ros) |
| Caméra | libcamera (compilée depuis sources) |

---

## Structure du repo

```
robot_ws/
├── src/
│   ├── communication/           # Pont WiFi vers les PAMIs
│   │   └── communication/
│   │       ├── wifi_bridge_node.py
│   │       └── fleet_manager_node.py
│   ├── navigation/              # Évitement d'obstacles
│   │   └── navigation/
│   │       ├── obstacle_avoidance_node.py
│   │       ├── navigation_node.py
│   │       └── lidar_node.py
│   ├── robot_bringup/           # Launch global + config centrale
│   │   ├── config/
│   │   │   └── robot_params.yaml
│   │   ├── scripts/
│   │   │   ├── calibrate_sensors.py
│   │   │   └── setup_network.sh
│   │   └── launch/
│   │       └── robot.launch.py
│   ├── strategy/                # Machine à états principale
│   │   └── strategy/
│   │       ├── state_machine_node.py
│   │       └── mission_planner_node.py
│   └── vision/                  # Détection ArUco + caisses
│       ├── vision/
│       │   ├── aruco_detector_node.py
│       │   ├── box_detector_node.py
│       │   ├── camera_node.py
│       │   └── pose_estimator.py
│       └── config/
│           ├── aruco_params.yaml
│           ├── box_detection_params.yaml
│           └── camera_params.yaml
├── teensy_firmware/             # Teensy 1 — moteurs + encodeurs
│   ├── src/
│   │   ├── main.cpp
│   │   ├── motor_node.cpp
│   │   ├── encoder_node.cpp
│   │   └── sensor_node.cpp
│   ├── lib/
│   │   ├── motor_driver/
│   │   └── encoders/
│   ├── test/
│   │   └── diagnostic.cpp       # Mode diagnostic sans micro-ROS
│   └── platformio.ini
├── teensy_firmware2/            # Teensy 2 — capteurs IR + pince
│   ├── src/
│   │   └── main.cpp
│   ├── include/
│   │   └── ir_config.h
│   ├── test/
│   │   └── ir_diag.cpp          # Mode diagnostic sans micro-ROS
│   └── platformio.ini
├── tools/
│   ├── testing/
│   │   ├── topic_monitor.py     # Monitoring topics + injection de test
│   │   ├── lidar_test.py        # Monitor LiDAR temps réel
│   │   ├── simulate_mission.py  # Simulation de mission
│   │   ├── ir_bench_viewer.py   # Visualisation capteurs IR
│   │   └── test_camera.py       # Test caméra
│   ├── calibration/
│   │   └── calibrate_camera.py
│   ├── visualization/
│   │   └── viewer_camera.py
│   └── chessboard.png           # Mire de calibration caméra
├── docker/
│   ├── Dockerfile.raspberry
│   ├── Dockerfile.teensy
│   ├── docker-compose.yaml
│   ├── entrypoint.sh
│   ├── build.sh
│   ├── start.sh
│   └── stop.sh
└── docs/
    └── ARCHITECTURE.md          # Documentation technique détaillée
```

---

## Lancer le robot

```bash
# Depuis la racine du repo
ros2 launch robot_bringup robot.launch.py team:=blue   # ou team:=yellow
```

Le paramètre `team` conditionne la logique de tri des caisses et le sens aux intersections.
Il est surchargeable en ligne de commande sans modifier le YAML.

---

## Paramétrage

Tous les paramètres sont centralisés dans `src/robot_bringup/config/robot_params.yaml` :
seuils d'évitement, vitesses, secteurs LiDAR, séquences de la pince, IP des PAMIs.

---

## Firmware Teensy

```bash
# Teensy 1 — moteurs/encodeurs
cd teensy_firmware/
pio run -e teensy_motors --target upload    # match
pio run -e motors_diag --target upload      # diagnostic

# Teensy 2 — IR/pince
cd teensy_firmware2/
pio run -e teensy_ir --target upload        # match
pio run -e ir_diag --target upload          # diagnostic
```

---

## Documentation

→ [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — Architecture complète : nodes ROS2, firmware Teensy, topics, difficultés rencontrées, perspectives.