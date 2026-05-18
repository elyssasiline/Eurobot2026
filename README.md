# Robot Principal | Eurobot 2025

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
│   ├── communication/       # Pont WiFi vers les PAMIs (wifi_bridge_node)
│   ├── navigation/          # Évitement d'obstacles (obstacle_avoidance_node)
│   ├── robot_bringup/       # Launch global + robot_params.yaml
│   ├── strategy/            # Machine à états principale (mission_node)
│   └── vision/              # Détection ArUco + caisses
├── teensy_firmware/         # Teensy 1 — moteurs + encodeurs
├── teensy_firmware2/        # Teensy 2 — capteurs IR + pince
├── tools/                   # Utilitaires de diagnostic
│   ├── lidar_test.py        # Monitor LiDAR temps réel
│   └── topic_monitor.py     # Monitoring topics + injection de test
├── docker/                  # Dockerfile.raspberry + docker-compose
└── docs/
    └── ARCHITECTURE.md      # Documentation technique détaillée
```

---

## Lancer le robot

```bash
# Dans le container Docker
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
cd teensy_firmware/
pio run -e teensy_motors --target upload    # Teensy 1 — moteurs/encodeurs
pio run -e motors_diag --target upload      # Teensy 1 — mode diagnostic

cd teensy_firmware2/
pio run -e teensy_ir --target upload        # Teensy 2 — IR/pince
pio run -e ir_diag --target upload          # Teensy 2 — mode diagnostic
```

---

## Documentation

→ [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — Architecture complète : nodes ROS2, firmware Teensy, topics, difficultés rencontrées, perspectives.