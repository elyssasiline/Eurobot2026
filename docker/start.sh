#!/bin/bash

# Permettre l'accès X11 pour les GUI
xhost +local:docker > /dev/null 2>&1

echo "Démarrage du conteneur ROS2..."
docker compose up -d

if [ $? -ne 0 ]; then
    echo "Erreur lors du démarrage du conteneur"
    exit 1
fi

echo "Connexion au conteneur..."
docker exec -it robot_ros2_dev /bin/bash