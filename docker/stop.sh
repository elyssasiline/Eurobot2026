#!/bin/bash

echo "Arrêt du conteneur ROS2..."
docker compose down

if [ $? -eq 0 ]; then
    echo "Conteneur arrêté"
else
    echo "Erreur lors de l'arrêt"
    exit 1
fi