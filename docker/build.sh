#!/bin/bash

echo "Construction de l'image Docker ROS2 Humble..."
echo "Cela peut prendre 20-30 minutes..."

# Utiliser docker compose au lieu de docker-compose
docker compose build

if [ $? -eq 0 ]; then
    echo "Image construite avec succès!"
else
    echo "Erreur lors de la construction"
    exit 1
fi