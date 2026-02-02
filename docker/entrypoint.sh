#!/bin/bash
set -e

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source camera_ros build
source /app/install/setup.bash

# Source workspace utilisateur si disponible
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

exec "$@"