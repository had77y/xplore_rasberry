#!/usr/bin/env bash
# Lance camera_node nativement sur le Pi (hors Docker).
# Pré-requis :
#   sudo apt install ros-humble-ros-base python3-picamera2 python3-opencv ros-humble-cv-bridge
#   colcon build --packages-select rover_xplore  (depuis ~/dev_ws)
#
# Usage :
#   ./scripts/start_camera.sh

set -e

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/dev_ws/install/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
    echo "[ERREUR] ROS 2 Humble introuvable à $ROS_SETUP"
    echo "         sudo apt install ros-humble-ros-base"
    exit 1
fi

source "$ROS_SETUP"

if [ ! -f "$WS_SETUP" ]; then
    echo "[ERREUR] Workspace non buildé — lance d'abord :"
    echo "         cd ~/dev_ws && colcon build --packages-select rover_xplore"
    exit 1
fi

source "$WS_SETUP"

echo "[camera] Démarrage camera_node natif (libcamera / picamera2)..."
exec ros2 run rover_xplore camera_node
