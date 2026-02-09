#!/bin/bash

###########################################
# CONFIG
###########################################

REPO_DIR="$HOME/IMT-Driverless-Stack"
PYTHON_STACK="$REPO_DIR/python_stack"
ROS_DISTRO="galactic"

###########################################
# CHECK DEPENDENCIES
###########################################

command -v python3 >/dev/null 2>&1 || { echo "❌ python3 non trouvé"; exit 1; }
command -v ros2 >/dev/null 2>&1 || { echo "❌ ros2 non trouvé"; exit 1; }

###########################################
# SOURCE ROS2
###########################################

source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

###########################################
# LAUNCH cone_fusion.py
###########################################

echo "🚀 Lancement du node cone_fusion.py..."
gnome-terminal -- bash -c "
    cd $PYTHON_STACK;
    echo '🟢 cone_fusion.py en cours...';
    python3 cone_fusion.py;
    exec bash
"

###########################################
# LAUNCH circuit_map.py
###########################################

echo "🚀 Lancement du node circuit_map.py..."
gnome-terminal -- bash -c "
    cd $PYTHON_STACK;
    echo '🟢 circuit_map.py en cours...';
    python3 circuit_map.py;
    exec bash
"

###########################################
# LAUNCH YOLO (avec accès dossier weights)
###########################################

echo "🚀 Lancement du node yolo_ros.py..."
gnome-terminal -- bash -c "
    cd $PYTHON_STACK;
    echo '📂 Vérification du dossier weights...';
    if [ ! -d weights ]; then
        echo '❌ Dossier weights introuvable ! Assurez-vous qu'il existe dans $PYTHON_STACK';
        exit 1;
    fi
    echo '🟢 yolo_ros.py en cours...';
    python3 yolo_ros.py;
    exec bash
"

echo "✅ cone_fusion.py, circuit_map.py et yolo_ros.py lancés."
