#!/bin/bash

###########################################
# CONFIG
###########################################

REPO_DIR="$HOME/IMT-Driverless-Stack"
PYTHON_STACK="$REPO_DIR/python_stack"
ROS_DISTRO="galactic"
SIM_PATH="$HOME/Formula-Student-Driverless-Simulator-binary"

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
# 1. LANCEMENT SIMULATEUR OU ROSBAG
###########################################
echo "🎮 Lancement Simulateur / Rosbag..."
gnome-terminal -- bash -c "
cd $SIM_PATH
# Remplace par 'ros2 bag play ...' si tu veux replay
./FSDS.sh -windowed -ResX=640 -ResY=480
exec bash" &
sleep 5

###########################################
# 2. LANCEMENT YOLO
###########################################
echo "🚀 Vérification du dossier weights pour YOLO..."
if [ ! -d "$PYTHON_STACK/weights" ]; then
    echo "❌ Dossier weights introuvable ! Assurez-vous qu'il existe dans $PYTHON_STACK"
    exit 1
fi

echo "🟢 Lancement yolo_ros.py..."
gnome-terminal -- bash -c "
cd $PYTHON_STACK
python3 yolo_ros.py
exec bash" &
sleep 2

###########################################
# 3. LANCEMENT GLOBAL DRIVE
###########################################
echo "🏎️ Lancement global_drive.py..."
gnome-terminal -- bash -c "
cd $PYTHON_STACK
python3 global_drive.py
exec bash" &
sleep 2

###########################################
# 4. LANCEMENT LIDAR ROS
###########################################
echo "🟢 Lancement lidar_ros.py..."
gnome-terminal -- bash -c "
cd $PYTHON_STACK
python3 lidar_ros.py
exec bash" &
sleep 1

###########################################
# 5. LANCEMENT LIDAR CLUSTER
###########################################
echo "🔵 Lancement lidar_cluster.py..."
gnome-terminal -- bash -c "
cd $PYTHON_STACK
python3 lidar_cluster.py
exec bash" &
sleep 1

###########################################
# 6. LANCEMENT CONE FUSION
###########################################
echo "🔴 Lancement cone_fusion.py..."
gnome-terminal -- bash -c "
cd $PYTHON_STACK
python3 cone_fusion.py
exec bash" &
sleep 1

###########################################
# 7. LANCEMENT CIRCUIT MAP
###########################################
echo "📍 Lancement circuit_map.py..."
gnome-terminal -- bash -c "
cd $PYTHON_STACK
python3 circuit_map.py
exec bash" &

echo "✅ Tous les nœuds ont été lancés dans l'ordre demandé."
