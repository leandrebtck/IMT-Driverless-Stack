#!/bin/bash

# --- CONFIGURATION ---
SIM_PATH="$HOME/Formula-Student-Driverless-Simulator-binary"

# Récupère le chemin du dossier où se trouve CE script (python_stack)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "🚀 LANCEMENT STACK IMT DRIVERLESS"

# 1. SIMULATEUR
echo "🎮 Lancement Simu..."
gnome-terminal --title="SIMULATEUR" -- bash -c "cd $SIM_PATH; ./FSDS.sh -windowed -ResX=640 -ResY=480; exec bash" &
sleep 5

# 2. ROS2 BRIDGE 
echo "🔌 Lancement Bridge..."
gnome-terminal --title="BRIDGE ROS2" -- bash -c "source /opt/ros/galactic/setup.bash; cd ~/Formula-Student-Driverless-Simulator/ros2; source install/setup.bash; ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py; exec bash" &
sleep 3

# 3. PERCEPTION (YOLO)
# On reste dans Workspace_ROS2 pour charger les messages (fs_msgs) nécessaires au script python
echo "👁️ Lancement YOLO..."
gnome-terminal --title="YOLO PERCEPTION" -- bash -c "source /opt/ros/galactic/setup.bash; cd ~/Workspace_ROS2; source install/setup.bash; python3 $SCRIPT_DIR/yolo_ros.py; exec bash" &
sleep 2

# 4. DRIVE (PILOTE)
echo "🏎️ Lancement Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "source /opt/ros/galactic/setup.bash; cd ~/Workspace_ROS2; source install/setup.bash; python3 $SCRIPT_DIR/global_drive.py; exec bash" &

echo "✅ Tout est lancé depuis : $SCRIPT_DIR"
