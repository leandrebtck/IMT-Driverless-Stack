#!/bin/bash

# ==========================================
# LAUNCHER - FSDS + YOLO (TEST CAMÉRA)
# ==========================================

# --- 1. CONFIGURATION CHEMINS ---
SIM_PATH="$HOME/Formula-Student-Driverless-Simulator-binary"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
INTERNAL_WS="$PROJECT_ROOT/ros_workspace"

# --- 2. GESTION DU WORKSPACE ROS ---
if [ -d "$INTERNAL_WS/src" ]; then
    echo "✅ Workspace interne détecté."
    if [ ! -f "$INTERNAL_WS/install/setup.bash" ]; then
        echo "⚠️  Compilation requise. Patientez..."
        cd "$INTERNAL_WS" && colcon build --symlink-install || { echo "❌ ÉCHEC COMPILATION"; exit 1; }
    fi
    ROS_CMD="source /opt/ros/galactic/setup.bash; source $INTERNAL_WS/install/setup.bash"
else
    echo "⚠️  Pas de workspace interne. Utilisation de ~/Workspace_ROS2..."
    ROS_CMD="source /opt/ros/galactic/setup.bash; source ~/Workspace_ROS2/install/setup.bash"
fi

echo "🚀 LANCEMENT STACK IMT DRIVERLESS (YOLO SEUL)"

# 1. SIMULATEUR
echo "🎮 Lancement Simu..."
gnome-terminal --title="SIMULATEUR" -- bash -c "cd $SIM_PATH; ./FSDS.sh -windowed -ResX=640 -ResY=480; exec bash" &
sleep 5

# 2. ROS2 BRIDGE 
echo "🔌 Lancement Bridge..."
gnome-terminal --title="BRIDGE ROS2" -- bash -c "source /opt/ros/galactic/setup.bash; cd ~/Formula-Student-Driverless-Simulator/ros2; source install/setup.bash; ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py; exec bash" &
sleep 3

# 3. PERCEPTION (YOLO)
echo "👁️ Lancement YOLO..."
gnome-terminal --title="YOLO PERCEPTION" -- bash -c "$ROS_CMD; python3 $SCRIPT_DIR/yolo_ros.py; exec bash" &
sleep 2

# 4. DRIVE (PILOTE)
echo "🏎️ Lancement Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "$ROS_CMD; python3 $SCRIPT_DIR/global_drive.py; exec bash" &

echo "✅ Tout est lancé depuis : $SCRIPT_DIR"
