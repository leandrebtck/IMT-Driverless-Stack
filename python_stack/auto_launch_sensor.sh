#!/bin/bash

# ==========================================
# LAUNCHER - FULL STACK (FUSION + RVIZ)
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

echo "🚀 DÉMARRAGE DE LA STACK COMPLÈTE..."

# 1. SIMULATEUR FSDS
echo "[1/7] Lancement Simu..."
gnome-terminal --title="SIMULATEUR" -- bash -c "cd $SIM_PATH; ./FSDS.sh -windowed -ResX=640 -ResY=480; exec bash" &
sleep 5

# 2. ROS2 BRIDGE
echo "[2/7] Lancement Bridge..."
gnome-terminal --title="ROS2 BRIDGE" -- bash -c "source /opt/ros/galactic/setup.bash; cd ~/Formula-Student-Driverless-Simulator/ros2; source install/setup.bash; ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py; exec bash" &
sleep 4

# 3. LIDAR STACK
echo "[3/7] Lancement LiDAR (Filtre + Cluster)..."
gnome-terminal --title="LIDAR PROCESSING" -- bash -c "
    $ROS_CMD;
    echo '---- 1. Filtre Sol ----';
    python3 $SCRIPT_DIR/lidar_ros.py & 
    sleep 1;
    echo '---- 2. Clustering DBSCAN ----';
    python3 $SCRIPT_DIR/lidar_cluster.py &
    wait" &
sleep 2

# 4. PERCEPTION (YOLO)
echo "[4/7] Lancement YOLO..."
gnome-terminal --title="YOLO PERCEPTION" -- bash -c "
    $ROS_CMD;
    python3 $SCRIPT_DIR/yolo_ros.py; 
    exec bash" &
sleep 2

# 5. SENSOR FUSION
echo "[5/7] Lancement FUSION..."
gnome-terminal --title="SENSOR FUSION" -- bash -c "
    $ROS_CMD;
    python3 $SCRIPT_DIR/sensor_fusion.py; 
    exec bash" &
sleep 1

# 6. RVIZ
echo "[6/7] Lancement RVIZ..."
gnome-terminal --title="RVIZ VISUALIZATION" -- bash -c "
    $ROS_CMD;
    rviz2; 
    exec bash" &
sleep 1

# 7. DRIVE
echo "[7/7] Lancement Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "
    $ROS_CMD;
    python3 $SCRIPT_DIR/global_drive.py; 
    exec bash" &

echo "✅ SYSTÈME COMPLET OPÉRATIONNEL !"
