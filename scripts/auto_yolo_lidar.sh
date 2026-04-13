#!/bin/bash
# ==========================================
# LAUNCHER - FSDS + YOLO + LIDAR (distance)
# ==========================================

# --- 1. DETECTION AUTOMATIQUE DE ROS ---
if [ -f "/opt/ros/iron/setup.bash" ]; then
    MY_ROS_DISTRO="iron"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    MY_ROS_DISTRO="galactic"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    MY_ROS_DISTRO="humble"
else
    echo "ERREUR : Aucune installation ROS detectee dans /opt/ros/"
    exit 1
fi
echo "ROS Version detectee : $MY_ROS_DISTRO"
ROS_SETUP="/opt/ros/$MY_ROS_DISTRO/setup.bash"

# --- 2. CONFIGURATION CHEMINS ---
SIM_PATH="$HOME/Formula-Student-Driverless-Simulator-binary"
BRIDGE_PATH="$HOME/Formula-Student-Driverless-Simulator/ros2"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PERCEPTION_DIR="$PROJECT_ROOT/python_scripts/1perception"
SLAM_DIR="$PROJECT_ROOT/python_scripts/2slam"
CONTROL_DIR="$PROJECT_ROOT/python_scripts/3control"
PERF_DIR="$PROJECT_ROOT/python_scripts/performance"
RVIZ_DIR="$PROJECT_ROOT/config/rviz"
WS_PATH="$PROJECT_ROOT/ros_workspace"

# --- 3. GESTION DU WORKSPACE ---
if [ -d "$WS_PATH/src" ] && [ ! -f "$WS_PATH/install/setup.bash" ]; then
    echo "Compilation du workspace requise. Patientez..."
    bash -c "source $ROS_SETUP && cd $WS_PATH && colcon build --symlink-install" || { echo "ECHEC COMPILATION"; exit 1; }
fi

if [ -f "$WS_PATH/install/setup.bash" ]; then
    ROS_CMD="source $ROS_SETUP; source $WS_PATH/install/setup.bash"
elif [ -f "$HOME/Workspace_ROS2/install/setup.bash" ]; then
    echo "Pas de workspace interne. Utilisation de ~/Workspace_ROS2..."
    ROS_CMD="source $ROS_SETUP; source $HOME/Workspace_ROS2/install/setup.bash"
else
    echo "Aucun workspace compile trouve. fs_msgs sera indisponible."
    ROS_CMD="source $ROS_SETUP"
fi

# --- 4. LANCEMENT ---
echo "Lancement du simulateur..."
gnome-terminal --title="SIMULATEUR" -- bash -c "
    cd $SIM_PATH;
    ./FSDS.sh -windowed -ResX=640 -ResY=480;
    exec bash" &

echo "Attente demarrage simulateur (10s)..."
sleep 10

# Attente confirmation utilisateur avant de continuer
if command -v zenity &>/dev/null; then
    zenity --info --title="IMT Driverless" \
        --text="Clique sur 'Run Simulation' dans le simulateur FSDS,\npuis clique sur OK pour continuer." \
        --ok-label="Simulation lancee — Continuer" --width=400 2>/dev/null || true
else
    read -p ">>> Clique sur 'Run Simulation' dans FSDS, puis appuie sur Entree..."
fi

echo "Lancement Bridge ROS2..."
gnome-terminal --title="BRIDGE ROS2" -- bash -c "
    source $ROS_SETUP;
    cd $BRIDGE_PATH;
    source install/setup.bash;
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py;
    exec bash" &

echo "Attente bridge (5s)..."
sleep 5

echo "Lancement YOLO + LiDAR..."
gnome-terminal --title="YOLO LIDAR" -- bash -c "
    $ROS_CMD;
    python3 $PERCEPTION_DIR/yolo_lidar.py;
    exec bash" &
sleep 2

echo "Lancement Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "
    $ROS_CMD;
    python3 $CONTROL_DIR/global_drive.py;
    exec bash" &

sleep 2
echo "Tout est lance depuis : $SCRIPT_DIR"
