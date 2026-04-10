#!/bin/bash
# ==========================================
# LAUNCHER - FSDS + YOLO Stéréo + SLAM/Carte
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
PERCEPTION_DIR="$PROJECT_ROOT/perception"
TOOLS_DIR="$PROJECT_ROOT/tools"
RVIZ_DIR="$PROJECT_ROOT/config/rviz"
WS_PATH="$PROJECT_ROOT/ros_workspace"
RVIZ_CONFIG="$RVIZ_DIR/slam.rviz"

ROS_CMD="source $ROS_SETUP; source $WS_PATH/install/setup.bash"

# --- 3. SIMULATEUR ---
echo "Lancement du simulateur..."
gnome-terminal --title="SIMULATEUR" -- bash -c "
    cd $SIM_PATH;
    ./FSDS.sh -windowed -ResX=640 -ResY=480;
    exec bash" &

echo "Attente demarrage simulateur (10s)..."
sleep 10

if command -v zenity &>/dev/null; then
    zenity --info --title="IMT Driverless — SLAM Stereo" \
        --text="Clique sur 'Run Simulation' dans le simulateur FSDS,\npuis clique sur OK pour continuer." \
        --ok-label="Simulation lancee — Continuer" --width=400 2>/dev/null || true
else
    read -p ">>> Clique sur 'Run Simulation' dans FSDS, puis appuie sur Entree..."
fi

# --- 4. BRIDGE ROS2 ---
echo "Lancement Bridge ROS2..."
gnome-terminal --title="BRIDGE ROS2" -- bash -c "
    source $ROS_SETUP;
    cd $BRIDGE_PATH;
    source install/setup.bash;
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py;
    exec bash" &
sleep 5

# --- 5. ODOM TF PUBLISHER (crée le frame fsds/map dans le TF) ---
echo "Lancement Odom TF Publisher..."
gnome-terminal --title="ODOM TF" -- bash -c "
    $ROS_CMD;
    python3 $PERCEPTION_DIR/odom_tf_publisher.py;
    exec bash" &
sleep 2

# --- 6. YOLO STEREO ---
echo "Lancement YOLO Stereo..."
gnome-terminal --title="YOLO STEREO" -- bash -c "
    $ROS_CMD;
    python3 $PERCEPTION_DIR/yolo_stereo.py;
    exec bash" &
sleep 3

# --- 7. CONE MAPPER (SLAM) ---
echo "Lancement Cone Mapper..."
gnome-terminal --title="CONE MAPPER" -- bash -c "
    $ROS_CMD;
    python3 $PERCEPTION_DIR/cone_mapper.py;
    exec bash" &
sleep 2

# --- 8. RVIZ ---
echo "Lancement RViz SLAM..."
gnome-terminal --title="RVIZ SLAM" -- bash -c "
    $ROS_CMD;
    if [ -f \"$RVIZ_CONFIG\" ]; then
        rviz2 -d \"$RVIZ_CONFIG\";
    else
        rviz2;
    fi;
    exec bash" &
sleep 2

# --- 9. DRIVE ---
echo "Lancement Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "
    $ROS_CMD;
    python3 $TOOLS_DIR/global_drive.py;
    exec bash" &

sleep 2
echo "SLAM Stereo stack lancee depuis : $SCRIPT_DIR"
echo "Topics : /slam/cone_map | /slam/car_path | /slam/stats"
