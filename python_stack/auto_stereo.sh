#!/bin/bash
# ==========================================
# LAUNCHER - FSDS + YOLO STEREO + DEPTH
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
WS_PATH="$HOME/IMT-Driverless-Stack/ros_workspace"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

ROS_CMD="source $ROS_SETUP; source $WS_PATH/install/setup.bash"

# --- 3. COPIE ET VERIFICATION DU SETTINGS.JSON ---
AIRSIM_SETTINGS="$HOME/Documents/AirSim/settings.json"
LOCAL_SETTINGS="$SCRIPT_DIR/settings.json"

mkdir -p "$HOME/Documents/AirSim"

if [ -f "$LOCAL_SETTINGS" ]; then
    echo "Copie du settings.json vers ~/Documents/AirSim/..."
    cp "$LOCAL_SETTINGS" "$AIRSIM_SETTINGS"
fi

if grep -q "cam_depth" "$AIRSIM_SETTINGS" 2>/dev/null; then
    echo "cam_depth present dans settings.json."
else
    echo "ATTENTION : cam_depth absent du settings.json charge."
fi

# --- 4. LANCEMENT ---
echo "Lancement du simulateur..."
gnome-terminal --title="SIMULATEUR" -- bash -c "
    cd $SIM_PATH;
    ./FSDS.sh -windowed -ResX=640 -ResY=480;
    exec bash" &

echo "Attente demarrage simulateur (10s)..."
sleep 10

echo "Lancement Bridge ROS2..."
gnome-terminal --title="BRIDGE ROS2" -- bash -c "
    source $ROS_SETUP;
    cd $BRIDGE_PATH;
    source install/setup.bash;
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py;
    exec bash" &

echo "Attente bridge (5s)..."
sleep 5

# Verification automatique de la cam_depth
PUB_COUNT=$(ros2 topic info /fsds/cam_depth/image_depth 2>/dev/null | grep "Publisher count" | awk '{print $3}')
if [ "$PUB_COUNT" = "1" ]; then
    echo "cam_depth OK - Publisher actif."
else
    echo "ATTENTION : /fsds/cam_depth/image_depth n'a pas de publisher. Fallback stereo/mono actif."
fi

echo "Lancement YOLO..."
gnome-terminal --title="YOLO PERCEPTION" -- bash -c "
    $ROS_CMD;
    python3 $SCRIPT_DIR/yolo_stereo.py;
    exec bash" &
sleep 2

echo "Lancement Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "
    $ROS_CMD;
    python3 $SCRIPT_DIR/global_drive.py;
    exec bash" &

echo "Tout est lance depuis : $SCRIPT_DIR"
