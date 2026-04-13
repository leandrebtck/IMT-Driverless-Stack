#!/bin/bash
# ============================================================
# LAUNCHER - Skidpad — YOLO Stéréo + SLAM Stéréo
# ============================================================

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
ROS_SETUP="/opt/ros/$MY_ROS_DISTRO/setup.bash"
echo "ROS : $MY_ROS_DISTRO"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SIM_PATH="$HOME/Formula-Student-Driverless-Simulator-binary"
BRIDGE_PATH="$HOME/Formula-Student-Driverless-Simulator/ros2"
PERCEPTION_DIR="$PROJECT_ROOT/python_scripts/1perception"
SLAM_DIR="$PROJECT_ROOT/python_scripts/2slam"
CONTROL_DIR="$PROJECT_ROOT/python_scripts/3control"
PERF_DIR="$PROJECT_ROOT/python_scripts/performance"
RVIZ_DIR="$PROJECT_ROOT/config/rviz"
WS_PATH="$PROJECT_ROOT/ros_workspace"
RVIZ_CONFIG="$RVIZ_DIR/skidpad_stereo.rviz"
ROS_CMD="source $ROS_SETUP; source $WS_PATH/install/setup.bash"

# Simulateur
gnome-terminal --title="SIMULATEUR" -- bash -c "
    cd $SIM_PATH; ./FSDS.sh -windowed -ResX=640 -ResY=480; exec bash" &
echo "Attente simulateur (10s)..."
sleep 10

if command -v zenity &>/dev/null; then
    zenity --info --title="Skidpad Stéréo" \
        --text="Charge la map SKIDPAD,\nclique 'Run Simulation', puis OK." \
        --ok-label="Continuer" --width=380 2>/dev/null || true
else
    read -p ">>> Map Skidpad chargée + Run Simulation → Entree..."
fi

gnome-terminal --title="BRIDGE ROS2" -- bash -c "
    source $ROS_SETUP; cd $BRIDGE_PATH; source install/setup.bash;
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py; exec bash" &
sleep 5

gnome-terminal --title="ODOM TF" -- bash -c "
    $ROS_CMD; python3 $PERCEPTION_DIR/odom_tf_publisher.py; exec bash" &
sleep 2

gnome-terminal --title="YOLO STEREO" -- bash -c "
    $ROS_CMD; python3 $PERCEPTION_DIR/yolo_stereo.py; exec bash" &
sleep 3

gnome-terminal --title="CONE MAPPER SKIDPAD STEREO" -- bash -c "
    $ROS_CMD; python3 $SLAM_DIR/cone_mapper_skidpad_stereo.py; exec bash" &
sleep 2

gnome-terminal --title="RVIZ" -- bash -c "
    $ROS_CMD;
    if [ -f \"$RVIZ_CONFIG\" ]; then rviz2 -d \"$RVIZ_CONFIG\"; else rviz2; fi;
    exec bash" &
sleep 2

gnome-terminal --title="SKIDPAD DRIVER" -- bash -c "
    $ROS_CMD;
    python3 $CONTROL_DIR/skidpad_driver.py --map /slam/cone_map --straight 13.0;
    exec bash" &

sleep 1
echo "Skidpad Stéréo lancé — topic: /slam/cone_map"
