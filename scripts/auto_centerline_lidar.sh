#!/bin/bash
# ============================================================
# LAUNCHER - FSDS + YOLO LiDAR + SLAM + Centerline Follower
# ============================================================

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
RVIZ_CONFIG="$RVIZ_DIR/slam_lidar.rviz"

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
    zenity --info --title="IMT Driverless — Centerline LiDAR" \
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

# --- 5. ODOM TF PUBLISHER ---
echo "Lancement Odom TF Publisher..."
gnome-terminal --title="ODOM TF" -- bash -c "
    $ROS_CMD;
    python3 $PERCEPTION_DIR/odom_tf_publisher.py;
    exec bash" &
sleep 2

# --- 6. YOLO LIDAR ---
echo "Lancement YOLO LiDAR..."
gnome-terminal --title="YOLO LIDAR" -- bash -c "
    $ROS_CMD;
    python3 $PERCEPTION_DIR/yolo_lidar.py;
    exec bash" &
sleep 3

# --- 7. CONE MAPPER LIDAR (SLAM) ---
echo "Lancement Cone Mapper LiDAR..."
gnome-terminal --title="CONE MAPPER LIDAR" -- bash -c "
    $ROS_CMD;
    python3 $SLAM_DIR/cone_mapper_lidar.py;
    exec bash" &
sleep 2

# --- 8. RVIZ ---
echo "Lancement RViz..."
gnome-terminal --title="RVIZ SLAM LIDAR" -- bash -c "
    $ROS_CMD;
    if [ -f \"$RVIZ_CONFIG\" ]; then
        rviz2 -d \"$RVIZ_CONFIG\";
    else
        rviz2;
    fi;
    exec bash" &
sleep 2

# --- 9. CENTERLINE FOLLOWER ---
echo "Lancement Centerline Follower (pilotage autonome)..."
gnome-terminal --title="CENTERLINE FOLLOWER" -- bash -c "
    $ROS_CMD;
    python3 $CONTROL_DIR/centerline_follower.py --map /slam_lidar/cone_map;
    exec bash" &

sleep 2
echo "Stack Centerline LiDAR lancee depuis : $SCRIPT_DIR"
echo "Topics : /slam_lidar/cone_map | /control_command"
