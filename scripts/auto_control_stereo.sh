#!/bin/bash
# =======================================================================
# LAUNCHER — FSDS + YOLO Stéréo + SLAM Stéréo + Contrôle autonome + Eval
# =======================================================================

# --- 1. DETECTION ROS ---
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
echo "ROS : $MY_ROS_DISTRO"
ROS_SETUP="/opt/ros/$MY_ROS_DISTRO/setup.bash"

# --- 2. CHEMINS ---
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
STEREO_MAP_TOPIC="/slam/cone_map"

# --- 3. SIMULATEUR ---
echo "[1/8] Lancement simulateur..."
gnome-terminal --title="SIMULATEUR" -- bash -c "
    cd $SIM_PATH;
    ./FSDS.sh -windowed -ResX=640 -ResY=480;
    exec bash" &

echo "Attente simulateur (10s)..."
sleep 10

if command -v zenity &>/dev/null; then
    zenity --info --title="IMT Driverless — Controle Stereo" \
        --text="Clique sur 'Run Simulation' dans FSDS,\npuis OK pour continuer." \
        --ok-label="Simulation lancee — Continuer" --width=400 2>/dev/null || true
else
    read -p ">>> Clique sur 'Run Simulation' dans FSDS, puis Entree..."
fi

# --- 4. BRIDGE ROS2 ---
echo "[2/8] Lancement Bridge ROS2..."
gnome-terminal --title="BRIDGE ROS2" -- bash -c "
    source $ROS_SETUP;
    cd $BRIDGE_PATH;
    source install/setup.bash;
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py;
    exec bash" &
sleep 5

# --- 5. ODOM TF ---
echo "[3/8] Lancement Odom TF Publisher..."
gnome-terminal --title="ODOM TF" -- bash -c "
    $ROS_CMD;
    python3 $PERCEPTION_DIR/odom_tf_publisher.py;
    exec bash" &
sleep 2

# --- 6. YOLO STEREO ---
echo "[4/8] Lancement YOLO Stereo..."
gnome-terminal --title="YOLO STEREO" -- bash -c "
    $ROS_CMD;
    python3 $PERCEPTION_DIR/yolo_stereo.py;
    exec bash" &
sleep 3

# --- 7. CONE MAPPER STEREO (SLAM) ---
echo "[5/8] Lancement Cone Mapper Stereo (SLAM)..."
gnome-terminal --title="SLAM STEREO" -- bash -c "
    $ROS_CMD;
    python3 $PERCEPTION_DIR/cone_mapper.py;
    exec bash" &
sleep 2

# --- 8. EVALUATEUR ---
echo "[6/8] Lancement Cone Evaluator..."
gnome-terminal --title="EVALUATEUR" -- bash -c "
    $ROS_CMD;
    python3 $PERCEPTION_DIR/cone_evaluator.py --map $STEREO_MAP_TOPIC;
    exec bash" &
sleep 2

# --- 9. CONTROLE AUTONOME ---
echo "[7/8] Lancement Controle Autonome (centerline follower)..."
gnome-terminal --title="CONTROLE AUTO" -- bash -c "
    $ROS_CMD;
    python3 $TOOLS_DIR/centerline_follower.py --map $STEREO_MAP_TOPIC;
    exec bash" &
sleep 2

# --- 10. RVIZ ---
echo "[8/8] Lancement RViz..."
gnome-terminal --title="RVIZ STEREO" -- bash -c "
    $ROS_CMD;
    if [ -f \"$RVIZ_CONFIG\" ]; then
        rviz2 -d \"$RVIZ_CONFIG\";
    else
        rviz2;
    fi;
    exec bash" &

sleep 1
echo ""
echo "Stack stereo + controle autonome lancee !"
echo "  SLAM topic   : $STEREO_MAP_TOPIC"
echo "  Stats SLAM   : /slam/stats"
echo "  Eval stats   : /evaluation/stats"
echo "  Controle     : /control_command"
echo ""
echo "Pour prendre la main manuellement, lance :"
echo "  python3 $TOOLS_DIR/global_drive.py"
