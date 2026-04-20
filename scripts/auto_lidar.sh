#!/bin/bash
# ==========================================
# LAUNCHER INTELLIGENT - FSDS + LIDAR SEUL
# Pipeline : Filtre Sol -> DBSCAN -> Cone Mapper
# Publie : /lidar/cone_markers (RViz) + /slam_lidar/cone_map (carte persistante)
# ==========================================

# --- 1. DETECTION AUTOMATIQUE DE ROS (Iron / Galactic / Humble) ---
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
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PERCEPTION_DIR="$PROJECT_ROOT/python_scripts/1perception"
SLAM_DIR="$PROJECT_ROOT/python_scripts/2slam"
CONTROL_DIR="$PROJECT_ROOT/python_scripts/3control"
PERF_DIR="$PROJECT_ROOT/python_scripts/performance"
RVIZ_DIR="$PROJECT_ROOT/config/rviz"
INTERNAL_WS="$PROJECT_ROOT/ros_workspace"

# --- 3. GESTION DU WORKSPACE ---
# Compilation automatique si le workspace n'est pas encore compile
if [ -d "$INTERNAL_WS/src" ] && [ ! -f "$INTERNAL_WS/install/setup.bash" ]; then
    echo "Compilation du workspace requise. Patientez..."
    bash -c "source $ROS_SETUP && cd $INTERNAL_WS && colcon build --symlink-install" || { echo "ECHEC COMPILATION"; exit 1; }
fi

if [ -f "$INTERNAL_WS/install/setup.bash" ]; then
    echo "Utilisation du Workspace interne : $INTERNAL_WS"
    WS_SETUP="$INTERNAL_WS/install/setup.bash"
elif [ -f "$HOME/Workspace_ROS2/install/setup.bash" ]; then
    echo "Utilisation du Workspace global : ~/Workspace_ROS2"
    WS_SETUP="$HOME/Workspace_ROS2/install/setup.bash"
else
    echo "Aucun workspace compile trouve. fs_msgs sera indisponible."
    WS_SETUP=""
fi

ROS_CMD="source $ROS_SETUP"
if [ -n "$WS_SETUP" ]; then
    ROS_CMD="$ROS_CMD; source $WS_SETUP"
fi

echo "LANCEMENT STACK LIDAR..."
sleep 1

# ==========================================
# 4. LANCEMENT DES PROCESSUS
# ==========================================

# A. SIMULATEUR
echo "Lancement Simulateur..."
gnome-terminal --title="SIMULATEUR" -- bash -c "
    cd $SIM_PATH;
    ./FSDS.sh -windowed -ResX=640 -ResY=480;
    exec bash" &
sleep 10

if command -v zenity &>/dev/null; then
    zenity --info --title="IMT Driverless" \
        --text="Clique sur 'Run Simulation' dans le simulateur FSDS,\npuis clique sur OK pour continuer." \
        --ok-label="Simulation lancee — Continuer" --width=400 2>/dev/null || true
else
    read -p ">>> Clique sur 'Run Simulation' dans FSDS, puis appuie sur Entree..."
fi

# B. ROS2 BRIDGE
echo "Lancement Bridge ROS2..."
gnome-terminal --title="BRIDGE ROS2" -- bash -c "
    $ROS_CMD;
    cd ~/Formula-Student-Driverless-Simulator/ros2;
    source install/setup.bash;
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py;
    exec bash" &
sleep 5

# C. ODOM TF PUBLISHER (necessaire pour cone_mapper_lidar : repere fsds/map)
echo "Lancement Odom TF Publisher..."
gnome-terminal --title="ODOM TF" -- bash -c "
    $ROS_CMD;
    echo '---- Odom TF Publisher ----';
    python3 $PERCEPTION_DIR/odom_tf_publisher.py;
    exec bash" &
sleep 2

# D. PIPELINE LIDAR (Filtre Sol + Clustering + Mapper)
echo "Lancement Pipeline LiDAR..."
gnome-terminal --title="LIDAR PIPELINE" -- bash -c "
    $ROS_CMD;

    echo '---- 1. Filtre Sol (Z > -0.40m) ----';
    python3 $PERCEPTION_DIR/lidar_ros.py &
    sleep 2;

    echo '---- 2. Clustering DBSCAN + Publication detections ----';
    python3 $PERCEPTION_DIR/lidar_cluster.py &
    sleep 2;

    echo '---- 3. Cone Mapper LiDAR (carte persistante) ----';
    python3 $SLAM_DIR/cone_mapper_lidar.py &

    wait" &
sleep 3

#E. GLOBAL DRIVE
echo "Lancement Global Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "
    $ROS_CMD;
    python3 $CONTROL_DIR/global_drive.py; 
    exec bash" &
sleep 3

# F. RVIZ
echo "Lancement RViz..."
gnome-terminal --title="RVIZ" -- bash -c "
    $ROS_CMD;
    echo 'RViz : Fixed Frame -> fsds/Lidar1';
    echo 'Add -> By topic -> /lidar/cone_markers -> MarkerArray';
    echo 'Add -> By topic -> /slam_lidar/cone_map -> MarkerArray';
    rviz2;
    exec bash" &

echo ""
echo "TOUS LES SYSTEMES SONT LANCES."
echo ""
echo "Topics utiles :"
echo "  /lidar/cone_markers      — clusters bruts (spheres vertes)"
echo "  /slam_lidar/cone_map     — carte persistante (jaune/bleu/gris)"
echo "  /slam_lidar/stats        — compteurs cones en temps reel"
echo ""
echo "Dans RViz : Fixed Frame -> fsds/Lidar1"
