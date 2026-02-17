#!/bin/bash

# ==========================================
# 🚀 LAUNCHER INTELLIGENT - FSDS + FULL STACK
# ==========================================

# --- 1. DÉTECTION AUTOMATIQUE DE ROS (Iron / Galactic / Humble) ---
if [ -f "/opt/ros/iron/setup.bash" ]; then
    MY_ROS_DISTRO="iron"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    MY_ROS_DISTRO="galactic"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    MY_ROS_DISTRO="humble"
else
    echo "❌ ERREUR : Aucune installation ROS détectée dans /opt/ros/"
    exit 1
fi

echo "✅ ROS Version détectée : $MY_ROS_DISTRO"
ROS_SETUP="/opt/ros/$MY_ROS_DISTRO/setup.bash"

# --- 2. CONFIGURATION CHEMINS ---
SIM_PATH="$HOME/Formula-Student-Driverless-Simulator-binary"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
INTERNAL_WS="$PROJECT_ROOT/ros_workspace"

# --- 3. GESTION DU WORKSPACE (Sourcer le bon fichier) ---
# On prépare la commande qui sera exécutée dans chaque terminal
if [ -f "$INTERNAL_WS/install/setup.bash" ]; then
    echo "📂 Utilisation du Workspace interne : $INTERNAL_WS"
    WS_SETUP="$INTERNAL_WS/install/setup.bash"
elif [ -f "$HOME/Workspace_ROS2/install/setup.bash" ]; then
    echo "📂 Utilisation du Workspace global : ~/Workspace_ROS2"
    WS_SETUP="$HOME/Workspace_ROS2/install/setup.bash"
else
    echo "⚠️  Aucun Workspace compilé trouvé. YOLO fonctionnera, mais Drive risque d'échouer."
    WS_SETUP=""
fi

# Commande de base pour chaque terminal : Source ROS + Source Workspace (si existe)
ROS_CMD="source $ROS_SETUP"
if [ -n "$WS_SETUP" ]; then
    ROS_CMD="$ROS_CMD; source $WS_SETUP"
fi

echo "🚀 LANCEMENT DE LA STACK COMPLETE..."
sleep 1

# ==========================================
# 4. LANCEMENT DES PROCESSUS
# ==========================================

# A. SIMULATEUR
echo "🎮 Lancement Simu..."
gnome-terminal --title="SIMULATEUR" -- bash -c "cd $SIM_PATH; ./FSDS.sh -windowed -ResX=640 -ResY=480; exec bash" &
sleep 5

# B. ROS2 BRIDGE
echo "🔌 Lancement Bridge..."
gnome-terminal --title="BRIDGE ROS2" -- bash -c "
    $ROS_CMD; 
    cd ~/Formula-Student-Driverless-Simulator/ros2; 
    source install/setup.bash; 
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py; 
    exec bash" &
sleep 3

# C. LIDAR STACK (Filtre + Cluster)
echo "🟢 Lancement LiDAR..."
gnome-terminal --title="LIDAR PROCESSING" -- bash -c "
    $ROS_CMD;
    echo '---- 1. Filtre Sol ----';
    python3 $SCRIPT_DIR/lidar_ros.py & 
    sleep 1;
    echo '---- 2. Clustering DBSCAN ----';
    python3 $SCRIPT_DIR/lidar_cluster.py &
    
    # Le wait empêche la fermeture en cas d'erreur
    wait" &
sleep 2

# D. PERCEPTION & FUSION (YOLO + Fusion Node)
echo "👁️ Lancement Perception (YOLO + FUSION)..."
gnome-terminal --title="PERCEPTION FUSION" -- bash -c "
    $ROS_CMD;
    
    echo '---- 1. YOLOv8 ----';
    python3 $SCRIPT_DIR/yolo_ros.py &
    sleep 2;
    
    echo '---- 2. Fusion Lidar-Camera ----';
    # On lance la fusion maintenant que YOLO et LiDAR tournent
    python3 $SCRIPT_DIR/fusion_node.py &
    
    wait" &
sleep 2

# E. DRIVE (PILOTE)
echo "🏎️ Lancement Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "
    $ROS_CMD;
    python3 $SCRIPT_DIR/global_drive.py; 
    exec bash" &

echo "✅ TOUS LES SYSTÈMES SONT LANCÉS."
