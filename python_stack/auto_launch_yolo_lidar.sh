#!/bin/bash

# --- 1. DÉTECTION INTELLIGENTE DE LA VERSION ROS ---
# On cherche quelle version est installée dans /opt/ros
if [ -d "/opt/ros/iron" ]; then
    MY_ROS_DISTRO="iron"
elif [ -d "/opt/ros/galactic" ]; then
    MY_ROS_DISTRO="galactic"
elif [ -d "/opt/ros/humble" ]; then
    MY_ROS_DISTRO="humble"
else
    echo "❌ ERREUR CRITIQUE : Aucune installation de ROS trouvée dans /opt/ros/"
    exit 1
fi

echo "🚀 LANCEMENT STACK (Version détectée : $MY_ROS_DISTRO)"

# --- 2. CONFIGURATION ---
SIM_PATH="$HOME/Formula-Student-Driverless-Simulator-binary"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Fonction pour charger l'environnement proprement
setup_env() {
    source /opt/ros/$MY_ROS_DISTRO/setup.bash
    
    # On ne charge le workspace QUE s'il existe vraiment
    if [ -f "$HOME/Workspace_ROS2/install/setup.bash" ]; then
        source "$HOME/Workspace_ROS2/install/setup.bash"
    elif [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
        source "$HOME/ros2_ws/install/setup.bash"
    else
        echo "⚠️  Workspace custom non trouvé (ceci est normal si tu n'as pas encore compilé fs_msgs)"
    fi
}
# On exporte la fonction et la variable pour les sous-terminaux
export -f setup_env
export MY_ROS_DISTRO

# --- 3. LANCEMENT DES TERMINAUX ---

# A. SIMULATEUR
gnome-terminal --title="SIMULATEUR" -- bash -c "cd $SIM_PATH; ./FSDS.sh -windowed -ResX=640 -ResY=480; exec bash" &
sleep 5

# B. BRIDGE ROS2
gnome-terminal --title="BRIDGE ROS2" -- bash -c "
    setup_env;
    cd ~/Formula-Student-Driverless-Simulator/ros2; 
    source install/setup.bash; 
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py; 
    exec bash" &
sleep 3

# C. LIDAR STACK
gnome-terminal --title="LIDAR PROCESSING" -- bash -c "
    setup_env;
    echo '---- 1. Filtre Sol ----';
    python3 $SCRIPT_DIR/lidar_ros.py & 
    sleep 1;
    echo '---- 2. Clustering DBSCAN ----';
    python3 $SCRIPT_DIR/lidar_cluster.py &
    wait" &
sleep 2

# D. PERCEPTION (YOLO)
# C'est ici que la fenêtre doit s'ouvrir
gnome-terminal --title="YOLO PERCEPTION" -- bash -c "
    setup_env;
    echo '👁️ Lancement YOLO...';
    python3 $SCRIPT_DIR/yolo_ros.py;
    echo '❌ Le script YOLO s est arrêté ! Regarde l erreur ci-dessus.';
    exec bash" &
sleep 2

# E. DRIVE (PILOTE)
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "
    setup_env;
    python3 $SCRIPT_DIR/global_drive.py;
    exec bash" &

echo "✅ TOUS LES SYSTÈMES SONT LANCÉS."
