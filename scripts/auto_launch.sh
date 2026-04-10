#!/bin/bash

# ==========================================
# LAUNCHER - FSDS + YOLO (TEST CAMÉRA SEULE)
# ==========================================

# --- 1. DÉTECTION AUTOMATIQUE DE ROS ---
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
PERCEPTION_DIR="$PROJECT_ROOT/perception"
TOOLS_DIR="$PROJECT_ROOT/tools"
RVIZ_DIR="$PROJECT_ROOT/config/rviz"
INTERNAL_WS="$PROJECT_ROOT/ros_workspace"

# --- 3. GESTION DU WORKSPACE ROS ---
if [ -d "$INTERNAL_WS/src" ]; then
    echo "✅ Workspace interne détecté : $INTERNAL_WS"
    
    # Compilation automatique si nécessaire
    if [ ! -f "$INTERNAL_WS/install/setup.bash" ]; then
        echo "⚠️  Compilation requise. Patientez..."
        # On source ROS avant de compiler
        bash -c "source $ROS_SETUP && cd $INTERNAL_WS && colcon build --symlink-install" || { echo "❌ ÉCHEC COMPILATION"; exit 1; }
    fi
    
    # Commande à injecter dans les terminaux
    ROS_CMD="source $ROS_SETUP; source $INTERNAL_WS/install/setup.bash"
    
elif [ -f "$HOME/Workspace_ROS2/install/setup.bash" ]; then
    echo "⚠️  Pas de workspace interne. Utilisation de ~/Workspace_ROS2..."
    ROS_CMD="source $ROS_SETUP; source $HOME/Workspace_ROS2/install/setup.bash"
else
    echo "⚠️  Aucun workspace trouvé. Seul ROS standard sera chargé."
    ROS_CMD="source $ROS_SETUP"
fi

echo "🚀 LANCEMENT STACK IMT DRIVERLESS (YOLO SEUL)"

# 1. SIMULATEUR
echo "🎮 Lancement Simu..."
gnome-terminal --title="SIMULATEUR" -- bash -c "cd $SIM_PATH; ./FSDS.sh -windowed -ResX=640 -ResY=480; exec bash" &
sleep 10

if command -v zenity &>/dev/null; then
    zenity --info --title="IMT Driverless" \
        --text="Clique sur 'Run Simulation' dans le simulateur FSDS,\npuis clique sur OK pour continuer." \
        --ok-label="Simulation lancee — Continuer" --width=400 2>/dev/null || true
else
    read -p ">>> Clique sur 'Run Simulation' dans FSDS, puis appuie sur Entree..."
fi

# 2. ROS2 BRIDGE 
echo "🔌 Lancement Bridge..."
# Le bridge a besoin de charger ROS avant ses propres dépendances
gnome-terminal --title="BRIDGE ROS2" -- bash -c "source $ROS_SETUP; cd ~/Formula-Student-Driverless-Simulator/ros2; source install/setup.bash; ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py; exec bash" &
sleep 3

# 3. PERCEPTION (YOLO)
echo "👁️ Lancement YOLO..."
gnome-terminal --title="YOLO PERCEPTION" -- bash -c "
    $ROS_CMD; 
    python3 $SCRIPT_DIR/yolo_ros.py; 
    exec bash" &
sleep 2

# 4. DRIVE (PILOTE)
echo "🏎️ Lancement Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "
    $ROS_CMD; 
    python3 $TOOLS_DIR/global_drive.py; 
    exec bash" &

sleep 2
echo "✅ Tout est lancé depuis : $SCRIPT_DIR"
