#!/bin/bash
# auto_eval.sh — Lance l'évaluateur SLAM + le dashboard graphique.
# Requiert que le SLAM LiDAR (slam_lidar.sh) soit déjà en cours d'exécution.

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

# --- 2. CHEMINS ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PERCEPTION_DIR="$PROJECT_ROOT/perception"
TOOLS_DIR="$PROJECT_ROOT/tools"
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

MAP_TOPIC="${1:-/slam_lidar/cone_map}"

echo "=== Evaluation SLAM — map topic : $MAP_TOPIC ==="

# 1. cone_evaluator : ground truth AirSim + comparaison
gnome-terminal --title="CONE EVALUATOR" -- bash -c "
  $ROS_CMD;
  python3 '$PERCEPTION_DIR/cone_evaluator.py' --map '$MAP_TOPIC';
  exec bash"

sleep 2

# 2. Dashboard graphique (tkinter, thread principal)
gnome-terminal --title="EVAL DASHBOARD" -- bash -c "
  $ROS_CMD;
  python3 '$TOOLS_DIR/eval_dashboard.py';
  exec bash"

echo "=== Dashboard lance. Utilisez Ctrl+C pour arreter. ==="
echo "    Pour evaluer le SLAM stereo : $0 /slam/cone_map"
