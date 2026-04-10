#!/bin/bash
# auto_eval.sh — Lance l'évaluateur SLAM + le dashboard graphique.
# Requiert que le SLAM LiDAR (slam_lidar.sh) soit déjà en cours d'exécution.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PERCEPTION_DIR="$PROJECT_ROOT/perception"
TOOLS_DIR="$PROJECT_ROOT/tools"
RVIZ_DIR="$PROJECT_ROOT/config/rviz"
WS_PATH="$PROJECT_ROOT/ros_workspace"

source /opt/ros/iron/setup.bash 2>/dev/null || source /opt/ros/galactic/setup.bash
source "$HOME/Formula-Student-Driverless-Simulator/ros2/install/setup.bash" 2>/dev/null
[ -f "$WS_PATH/install/setup.bash" ] && source "$WS_PATH/install/setup.bash"

MAP_TOPIC="${1:-/slam_lidar/cone_map}"

echo "=== Évaluation SLAM — map topic : $MAP_TOPIC ==="

# 1. cone_evaluator : ground truth AirSim + comparaison
gnome-terminal --title="CONE EVALUATOR" -- bash -c "
  source /opt/ros/iron/setup.bash 2>/dev/null || source /opt/ros/galactic/setup.bash
  source "$HOME/Formula-Student-Driverless-Simulator/ros2/install/setup.bash" 2>/dev/null
  [ -f '$WS_PATH/install/setup.bash' ] && source '$WS_PATH/install/setup.bash'
  python3 '$PERCEPTION_DIR/cone_evaluator.py' --map '$MAP_TOPIC'
  exec bash"

sleep 2

# 2. Dashboard graphique (tkinter, thread principal)
gnome-terminal --title="EVAL DASHBOARD" -- bash -c "
  source /opt/ros/iron/setup.bash 2>/dev/null || source /opt/ros/galactic/setup.bash
  source "$HOME/Formula-Student-Driverless-Simulator/ros2/install/setup.bash" 2>/dev/null
  [ -f '$WS_PATH/install/setup.bash' ] && source '$WS_PATH/install/setup.bash'
  python3 '$TOOLS_DIR/eval_dashboard.py'
  exec bash"

echo "=== Dashboard lancé. Utilisez Ctrl+C pour arrêter. ==="
echo "    Pour évaluer le SLAM stéréo : $0 /slam/cone_map"
