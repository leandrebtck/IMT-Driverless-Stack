#!/bin/bash

###########################################
# CONFIG
###########################################

REPO_RAW="https://raw.githubusercontent.com/leandrebtck/IMT-Driverless-Stack/main"
TMP_DIR="/tmp/imt_driverless_cone_fusion"
ROS_DISTRO="galactic"

mkdir -p "$TMP_DIR"

###########################################
# CHECK DEPENDENCIES
###########################################

command -v python3 >/dev/null 2>&1 || { echo "❌ python3 non trouvé"; exit 1; }
command -v curl >/dev/null 2>&1 || { echo "❌ curl non trouvé"; exit 1; }
command -v ros2 >/dev/null 2>&1 || { echo "❌ ros2 non trouvé"; exit 1; }
command -v gnome-terminal >/dev/null 2>&1 || { echo "❌ gnome-terminal non trouvé"; exit 1; }

###########################################
# DOWNLOAD cone_fusion.py AND circuit_map.py
###########################################

echo "📥 Téléchargement de cone_fusion.py..."
curl -s "$REPO_RAW/python_stack/cone_fusion.py" -o "$TMP_DIR/cone_fusion.py"
chmod +x "$TMP_DIR/cone_fusion.py"

echo "📥 Téléchargement de circuit_map.py (PyQtGraph)..."
curl -s "$REPO_RAW/python_stack/circuit_map.py" -o "$TMP_DIR/circuit_map.py"
chmod +x "$TMP_DIR/circuit_map.py"

###########################################
# SOURCE ROS2
###########################################

source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

###########################################
# LAUNCH cone_fusion.py
###########################################

echo "🚀 Lancement du node cone_fusion.py..."
gnome-terminal -- bash -c "
    cd \"$TMP_DIR\";
    echo '🟢 cone_fusion.py en cours...';
    python3 cone_fusion.py;
    exec bash
" &

sleep 2  # laisser un petit délai pour que cone_fusion démarre

###########################################
# LAUNCH circuit_map.py (PyQtGraph)
###########################################

echo "🚀 Lancement du node circuit_map.py (PyQtGraph)..."
gnome-terminal -- bash -c "
    cd \"$TMP_DIR\";
    echo '🟢 circuit_map.py en cours...';
    python3 circuit_map.py;
    exec bash
" &

echo "✅ cone_fusion.py et circuit_map.py lancés avec PyQtGraph"
