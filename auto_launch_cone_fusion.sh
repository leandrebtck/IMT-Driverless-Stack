#!/bin/bash

###########################################
# CONFIG
###########################################

REPO_RAW="https://raw.githubusercontent.com/leandrebtck/IMT-Driverless-Stack/main"
TMP_DIR="/tmp/imt_driverless_cone_fusion"
VENV_DIR="/tmp/imt_driverless_env"
ROS_DISTRO="galactic"

mkdir -p "$TMP_DIR"
mkdir -p "$VENV_DIR"

###########################################
# CHECK DEPENDENCIES
###########################################

for cmd in python3 curl ros2 gnome-terminal; do
    command -v $cmd >/dev/null 2>&1 || { echo "❌ $cmd non trouvé"; exit 1; }
done

###########################################
# CREATE OR ACTIVATE VENV
###########################################

if [ ! -d "$VENV_DIR/bin" ]; then
    echo "🐍 Création de l'environnement virtuel Python..."
    python3 -m venv "$VENV_DIR"
    source "$VENV_DIR/bin/activate"
    pip install --upgrade pip
    pip install pyqtgraph PyQt5>=5.15
else
    echo "🐍 Activation de l'environnement virtuel Python..."
    source "$VENV_DIR/bin/activate"
fi

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
    source \"$VENV_DIR/bin/activate\";
    cd \"$TMP_DIR\";
    echo '🟢 cone_fusion.py en cours...';
    python3 cone_fusion.py;
    exec bash
" &

sleep 2

###########################################
# LAUNCH circuit_map.py (PyQtGraph)
###########################################

echo "🚀 Lancement du node circuit_map.py (PyQtGraph)..."
gnome-terminal -- bash -c "
    source \"$VENV_DIR/bin/activate\";
    cd \"$TMP_DIR\";
    echo '🟢 circuit_map.py en cours...';
    python3 circuit_map.py;
    exec bash
" &

echo "✅ cone_fusion.py et circuit_map.py lancés avec PyQtGraph dans venv Python isolé"
