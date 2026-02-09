#!/bin/bash

###########################################
# CONFIG
###########################################

REPO_RAW="https://raw.githubusercontent.com/leandrebtck/IMT-Driverless-Stack/main"
ROS_DISTRO="galactic"

###########################################
# CHECK DEPENDENCIES
###########################################

command -v python3 >/dev/null 2>&1 || {
    echo "❌ python3 non trouvé"; exit 1;
}

command -v curl >/dev/null 2>&1 || {
    echo "❌ curl non trouvé"; exit 1;
}

###########################################
# DOWNLOAD cone_fusion.py (TEMP)
###########################################

TMP_DIR="/tmp/imt_driverless_cone_fusion"
mkdir -p "$TMP_DIR"

echo "📥 Téléchargement de cone_fusion.py..."
curl -s "$REPO_RAW/python_stack/cone_fusion.py" -o "$TMP_DIR/cone_fusion.py"

if [ ! -f "$TMP_DIR/cone_fusion.py" ]; then
    echo "❌ Échec téléchargement cone_fusion.py"
    exit 1
fi

chmod +x "$TMP_DIR/cone_fusion.py"

###########################################
# TERMINAL — LANCER CONE FUSION
###########################################

echo "🚀 Lancement du node cone_fusion..."

gnome-terminal -- bash -c "
    echo '🔧 Sourcing ROS2...';
    source /opt/ros/$ROS_DISTRO/setup.bash;

    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash;
        echo 'Workspace ROS2 sourcé';
    fi

    cd \"$TMP_DIR\";

    echo '🟢 cone_fusion.py en cours...';
    python3 cone_fusion.py;

    exec bash
" &

echo "✅ cone_fusion lancé"
