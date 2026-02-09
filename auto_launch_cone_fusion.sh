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

###########################################
# DOWNLOAD cone_fusion.py
###########################################

echo "📥 Téléchargement de cone_fusion.py..."
curl -s "$REPO_RAW/python_stack/cone_fusion.py" -o "$TMP_DIR/cone_fusion.py"
chmod +x "$TMP_DIR/cone_fusion.py"

###########################################
# SOURCE ROS2
###########################################

source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

###########################################
# LAUNCH cone_fusion.py DIRECTLY
###########################################

echo "🚀 Lancement du node cone_fusion.py..."
gnome-terminal -- bash -c "
    cd \"$TMP_DIR\";
    echo '🟢 cone_fusion.py en cours...';
    python3 cone_fusion.py;
    exec bash
"

echo "✅ cone_fusion.py lancé"
