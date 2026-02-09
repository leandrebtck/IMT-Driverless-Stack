#!/bin/bash

###########################################
# CONFIG
###########################################

REPO_RAW="https://raw.githubusercontent.com/leandrebtck/IMT-Driverless-Stack/main"
ROS_DISTRO="galactic"

TMP_DIR="/tmp/imt_driverless_cone_fusion"
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

if [ ! -f "$TMP_DIR/cone_fusion.py" ]; then
    echo "❌ Échec téléchargement cone_fusion.py"
    exit 1
fi

chmod +x "$TMP_DIR/cone_fusion.py"

###########################################
# SOURCE ROS2
###########################################

source /opt/ros/$ROS_DISTRO/setup.bash

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "Workspace ROS2 sourcé"
fi

###########################################
# WAIT FOR /lidar/obstacles
###########################################

echo "⏳ Attente que /lidar/obstacles soit publié..."

MAX_WAIT=30
COUNT=0
while ! ros2 topic list | grep -q '/lidar/obstacles'; do
    sleep 1
    COUNT=$((COUNT+1))
    if [ $COUNT -ge $MAX_WAIT ]; then
        echo "❌ /lidar/obstacles introuvable après $MAX_WAIT secondes"
        exit 1
    fi
done

echo "✅ /lidar/obstacles détecté"

###########################################
# LAUNCH cone_fusion.py
###########################################

echo "🚀 Lancement du node cone_fusion.py dans un nouveau terminal..."
gnome-terminal -- bash -c "
    cd \"$TMP_DIR\";
    echo '🟢 cone_fusion.py en cours...';
    python3 cone_fusion.py;
    exec bash
"

echo "✅ cone_fusion.py lancé"
