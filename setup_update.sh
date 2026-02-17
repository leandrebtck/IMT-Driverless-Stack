#!/bin/bash
set -e
trap 'echo "❌ Erreur à la ligne $LINENO"' ERR

echo "========================================="
echo "     FSDS INSTALL / UPDATE SCRIPT"
echo "========================================="

############################################
# UTILITAIRES
############################################

command_exists() {
    command -v "$1" >/dev/null 2>&1
}

add_to_bashrc_if_missing() {
    if ! grep -Fxq "$1" ~/.bashrc; then
        echo "$1" >> ~/.bashrc
    fi
}

############################################
# 1) SIMULATEUR BINAIRES
############################################

echo "[1] Vérification simulateur FSDS..."

SIM_DIR="$HOME/Formula-Student-Driverless-Simulator-binary"
mkdir -p "$SIM_DIR"
cd "$SIM_DIR"

if [ ! -f "./FSDS.sh" ]; then
    echo "➡ Téléchargement simulateur FSDS v2.2.0..."
    wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip -O fsds.zip
    unzip -o fsds.zip
    rm fsds.zip
else
    echo "✅ Simulateur déjà présent."
fi

############################################
# 2) CMAKE
############################################

echo "[2] Vérification CMake..."
if ! command_exists cmake; then
    sudo snap install cmake --classic
    add_to_bashrc_if_missing 'export PATH=/snap/bin:$PATH'
else
    echo "✅ CMake déjà installé."
fi

############################################
# 3) ROS2 AUTO-DETECTION
############################################

echo "[3] Détection ROS2..."

ROS_DISTRO_FOUND=""

# Vérifie si ROS_DISTRO est déjà défini
if [ -n "$ROS_DISTRO" ]; then
    ROS_DISTRO_FOUND="$ROS_DISTRO"
fi

# Sinon regarde dans /opt/ros
if [ -z "$ROS_DISTRO_FOUND" ] && [ -d "/opt/ros" ]; then
    ROS_DISTRO_FOUND=$(ls /opt/ros | grep -E "iron|galactic" | head -n 1)
fi

# Cas : aucune version trouvée → installer Galactic
if [ -z "$ROS_DISTRO_FOUND" ]; then
    ROS_DISTRO_FOUND="galactic"
    echo "➡ Aucune version de ROS trouvée. Installation ROS Galactic..."

    sudo apt update
    sudo apt install -y curl gnupg2 lsb-release

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-galactic-desktop
else
    echo "✅ ROS2 détecté : $ROS_DISTRO_FOUND"
fi

# Source automatique
source /opt/ros/$ROS_DISTRO_FOUND/setup.bash
add_to_bashrc_if_missing "source /opt/ros/$ROS_DISTRO_FOUND/setup.bash"

############################################
# 4) DÉPENDANCES ROS2
############################################

echo "[4] Installation dépendances ROS2 pour $ROS_DISTRO_FOUND..."

sudo apt install -y python3-colcon-common-extensions libyaml-cpp-dev libcurl4-openssl-dev

if [ "$ROS_DISTRO_FOUND" = "galactic" ]; then
    sudo apt install -y \
        ros-galactic-cv-bridge \
        ros-galactic-image-transport \
        ros-galactic-tf2-geometry-msgs \
        ros-galactic-joy \
        ros-galactic-sensor-msgs-py \
        ros-galactic-vision-msgs
elif [ "$ROS_DISTRO_FOUND" = "iron" ]; then
    sudo apt install -y \
        ros-iron-cv-bridge \
        ros-iron-image-transport \
        ros-iron-tf2-geometry-msgs \
        ros-iron-joy \
        ros-iron-sensor-msgs-py \
        ros-iron-vision-msgs
fi

############################################
# 5) CLONE OU UPDATE FSDS
############################################

echo "[5] Vérification repo FSDS..."

FSDS_DIR="$HOME/Formula-Student-Driverless-Simulator"

if [ ! -d "$FSDS_DIR" ]; then
    git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules
fi

cd "$FSDS_DIR"
git checkout tags/v2.2.0
git submodule update --init --recursive || true

############################################
# 6) AIRLIB
############################################

echo "[6] Vérification AirLib..."

AIRSIM_DIR="$FSDS_DIR/AirSim"

if [ ! -d "$AIRSIM_DIR" ]; then
    echo "❌ Dossier AirSim introuvable !"
else
    if [ ! -d "$AIRSIM_DIR/build" ]; then
        echo "➡ Compilation AirLib..."
        cd "$AIRSIM_DIR"
        ./setup.sh
    else
        echo "✅ AirLib déjà compilé."
    fi
fi

############################################
# 7) BUILD ROS2 BRIDGE
############################################

echo "[7] Build ROS2 bridge..."

cd "$FSDS_DIR/ros2"

if [ ! -d "install" ]; then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    echo "➡ Rebuild rapide (update code éventuel)..."
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

add_to_bashrc_if_missing "source $FSDS_DIR/ros2/install/setup.bash"

############################################
# 8) PYTHON
############################################

echo "[8] Installation dépendances Python..."

sudo apt install -y python3-pip python3-venv
python3 -m pip install --upgrade pip

python3 -m pip install ultralytics pynput opencv-python pyqtgraph PyQt5 scikit-learn

PY_REQUIREMENTS="$FSDS_DIR/python/requirements.txt"
if [ -f "$PY_REQUIREMENTS" ]; then
    python3 -m pip install -r "$PY_REQUIREMENTS"
fi

############################################
# 9) OUTILS FENÊTRES
############################################

echo "[9] Outils gestion fenêtres..."
sudo apt install -y xdotool wmctrl

############################################
# FIN
############################################

echo ""
echo "========================================="
echo "     INSTALLATION / UPDATE TERMINÉE 🎉"
echo "========================================="
echo ""
echo "ROS2 utilisé : $ROS_DISTRO_FOUND"
echo ""
