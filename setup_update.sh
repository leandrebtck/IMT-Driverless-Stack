#!/bin/bash
set -e
trap 'echo "❌ Erreur à la ligne $LINENO"' ERR

echo "========================================="
echo "     FSDS INSTALL / UPDATE SCRIPT"
echo "========================================="

# --- FONCTIONS UTILITAIRES ---
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

if [ ! -f "$SIM_DIR/FSDS.sh" ]; then
    echo "➡ Téléchargement simulateur FSDS v2.2.0..."
    # On télécharge dans /tmp pour éviter de polluer si ça fail
    wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip -O /tmp/fsds.zip
    unzip -o /tmp/fsds.zip -d "$SIM_DIR"
    # FSDS se dézippe souvent dans un sous-dossier, on s'assure que FSDS.sh est au bon endroit
    if [ -d "$SIM_DIR/LinuxNoEditor" ]; then
        mv "$SIM_DIR/LinuxNoEditor"/* "$SIM_DIR/"
        rmdir "$SIM_DIR/LinuxNoEditor"
    fi
    chmod +x "$SIM_DIR/FSDS.sh"
    rm /tmp/fsds.zip
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
# 3) DÉTECTION INTELLIGENTE ROS2 (CRITIQUE)
############################################
echo "[3] Configuration ROS2..."

# Détection de la version Ubuntu
UBUNTU_VER=$(lsb_release -rs)
echo "   ℹ️  Ubuntu version : $UBUNTU_VER"

if [ "$UBUNTU_VER" == "20.04" ]; then
    TARGET_ROS="galactic"
elif [ "$UBUNTU_VER" == "22.04" ]; then
    TARGET_ROS="iron"
else
    echo "⚠️  Version Ubuntu non standard ($UBUNTU_VER). On tente Iron par défaut."
    TARGET_ROS="iron"
fi

# Vérification si ROS est déjà installé
if [ -f "/opt/ros/$TARGET_ROS/setup.bash" ]; then
    echo "✅ ROS2 $TARGET_ROS est déjà installé."
else
    echo "➡ Installation de ROS2 $TARGET_ROS..."
    
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release
    
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    sudo apt install -y ros-$TARGET_ROS-desktop
fi

# Sourcing immédiat pour la suite du script
source /opt/ros/$TARGET_ROS/setup.bash
add_to_bashrc_if_missing "source /opt/ros/$TARGET_ROS/setup.bash"

echo "✅ Environnement ROS2 ($TARGET_ROS) chargé."

############################################
# 4) DÉPENDANCES ROS2
############################################
echo "[4] Installation dépendances ROS2..."

sudo apt install -y python3-colcon-common-extensions libyaml-cpp-dev libcurl4-openssl-dev

# Installation dynamique des paquets selon la version détectée
sudo apt install -y \
    ros-$TARGET_ROS-cv-bridge \
    ros-$TARGET_ROS-image-transport \
    ros-$TARGET_ROS-tf2-geometry-msgs \
    ros-$TARGET_ROS-joy \
    ros-$TARGET_ROS-sensor-msgs-py \
    ros-$TARGET_ROS-vision-msgs

############################################
# 5) CLONE OU UPDATE FSDS
############################################
echo "[5] Vérification repo FSDS (Code)..."

FSDS_DIR="$HOME/Formula-Student-Driverless-Simulator"

if [ ! -d "$FSDS_DIR" ]; then
    git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules "$FSDS_DIR"
fi

cd "$FSDS_DIR"
# Force le checkout propre
git fetch --all --tags
git checkout tags/v2.2.0
git submodule update --init --recursive

############################################
# 6) AIRLIB
############################################
echo "[6] Compilation AirLib..."

AIRSIM_DIR="$FSDS_DIR/AirSim"
if [ ! -d "$AIRSIM_DIR/build" ]; then
    cd "$AIRSIM_DIR"
    ./setup.sh
else
    echo "✅ AirLib déjà compilé."
fi

############################################
# 7) BUILD ROS2 BRIDGE
############################################
echo "[7] Build ROS2 bridge..."

cd "$FSDS_DIR/ros2"

# On source ROS avant de builder
source /opt/ros/$TARGET_ROS/setup.bash

if [ ! -d "install" ]; then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    echo "➡ Rebuild rapide..."
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

# Ajout au bashrc (seulement si pas déjà fait)
grep -qxF "source $FSDS_DIR/ros2/install/setup.bash" ~/.bashrc || echo "source $FSDS_DIR/ros2/install/setup.bash" >> ~/.bashrc

############################################
# 8) PYTHON
############################################
echo "[8] Installation dépendances Python..."

sudo apt install -y python3-pip python3-venv
python3 -m pip install --upgrade pip

# Installation des libs essentielles + YOLO
python3 -m pip install ultralytics pynput opencv-python pyqtgraph PyQt5 scikit-learn "numpy<2.0"

# Requirements FSDS
PY_REQUIREMENTS="$FSDS_DIR/python/requirements.txt"
if [ -f "$PY_REQUIREMENTS" ]; then
    python3 -m pip install -r "$PY_REQUIREMENTS"
fi

############################################
# 9) OUTILS SYSTEME
############################################
echo "[9] Outils fenêtre..."
sudo apt install -y xdotool wmctrl

echo ""
echo "========================================="
echo "     INSTALLATION TERMINÉE 🎉"
echo "========================================="
echo "ROS Version : $TARGET_ROS"
echo "N'oublie pas de taper : source ~/.bashrc"
