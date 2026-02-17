#!/bin/bash
set -e

echo "=============================="
echo "   INSTALLATION / UPDATE FSDS"
echo "=============================="

############################################
# FONCTIONS UTILITAIRES
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
# 1) SIMULATEUR BINARY
############################################

echo "[1] Vérification simulateur FSDS..."

SIM_DIR="$HOME/Formula-Student-Driverless-Simulator-binary"

if [ ! -d "$SIM_DIR" ]; then
    mkdir -p "$SIM_DIR"
fi

cd "$SIM_DIR"

if [ ! -f "./FSDS.sh" ]; then
    echo "➡ Téléchargement simulateur..."
    wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip -O fsds.zip
    unzip -o fsds.zip
    rm fsds.zip
else
    echo "✅ Simulateur déjà installé."
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
# 3) ROS2
############################################

echo "[3] Vérification ROS2 Galactic..."

if [ ! -d "/opt/ros/galactic" ]; then
    echo "➡ Installation ROS2 Galactic..."
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
    echo "✅ ROS2 déjà installé."
fi

add_to_bashrc_if_missing "source /opt/ros/galactic/setup.bash"


############################################
# 4) DÉPENDANCES ROS2
############################################

echo "[4] Installation dépendances ROS2 (si nécessaire)..."

sudo apt install -y \
    python3-colcon-common-extensions \
    libyaml-cpp-dev \
    libcurl4-openssl-dev \
    ros-galactic-cv-bridge \
    ros-galactic-image-transport \
    ros-galactic-tf2-geometry-msgs \
    ros-galactic-joy \
    ros-galactic-sensor-msgs-py


############################################
# 5) CLONE FSDS
############################################

echo "[5] Vérification repo FSDS..."

FSDS_DIR="$HOME/Formula-Student-Driverless-Simulator"

if [ ! -d "$FSDS_DIR" ]; then
    git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules
    cd "$FSDS_DIR"
    git checkout tags/v2.2.0
else
    echo "✅ Repo déjà cloné."
fi


############################################
# 6) AIRLIB
############################################

echo "[6] Vérification AirLib..."

if [ ! -d "$FSDS_DIR/AirSim/build" ]; then
    cd "$FSDS_DIR/AirSim"
    ./setup.sh
else
    echo "✅ AirLib déjà compilé."
fi


############################################
# 7) BUILD ROS2 BRIDGE
############################################

echo "[7] Vérification build ROS2 bridge..."

if [ ! -d "$FSDS_DIR/ros2/install" ]; then
    cd "$FSDS_DIR/ros2"
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    echo "✅ ROS2 bridge déjà compilé."
fi

add_to_bashrc_if_missing "source $FSDS_DIR/ros2/install/setup.bash"


############################################
# 8) PYTHON
############################################

echo "[8] Vérification dépendances Python..."

sudo apt install -y python3-pip python3-venv

pip3 install --upgrade pip

pip3 install ultralytics opencv-python pynput pyqtgraph PyQt5 scikit-learn

PY_REQUIREMENTS="$FSDS_DIR/python/requirements.txt"

if [ -f "$PY_REQUIREMENTS" ]; then
    pip3 install -r "$PY_REQUIREMENTS"
fi


############################################
# 9) OUTILS FENÊTRES
############################################

echo "[9] Vérification outils fenêtres..."

sudo apt install -y xdotool wmctrl


############################################
# FIN
############################################

echo ""
echo "============================================"
echo " INSTALLATION / UPDATE TERMINÉE 🎉"
echo "============================================"
echo ""
