#!/bin/bash
set -e

echo "=============================="
echo " INSTALLATION FSDS + ROS2     "
echo "=============================="

###############################
# 1) Simulateur FSDS Binary
###############################

echo "[1/10] Installation du simulateur FSDS..."
mkdir -p ~/Formula-Student-Driverless-Simulator-binary
cd ~/Formula-Student-Driverless-Simulator-binary

if [ ! -f "./fsds-v2.2.0-linux.zip" ]; then
    wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip -O fsds.zip
    unzip fsds.zip
    rm fsds.zip
else
    echo "Simulateur déjà téléchargé."
fi


################################
# 2) Installation CMake
################################

echo "[2/10] Mise à jour CMake..."
sudo apt remove --purge -y cmake || true
sudo snap install cmake --classic
echo 'export PATH=/snap/bin:$PATH' >> ~/.bashrc
source ~/.bashrc


################################
# 3) Dépôts ROS2 Galactic
################################

echo "[3/10] Dépôts ROS2 Galactic..."
sudo apt update
sudo apt install -y curl gnupg2 lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update


################################
# 4) Installation ROS Galactic
################################

echo "[4/10] Installation ROS2 Galactic..."
sudo apt install -y ros-galactic-desktop

echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
source /opt/ros/galactic/setup.bash


################################
# 5) Dépendances FSDS ROS2
################################

echo "[5/10] Installation dépendances FSDS ROS2..."
sudo apt install -y \
    python3-colcon-common-extensions \
    libyaml-cpp-dev \
    libcurl4-openssl-dev \
    ros-galactic-cv-bridge \
    ros-galactic-image-transport \
    ros-galactic-tf2-geometry-msgs \
    ros-galactic-joy


################################
# 6) Installation Git + Clonage FSDS
################################

echo "[6/10] Installation Git + clonage FSDS..."
sudo apt install -y git

cd ~
if [ ! -d "~/Formula-Student-Driverless-Simulator" ]; then
    git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules
fi

cd ~/Formula-Student-Driverless-Simulator
git checkout tags/v2.2.0


################################
# 7) Setup AirLib
################################

echo "[7/10] AirLib setup (long)..."
cd ~/Formula-Student-Driverless-Simulator/AirSim
./setup.sh


################################
# 8) Build du ROS2 bridge
################################

echo "[8/10] Compilation ROS2 bridge..."
cd ~/Formula-Student-Driverless-Simulator/ros2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


################################
# 9) Ajout au .bashrc
################################

echo "[9/10] Ajout du setup ROS FSDS dans .bashrc..."
echo "source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash" >> ~/.bashrc

################################
# 10) Installation de l'nstallation des outils nécessaires pour automatiser des interactions avec la fenêtre active

# xdotool : permet de simuler des entrées clavier/souris et manipuler les fenêtres
# wmctrl  : permet de gérer et contrôler les fenêtres (changer de bureau, focus, etc.)
################################

sudo apt install -y xdotool
sudo apt install wmctrl

################################
# 11) Fin
################################

echo ""
echo "============================================"
echo " INSTALLATION TERMINÉE 🎉"
echo "============================================"
echo ""
echo "Démarrer le simulateur :"
echo "   cd ~/Formula-Student-Driverless-Simulator-binary"
echo "   ./FSDS.sh"
echo ""
echo "Démarrer le ROS bridge (dans un autre terminal) :"
echo "   cd ~/Formula-Student-Driverless-Simulator/ros2"
echo "   source install/setup.bash"
echo "   ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py"
echo ""
