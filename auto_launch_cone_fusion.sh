#!/bin/bash

###########################################
# CONFIG
###########################################

SIM_PATH="$HOME/Formula-Student-Driverless-Simulator-binary"
SCRIPT_DIR="$HOME/IMT-Driverless-Stack/python_stack"
ROS_DISTRO="galactic"

# S'assurer que le dossier temporaire existe
mkdir -p /tmp/imt_driverless_cone_fusion

# SOURCE ROS2
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

###########################################
# 1. LANCEMENT SIMULATEUR ou ROSBAG
###########################################
echo "🎮 Lancement Simulateur / Rosbag..."
gnome-terminal --title="SIMULATEUR / ROSBAG" -- bash -c "
cd $SIM_PATH
# FSDS.sh pour simulateur ou remplacer par 'ros2 bag play ...' si replay
./FSDS.sh -windowed -ResX=640 -ResY=480
exec bash" &
sleep 5

###########################################
# 2. LANCEMENT GLOBAL DRIVE
###########################################
echo "🏎️ Lancement Global Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
cd $SCRIPT_DIR
python3 global_drive.py
exec bash" &
sleep 2

###########################################
# 3. LANCEMENT LIDAR ROS
###########################################
echo "🟢 Lancement lidar_ros.py (filtre sol)..."
gnome-terminal --title="LIDAR ROS" -- bash -c "
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
cd $SCRIPT_DIR
python3 lidar_ros.py
exec bash" &
sleep 1

###########################################
# 4. LANCEMENT LIDAR CLUSTER
###########################################
echo "🔵 Lancement lidar_cluster.py (nuage -> objets locaux)..."
gnome-terminal --title="LIDAR CLUSTER" -- bash -c "
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
cd $SCRIPT_DIR
python3 lidar_cluster.py
exec bash" &
sleep 1

###########################################
# 5. LANCEMENT CONE FUSION
###########################################
echo "🔴 Lancement cone_fusion.py (carte globale)..."
gnome-terminal --title="CONE FUSION" -- bash -c "
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
cd $SCRIPT_DIR
python3 cone_fusion.py
exec bash" &
sleep 1

###########################################
# 6. LANCEMENT CIRCUIT MAP
###########################################
echo "📍 Lancement circuit_map.py (affichage)..."
gnome-terminal --title="CIRCUIT MAP" -- bash -c "
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
cd $SCRIPT_DIR
python3 circuit_map.py
exec bash" &

echo "✅ Tous les nœuds ont été lancés dans l'ordre demandé."
