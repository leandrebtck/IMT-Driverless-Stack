#!/bin/bash

# --- CONFIGURATION ---
SIM_PATH="$HOME/Formula-Student-Driverless-Simulator-binary"


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "🚀 LANCEMENT STACK COMPLETE (CAM + LIDAR)"
echo "📂 Dossier de travail détecté : $SCRIPT_DIR"

# 1. SIMULATEUR
echo "🎮 Lancement Simu..."
gnome-terminal --title="SIMULATEUR" -- bash -c "cd $SIM_PATH; ./FSDS.sh -windowed -ResX=640 -ResY=480; exec bash" &
sleep 5

# 2. ROS2 BRIDGE 
# Le bridge a souvent son propre workspace (dans le dossier du simu)
echo "🔌 Lancement Bridge..."
gnome-terminal --title="BRIDGE ROS2" -- bash -c "source /opt/ros/galactic/setup.bash; cd ~/Formula-Student-Driverless-Simulator/ros2; source install/setup.bash; ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py; exec bash" &
sleep 3

# 3. LIDAR STACK (NOUVEAU)
echo "🟢 Lancement LiDAR (Filtre + Cluster)..."
gnome-terminal --title="LIDAR PROCESSING" -- bash -c "
    source /opt/ros/galactic/setup.bash; 
    # On source aussi le workspace au cas où tu utiliserais des msgs custom plus tard
    source ~/Workspace_ROS2/install/setup.bash; 
    
    echo '---- 1. Filtre Sol ----';
    python3 $SCRIPT_DIR/lidar_ros.py & 
    sleep 1;
    
    echo '---- 2. Clustering DBSCAN ----';
    python3 $SCRIPT_DIR/lidar_cluster.py &
    
    wait" &
sleep 2

# 4. PERCEPTION (YOLO)
echo "👁️ Lancement YOLO..."
gnome-terminal --title="YOLO PERCEPTION" -- bash -c "
    source /opt/ros/galactic/setup.bash; 
    # INDISPENSABLE : On charge ton workspace pour que ROS trouve tes paquets
    source ~/Workspace_ROS2/install/setup.bash;
    python3 $SCRIPT_DIR/yolo_ros.py; 
    exec bash" &
sleep 2

# 5. DRIVE (PILOTE)
echo "🏎️ Lancement Drive..."
gnome-terminal --title="GLOBAL DRIVE" -- bash -c "
    source /opt/ros/galactic/setup.bash; 
    # INDISPENSABLE : C'est ici que se trouve fs_msgs !
    source ~/Workspace_ROS2/install/setup.bash;
    python3 $SCRIPT_DIR/global_drive.py; 
    exec bash" &

echo "✅ TOUS LES SYSTÈMES SONT LANCÉS."