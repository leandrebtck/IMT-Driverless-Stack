#!/bin/bash

###########################################
# CONFIG
###########################################

SIM_DIR="$HOME/Formula-Student-Driverless-Simulator-binary"
SIM_CMD="./FSDS.sh -windowed -ResX=1280 -ResY=720"

# Nom exact de la fenêtre
WINDOW_NAME="Formula Student Driverless Simulator"

# Clic légèrement plus haut (Y + 5)
CLICK_X=289
CLICK_Y=210

###########################################
# TERMINAL 1 — LANCER LE SIMULATEUR
###########################################

echo "Ouverture du SIMULATEUR dans un terminal..."
gnome-terminal -- bash -c "
    cd \"$SIM_DIR\";
    echo 'Lancement FSDS...';
    $SIM_CMD;
    exec bash
" &

###########################################
# DÉTECTION DE LA FENÊTRE
###########################################

echo "⏳ Attente de l'ouverture de la fenêtre du simulateur..."

for i in {1..40}; do
    WIN_ID=$(wmctrl -l | grep -F "$WINDOW_NAME" | awk '{print $1}')
    if [ ! -z "$WIN_ID" ]; then
        echo "✅ Fenêtre détectée : $WIN_ID"
        break
    fi
    echo "   Tentative $i/40..."
    sleep 4
done

if [ -z "$WIN_ID" ]; then
    echo "❌ Impossible de détecter FSDS."
    exit 1
fi

###########################################
# POSITIONNEMENT & CLIC
###########################################

echo "Mise en position 1280x720 en haut à gauche..."
wmctrl -i -r "$WIN_ID" -e 0,0,0,1280,720
sleep 1

echo "Focus sur la fenêtre..."
wmctrl -i -a "$WIN_ID"
sleep 1

echo "Clique sur 'Run Simulation' ($CLICK_X,$CLICK_Y)..."
xdotool mousemove --window "$WIN_ID" $CLICK_X $CLICK_Y click 1


sleep 3  # Important !!

###########################################
# TERMINAL 2 — LANCER LE ROS2 BRIDGE
###########################################

echo "🔌 Lancement du ROS2 bridge dans un nouveau terminal..."
gnome-terminal -- bash -c "
    cd ~/Formula-Student-Driverless-Simulator/ros2;
    source install/setup.bash;
    echo 'Bridge FSDS → ROS2';
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py;
    exec bash
" &

sleep 1

###########################################
# TERMINAL 3 — LISTER LES TOPICS + READY
###########################################

echo "📡 Terminal ROS2 pour tester les topics..."
gnome-terminal -- bash -c "
    cd ~/Formula-Student-Driverless-Simulator/ros2;
    source install/setup.bash;
sleep 1
    echo 'Topics disponibles :';
    ros2 topic list;

    exec bash
" &

echo " FSDS + Bridge + Terminal ROS sont prêts !"
