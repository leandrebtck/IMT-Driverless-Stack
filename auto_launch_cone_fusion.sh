#!/bin/bash

###########################################
# CONFIGURATION
###########################################

# Adapte ce chemin si ton dossier s'appelle différemment
REPO_DIR="$HOME/IMT-Driverless-Stack" 
# Je suppose que tes scripts sont à la racine ou dans un sous-dossier specifique.
# Si tes fichiers sont à la racine du repo, laisse juste $REPO_DIR
PYTHON_STACK="$REPO_DIR" 

ROS_DISTRO="galactic" # ou "humble" ou "foxy" selon ta version

###########################################
# FONCTIONS UTILITAIRES
###########################################

# Fonction pour ouvrir un onglet/fenêtre proprement
launch_node() {
    local title=$1
    local script_name=$2
    
    echo "🚀 Lancement de $title..."
    gnome-terminal --tab --title="$title" -- bash -c "
        source /opt/ros/$ROS_DISTRO/setup.bash;
        [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash;
        cd $PYTHON_STACK;
        echo '-----------------------------------';
        echo '🟢 PROCESS: $title';
        echo '📂 SCRIPT : $script_name';
        echo '-----------------------------------';
        python3 $script_name;
        echo '❌ Processus terminé.';
        exec bash
    "
    sleep 1 # Petite pause pour laisser le temps au noeud de s'initialiser
}

###########################################
# VÉRIFICATION DES DÉPENDANCES
###########################################

command -v python3 >/dev/null 2>&1 || { echo "❌ python3 non trouvé"; exit 1; }
command -v ros2 >/dev/null 2>&1 || { echo "❌ ros2 non trouvé"; exit 1; }

if [ ! -d "$PYTHON_STACK" ]; then
    echo "❌ Le dossier $PYTHON_STACK n'existe pas ! Vérifie la variable REPO_DIR."
    exit 1
fi

###########################################
# LANCEMENT DE LA STACK (ORDRE LOGIQUE)
###########################################

echo "=========================================="
echo "🏎️  IMT DRIVERLESS - DÉMARRAGE SYSTÈME"
echo "=========================================="

# 1. PILOTAGE (Pour bouger la voiture)
launch_node "1. Global Pilot (Clavier)" "global_drive.py"

# 2. PERCEPTION LIDAR (La chaîne de traitement)
# D'abord on filtre le sol
launch_node "2. Lidar Filter (Sol)" "lidar_ros.py"

# Ensuite on clusterise (transforme points en objets)
launch_node "3. Lidar Cluster (DBSCAN)" "lidar_cluster.py"

# 3. MAPPING & SLAM
# On fusionne les objets dans la carte globale
launch_node "4. Cone Fusion (Mapping)" "cone_fusion.py"

# 4. VISUALISATION
# On affiche la carte
launch_node "5. Circuit Map (Visu)" "circuit_map.py"

# 5. PERCEPTION CAMERA (Optionnel / Indépendant pour l'instant)
# Vérification spécifique pour YOLO
echo "🚀 Lancement de 6. YOLO Perception..."
gnome-terminal --tab --title="6. YOLO Perception" -- bash -c "
    source /opt/ros/$ROS_DISTRO/setup.bash;
    cd $PYTHON_STACK;
    if [ ! -d weights ]; then
        echo '⚠️  ATTENTION : Dossier weights introuvable !';
        echo 'Le script yolo_ros.py risque de planter.';
    fi
    python3 yolo_ros.py;
    exec bash
"

echo "✅ Tous les nœuds ont été lancés dans des onglets séparés."
echo "💡 Astuce : Si rien ne s'affiche, vérifie que le simulateur FSDS est bien lancé."
