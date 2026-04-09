# Installation

## Installation depuis zéro

Script automatique qui installe tout en une commande :

```bash
sudo apt update && sudo apt install -y curl
bash <(curl -s https://raw.githubusercontent.com/leandrebtck/IMT-Driverless-Stack/main/setup_update.sh)
```

Ce script installe automatiquement :

- [x] Simulateur FSDS v2.2.0 (binaire)
- [x] ROS2 (Galactic sur Ubuntu 20.04, Iron sur Ubuntu 22.04+)
- [x] Bridge ROS2 FSDS (compilé depuis les sources)
- [x] AirLib (dépendance bridge)
- [x] Toutes les dépendances Python
- [x] Outils système (`xdotool`, `wmctrl`, `zenity`)

---

## Mise à jour

Si le repo est déjà installé :

```bash
cd ~/IMT-Driverless-Stack && git pull origin main
bash setup_update.sh
```

---

## Installation manuelle des dépendances Python

```bash
python3 -m pip install "numpy<2.0" ultralytics opencv-python pynput \
    pyqtgraph PyQt5 scikit-learn torch torchvision psutil

# GPU NVIDIA uniquement (optionnel)
python3 -m pip install pynvml || echo "pynvml optionnel"
```

---

## Paquets ROS2 requis

=== "ROS2 Iron (Ubuntu 22.04)"
    ```bash
    sudo apt install -y \
        ros-iron-cv-bridge \
        ros-iron-image-transport \
        ros-iron-tf2-geometry-msgs \
        ros-iron-joy \
        ros-iron-sensor-msgs-py \
        ros-iron-vision-msgs \
        ros-iron-image-geometry \
        ros-iron-message-filters \
        ros-iron-tf2-ros \
        ros-iron-nav-msgs \
        ros-iron-visualization-msgs
    ```

=== "ROS2 Galactic (Ubuntu 20.04)"
    ```bash
    sudo apt install -y \
        ros-galactic-cv-bridge \
        ros-galactic-image-transport \
        ros-galactic-tf2-geometry-msgs \
        ros-galactic-joy \
        ros-galactic-sensor-msgs-py \
        ros-galactic-vision-msgs \
        ros-galactic-image-geometry \
        ros-galactic-message-filters \
        ros-galactic-tf2-ros \
        ros-galactic-nav-msgs \
        ros-galactic-visualization-msgs
    ```

---

## Compilation du workspace ROS2

```bash
cd ~/IMT-Driverless-Stack/ros_workspace
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

---

## Vérification de l'installation

```bash
# Vérifier ROS2
ros2 --version

# Vérifier YOLO
python3 -c "from ultralytics import YOLO; print('YOLO OK')"

# Vérifier le modèle
ls ~/IMT-Driverless-Stack/python_stack/weights/best_FINAL.pt

# Vérifier le simulateur
ls ~/Formula-Student-Driverless-Simulator-binary/FSDS.sh
```

---

## Structure des dossiers après installation

```
~/
├── Formula-Student-Driverless-Simulator-binary/   ← binaire simulateur
│   └── FSDS.sh
├── Formula-Student-Driverless-Simulator/          ← sources + bridge ROS2
│   ├── ros2/install/setup.bash
│   └── python/                                    ← client Python FSDS
└── IMT-Driverless-Stack/                          ← ce repo
    ├── python_stack/
    │   └── weights/best_FINAL.pt                  ← modèle YOLO
    └── ros_workspace/                             ← fs_msgs compilé
```

---

## Sourcing de l'environnement

Ajouter au `~/.bashrc` (fait automatiquement par le script d'installation) :

```bash
source /opt/ros/<distro>/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
source ~/IMT-Driverless-Stack/ros_workspace/install/setup.bash
```
