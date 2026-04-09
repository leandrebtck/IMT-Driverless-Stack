# IMT Driverless Stack

**Stack de perception et SLAM temps réel pour la compétition Formula Student Driverless.**

Développée à l'IMT, testée sur le simulateur FSDS v2.2.0.
Compatible **ROS2 Galactic**, **ROS2 Iron** et **ROS2 Humble**.

---

## Démarrage rapide

### Installation depuis zéro

```bash
sudo apt update && sudo apt install -y curl
bash <(curl -s https://raw.githubusercontent.com/leandrebtck/IMT-Driverless-Stack/main/setup_update.sh)
```

### Si déjà cloné

```bash
cd ~/IMT-Driverless-Stack && git pull origin main
```

### Lancer le launcher graphique

```bash
python3 ~/IMT-Driverless-Stack/launcher.py
```

---

## Pipelines disponibles

| Pipeline | Script | Description |
|---|---|---|
| **SLAM LiDAR** *(recommandé)* | `slam_lidar.sh` | YOLO + LiDAR + Kalman + carte globale |
| **SLAM Stéréo** | `slam_stereo.sh` | YOLO + vision stéréo + carte globale |
| **LiDAR seul** | `auto_lidar.sh` | DBSCAN sans YOLO |
| **YOLO caméra** | `auto_launch.sh` | Détection sans SLAM |

---

## Compatibilité OS / ROS2

| OS | ROS2 |
|---|---|
| Ubuntu 20.04 | Galactic |
| Ubuntu 22.04+ | Iron |

---

## Contrôle clavier (`global_drive.py`)

| Touche | Action |
|---|---|
| `Z` | Avancer (gaz) |
| `S` | Freiner |
| `Q` | Gauche |
| `D` | Droite |
| `R` | Reset position |

---

## Liens utiles

- [Dépôt GitHub](https://github.com/leandrebtck/IMT-Driverless-Stack)
- [Simulateur FSDS](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/)
- [ROS2 Iron](https://docs.ros.org/en/iron/)
