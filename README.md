<p align="center">
  <img src="docs/assets/logo_imtensity_nobg.png" alt="IMTensity Logo" width="600"/>
</p>

# Documentation complète

📖 **[https://leandrebtck.github.io/IMT-Driverless-Stack/](https://leandrebtck.github.io/IMT-Driverless-Stack/)**

Architecture, topics ROS2, paramètres, guide de déploiement sur véhicule réel.

## SETUP_IMT_V1
Installateur automatique pour Formula Student Driverless Simulator + ROS 2 Galactic + ROS-Bridge / dépendances requises  

Ce script bash configure **automatiquement** un environnement complet sur Ubuntu 20.04 :  
- Téléchargement et installation du simulateur FSDS v2.2.0 (binaires)  
- Installation de ROS 2 Galactic Desktop  
- Installation des dépendances nécessaires (CMake récent, paquets système, librairies ROS & AirSim)  
- Clonage du dépôt FSDS officiel & compilation du ROS2-bridge  
- Configuration automatique de l’environnement (`.bashrc`)  

---

## Objectif: 

Faciliter la mise en place d’un environnement complet FSDS + ROS2 pour n’importe quel utilisateur, sans configuration manuelle fastidieuse.  
L’utilisateur peut lancer le simulateur + le ROS bridge en quelques commandes seulement.  

---

## Première Installation  

Depuis un terminal, lancez :  

```bash
sudo apt update && sudo apt install -y curl
bash <(curl -s https://raw.githubusercontent.com/leandrebtck/IMT-Driverless-Stack/main/SETUP_IMT_V1.sh)
```
### Pour mettre à jour et vérifier l'installation *_*
```bash
sudo apt update && sudo apt install -y curl
bash <(curl -s https://raw.githubusercontent.com/leandrebtck/IMT-Driverless-Stack/main/setup_update.sh)
```
## Launcher graphique (recommandé)

Lance l’interface graphique qui liste tous les scripts disponibles et permet de les exécuter en un clic :

```bash
python3 ~/IMT-Driverless-Stack/launcher.py
```

---

## Scripts de lancement

### `auto_launch_FSDS.sh` — Simulateur seul
Lance FSDS, attend son ouverture, clique automatiquement sur *Run Simulation*, puis ouvre le ROS2 bridge.
```bash
cd ~/IMT-Driverless-Stack && bash auto_launch_FSDS.sh
```

### `auto_launch.sh` — FSDS + YOLO (caméra seule)
```bash
cd ~/IMT-Driverless-Stack && bash python_stack/auto_launch.sh
```

### `auto_stereo.sh` — FSDS + YOLO stéréo + profondeur
```bash
cd ~/IMT-Driverless-Stack && bash python_stack/auto_stereo.sh
```

### `auto_yolo_lidar.sh` — FSDS + YOLO + LiDAR *(recommandé)*
Détection des cônes (couleur + distance mesurée par LiDAR). Publie :
- `/yolo_lidar/debug_image` — image annotée (bbox + couleur + distance)
- `/yolo_lidar/cone_markers` — markers 3D colorés pour RViz
```bash
cd ~/IMT-Driverless-Stack && bash python_stack/auto_yolo_lidar.sh
```

### `auto_launch_yolo_lidar.sh` — FSDS + YOLO + LiDAR + Fusion + RViz
```bash
cd ~/IMT-Driverless-Stack && bash python_stack/auto_launch_yolo_lidar.sh
```
Pour visualiser dans RViz : *Fixed Frame* → `fsds/Lidar1`, puis *Add → By topic → /lidar/cone_markers → MarkerArray*

### `auto_launch_sensor.sh` — Stack complète + sensor fusion + RViz
```bash
cd ~/IMT-Driverless-Stack && bash python_stack/auto_launch_sensor.sh
```
> Note : la fusion devient imprécise au-delà de ~4-5 m/s.

---

## Contrôle clavier

Tous les scripts lancent `global_drive.py` (terminal **GLOBAL DRIVE**) :

| Touche | Action  |
|--------|---------|
| `Z`    | Avancer |
| `S`    | Reculer |
| `Q`    | Gauche  |
| `D`    | Droite  |

---

## Compatibilité

| OS            | ROS          |
|---------------|--------------|
| Ubuntu 20.04  | ROS Galactic |
| Ubuntu 22.04+ | ROS Iron     |

---

## Git pull
```bash
git clone https://github.com/leandrebtck/IMT-Driverless-Stack.git && cd IMT-Driverless-Stack
```
Si déjà cloné : 
```bash
cd ~/IMT-Driverless-Stack && git pull origin main
```
