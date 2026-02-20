# SETUP_IMT_V1  
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
# Lancement automatique : auto_launch_FSDS.sh

Ce dépôt inclut un exécutable permettant de tout lancer automatiquement :

- Lancer FSDS en 1280×720
- Attendre son ouverture
- Cliquer sur Run Simulation
- Lancer le ROS2 bridge
- Lancer un terminal ROS2 prêt à l’emploi et lister les topics disponibles

## Lancer automatiquement FSDS + ROS2 (en 1 seule commande)

Sans télécharger manuellement le script, lancez :
```bash
bash <(curl -s https://raw.githubusercontent.com/leandrebtck/IMT-Driverless-Stack/main/auto_launch_FSDS.sh)
```
meme commande en local (après telechargement des scripts):
```bash
cd ~/IMT-Driverless-Stack && bash auto_launch_FSDS.sh
```
## Lancer automatiquement FSDS + ROS2 + YOLO (uniquement)

```bash
bash -c "[ -d ~/IMT-Driverless-Stack ] && (cd ~/IMT-Driverless-Stack && git pull) || git clone https://github.com/leandrebtck/IMT-Driverless-Stack.git ~/IMT-Driverless-Stack; chmod +x ~/IMT-Driverless-Stack/python_stack/auto_launch.sh; ~/IMT-Driverless-Stack/python_stack/auto_launch.sh"
```
meme commande en local:
```bash
cd ~/IMT-Driverless-Stack && chmod +x python_stack/auto_launch.sh && ./python_stack/auto_launch.sh
```

## Lancer automatiquement FDSD + ROS2 + CAMERA(YOLO) + LIDAR (1 commande)

```bash
bash -c "[ -d ~/IMT-Driverless-Stack ] && (cd ~/IMT-Driverless-Stack && git pull) || git clone https://github.com/leandrebtck/IMT-Driverless-Stack.git ~/IMT-Driverless-Stack; chmod +x ~/IMT-Driverless-Stack/python_stack/auto_launch_yolo_lidar.sh; ~/IMT-Driverless-Stack/python_stack/auto_launch_yolo_lidar.sh"
```
Pour plus de visibilité :

```bash
source /opt/ros/galactic/setup.bash
rviz2
```
Puis : 
 -Fixed Frame (en haut a gauche), cliquez dessus et selectionnez à droite fsds/Lidar1
 -Add (en bas à gauche) ---> By topic ---> lidar -> cone_markers -> MarkerArray -> OK
 -Puis appuyez sur la flèche à côté de MarkerArray et vérifiez bien que le topic est : /lidar/cone_markers
 
## Lancer automatiquement tout + sensor fusion + rviz : 

```bash
bash -c "[ -d ~/IMT-Driverless-Stack ] && (cd ~/IMT-Driverless-Stack && git pull) || git clone https://github.com/leandrebtck/IMT-Driverless-Stack.git ~/IMT-Driverless-Stack; chmod +x ~/IMT-Driverless-Stack/python_stack/auto_launch_sensor.sh; ~/IMT-Driverless-Stack/python_stack/auto_launch_sensor.sh"
```
à noter qu'à partir de environ 4/5 m/s , la fusion devient très peu précise
De plus, il est nécessaire de passer yolo_ros en mode headless pour + de performance
 
