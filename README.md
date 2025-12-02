# SETUP_IMT_V1  
Installateur automatique pour Formula Student Driverless Simulator + ROS 2 Galactic + ROS-Bridge / dépendances requises  

Ce script bash configure **automatiquement** un environnement complet sur Ubuntu 20.04 :  
- Téléchargement et installation du simulateur FSDS v2.2.0 (binaires)  
- Installation de ROS 2 Galactic Desktop  
- Installation des dépendances nécessaires (CMake récent, paquets système, librairies ROS & AirSim)  
- Clonage du dépôt FSDS officiel & compilation du ROS2-bridge  
- Configuration automatique de l’environnement (`.bashrc`)  

---

## Objectif  

Faciliter la mise en place d’un environnement complet FSDS + ROS2 pour n’importe quel utilisateur, sans configuration manuelle fastidieuse.  
L’utilisateur peut lancer le simulateur + le ROS bridge en quelques commandes seulement.  

---

## Installation  

Depuis un terminal, lancez :  

```bash
bash <(curl -s https://raw.githubusercontent.com/leandrebtck/SETUP_IMT_V1/main/SETUP_IMT_V1.sh)
