# Lancement

## Méthode recommandée — Launcher GUI

```bash
python3 ~/IMT-Driverless-Stack/launcher.py
```

Double-clic sur un script pour le lancer. Le panneau de droite affiche les ressources en temps réel.

---

## Pipeline SLAM LiDAR *(le plus précis)*

```bash
cd ~/IMT-Driverless-Stack/python_stack
bash slam_lidar.sh
```

Lance dans l'ordre :

1. **FSDS** — simulateur (attend 10 s)
2. **Dialogue** — attendre "Run Simulation" puis confirmer
3. **Bridge ROS2** — `fsds_ros2_bridge` (attend 5 s)
4. **odom_tf_publisher** — crée `fsds/map` dans le TF (attend 2 s)
5. **YOLO LiDAR** — détection + Kalman (attend 3 s)
6. **Cone Mapper LiDAR** — SLAM (attend 2 s)
7. **RViz** — avec config `slam_lidar.rviz` si présente
8. **Global Drive** — contrôle clavier

**Topics utiles :**

- `/slam_lidar/cone_map` — carte globale des cônes (MarkerArray)
- `/slam_lidar/stats` — statistiques texte
- `/slam/car_path` — trajectoire de la voiture

---

## Pipeline SLAM Stéréo

```bash
cd ~/IMT-Driverless-Stack/python_stack
bash slam_stereo.sh
```

Même séquence que le SLAM LiDAR, avec `yolo_stereo.py` et `cone_mapper.py` à la place.

---

## Pipeline LiDAR seul (sans YOLO)

```bash
cd ~/IMT-Driverless-Stack/python_stack
bash auto_lidar.sh
```

Lance :

1. FSDS + Bridge + odom_tf
2. `lidar_ros.py` — filtre sol (garde les points `z > −0.40 m`)
3. `lidar_cluster.py` — clustering DBSCAN
4. `cone_mapper_lidar.py` — carte persistante
5. Global Drive + RViz

**Dans RViz :** Fixed Frame → `fsds/Lidar1`

---

## Évaluateur de précision

```bash
cd ~/IMT-Driverless-Stack/python_stack
bash auto_eval.sh
```

Lance `cone_evaluator.py` + `eval_dashboard.py` (jauge de précision graphique).

Nécessite que le SLAM LiDAR soit déjà lancé (il lit `/slam_lidar/cone_map`).

---

## Lancer les composants individuellement

```bash
cd ~/IMT-Driverless-Stack/python_stack

# TF (indispensable en premier)
python3 odom_tf_publisher.py

# Détection
python3 yolo_lidar.py        # YOLO + LiDAR + Kalman
python3 yolo_stereo.py       # YOLO + stéréo

# SLAM
python3 cone_mapper_lidar.py # SLAM LiDAR
python3 cone_mapper.py       # SLAM stéréo

# Évaluation
python3 cone_evaluator.py --map /slam_lidar/cone_map
python3 eval_dashboard.py

# Monitoring
python3 system_monitor.py

# Conduite
python3 global_drive.py
```

---

## Visualisation dans RViz

### Configuration SLAM LiDAR

| Paramètre | Valeur |
|---|---|
| Fixed Frame | `fsds/map` |
| Topic cônes | `/slam_lidar/cone_map` → MarkerArray |
| Topic trajectoire | `/slam/car_path` → Path |
| Topic flèches GT | `/evaluation/markers` → MarkerArray |
| Topic image | `/yolo_lidar/debug_image` → Image |

### Configuration LiDAR seul

| Paramètre | Valeur |
|---|---|
| Fixed Frame | `fsds/Lidar1` |
| Topic clusters | `/lidar/cone_markers` → MarkerArray |
| Topic carte | `/slam_lidar/cone_map` → MarkerArray |

---

## Commandes utiles

```bash
# Lister tous les topics actifs
ros2 topic list

# Voir les stats SLAM en temps réel
ros2 topic echo /slam_lidar/stats

# Voir les stats d'évaluation
ros2 topic echo /evaluation/stats

# Voir l'arbre TF
ros2 run tf2_tools view_frames

# Vérifier la fréquence d'un topic
ros2 topic hz /perception/lidar_detections
```
