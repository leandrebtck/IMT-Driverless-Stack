# Lancement

## Méthode recommandée — Launcher GUI

```bash
python3 ~/IMT-Driverless-Stack/launcher.py
```

Double-clic sur un script pour le lancer. Le panneau de droite affiche les ressources en temps réel.

---

## Pipelines disponibles

| Script | Pipeline | Commande |
|---|---|---|
| `slam_lidar.sh` | SLAM LiDAR *(recommandé)* | YOLO + LiDAR + SLAM + conduite manuelle |
| `slam_stereo.sh` | SLAM Stéréo | YOLO stéréo + SLAM stéréo + conduite manuelle |
| `auto_centerline_lidar.sh` | Autonome circuit LiDAR | SLAM LiDAR + `centerline_follower.py` |
| `auto_control_stereo.sh` | Autonome circuit stéréo | SLAM stéréo + `centerline_follower.py` |
| `skidpad_lidar.sh` | Skidpad LiDAR | SLAM Skidpad LiDAR + `skidpad_driver.py` |
| `skidpad_stereo.sh` | Skidpad stéréo | SLAM Skidpad stéréo + `skidpad_driver.py` |
| `acceleration_lidar.sh` | Accélération LiDAR | SLAM LiDAR + `acceleration_driver.py` |
| `auto_lidar.sh` | LiDAR seul (sans YOLO) | DBSCAN + SLAM + conduite manuelle |
| `auto_eval.sh` | Évaluation SLAM | `cone_evaluator.py` + `eval_dashboard.py` |

---

## Pipeline SLAM LiDAR *(le plus précis)*

```bash
bash ~/IMT-Driverless-Stack/scripts/slam_lidar.sh
```

Lance dans l'ordre :

1. **FSDS** — simulateur (attend 10 s)
2. **Dialogue** — attendre "Run Simulation" puis confirmer
3. **Bridge ROS2** — `fsds_ros2_bridge` (attend 5 s)
4. **odom_tf_publisher** — crée `fsds/map` dans le TF (attend 2 s)
5. **YOLO LiDAR** — détection + Kalman (attend 3 s)
6. **Cone Mapper LiDAR** — SLAM (attend 2 s)
7. **RViz** — avec config `slam_lidar.rviz`
8. **Global Drive** — contrôle clavier

**Topics utiles :**

- `/slam_lidar/cone_map` — carte globale des cônes (MarkerArray)
- `/slam_lidar/stats` — statistiques texte
- `/slam/car_path` — trajectoire de la voiture

---

## Pipeline Skidpad

```bash
bash ~/IMT-Driverless-Stack/scripts/skidpad_lidar.sh
# ou
bash ~/IMT-Driverless-Stack/scripts/skidpad_stereo.sh
```

Lance le SLAM dédié Skidpad (`cone_mapper_skidpad_lidar.py` ou `_stereo.py`) puis `skidpad_driver.py`.

Le SLAM Skidpad affiche les **lignes jaune↔bleu** (centerline) au lieu des lignes latérales.

**Paramètres du driver :**

```bash
python3 python_scripts/3control/skidpad_driver.py --map /slam_skidpad_lidar/cone_map --straight 13.0
```

| Argument | Défaut | Description |
|---|---|---|
| `--map` | `/slam_skidpad_stereo/cone_map` | Topic SLAM pour détection cônes orange sortie |
| `--straight` | `13.0` | Distance ligne droite avant d'entrer dans les cercles (m) |

---

## Pipeline Accélération

```bash
bash ~/IMT-Driverless-Stack/scripts/acceleration_lidar.sh
```

Lance SLAM LiDAR + `acceleration_driver.py` (75 m plein gaz, freinage à 65 m).

**Paramètres du driver :**

```bash
python3 python_scripts/3control/acceleration_driver.py --dist 75.0 --brake 10.0 --map /slam_lidar/cone_map
```

| Argument | Défaut | Description |
|---|---|---|
| `--dist` | `75.0` | Distance totale épreuve (m) |
| `--brake` | `10.0` | Marge de freinage avant la fin (m) |
| `--map` | `/slam_lidar/cone_map` | Topic SLAM pour détection cônes orange arrivée |

---

## Lancer les composants individuellement

```bash
# ── perception/ ──────────────────────────────────────
python3 python_scripts/1perception/odom_tf_publisher.py     # TF (indispensable en premier)
python3 python_scripts/1perception/yolo_lidar.py            # YOLO + LiDAR + Kalman
python3 python_scripts/1perception/yolo_stereo.py           # YOLO + stéréo

# ── slam/ ─────────────────────────────────────────────
python3 python_scripts/2slam/cone_mapper_lidar.py           # SLAM LiDAR
python3 python_scripts/2slam/cone_mapper.py                 # SLAM stéréo
python3 python_scripts/2slam/cone_mapper_skidpad_lidar.py   # SLAM Skidpad LiDAR
python3 python_scripts/2slam/cone_mapper_skidpad_stereo.py  # SLAM Skidpad stéréo

# ── control/ ──────────────────────────────────────────
python3 python_scripts/3control/centerline_follower.py --map /slam_lidar/cone_map
python3 python_scripts/3control/skidpad_driver.py --map /slam_skidpad_lidar/cone_map
python3 python_scripts/3control/acceleration_driver.py
python3 python_scripts/3control/global_drive.py             # conduite manuelle
python3 python_scripts/3control/cone_evaluator.py           # évaluateur (nécessite SLAM actif)
python3 python_scripts/performance/eval_dashboard.py           # dashboard précision
python3 python_scripts/performance/system_monitor.py           # monitoring ressources
```

!!! warning "Sourcer l'environnement ROS2 avant chaque commande"
    ```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    source ~/IMT-Driverless-Stack/ros_workspace/install/setup.bash
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

### Configuration Skidpad

| Paramètre | Valeur |
|---|---|
| Fixed Frame | `fsds/map` |
| Topic cônes | `/slam_skidpad_lidar/cone_map` → MarkerArray |
| Config RViz | `config/rviz/skidpad_lidar.rviz` |

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
