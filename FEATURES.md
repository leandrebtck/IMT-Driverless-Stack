# IMT Driverless Stack — Fonctionnalités

> Stack de perception et SLAM temps réel pour la compétition **Formula Student Driverless**.
> Développée à l'IMT, testée sur le simulateur FSDS v2.2.0.
> Compatible **ROS2 Galactic**, **ROS2 Iron**, **ROS2 Humble**.

---

## 1. Détection de cônes par caméra stéréo (`yolo_lidar.py` / `yolo_stereo.py`)

### YOLO stéréo (`yolo_stereo.py`)
- Inférence **YOLOv8** simultanée sur les deux flux caméra (batch gauche + droite)
- Modèle entraîné : `weights/best_FINAL.pt` — 3 classes : **JAUNE (0), BLEU (1), ORANGE (2)**
- **Estimation de profondeur stéréo** : matching des bounding boxes entre cam1 et cam2
  - Contrainte épipolaire : tolérance verticale ±15 px
  - Disparité valide : 8–100 px → profondeur 1.3–16.6 m
  - Formule : `Z = f × B / (cx_gauche − cx_droite)` avec `f=208 px`, `B=0.64 m`
  - Plage fiable retenue : **2–12 m**
- **Fallback monoculaire** (hors plage stéréo) : estimation par taille apparente du cône (`h_cone_réel = 0.325 m`)
- **Correction HSV** : reclassification automatique des cônes mal détectés orange/jaune par analyse couleur
- **Correction de biais** : décalage empirique par classe (`−1.5 m` jaune, `−2.0 m` bleu)
- Publication sur `/perception/stereo_detections` (`vision_msgs/Detection2DArray`)

### YOLO + LiDAR (`yolo_lidar.py`)
- Inférence YOLO sur caméra gauche uniquement
- **Projection LiDAR dans l'image** via `PinholeCameraModel` + TF2
- Association bbox ↔ points LiDAR dans la bounding box YOLO
- **Rejet d'outliers LiDAR** : points dont la distance dépasse `médiane ± 0.5 m`
- **Position 3D** par centroïde pondéré des points LiDAR valides
- **Filtre de Kalman par cône** :
  - État : position 3D `[px, py, pz]` dans le repère LiDAR
  - Modèle : cône statique (`F = I`)
  - Bruit mesure : `R_std = 0.20 m`, bruit processus : `Q_std = 0.02 m`
  - Convergence : ~5 mises à jour → précision ±5 cm
- **Confirmation** : cône publié uniquement après `MIN_HITS = 3` détections consécutives
- **Suppression** : cône supprimé après `MAX_MISSES = 8` frames sans détection
- Publication sur `/perception/lidar_detections` (`vision_msgs/Detection2DArray`)

---

## 2. SLAM — Cartographie globale des cônes

### SLAM Stéréo (`cone_mapper.py`)
- Entrée : `/perception/stereo_detections`
- Repère de travail : `fsds/cam1` → transformé en `fsds/map` via TF2
- Profondeur max : **4 m** (stéréo moins fiable au-delà)
- **Suspension des ajouts en virage** : détection du taux de lacet depuis `/testing_only/odom` (seuil `0.3 rad/s`)
- **Fusion des cônes** : distance de merge `1.5 m` — moyenne pondérée sur `COUNT_CAP = 20` mesures
- **Compteur de confiance** `MIN_COUNT = 3` avant publication
- Tracé des **lignes de bord de piste** (Catmull-Rom) :
  - Cônes jaunes → ligne droite (côté droit)
  - Cônes bleus → ligne gauche (côté gauche)
  - Tri par ordre de détection (ID croissant = ordre de passage)
  - Stride `LINE_STRIDE = 3` (1 cône sur 3 comme point de contrôle)
  - Rupture de ligne si écart > `MAX_LINE_STEP = 7 m`
- Publications : `/slam/cone_map` (MarkerArray latché) + `/slam/stats` (String)

### SLAM LiDAR (`cone_mapper_lidar.py`)
- Entrée : `/perception/lidar_detections`
- Repère de travail : `fsds/Lidar1` → `fsds/map`
- Profondeur max : **8 m** ligne droite, **5 m** en virage (LiDAR plus fiable)
- Mêmes mécanismes de fusion et de suspension virage que le SLAM stéréo
- Stride `LINE_STRIDE = 1` (tous les cônes connectés — LiDAR plus précis)
- Pas de lignes orange (trop peu de cônes orange pour être stables)
- Publications : `/slam_lidar/cone_map` + `/slam_lidar/stats`

---

## 3. Gestion du repère TF (`odom_tf_publisher.py`)

- Souscrit à `/testing_only/odom` (`nav_msgs/Odometry`)
- **Publie le TF dynamique** `fsds/map → fsds/FSCar` à chaque mesure d'odométrie
- Sans ce nœud, `fsds/map` n'existe pas dans l'arbre TF → les SLAM ne peuvent pas fonctionner
- Publie également le **tracé du chemin parcouru** sur `/slam/car_path` (`nav_msgs/Path`) pour RViz

---

## 4. Évaluation de la précision (`cone_evaluator.py`)

- Souscrit à `/testing_only/track` (`fs_msgs/Track`) pour la **vérité terrain** des cônes
- Souscrit à `/slam_lidar/cone_map` (ou tout topic configurable via `--map`) pour la carte SLAM
- **Association la plus proche** : chaque cône détecté est apparié au GT le plus proche (seuil 3 m)
- **Métriques calculées** :
  - Erreur moyenne (m)
  - RMSE (m)
  - Erreur min/max (m)
  - Nombre de fantômes (cônes détectés sans correspondance GT)
- Publication `/evaluation/stats` (String) — lisible par `eval_dashboard.py`
- Publication `/evaluation/markers` (MarkerArray) — flèches d'erreur et labels dans RViz

---

## 5. Dashboard d'évaluation (`eval_dashboard.py`)

- Interface **tkinter** dark theme, lancée indépendamment
- **Jauge circulaire** de précision : 0 % (erreur ≥ 1.5 m) → 100 % (erreur = 0 m)
  - Vert > 70 %, Jaune 40–70 %, Rouge < 40 %
- Métriques en temps réel : erreur moy, RMSE, min, max, cônes détectés, fantômes
- **Historique sparkline** sur 120 points
- ROS2 dans un thread daemon (non bloquant pour l'UI)

---

## 6. Monitoring des ressources (`system_monitor.py`)

- Interface **tkinter** dark theme — monitore uniquement les processus de **perception embarquée** (FSDS exclu)
- **3 catégories surveillées** :
  1. Scripts perception Python : `cone_mapper_lidar`, `yolo_lidar`, `yolo_stereo`, `cone_mapper`, `odom_tf_publisher`, `cone_evaluator`
  2. Middleware ROS2 : `ros2`, `fastrtps`, `fastdds`, `cyclonedds`
  3. Drivers capteurs : `velodyne`, `ouster`, `realsense`, `usb_cam`, `v4l2`, `zed`
- **Métriques affichées** (valeurs absolues, pas de %) :
  - RAM en **Go** (RSS par processus)
  - CPU en **cœurs équivalents** (sum cpu_percent / 100)
  - VRAM GPU en **Go** via `nvidia-smi --query-compute-apps`
- **Tableau de répartition** : ligne par catégorie + ligne **TOTAL embarqué**
- **Max historique** affiché en jaune pour RAM, CPU, VRAM
- **Sparklines** de tendance sur 60 secondes
- Détail par processus : nom, catégorie, PID, RAM, CPU, VRAM, statut
- Intégré dans `launcher.py` via un panneau latéral (résumé condensé)

---

## 7. Contrôle manuel (`global_drive.py`)

- Publication sur `/control_command` (`fs_msgs/ControlCommand`)
- Contrôles clavier via **pynput** (non bloquant, thread séparé) :
  - `Z` — Gaz (throttle)
  - `S` — Frein
  - `Q` — Gauche (steering)
  - `D` — Droite
  - `R` — Reset (appel service `/reset`)
- Fréquence de publication : **10 Hz**

---

## 8. Launcher GUI (`launcher.py`)

- Interface **tkinter** centralisée pour lancer les scripts du projet
- Liste automatique de tous les fichiers `.sh` du repo
- Double-clic pour lancer un script dans un terminal
- **Panneau de monitoring** intégré (résumé RAM/CPU/VRAM en temps réel)
- Bouton "Ouvrir dashboard complet" → lance `system_monitor.py`

---

## 9. Scripts de lancement

| Script | Description |
|---|---|
| `slam_stereo.sh` | Lancement complet : FSDS + Bridge + odom_tf + YOLO stéréo + SLAM stéréo + RViz + Drive |
| `slam_lidar.sh` | Lancement complet : FSDS + Bridge + odom_tf + YOLO LiDAR + SLAM LiDAR + RViz + Drive |
| `auto_launch.sh` | YOLO caméra seule (sans SLAM) |
| `auto_stereo.sh` | YOLO stéréo sans SLAM |
| `auto_yolo_lidar.sh` | YOLO + LiDAR sans SLAM |
| `auto_eval.sh` | Lancement évaluateur + dashboard |

---

## 10. Installation et mise à jour

- **`SETUP_IMT_V1.sh`** : installation complète depuis zéro (FSDS, ROS2 Galactic, bridge, dépendances Python)
- **`setup_update.sh`** : mise à jour intelligente — détecte Ubuntu (20.04/22.04) et choisit automatiquement ROS2 Galactic ou Iron, idempotent (ne réinstalle pas ce qui existe déjà)

---

## Modèle YOLO

- Fichier : `python_stack/weights/best_FINAL.pt`
- Entraîné sur images FSDS
- 3 classes : `0 = JAUNE`, `1 = BLEU`, `2 = ORANGE`
- Utilisé par `yolo_stereo.py` et `yolo_lidar.py`
- Chargé sur GPU automatiquement si CUDA disponible
