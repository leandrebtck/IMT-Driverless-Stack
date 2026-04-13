# Pipelines détaillés

## Pipeline YOLO + LiDAR (recommandé)

### Étape 1 — Synchronisation temporelle

`yolo_lidar.py` synchronise les deux capteurs avec `ApproximateTimeSynchronizer` :

```python
ts = message_filters.ApproximateTimeSynchronizer(
    [sub_image, sub_lidar], queue_size=10, slop=0.1
)
ts.registerCallback(self._callback)
```

- Caméra : `/fsds/cam1/image_color`
- LiDAR : `/lidar/Lidar1`
- Tolérance de synchronisation : **0.1 s**

### Étape 2 — Détection YOLO

```
image (640×480) → YOLO (GPU) → [(bbox, class_id, score), ...]
```

Modèle chargé une seule fois au démarrage sur `cuda` si disponible, sinon `cpu`.

### Étape 3 — Projection LiDAR dans l'image

Pour chaque point `(X, Y, Z)` dans `fsds/Lidar1` :

1. Transformation vers `fsds/cam1` via TF2
2. Projection pixel via `PinholeCameraModel.project3dToPixel()`
3. Filtre : conserver uniquement les points tombant dans une bounding box YOLO avec `z > 0`

### Étape 4 — Rejet des outliers

```python
dist = sqrt(x² + y²)          # distance horizontale
median = np.median(distances)
keep = |dist - median| < 0.5  # OUTLIER_STD = 0.5 m
position = centroïde_pondéré(points_valides)
```

### Étape 5 — Filtre de Kalman par cône

```python
# Association mesure → tracker existant
tracker = find_nearest(x, y, z, max_dist=1.5)  # ASSOC_DIST

if tracker:
    tracker.kalman.update(x, y, z)   # mise à jour Kalman
    tracker.hits += 1
else:
    create_new_tracker(x, y, z)       # nouveau cône

# Publication si suffisamment confirmé
if tracker.hits >= 3:  # MIN_HITS
    publish(tracker)
```

### Étape 6 — Publication

```
/perception/lidar_detections  →  python_scripts/2slam/cone_mapper_lidar.py
/yolo_lidar/cone_markers      →  RViz (visualisation)
/yolo_lidar/debug_image       →  RViz (image annotée)
```

---

## Pipeline SLAM LiDAR

### Étape 1 — Filtrage par profondeur

```python
depth = sqrt(x² + y²)   # distance horizontale dans fsds/Lidar1

if abs(yaw_rate) < 0.3:   # ligne droite
    max_depth = 8.0
else:                      # virage
    max_depth = 5.0

if depth > max_depth:
    continue  # ignoré
```

### Étape 2 — Transformation dans `fsds/map`

```python
pt_lidar = PointStamped(frame='fsds/Lidar1', x=x, y=y, z=z)
pt_map   = tf_buffer.transform(pt_lidar, 'fsds/map')
```

### Étape 3 — Fusion des cônes

```python
# Chercher un cône existant à moins de MERGE_DIST = 1.5 m
cone = find_nearest_cone(x_map, y_map, max_dist=1.5)

if cone:
    # Moyenne pondérée sur COUNT_CAP = 20 dernières mesures
    n = min(cone.count, 20)
    cone.x = (cone.x * n + x_new) / (n + 1)
    cone.count += 1
else:
    create_cone(x_map, y_map, count=1)

# Publication si count >= MIN_COUNT = 3
```

### Étape 4 — Détection virage (anti-pollution)

```python
# Depuis /testing_only/odom
yaw_rate = odom.twist.twist.angular.z

if abs(yaw_rate) >= 0.3:   # TURN_YAW_RATE
    return  # aucun ajout en virage
```

!!! info "Pourquoi suspendre en virage ?"
    En virage, la voiture tourne rapidement. Si l'odométrie est en léger retard
    par rapport aux capteurs, la transformation TF peut décaler les positions
    des cônes de plusieurs dizaines de centimètres, polluant la carte.

### Étape 5 — Tracé des lignes Catmull-Rom

```python
# Tri par ordre de détection (ID croissant = ordre de passage)
yellow = sorted(cones_jaunes, key=lambda c: c.id)
blue   = sorted(cones_bleus,  key=lambda c: c.id)

# Segmentation si gap > MAX_LINE_STEP = 8 m
segments = split_at_gaps(yellow, max_gap=8.0)

# Spline Catmull-Rom sur chaque segment
for seg in segments:
    points = catmull_rom(seg, stride=LINE_STRIDE)
    publish_line(points)
```

La spline Catmull-Rom passe exactement par les points de contrôle et produit
des courbes lisses sans artefacts aux extrémités grâce aux points fantômes.

### Étape 6 — Publication (2 Hz, QoS latché)

```
/slam_lidar/cone_map  →  RViz + cone_evaluator.py
/slam_lidar/stats     →  logs, monitoring
```

---

## Pipeline YOLO Stéréo

### Estimation de profondeur

```python
# Matching des bounding boxes gauche / droite
for cone_g in detections_gauche:
    for cone_d in detections_droite:
        # Contrainte épipolaire
        if abs(cone_g.cy - cone_d.cy) > 15:    # CY_TOLERANCE_PX
            continue
        # Contrainte géométrique
        disp = cone_g.cx - cone_d.cx
        if not (8 < disp < 100):                 # DISP_MIN / DISP_MAX
            continue
        # Profondeur stéréo
        Z = 208.0 * 0.64 / disp                  # f * B / disparité

# Fallback monoculaire si hors plage stéréo [2, 12] m
Z_mono = 208.0 * 0.325 / h_bbox_pixels           # f * H_réel / h_pixels
```

### Correction HSV

```python
roi = image[y1:y2, x1:x2]
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
# H ∈ [10, 25] = orange
orange_ratio = (10 < hsv[:,:,0]) & (hsv[:,:,0] < 25)
if orange_ratio.mean() > seuil:
    class_id = 2  # ORANGE
```

---

## Schéma temporel des communications

```
t=0       Bridge publie cam1 (30 Hz), LiDAR (10 Hz), odom (50 Hz)

t=0       odom_tf_publisher → TF fsds/map→fsds/FSCar (50 Hz)

t=0.033   yolo_lidar reçoit image+LiDAR synchronisés
           YOLO inference (~50 ms GPU)
           projection + Kalman + publication lidar_detections (~10 Hz)

t=0.083   cone_mapper_lidar reçoit lidar_detections
           transform TF → fsds/map
           merge cône
           publication slam_lidar/cone_map (toutes les 0.5 s)

           cone_evaluator reçoit slam_lidar/cone_map
           comparaison GT → publication evaluation/stats (10 Hz)

           eval_dashboard reçoit evaluation/stats
           mise à jour GUI (1 Hz)
```
