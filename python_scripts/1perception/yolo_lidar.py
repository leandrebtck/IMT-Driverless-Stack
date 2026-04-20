#!/usr/bin/env python3
"""
yolo_lidar.py — YOLO + LiDAR avec filtrage de Kalman par cône.
Compatible ROS Iron et ROS Galactic.

Pipeline par frame :
  1. YOLO détecte les cônes (bbox + couleur)
  2. Points LiDAR projetés dans l'image → association bbox ↔ points
  3. Rejet des outliers LiDAR (distance > médiane ± 0.5 m)
  4. Estimation de position 3D par centroïde pondéré
  5. Association détections → trackers existants (distance 3D)
  6. Mise à jour du filtre de Kalman de chaque tracker
  7. Publication des cônes confirmés (hits ≥ MIN_HITS)

Subscriptions :
  /fsds/cam1/image_color   — flux caméra
  /lidar/Lidar1            — nuage de points LiDAR

Publications :
  /yolo_lidar/debug_image  — image annotée
  /yolo_lidar/cone_markers — markers 3D (repère LiDAR)
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from config_loader import CFG

import math
import numpy as np
import cv2
import torch
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from tf2_ros import Buffer, TransformListener
from image_geometry import PinholeCameraModel
import message_filters

try:
    import sensor_msgs_py.point_cloud2 as pc2
except ImportError:
    from sensor_msgs import point_cloud2 as pc2


# ────────────────────────────────────────────────────────────────────────────
class KalmanCone:
    """
    Filtre de Kalman pour un cône statique (position 3D).

    Modèle :
      - État       : x = [px, py, pz]  (position dans le repère LiDAR)
      - Transition : F = I  (cône immobile → pas de dynamique)
      - Observation: H = I  (on mesure directement la position)
      - Bruit process Q  : faible (cône ne bouge pas)
      - Bruit mesure  R  : ~20 cm (bruit LiDAR)

    Convergence : après ~5 mises à jour, l'incertitude P est divisée par 5
    et la position estimée est stable à ±5 cm.
    """

    def __init__(self, x: float, y: float, z: float,
                 r_std: float = 0.20,
                 q_std: float = 0.02):
        self.x = np.array([x, y, z], dtype=np.float64)
        self.P = np.eye(3) * 1.0          # incertitude initiale élevée
        self.Q = np.eye(3) * q_std ** 2   # bruit processus (cône statique)
        self.R = np.eye(3) * r_std ** 2   # bruit mesure LiDAR

    def update(self, mx: float, my: float, mz: float) -> np.ndarray:
        """Intègre une mesure LiDAR et retourne la position estimée."""
        z     = np.array([mx, my, mz], dtype=np.float64)
        P_pre = self.P + self.Q                     # prédiction (F=I)
        S     = P_pre + self.R                      # innovation covariance
        K     = P_pre @ np.linalg.inv(S)            # gain de Kalman
        self.x = self.x + K @ (z - self.x)          # mise à jour état
        self.P = (np.eye(3) - K) @ P_pre            # mise à jour incertitude
        return self.x.copy()

    @property
    def position(self) -> np.ndarray:
        return self.x.copy()

    @property
    def uncertainty(self) -> float:
        """Trace de P : indicateur global d'incertitude."""
        return float(np.trace(self.P))


# ────────────────────────────────────────────────────────────────────────────
class ConeTracker:
    """
    Tracker pour un seul cône : filtre de Kalman + compteurs hits/misses.

    - hits   : nombre de frames consécutives avec une association LiDAR valide
    - misses : nombre de frames consécutives sans association
    Le cône est affiché seulement si hits ≥ MIN_HITS (évite les faux positifs).
    Il est supprimé si misses ≥ MAX_MISSES.
    """

    _next_id = 0

    def __init__(self, x, y, z, color, conf, bbox):
        self.id     = ConeTracker._next_id
        ConeTracker._next_id += 1
        self.color  = color
        self.conf   = conf
        self.bbox   = bbox      # (x1, y1, x2, y2) dans l'image
        self.hits   = 1
        self.misses = 0
        # Position brute de la dernière mesure (repère LiDAR courant)
        self._raw_pos = np.array([x, y, z], dtype=np.float64)

    def update(self, x, y, z, color, conf, bbox):
        self._raw_pos = np.array([x, y, z], dtype=np.float64)
        self.color  = color
        self.conf   = conf
        self.bbox   = bbox
        self.hits  += 1
        self.misses = 0

    def miss(self):
        self.misses += 1

    @property
    def position(self):
        return self._raw_pos.copy()

    @property
    def confirmed(self):
        return self.hits >= YoloLidarNode.MIN_HITS


# ────────────────────────────────────────────────────────────────────────────
class YoloLidarNode(Node):

    COLORS_BGR = {0: (0, 255, 255), 1: (255, 0, 0), 2: (0, 140, 255)}
    NAMES      = {0: "JAUNE",       1: "BLEU",       2: "ORANGE"}
    COLORS_RGB = {0: (1.0, 1.0, 0.0), 1: (0.0, 0.0, 1.0), 2: (1.0, 0.5, 0.0)}

    # ── Paramètres chargés depuis config.yaml ────────────────────────────────
    Z_GROUND     = CFG['lidar']['z_ground_threshold_m']
    MARGIN       = CFG['lidar']['bbox_margin_px']
    MARGIN_RATIO = CFG['lidar']['bbox_margin_ratio']
    MIN_PTS      = CFG['lidar']['min_points_in_bbox']
    OUTLIER_STD  = CFG['lidar']['outlier_std_m']
    MIN_HITS     = CFG['tracking']['min_hits']
    MAX_MISSES   = CFG['tracking']['max_misses']
    ASSOC_DIST   = CFG['tracking']['association_distance_m']

    def __init__(self):
        super().__init__('yolo_lidar_node')

        # YOLO
        script_dir   = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')
        if not os.path.exists(weights_path):
            raise FileNotFoundError(f"Poids YOLO introuvables : {weights_path}")
        self.model  = YOLO(weights_path)
        device      = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"YOLO chargé sur : {device}")

        # TF2
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Caméra
        self.bridge     = CvBridge()
        self.cam_model  = PinholeCameraModel()
        self.cam_width  = 0
        self.cam_height = 0

        # Display thread (même pattern que yolo_stereo)
        self._display_frame = None
        self._display_lock  = threading.Lock()

        # Guard : empêche l'accumulation de callbacks si YOLO est lent
        self._processing = False

        # Trackers actifs
        self.trackers: list[ConeTracker] = []

        # Subscribers synchronisés (même pattern que yolo_stereo)
        _t = CFG['topics']
        self.sub_image = message_filters.Subscriber(
            self, Image, _t['camera_left'],
            qos_profile=qos_profile_sensor_data)
        self.sub_lidar = message_filters.Subscriber(
            self, PointCloud2, _t['lidar'],
            qos_profile=qos_profile_sensor_data)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_image, self.sub_lidar], queue_size=5, slop=0.1)
        self.ts.registerCallback(self._sync_cb)

        # Publishers
        self.pub_debug   = self.create_publisher(Image,            _t['debug_image_yolo_lidar'],   10)
        self.pub_markers = self.create_publisher(MarkerArray,      _t['debug_markers_yolo_lidar'],  10)
        self.pub_dets    = self.create_publisher(Detection2DArray, _t['lidar_detections'], 10)

        self.get_logger().info(
            f"YoloLidarNode prêt (hits≥{self.MIN_HITS}, misses<{self.MAX_MISSES}, "
            f"assoc={self.ASSOC_DIST}m)"
        )

    # ── Caméra ───────────────────────────────────────────────────────────────
    def _setup_camera(self, width, height):
        f    = width / 2.0
        info = CameraInfo()
        info.width, info.height = width, height
        info.k = [f, 0.0, width/2.0, 0.0, f, height/2.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, width/2.0, 0.0, 0.0, f, height/2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.cam_model.fromCameraInfo(info)
        self.cam_width, self.cam_height = width, height
        self.get_logger().info(f"Modèle caméra : {width}×{height}")

    # ── Callback synchronisé image + LiDAR ──────────────────────────────────
    def _sync_cb(self, img_msg, lidar_msg):
        # Skip si le pipeline précédent n'est pas terminé (évite l'accumulation)
        if self._processing:
            return
        self._processing = True
        try:
            self._process_frame(img_msg, lidar_msg)
        finally:
            self._processing = False

    def _process_frame(self, img_msg, lidar_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"image conversion : {e}")
            return

        H, W = frame.shape[:2]
        if W != self.cam_width or H != self.cam_height:
            self._setup_camera(W, H)

        # 1. TF LiDAR → caméra (extraction matrice 4×4 pour batch numpy)
        try:
            tf = self.tf_buffer.lookup_transform(
                'fsds/cam1', 'fsds/Lidar1', rclpy.time.Time(seconds=0))
        except Exception as e:
            self.get_logger().warning(f"TF non prêt : {e}", throttle_duration_sec=2)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        # Quaternion → matrice de rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),     1-2*(qx*qx+qy*qy)]
        ], dtype=np.float64)
        T = np.array([t.x, t.y, t.z], dtype=np.float64)

        # 2. Lecture points + filtre sol
        try:
            raw = list(pc2.read_points(lidar_msg, field_names=('x', 'y', 'z'), skip_nans=True))
        except Exception as e:
            self.get_logger().error(f"Lecture PointCloud2 : {e}")
            return
        if not raw:
            return
        pts = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in raw],
                       dtype=np.float64)
        pts = pts[pts[:, 2] > self.Z_GROUND]
        if len(pts) == 0:
            return

        # 3. YOLO
        results = self.model(frame, verbose=False, conf=0.5)
        yolo_dets = []   # list of {x1,y1,x2,y2, key, conf}
        if results and len(results[0].boxes):
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                name   = self.model.names[cls_id].lower()
                key    = 2 if ('orange' in name or 'large' in name) else \
                         1 if 'blue'   in name else \
                         0 if 'yellow' in name else cls_id
                yolo_dets.append({'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                                  'key': key, 'conf': conf})

        # 4. Projection LiDAR → pixels (batch numpy, ~100x plus rapide)
        # Transformation LiDAR → repère physique caméra : p_cam = R @ p_lid + T
        pts_cam = (R @ pts.T).T + T                    # (N, 3) dans repère cam1
        # Repère physique cam1 (X=avant,Y=gauche,Z=haut) → repère optique (X=droite,Y=bas,Z=profondeur)
        z_opt = pts_cam[:, 0]                           # X_phys = profondeur
        x_opt = -pts_cam[:, 1]                          # -Y_phys = droite
        y_opt = -pts_cam[:, 2]                          # -Z_phys = bas

        # Filtre : devant la caméra uniquement
        valid = z_opt > 0.1
        z_opt, x_opt, y_opt = z_opt[valid], x_opt[valid], y_opt[valid]
        pts_valid = pts[valid]                          # coordonnées LiDAR originales

        # Projection pinhole → pixels
        fx = self.cam_model.fx()
        fy = self.cam_model.fy()
        cx_cam = self.cam_model.cx()
        cy_cam = self.cam_model.cy()
        u_all = (fx * x_opt / z_opt + cx_cam).astype(np.int32)
        v_all = (fy * y_opt / z_opt + cy_cam).astype(np.int32)

        # Filtre : dans l'image
        in_img = (u_all >= 0) & (u_all < W) & (v_all >= 0) & (v_all < H)
        u_px    = u_all[in_img]
        v_px    = v_all[in_img]
        pts_img = pts_valid[in_img]                     # (M, 3) coords LiDAR
        dist_xy = np.linalg.norm(pts_img[:, :2], axis=1)  # distance XY dans repère LiDAR

        # 5. Fusion YOLO ↔ LiDAR : extraction de position 3D par bbox (vectorisé)
        detections_3d = []   # list of {x,y,z, key, conf, bbox}
        for det in yolo_dets:
            bx1, by1, bx2, by2 = det['x1'], det['y1'], det['x2'], det['y2']
            m = max(self.MARGIN, int(self.MARGIN_RATIO * min(bx2 - bx1, by2 - by1)))

            # Masque des points LiDAR dans la bbox (vectorisé)
            mask = ((u_px >= bx1 - m) & (u_px <= bx2 + m) &
                    (v_px >= by1 - m) & (v_px <= by2 + m))
            n_inside = np.count_nonzero(mask)
            if n_inside < self.MIN_PTS:
                continue

            # Rejet des outliers : points à ±OUTLIER_STD de la médiane
            dists_in = dist_xy[mask]
            median   = np.median(dists_in)
            inlier_mask = np.abs(dists_in - median) <= self.OUTLIER_STD
            if np.count_nonzero(inlier_mask) < self.MIN_PTS:
                continue

            pts_inlier  = pts_img[mask][inlier_mask]    # (K, 3)
            dist_inlier = dists_in[inlier_mask]

            # Centroïde pondéré (poids ∝ 1/distance)
            inv_d = 1.0 / np.maximum(dist_inlier, 0.1)
            w     = inv_d / inv_d.sum()
            centroid = (pts_inlier * w[:, None]).sum(axis=0)

            detections_3d.append({
                'x': float(centroid[0]), 'y': float(centroid[1]), 'z': float(centroid[2]),
                'dist': float(median),
                'key': det['key'], 'conf': det['conf'],
                'bbox': (bx1, by1, bx2, by2)
            })

        # 6. Association détections 3D ↔ trackers (greedy par distance 3D min)
        #    Seuls les trackers fraîchement mis à jour (misses <= 1) participent
        #    car les positions périmées dans le repère LiDAR mobile sont faussées.
        matched_t = set()
        matched_d = set()
        pairs = []   # (det_idx, tracker_idx)

        for d_idx, det in enumerate(detections_3d):
            best_t, best_dist = None, self.ASSOC_DIST
            for t_idx, tr in enumerate(self.trackers):
                if t_idx in matched_t or tr.misses > 1:
                    continue
                tp = tr.position
                d3 = math.sqrt((det['x'] - tp[0])**2 +
                               (det['y'] - tp[1])**2 +
                               (det['z'] - tp[2])**2)
                if d3 < best_dist:
                    best_dist, best_t = d3, t_idx
            if best_t is not None:
                pairs.append((d_idx, best_t))
                matched_t.add(best_t)
                matched_d.add(d_idx)

        # Mises à jour
        for d_idx, t_idx in pairs:
            det = detections_3d[d_idx]
            self.trackers[t_idx].update(
                det['x'], det['y'], det['z'],
                det['key'], det['conf'], det['bbox'])

        # Nouveaux trackers pour les détections non associées
        for d_idx, det in enumerate(detections_3d):
            if d_idx not in matched_d:
                self.trackers.append(
                    ConeTracker(det['x'], det['y'], det['z'],
                                det['key'], det['conf'], det['bbox']))

        # Miss pour les trackers non associés
        for t_idx, tr in enumerate(self.trackers):
            if t_idx not in matched_t:
                tr.miss()

        # Purge des trackers trop vieux sans détection
        self.trackers = [tr for tr in self.trackers if tr.misses < self.MAX_MISSES]

        # 7. Publication markers + image debug
        self._publish(lidar_msg, img_msg.header, detections_3d, frame, W, H)

    # ── Publication ──────────────────────────────────────────────────────────
    def _publish(self, lidar_msg, img_header, detections_3d, frame, W, H):
        marker_array = MarkerArray()
        debug        = frame.copy()

        # Efface les anciens markers (DELETE_ALL)
        del_mk        = Marker()
        del_mk.action = Marker.DELETEALL
        del_mk.header.frame_id = lidar_msg.header.frame_id
        del_mk.header.stamp    = lidar_msg.header.stamp
        marker_array.markers.append(del_mk)

        # Dessine toutes les detections YOLO (même non confirmées) sur l'image debug
        for det in detections_3d:
            x1, y1, x2, y2 = det['bbox']
            color = self.COLORS_BGR.get(det['key'], (0, 255, 0))
            label = self.NAMES.get(det['key'], str(det['key']))
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 1)
            cv2.putText(debug, f"{label} {det['dist']:.1f}m",
                        (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        # Markers + annotations pour les trackers confirmés ET fraîchement mis à jour
        for tr in self.trackers:
            if not tr.confirmed or tr.misses > 0:
                continue
            pos = tr.position
            x1, y1, x2, y2 = tr.bbox
            color = self.COLORS_BGR.get(tr.color, (0, 255, 0))
            label = self.NAMES.get(tr.color, str(tr.color))
            dist  = math.sqrt(pos[0]**2 + pos[1]**2)

            # Bbox épaisse + label complet sur l'image
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            full_label = f"{label} {dist:.1f}m [K{tr.hits}]"
            (tw, _), _ = cv2.getTextSize(full_label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(debug, (x1, y1 - 20), (x1 + tw, y1), color, -1)
            cv2.putText(debug, full_label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

            # Marker sphère en RViz
            mk = Marker()
            mk.header.frame_id    = lidar_msg.header.frame_id
            mk.header.stamp       = lidar_msg.header.stamp
            mk.ns                 = 'yolo_lidar_cones'
            mk.id                 = tr.id
            mk.type               = Marker.CYLINDER
            mk.action             = Marker.ADD
            mk.pose.position.x    = float(pos[0])
            mk.pose.position.y    = float(pos[1])
            mk.pose.position.z    = float(pos[2])
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = 0.28
            mk.scale.z = 0.33
            r, g, b = self.COLORS_RGB.get(tr.color, (0.5, 0.5, 0.5))
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 1.0
            mk.lifetime.nanosec = 300_000_000   # 0.3 s
            marker_array.markers.append(mk)

        self.pub_markers.publish(marker_array)

        # Publication Detection2DArray pour le SLAM lidar (cone_mapper_lidar.py)
        # Timestamp du scan LiDAR original (pas now) pour que le SLAM utilise le bon TF
        det_array = Detection2DArray()
        det_array.header.frame_id = lidar_msg.header.frame_id
        det_array.header.stamp    = lidar_msg.header.stamp
        for tr in self.trackers:
            if not tr.confirmed or tr.misses > 0:
                continue
            pos  = tr.position
            d2d  = Detection2D()
            d2d.header = det_array.header
            hyp  = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(tr.color)
            hyp.hypothesis.score    = tr.conf
            # Position 3D dans le repère fsds/Lidar1 (coordonnées brutes LiDAR)
            hyp.pose.pose.position.x = float(pos[0])
            hyp.pose.pose.position.y = float(pos[1])
            hyp.pose.pose.position.z = float(pos[2])
            d2d.results.append(hyp)
            det_array.detections.append(d2d)
        self.pub_dets.publish(det_array)

        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            dbg_msg.header = img_header
            self.pub_debug.publish(dbg_msg)
        except Exception:
            pass

        with self._display_lock:
            self._display_frame = debug


# ── Boucle d'affichage dans un thread dédié ─────────────────────────────────
# Séparation nécessaire pour ne pas bloquer le spin ROS avec cv2.imshow
# (même pattern que yolo_stereo.py)
# ────────────────────────────────────────────────────────────────────────────
def display_loop(node):
    import time
    while rclpy.ok():
        with node._display_lock:
            frame = node._display_frame
        if frame is not None:
            cv2.imshow("YOLO + LiDAR", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            time.sleep(0.033)
    cv2.destroyAllWindows()


# ────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = YoloLidarNode()

    display_thread = threading.Thread(target=display_loop, args=(node,), daemon=True)
    display_thread.start()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
