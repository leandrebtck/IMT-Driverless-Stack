#!/usr/bin/env python3
"""
yolo_lidar.py
Node ROS2 autonome : YOLO (détection + couleur des cônes) + LiDAR (distance).
Compatible ROS Iron et ROS Galactic.

Subscriptions :
  /fsds/cam1/image_color   — flux caméra
  /lidar/Lidar1            — nuage de points brut

Publications :
  /yolo_lidar/debug_image  — image annotée (bbox + couleur + distance)
  /yolo_lidar/cone_markers — markers 3D colorés dans le repère LiDAR
"""

import os
import numpy as np
import cv2
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from image_geometry import PinholeCameraModel

# Compatibilité Galactic / Iron pour sensor_msgs_py
try:
    import sensor_msgs_py.point_cloud2 as pc2
except ImportError:
    from sensor_msgs import point_cloud2 as pc2


class YoloLidarNode(Node):

    # Couleurs OpenCV (BGR)
    COLORS_BGR = {0: (0, 255, 255), 1: (255, 0, 0), 2: (0, 140, 255)}
    NAMES      = {0: "JAUNE",       1: "BLEU",       2: "ORANGE"}
    # Couleurs RViz (RGB normalisé)
    COLORS_RGB = {0: (1.0, 1.0, 0.0), 1: (0.0, 0.0, 1.0), 2: (1.0, 0.5, 0.0)}

    Z_GROUND      = -0.40   # seuil filtrage sol (m)
    MARGIN        = 15      # marge de base en pixels autour de la bbox YOLO
    MARGIN_RATIO  = 0.25    # marge adaptative = 25 % de la plus petite dimension de la bbox
    MIN_PTS       = 2       # nb minimum de points LiDAR pour valider une distance

    def __init__(self):
        super().__init__('yolo_lidar_node')

        # ── 1. Chargement YOLO ──────────────────────────────────────────────
        script_dir   = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')
        if not os.path.exists(weights_path):
            raise FileNotFoundError(f"Poids YOLO introuvables : {weights_path}")

        self.model = YOLO(weights_path)
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"YOLO chargé sur : {device}")

        # ── 2. TF2 ──────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 3. Modèle caméra — mis à jour dynamiquement à la 1ère image ─────
        self.bridge     = CvBridge()
        self.cam_model  = PinholeCameraModel()
        self.cam_width  = 0
        self.cam_height = 0

        # ── 4. Cache image ───────────────────────────────────────────────────
        self.latest_image  = None
        self.latest_header = None

        # ── 5. Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Image, '/fsds/cam1/image_color',
            self._image_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, '/lidar/Lidar1',
            self._lidar_cb, qos_profile_sensor_data
        )

        # ── 6. Publishers ────────────────────────────────────────────────────
        self.pub_debug   = self.create_publisher(Image,       '/yolo_lidar/debug_image',  10)
        self.pub_markers = self.create_publisher(MarkerArray, '/yolo_lidar/cone_markers', 10)

        self.get_logger().info("YoloLidarNode prêt.")

    # ────────────────────────────────────────────────────────────────────────
    def _setup_camera(self, width, height):
        """Met à jour le modèle caméra selon les dimensions réelles de l'image."""
        f  = width / 2.0
        cx = width  / 2.0
        cy = height / 2.0
        info = CameraInfo()
        info.width, info.height = width, height
        info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.cam_model.fromCameraInfo(info)
        self.cam_width  = width
        self.cam_height = height
        self.get_logger().info(f"Modèle caméra initialisé : {width}×{height}")

    def _image_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            H, W = img.shape[:2]
            # Mise à jour du modèle si la résolution change
            if W != self.cam_width or H != self.cam_height:
                self._setup_camera(W, H)
            self.latest_image  = img
            self.latest_header = msg.header
        except Exception as e:
            self.get_logger().error(f"image_cb : {e}")

    # ────────────────────────────────────────────────────────────────────────
    def _lidar_cb(self, msg):
        if self.latest_image is None or self.cam_width == 0:
            return

        # 1. Transform LiDAR → caméra
        try:
            tf = self.tf_buffer.lookup_transform(
                'fsds/cam1', 'fsds/Lidar1',
                rclpy.time.Time(seconds=0)
            )
        except Exception as e:
            self.get_logger().warning(f"TF non prêt : {e}", throttle_duration_sec=2)
            return

        # 2. Lecture + filtrage sol (compatible Galactic et Iron)
        try:
            raw = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        except Exception as e:
            self.get_logger().error(f"Lecture PointCloud2 : {e}")
            return
        if not raw:
            return
        pts = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in raw], dtype=np.float32)
        pts = pts[pts[:, 2] > self.Z_GROUND]
        if len(pts) == 0:
            return

        # 3. YOLO sur l'image courante
        frame      = self.latest_image.copy()
        H, W       = frame.shape[:2]
        results    = self.model(frame, verbose=False, conf=0.5)
        detections = []   # (x1, y1, x2, y2, key_id, conf)

        if results and len(results[0].boxes):
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                name   = self.model.names[cls_id].lower()
                if 'orange' in name or 'large' in name:
                    key = 2
                elif 'blue' in name:
                    key = 1
                elif 'yellow' in name:
                    key = 0
                else:
                    key = cls_id
                detections.append((x1, y1, x2, y2, key, conf))

        # 4. Projection LiDAR → pixels
        projected = []   # (u, v, dist, x_lid, y_lid, z_lid)
        for pt in pts:
            p = PointStamped()
            p.header.frame_id   = 'fsds/Lidar1'
            p.point.x, p.point.y, p.point.z = float(pt[0]), float(pt[1]), float(pt[2])
            try:
                p_cam = tf2_geometry_msgs.do_transform_point(p, tf)
            except Exception:
                continue

            # Repère physique → repère optique (Z=profondeur, X=droite, Y=bas)
            z_opt =  p_cam.point.x
            x_opt = -p_cam.point.y
            y_opt = -p_cam.point.z

            if z_opt < 0.1:
                continue

            u, v = self.cam_model.project3dToPixel((x_opt, y_opt, z_opt))
            u, v = int(u), int(v)
            if 0 <= u < W and 0 <= v < H:
                dist = float(np.linalg.norm(pt[:2]))   # distance horizontale (XY)
                projected.append((u, v, dist, float(pt[0]), float(pt[1]), float(pt[2])))

        # 5. Association bbox ↔ points LiDAR + rendu
        marker_array = MarkerArray()
        debug = frame.copy()

        for i, (x1, y1, x2, y2, key, conf) in enumerate(detections):
            color = self.COLORS_BGR.get(key, (0, 255, 0))
            label = self.NAMES.get(key, str(key))

            # Marge adaptative : max(marge fixe, 25% de la plus petite dim de la bbox)
            m = max(self.MARGIN, int(self.MARGIN_RATIO * min(x2 - x1, y2 - y1)))

            inside = [
                (u, v, d, px, py, pz)
                for u, v, d, px, py, pz in projected
                if x1 - m <= u <= x2 + m and y1 - m <= v <= y2 + m
            ]

            if len(inside) >= self.MIN_PTS:
                dist_m   = float(np.median([p[2] for p in inside]))
                cx3d     = float(np.mean([p[3] for p in inside]))
                cy3d     = float(np.mean([p[4] for p in inside]))
                cz3d     = float(np.mean([p[5] for p in inside]))
                dist_txt = f"{dist_m:.1f}m"
            else:
                dist_txt = "?"
                cx3d = cy3d = cz3d = 0.0

            # Dessin bbox + label + distance
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            full_label = f"{label} {conf:.2f} {dist_txt}"
            (tw, th), _ = cv2.getTextSize(full_label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(debug, (x1, y1 - 20), (x1 + tw, y1), color, -1)
            cv2.putText(debug, full_label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)

            # Marker 3D (seulement si position LiDAR connue)
            if len(inside) >= self.MIN_PTS:
                mk = Marker()
                mk.header.frame_id    = msg.header.frame_id
                mk.header.stamp       = self.get_clock().now().to_msg()
                mk.ns                 = "yolo_lidar_cones"
                mk.id                 = i
                mk.type               = Marker.SPHERE
                mk.action             = Marker.ADD
                mk.pose.position.x    = cx3d
                mk.pose.position.y    = cy3d
                mk.pose.position.z    = cz3d
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = mk.scale.z = 0.4
                r, g, b = self.COLORS_RGB.get(key, (0.5, 0.5, 0.5))
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 1.0
                mk.lifetime.nanosec = 200_000_000   # 0.2 s
                marker_array.markers.append(mk)

        # 6. Publication
        self.pub_markers.publish(marker_array)
        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            if self.latest_header:
                dbg_msg.header = self.latest_header
            self.pub_debug.publish(dbg_msg)
        except Exception:
            pass

        cv2.imshow("YOLO + LiDAR", debug)
        cv2.waitKey(1)


# ────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = YoloLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
