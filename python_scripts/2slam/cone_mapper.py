#!/usr/bin/env python3
"""
cone_mapper.py — SLAM / Cartographie des cônes en temps réel.
Compatible ROS Galactic et ROS Iron.

Géométrie de piste connue :
  - Largeur de piste : 6 m
  - Espacement entre cônes (même côté) : ~5 m (moins en virage)
  - Nombre de tours max : 10

  → MERGE_DIST = 1.5 m  (bien inférieur à la moitié de 5 m pour ne pas fusionner
                          deux cônes différents)
  → MAX_LINE_STEP = 7 m (tolère un cône manqué : 5 m + marge ; le nearest-neighbor
                          s'arrête si le gap est plus grand, pas de trait parasite)
  → COUNT_CAP = 20       (10 tours × ~2 détections/tour → position stable)
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from config_loader import CFG

import math
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Point as GPoint
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

CAM_FRAME = CFG['frames']['cam_left']
MAP_FRAME = CFG['frames']['map']

COLORS_RGB = {
    0:  (1.0, 1.0, 0.0),   # JAUNE  — droite de la piste
    1:  (0.0, 0.2, 1.0),   # BLEU   — gauche de la piste
    2:  (1.0, 0.4, 0.0),   # ORANGE — départ / arrivée
    -1: (0.5, 0.5, 0.5),
}
COLOR_NAME = {0: 'JAUNE', 1: 'BLEU', 2: 'ORANGE', -1: '?'}
COLOR_PRIO = {2: 3, 0: 2, 1: 2, -1: 0}


class ConeMapperNode(Node):

    # ── Paramètres de fusion ─────────────────────────────────────────────────
    MERGE_DIST   = CFG['slam']['stereo']['merge_distance_m']
    COUNT_CAP    = CFG['slam']['count_cap']
    MAX_DEPTH_STRAIGHT = CFG['slam']['stereo']['max_depth_straight_m']
    MAX_DEPTH_TURN     = CFG['slam']['stereo']['max_depth_turn_m']
    TURN_YAW_RATE      = CFG['slam']['turn_yaw_rate_threshold']
    PUBLISH_HZ    = CFG['slam']['publish_hz']
    MIN_COUNT     = CFG['slam']['min_count']
    MIN_SCORE     = 0.0
    MAX_LINE_STEP = CFG['slam']['max_line_step_m']
    LINE_STRIDE   = CFG['slam']['line_stride_stereo']

    def __init__(self):
        super().__init__('cone_mapper_node')

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cones    = {}   # {id: {x,y,z, color, count}}
        self.next_id  = 0
        self.yaw_rate = 0.0

        _t = CFG['topics']
        self.create_subscription(
            Detection2DArray, _t['stereo_detections'],
            self._detections_cb, 10)
        self.create_subscription(
            Odometry, _t['odometry'],
            self._odom_cb, qos_profile_sensor_data)

        self.pub_map  = self.create_publisher(MarkerArray, _t['slam_stereo_map'],   LATCHED_QOS)
        self.pub_stat = self.create_publisher(String,      _t['slam_stereo_stats'], 10)

        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_map)

        self.get_logger().info(
            f"ConeMapperNode prêt — merge={self.MERGE_DIST}m "
            f"max_depth={self.MAX_DEPTH_STRAIGHT}m step_ligne={self.MAX_LINE_STEP}m"
        )

    # ── Odométrie → yaw rate ─────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.yaw_rate = msg.twist.twist.angular.z

    # ── Callback détections ──────────────────────────────────────────────────
    def _detections_cb(self, msg: Detection2DArray):
        if not msg.detections:
            return

        turning   = abs(self.yaw_rate) >= self.TURN_YAW_RATE
        max_depth = self.MAX_DEPTH_TURN if turning else self.MAX_DEPTH_STRAIGHT

        # TF caméra → monde (timestamp exact, fallback latest)
        # TF : transform le plus récent disponible. L'erreur de position induite
        # (vitesse × latence ≈ 3 cm à 1.5 m/s) est bien inférieure à l'erreur
        # de profondeur stéréo (±1-2 m). Évite les échecs fréquents sur timestamp exact.
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, CAM_FRAME, rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception as e:
            self.get_logger().warning(f"TF non prêt : {e}", throttle_duration_sec=2)
            return

        for det in msg.detections:
            if not det.results:
                continue

            hyp   = det.results[0]
            depth = hyp.pose.pose.position.x
            score = hyp.hypothesis.score

            if depth <= 0.1 or depth > max_depth:
                continue
            if abs(score) < self.MIN_SCORE:
                continue

            color = self._parse_color(hyp.hypothesis.class_id)

            p = PointStamped()
            p.header.frame_id = CAM_FRAME
            p.header.stamp    = msg.header.stamp
            p.point.x = hyp.pose.pose.position.x
            p.point.y = hyp.pose.pose.position.y
            p.point.z = hyp.pose.pose.position.z

            try:
                pw = tf2_geometry_msgs.do_transform_point(p, tf)
            except Exception:
                continue

            wx, wy, wz = pw.point.x, pw.point.y, pw.point.z

            # Recherche du cône le plus proche
            best_id, best_dist = None, self.MERGE_DIST
            for cid, cone in self.cones.items():
                d = math.sqrt((cone['x'] - wx)**2 + (cone['y'] - wy)**2)
                if d < best_dist:
                    best_dist, best_id = d, cid

            if best_id is not None:
                c     = self.cones[best_id]
                n_eff = min(c['count'], self.COUNT_CAP)
                c['x']     = (c['x'] * n_eff + wx) / (n_eff + 1)
                c['y']     = (c['y'] * n_eff + wy) / (n_eff + 1)
                c['z']     = (c['z'] * n_eff + wz) / (n_eff + 1)
                c['count'] += 1
                if COLOR_PRIO.get(color, 0) > COLOR_PRIO.get(c['color'], 0):
                    c['color'] = color
            else:
                # En virage on n'ajoute pas de nouveaux cônes (positions peu fiables)
                if turning:
                    continue
                self.cones[self.next_id] = {
                    'x': wx, 'y': wy, 'z': wz,
                    'color': color, 'count': 1,
                }
                self.get_logger().info(
                    f"Cône #{self.next_id} ({COLOR_NAME.get(color,'?')}) à ({wx:.1f}, {wy:.1f})")
                self.next_id += 1

    # ── Publication ──────────────────────────────────────────────────────────
    def _publish_map(self):
        now          = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        by_color     = {0: [], 1: [], 2: []}

        for cid, cone in self.cones.items():
            if cone['count'] < self.MIN_COUNT:
                continue

            r, g, b = COLORS_RGB.get(cone['color'], (0.5, 0.5, 0.5))

            mk = Marker()
            mk.header.frame_id    = MAP_FRAME
            mk.header.stamp       = now
            mk.ns                 = 'cone_map'
            mk.id                 = cid
            mk.type               = Marker.CYLINDER
            mk.action             = Marker.ADD
            mk.pose.position.x    = cone['x']
            mk.pose.position.y    = cone['y']
            mk.pose.position.z    = cone['z']
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = 0.28
            mk.scale.z = 0.33
            mk.color.r, mk.color.g, mk.color.b = r, g, b
            mk.color.a       = 1.0
            mk.lifetime.sec  = mk.lifetime.nanosec = 0
            marker_array.markers.append(mk)

            txt = Marker()
            txt.header            = mk.header
            txt.ns                = 'cone_labels'
            txt.id                = cid + 10000
            txt.type              = Marker.TEXT_VIEW_FACING
            txt.action            = Marker.ADD
            txt.pose.position.x   = cone['x']
            txt.pose.position.y   = cone['y']
            txt.pose.position.z   = cone['z'] + 0.5
            txt.pose.orientation.w = 1.0
            txt.scale.z           = 0.25
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a           = 0.9
            txt.text              = f"#{cid} ×{cone['count']}"
            txt.lifetime.sec = txt.lifetime.nanosec = 0
            marker_array.markers.append(txt)

            if cone['color'] in by_color:
                by_color[cone['color']].append((cone['x'], cone['y'], cone['z']))

        # Lignes vertes transversales : relient chaque cône jaune au cône bleu le plus proche
        yellow_cones = by_color[0]  # liste de (x, y, z)
        blue_cones   = by_color[1]
        if yellow_cones and blue_cones:
            ln = Marker()
            ln.header.frame_id    = MAP_FRAME
            ln.header.stamp       = now
            ln.ns                 = 'track_cross_lines'
            ln.id                 = 20000
            ln.type               = Marker.LINE_LIST
            ln.action             = Marker.ADD
            ln.pose.orientation.w = 1.0
            ln.scale.x            = 0.08
            ln.color.r, ln.color.g, ln.color.b = 0.0, 1.0, 0.0  # vert
            ln.color.a            = 0.85
            ln.lifetime.sec = ln.lifetime.nanosec = 0

            for (yx, yy, yz) in yellow_cones:
                # Trouver le cône bleu le plus proche
                best_dist = float('inf')
                best_blue = None
                for (bx, by, bz) in blue_cones:
                    d = math.sqrt((yx - bx)**2 + (yy - by)**2)
                    if d < best_dist:
                        best_dist = d
                        best_blue = (bx, by, bz)
                if best_blue is not None:
                    p1 = GPoint(); p1.x = yx; p1.y = yy; p1.z = yz
                    p2 = GPoint(); p2.x = best_blue[0]; p2.y = best_blue[1]; p2.z = best_blue[2]
                    ln.points.append(p1)
                    ln.points.append(p2)

            marker_array.markers.append(ln)

        self.pub_map.publish(marker_array)

        counts = {}
        for cone in self.cones.values():
            counts[cone['color']] = counts.get(cone['color'], 0) + 1
        stat = String()
        stat.data = (
            f"{len(self.cones)} cônes | yaw={self.yaw_rate:.2f} rad/s | "
            f"J={counts.get(0,0)} B={counts.get(1,0)} O={counts.get(2,0)}"
        )
        self.pub_stat.publish(stat)

    # ── Helpers ──────────────────────────────────────────────────────────────
    @staticmethod
    def _catmull_rom(pts, steps=12):
        """
        Interpole une liste de points avec une spline Catmull-Rom.
        Chaque segment entre deux cônes génère `steps` points intermédiaires.
        La tangente en chaque point est calculée à partir des deux voisins →
        les angles pointus disparaissent même si un cône est légèrement décalé.
        """
        if len(pts) < 2:
            return pts
        # Points fantômes aux extrémités pour une tangente correcte
        ext = [pts[0]] + list(pts) + [pts[-1]]
        result = []
        for i in range(1, len(ext) - 2):
            p0, p1, p2, p3 = ext[i-1], ext[i], ext[i+1], ext[i+2]
            for s in range(steps):
                t  = s / steps
                t2 = t * t
                t3 = t2 * t
                def cr(a, b, c, d):
                    return 0.5 * (2*b + (-a+c)*t + (2*a-5*b+4*c-d)*t2 + (-a+3*b-3*c+d)*t3)
                result.append((cr(p0[0],p1[0],p2[0],p3[0]),
                                cr(p0[1],p1[1],p2[1],p3[1]),
                                cr(p0[2],p1[2],p2[2],p3[2])))
        result.append(pts[-1])
        return result

    @staticmethod
    def _angle_segments(pts, max_step):
        """
        Trie les points par angle depuis leur centroïde commun (ordre naturel
        sur un circuit fermé) puis découpe en segments là où le gap entre deux
        cônes consécutifs dépasse max_step (cône manquant ou fin de section).

        Cette méthode élimine les zigzags : chaque cône est placé dans l'ordre
        angulaire du circuit, jamais en aller-retour.
        """
        if len(pts) < 2:
            return [pts] if len(pts) == 1 else []

        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        ordered = sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))

        # Découpe en segments si gap > max_step
        segments  = []
        seg       = [ordered[0]]
        for i in range(1, len(ordered)):
            dx = ordered[i][0] - ordered[i-1][0]
            dy = ordered[i][1] - ordered[i-1][1]
            if math.sqrt(dx*dx + dy*dy) > max_step:
                if len(seg) >= 2:
                    segments.append(seg)
                seg = [ordered[i]]
            else:
                seg.append(ordered[i])
        if len(seg) >= 2:
            segments.append(seg)
        return segments

    @staticmethod
    def _parse_color(class_id_str: str) -> int:
        try:
            c = int(class_id_str)
            return c if c in (0, 1, 2) else -1
        except (ValueError, TypeError):
            return -1


def main(args=None):
    rclpy.init(args=args)
    node = ConeMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
