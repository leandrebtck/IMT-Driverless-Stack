#!/usr/bin/env python3
"""
cone_mapper_lidar.py — SLAM LiDAR : cartographie des cônes en temps réel.
Compatible ROS Galactic et ROS Iron.

Utilise les détections publiées par yolo_lidar.py (positions 3D déjà filtrées
par Kalman dans le repère fsds/Lidar1) et le TF publié par odom_tf_publisher.py
pour construire une carte globale persistante dans fsds/map.

Différences vs cone_mapper.py (version stéréo) :
  - Source       : /perception/lidar_detections  (au lieu de stereo_detections)
  - Repère source: fsds/Lidar1                   (au lieu de fsds/cam1)
  - Profondeur   : distance horizontale sqrt(x²+y²) (LiDAR mesure en XY)
  - MAX_DEPTH    : 8 m (LiDAR plus fiable que stéréo à longue distance)
  - Publications : /slam_lidar/cone_map et /slam_lidar/stats
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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

LIDAR_FRAME = CFG['frames']['lidar']
MAP_FRAME   = CFG['frames']['map']

COLORS_RGB = {
    0:  (1.0, 1.0, 0.0),
    1:  (0.0, 0.2, 1.0),
    2:  (1.0, 0.4, 0.0),
    -1: (0.5, 0.5, 0.5),
}
COLOR_NAME = {0: 'JAUNE', 1: 'BLEU', 2: 'ORANGE', -1: '?'}
COLOR_PRIO = {2: 3, 0: 2, 1: 2, -1: 0}


class ConeMapperLidarNode(Node):

    MERGE_DIST         = CFG['slam']['lidar']['merge_distance_m']
    COUNT_CAP          = CFG['slam']['count_cap']
    PUBLISH_HZ         = CFG['slam']['publish_hz']
    MIN_COUNT          = CFG['slam']['min_count']
    MIN_SCORE          = 0.0

    MAX_DEPTH_STRAIGHT = CFG['slam']['lidar']['max_depth_straight_m']
    MAX_DEPTH_TURN     = CFG['slam']['lidar']['max_depth_turn_m']
    # LiDAR est fiable même en virage → on n'utilise pas le blocage de virage
    TURN_YAW_RATE      = CFG['slam']['turn_yaw_rate_threshold']

    MAX_LINE_STEP      = CFG['slam']['max_line_step_m']
    LINE_STRIDE        = CFG['slam']['line_stride_lidar']

    def __init__(self):
        super().__init__('cone_mapper_lidar_node')

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cones    = {}
        self.next_id  = 0
        self.yaw_rate = 0.0

        _t = CFG['topics']
        self.create_subscription(
            Detection2DArray, _t['lidar_detections'],
            self._detections_cb, 10)
        self.create_subscription(
            Odometry, _t['odometry'],
            self._odom_cb, qos_profile_sensor_data)

        self.pub_map  = self.create_publisher(MarkerArray, _t['slam_lidar_map'],   LATCHED_QOS)
        self.pub_stat = self.create_publisher(String,      _t['slam_lidar_stats'], 10)

        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_map)

        self.get_logger().info(
            f"ConeMapperLidarNode prêt — merge={self.MERGE_DIST}m "
            f"max_depth={self.MAX_DEPTH_STRAIGHT}m step={self.MAX_LINE_STEP}m"
        )

    def _odom_cb(self, msg: Odometry):
        self.yaw_rate = msg.twist.twist.angular.z

    def _detections_cb(self, msg: Detection2DArray):
        if not msg.detections:
            return

        # LiDAR : positions fiables même en virage → on n'utilise pas le blocage.
        # On garde uniquement la réduction de profondeur en virage (mesures plus incertaines).
        turning   = False   # jamais bloqué pour LiDAR
        max_depth = self.MAX_DEPTH_STRAIGHT

        # TF : on utilise toujours le transform le plus récent disponible.
        # Le timestamp exact échoue souvent (décalage LiDAR/TF) ; le TF "latest"
        # introduit une erreur position ≈ vitesse × latence (~3 cm à 1.5 m/s) — acceptable.
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, LIDAR_FRAME, rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception as e:
            self.get_logger().warning(f"TF non prêt : {e}", throttle_duration_sec=2)
            return

        for det in msg.detections:
            if not det.results:
                continue
            hyp   = det.results[0]
            score = hyp.hypothesis.score

            if abs(score) < self.MIN_SCORE:
                continue

            # Profondeur = distance horizontale dans le repère LiDAR
            lx = hyp.pose.pose.position.x
            ly = hyp.pose.pose.position.y
            depth = math.sqrt(lx * lx + ly * ly)

            if depth <= 0.1 or depth > max_depth:
                continue

            color = self._parse_color(hyp.hypothesis.class_id)

            p = PointStamped()
            p.header.frame_id = LIDAR_FRAME
            p.header.stamp    = msg.header.stamp
            p.point.x = hyp.pose.pose.position.x
            p.point.y = hyp.pose.pose.position.y
            p.point.z = hyp.pose.pose.position.z

            try:
                pw = tf2_geometry_msgs.do_transform_point(p, tf)
            except Exception:
                continue

            wx, wy, wz = pw.point.x, pw.point.y, pw.point.z

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
                if turning:
                    continue
                self.cones[self.next_id] = {
                    'x': wx, 'y': wy, 'z': wz,
                    'color': color, 'count': 1,
                }
                self.get_logger().info(
                    f"Cône #{self.next_id} ({COLOR_NAME.get(color,'?')}) "
                    f"à ({wx:.1f}, {wy:.1f})")
                self.next_id += 1

    def _publish_map(self):
        now          = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        by_color     = {0: [], 1: [], 2: []}   # liste de (cid, x, y, z)

        for cid, cone in self.cones.items():
            if cone['count'] < self.MIN_COUNT:
                continue
            r, g, b = COLORS_RGB.get(cone['color'], (0.5, 0.5, 0.5))

            mk = Marker()
            mk.header.frame_id    = MAP_FRAME
            mk.header.stamp       = now
            mk.ns                 = 'cone_map_lidar'
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
            txt.ns                = 'cone_labels_lidar'
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
                by_color[cone['color']].append((cid, cone['x'], cone['y'], cone['z']))

        # Orange ignoré : positions trop instables (cônes de départ/arrivée peu nombreux)
        line_cfg = {
            0: ('track_yellow_l', 20000, (1.0, 1.0, 0.0)),
            1: ('track_blue_l',   21000, (0.0, 0.4, 1.0)),
        }
        for color_id, (ns, base_id, (r, g, b)) in line_cfg.items():
            entries = by_color[color_id]
            if len(entries) < 2:
                continue
            # Tri par ID = ordre de détection = ordre de conduite (stable quelle que
            # soit la position du centroïde → les premières lignes ne s'effacent plus)
            entries.sort(key=lambda e: e[0])
            pts = [(x, y, z) for (_, x, y, z) in entries]
            segments = self._gap_segments(pts, self.MAX_LINE_STEP)
            for seg_idx, seg in enumerate(segments):
                ctrl   = seg[::self.LINE_STRIDE]
                if seg[-1] != ctrl[-1]:
                    ctrl = ctrl + [seg[-1]]
                smooth = self._catmull_rom(ctrl, steps=15)
                ln = Marker()
                ln.header.frame_id    = MAP_FRAME
                ln.header.stamp       = now
                ln.ns                 = ns
                ln.id                 = base_id + seg_idx
                ln.type               = Marker.LINE_STRIP
                ln.action             = Marker.ADD
                ln.pose.orientation.w = 1.0
                ln.scale.x            = 0.10
                ln.color.r, ln.color.g, ln.color.b = r, g, b
                ln.color.a            = 0.85
                ln.lifetime.sec = ln.lifetime.nanosec = 0
                for (x, y, z) in smooth:
                    gp = GPoint(); gp.x = x; gp.y = y; gp.z = z
                    ln.points.append(gp)
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

    @staticmethod
    def _catmull_rom(pts, steps=15):
        if len(pts) < 2:
            return pts
        ext = [pts[0]] + list(pts) + [pts[-1]]
        result = []
        for i in range(1, len(ext) - 2):
            p0, p1, p2, p3 = ext[i-1], ext[i], ext[i+1], ext[i+2]
            for s in range(steps):
                t  = s / steps
                t2 = t * t
                t3 = t2 * t
                def cr(a, b, c, d):
                    return 0.5*(2*b+(-a+c)*t+(2*a-5*b+4*c-d)*t2+(-a+3*b-3*c+d)*t3)
                result.append((cr(p0[0],p1[0],p2[0],p3[0]),
                               cr(p0[1],p1[1],p2[1],p3[1]),
                               cr(p0[2],p1[2],p2[2],p3[2])))
        result.append(pts[-1])
        return result

    @staticmethod
    def _gap_segments(pts, max_step):
        """Découpe une liste de points (déjà triés par ordre de conduite) en
        segments continus. Coupe uniquement si le gap entre deux points
        consécutifs dépasse max_step (cône manquant ou fin de section)."""
        if len(pts) < 2:
            return [pts] if len(pts) == 1 else []
        segments = []
        seg      = [pts[0]]
        for i in range(1, len(pts)):
            dx = pts[i][0] - pts[i-1][0]
            dy = pts[i][1] - pts[i-1][1]
            if math.sqrt(dx*dx + dy*dy) > max_step:
                if len(seg) >= 2:
                    segments.append(seg)
                seg = [pts[i]]
            else:
                seg.append(pts[i])
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
    node = ConeMapperLidarNode()
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
