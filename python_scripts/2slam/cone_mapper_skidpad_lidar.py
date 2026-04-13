#!/usr/bin/env python3
"""
cone_mapper_skidpad_lidar.py — SLAM LiDAR dédié Skidpad.

Différences vs cone_mapper_lidar.py :
  - Jamais de blocage en virage (le Skidpad est intégralement composé de virages)
  - Pas de lignes latérales jaune/bleu
  - Lignes centerline : pour chaque cône jaune, trait vers son cône bleu le plus
    proche + sphère blanche au midpoint → visualise directement la trajectoire
  - Publie sur /slam_skidpad_lidar/cone_map
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

LIDAR_FRAME = CFG['frames']['lidar']
MAP_FRAME   = CFG['frames']['map']

MAP_TOPIC   = '/slam_skidpad_lidar/cone_map'
STAT_TOPIC  = '/slam_skidpad_lidar/stats'
MARKER_NS   = 'cone_map_skidpad_lidar'

MAX_PAIR_DIST = 14.0   # m — distance max jaune↔bleu pour former une paire centerline

COLORS_RGB = {
    0:  (1.0, 1.0, 0.0),
    1:  (0.0, 0.2, 1.0),
    2:  (1.0, 0.4, 0.0),
    -1: (0.5, 0.5, 0.5),
}
COLOR_NAME = {0: 'JAUNE', 1: 'BLEU', 2: 'ORANGE', -1: '?'}
COLOR_PRIO = {2: 3, 0: 2, 1: 2, -1: 0}


class ConeMapperSkidpadLidarNode(Node):

    MERGE_DIST  = CFG['slam']['lidar']['merge_distance_m']
    COUNT_CAP   = CFG['slam']['count_cap']
    PUBLISH_HZ  = CFG['slam']['publish_hz']
    MIN_COUNT   = CFG['slam']['min_count']
    MAX_DEPTH   = CFG['slam']['lidar']['max_depth_straight_m']

    def __init__(self):
        super().__init__('cone_mapper_skidpad_lidar')

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cones   = {}
        self.next_id = 0

        _t = CFG['topics']
        self.create_subscription(
            Detection2DArray, _t['lidar_detections'],
            self._detections_cb, 10)
        self.create_subscription(
            Odometry, _t['odometry'],
            self._odom_cb, qos_profile_sensor_data)

        self.pub_map  = self.create_publisher(MarkerArray, MAP_TOPIC,  LATCHED_QOS)
        self.pub_stat = self.create_publisher(String,      STAT_TOPIC, 10)

        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_map)
        self.get_logger().info(
            f"ConeMapperSkidpadLidar — merge={self.MERGE_DIST}m  "
            f"max_depth={self.MAX_DEPTH}m → {MAP_TOPIC}")

    def _odom_cb(self, msg: Odometry):
        pass   # pas de blocage virage pour le Skidpad

    def _detections_cb(self, msg: Detection2DArray):
        if not msg.detections:
            return
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
            lx    = hyp.pose.pose.position.x
            ly    = hyp.pose.pose.position.y
            depth = math.sqrt(lx * lx + ly * ly)
            if depth <= 0.1 or depth > self.MAX_DEPTH:
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
                self.cones[self.next_id] = {
                    'x': wx, 'y': wy, 'z': wz,
                    'color': color, 'count': 1,
                }
                self.next_id += 1

    def _publish_map(self):
        now          = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        yellow, blue = [], []

        for cid, cone in self.cones.items():
            if cone['count'] < self.MIN_COUNT:
                continue
            r, g, b = COLORS_RGB.get(cone['color'], (0.5, 0.5, 0.5))

            # Cylindre cône
            mk = Marker()
            mk.header.frame_id    = MAP_FRAME
            mk.header.stamp       = now
            mk.ns                 = MARKER_NS
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
            mk.color.a = 1.0
            mk.lifetime.sec = mk.lifetime.nanosec = 0
            marker_array.markers.append(mk)

            if cone['color'] == 0:
                yellow.append((cid, cone['x'], cone['y'], cone['z']))
            elif cone['color'] == 1:
                blue.append((cid, cone['x'], cone['y'], cone['z']))

        # ── Lignes centerline : jaune ↔ bleu + midpoint ───────────────────────
        self._add_centerline_pairs(yellow, blue, now, marker_array)

        self.pub_map.publish(marker_array)

        counts = {}
        for cone in self.cones.values():
            counts[cone['color']] = counts.get(cone['color'], 0) + 1
        stat = String()
        stat.data = (f"{len(self.cones)} cônes | "
                     f"J={counts.get(0,0)} B={counts.get(1,0)} O={counts.get(2,0)}")
        self.pub_stat.publish(stat)

    def _add_centerline_pairs(self, yellow, blue, now, marker_array):
        """
        Pour chaque cône jaune, trace un trait vers son cône bleu le plus proche
        (si distance < MAX_PAIR_DIST) et ajoute une sphère blanche au midpoint.
        Visualise directement la centerline de conduite.
        """
        if not yellow or not blue:
            return
        for i, (ycid, yx, yy, yz) in enumerate(yellow):
            best_d, best_b = MAX_PAIR_DIST, None
            for (bcid, bx, by, bz) in blue:
                d = math.hypot(yx - bx, yy - by)
                if d < best_d:
                    best_d, best_b = d, (bx, by, bz)
            if best_b is None:
                continue
            bx, by, bz = best_b

            # Trait jaune↔bleu
            ln = Marker()
            ln.header.frame_id    = MAP_FRAME
            ln.header.stamp       = now
            ln.ns                 = 'centerline_pairs'
            ln.id                 = 30000 + i
            ln.type               = Marker.LINE_STRIP
            ln.action             = Marker.ADD
            ln.pose.orientation.w = 1.0
            ln.scale.x            = 0.07
            ln.color.r, ln.color.g, ln.color.b = 0.0, 1.0, 0.5
            ln.color.a            = 0.75
            ln.lifetime.sec = ln.lifetime.nanosec = 0
            p1 = GPoint(); p1.x, p1.y, p1.z = yx, yy, yz
            p2 = GPoint(); p2.x, p2.y, p2.z = bx, by, bz
            ln.points.extend([p1, p2])
            marker_array.markers.append(ln)

            # Sphère blanche au midpoint
            mx, my, mz = (yx + bx) * 0.5, (yy + by) * 0.5, (yz + bz) * 0.5
            sp = Marker()
            sp.header.frame_id    = MAP_FRAME
            sp.header.stamp       = now
            sp.ns                 = 'centerline_midpoints'
            sp.id                 = 31000 + i
            sp.type               = Marker.SPHERE
            sp.action             = Marker.ADD
            sp.pose.position.x    = mx
            sp.pose.position.y    = my
            sp.pose.position.z    = mz
            sp.pose.orientation.w = 1.0
            sp.scale.x = sp.scale.y = sp.scale.z = 0.25
            sp.color.r = sp.color.g = sp.color.b = 1.0
            sp.color.a            = 0.9
            sp.lifetime.sec = sp.lifetime.nanosec = 0
            marker_array.markers.append(sp)

    @staticmethod
    def _parse_color(class_id_str: str) -> int:
        try:
            c = int(class_id_str)
            return c if c in (0, 1, 2) else -1
        except (ValueError, TypeError):
            return -1


def main(args=None):
    rclpy.init(args=args)
    node = ConeMapperSkidpadLidarNode()
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
