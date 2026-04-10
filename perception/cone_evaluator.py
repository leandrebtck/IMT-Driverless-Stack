#!/usr/bin/env python3
"""
cone_evaluator.py — Comparaison cônes détectés vs cônes réels.

Ground truth : /testing_only/track  (fs_msgs/msg/Track, publié par le bridge FSDS)
Carte SLAM   : /slam_lidar/cone_map (ou /slam/cone_map)

Publications :
  /evaluation/stats   — String de métriques (lu par eval_dashboard.py)
  /evaluation/markers — MarkerArray (flèches d'erreur + GT en blanc dans RViz)
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config_loader import CFG

import math
import argparse
import subprocess
import threading
import yaml
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import Point as GPoint

try:
    from fs_msgs.msg import Track, Cone as FsCone
    FS_MSGS_OK = True
except ImportError:
    FS_MSGS_OK = False

MAP_FRAME = CFG['frames']['map']

LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

# fs_msgs Cone.color → notre convention (0=jaune, 1=bleu, 2=orange, -1=inconnu)
FS_COLOR = {0: 1, 1: 0, 2: 2, 3: 2, 4: -1}


def _error_color(err_m: float):
    t = min(err_m / 1.5, 1.0)
    return (t, 1.0 - t, 0.0)


class ConeEvaluatorNode(Node):

    def __init__(self, map_topic: str):
        super().__init__('cone_evaluator_node')

        self.gt_cones    = []   # [{x,y,z,color}] — rempli au 1er message track
        self._gt_pub_done = False

        # ── Ground truth via /testing_only/track ─────────────────────────────
        if FS_MSGS_OK:
            self.create_subscription(
                Track, CFG['topics']['ground_truth_track'],
                self._track_cb,
                QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE
                )
            )
            self.get_logger().info(f"Souscription à {CFG['topics']['ground_truth_track']} (fs_msgs/Track)")
        else:
            self.get_logger().error(
                "fs_msgs non disponible — sourcer le workspace FSDS :\n"
                "  source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash"
            )

        # ── Carte SLAM ────────────────────────────────────────────────────────
        self.create_subscription(
            MarkerArray, map_topic,
            self._map_cb,
            LATCHED_QOS
        )

        self.pub_viz  = self.create_publisher(MarkerArray, CFG['topics']['eval_markers'], LATCHED_QOS)
        self.pub_stat = self.create_publisher(String,      CFG['topics']['eval_stats'],   10)

        # Fallback : si le message GT a été publié avant notre démarrage (VOLATILE),
        # on le récupère via ros2 topic echo après 3 secondes.
        self._gt_fallback_timer = self.create_timer(3.0, self._gt_fallback_once)

        self.get_logger().info(f"Écoute {map_topic} | GT: /testing_only/track")

    # ── Réception ground truth ────────────────────────────────────────────────
    def _track_cb(self, msg: 'Track'):
        if self.gt_cones:   # on ne recharge qu'une fois
            return
        for cone in msg.track:
            self.gt_cones.append({
                'x':     cone.location.x,
                'y':     cone.location.y,
                'z':     cone.location.z,
                'color': FS_COLOR.get(cone.color, -1)
            })
        self.get_logger().info(
            f"✓ {len(self.gt_cones)} cônes ground truth reçus depuis /testing_only/track"
        )
        if not self._gt_pub_done:
            self._publish_gt_markers()
            self._gt_pub_done = True

    # ── Fallback GT (si message publié avant notre démarrage) ────────────────
    def _gt_fallback_once(self):
        """Timer one-shot : annule après la première exécution."""
        self._gt_fallback_timer.cancel()
        if self.gt_cones:
            return   # déjà reçu via subscription
        self.get_logger().info(
            "GT non reçu via subscription — tentative via ros2 topic echo…"
        )
        threading.Thread(target=self._fetch_gt_subprocess, daemon=True).start()

    def _fetch_gt_subprocess(self):
        """Lit le topic GT via sous-processus et peuple self.gt_cones."""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'echo', '--once',
                 CFG['topics']['ground_truth_track']],
                capture_output=True, text=True, timeout=15
            )
            if result.returncode != 0 or not result.stdout.strip():
                self.get_logger().error("ros2 topic echo GT : aucune réponse")
                return
            data = yaml.safe_load(result.stdout)
            if not isinstance(data, dict) or 'track' not in data:
                self.get_logger().error("Format GT inattendu")
                return
            for cone in data['track']:
                loc = cone.get('location', {})
                self.gt_cones.append({
                    'x':     float(loc.get('x', 0.0)),
                    'y':     float(loc.get('y', 0.0)),
                    'z':     float(loc.get('z', 0.0)),
                    'color': FS_COLOR.get(cone.get('color', 4), -1)
                })
            self.get_logger().info(
                f"✓ {len(self.gt_cones)} cônes GT récupérés via fallback"
            )
            if not self._gt_pub_done:
                self._publish_gt_markers()
                self._gt_pub_done = True
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                "Timeout fallback GT — le bridge FSDS est-il lancé ?"
            )
        except Exception as e:
            self.get_logger().error(f"Fallback GT échoué : {e}")

    # ── Marqueurs GT (blanc semi-transparent) ─────────────────────────────────
    def _publish_gt_markers(self):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for i, c in enumerate(self.gt_cones):
            mk = Marker()
            mk.header.frame_id    = MAP_FRAME
            mk.header.stamp       = now
            mk.ns                 = 'gt_cones'
            mk.id                 = i
            mk.type               = Marker.CYLINDER
            mk.action             = Marker.ADD
            mk.pose.position.x    = c['x']
            mk.pose.position.y    = c['y']
            mk.pose.position.z    = c['z']
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = 0.28
            mk.scale.z = 0.33
            mk.color.r = mk.color.g = mk.color.b = 1.0
            mk.color.a = 0.45
            mk.lifetime.sec = mk.lifetime.nanosec = 0
            arr.markers.append(mk)
        self.pub_viz.publish(arr)

    # ── Comparaison carte SLAM ↔ GT ───────────────────────────────────────────
    def _map_cb(self, msg: MarkerArray):
        # Extrait les cylindres (cônes détectés)
        detected = [
            {'x': mk.pose.position.x,
             'y': mk.pose.position.y,
             'z': mk.pose.position.z,
             'id': mk.id}
            for mk in msg.markers
            if mk.ns in ('cone_map', 'cone_map_lidar') and mk.type == Marker.CYLINDER
        ]

        n_det = len(detected)
        n_gt  = len(self.gt_cones)

        self.get_logger().info(
            f"Carte reçue : {n_det} cônes détectés | GT : {n_gt} cônes",
            throttle_duration_sec=2
        )

        stat = String()

        if not self.gt_cones or n_det == 0:
            stat.data = (
                f"Comparaison 0/{n_det} cônes | "
                f"moy=0.000m | RMSE=0.000m | min=0.000m max=0.000m | fantômes=0 | "
                f"GT={'en attente' if not self.gt_cones else 'ok'}"
            )
            self.pub_stat.publish(stat)
            return

        # ── Association détecté → GT le plus proche ───────────────────────────
        now    = self.get_clock().now().to_msg()
        arr    = MarkerArray()
        errors = []
        ghosts = 0

        for det in detected:
            best_gt, best_dist = None, float('inf')
            for gt in self.gt_cones:
                d = math.sqrt((det['x']-gt['x'])**2 + (det['y']-gt['y'])**2)
                if d < best_dist:
                    best_dist, best_gt = d, gt

            if best_dist > 3.0:
                ghosts += 1
                continue

            errors.append(best_dist)
            r, g, b = _error_color(best_dist)
            base_id = det['id'] * 10

            # Flèche détecté → réel
            arrow = Marker()
            arrow.header.frame_id    = MAP_FRAME
            arrow.header.stamp       = now
            arrow.ns                 = 'error_arrows'
            arrow.id                 = base_id
            arrow.type               = Marker.ARROW
            arrow.action             = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.scale.x = 0.05; arrow.scale.y = 0.10; arrow.scale.z = 0.10
            arrow.color.r = r; arrow.color.g = g; arrow.color.b = b; arrow.color.a = 0.9
            arrow.lifetime.sec = arrow.lifetime.nanosec = 0
            p0 = GPoint(); p0.x = det['x'];     p0.y = det['y'];     p0.z = 0.2
            p1 = GPoint(); p1.x = best_gt['x']; p1.y = best_gt['y']; p1.z = 0.2
            arrow.points = [p0, p1]
            arr.markers.append(arrow)

            # Texte erreur
            txt = Marker()
            txt.header.frame_id    = MAP_FRAME
            txt.header.stamp       = now
            txt.ns                 = 'error_labels'
            txt.id                 = base_id + 1
            txt.type               = Marker.TEXT_VIEW_FACING
            txt.action             = Marker.ADD
            txt.pose.position.x    = (det['x'] + best_gt['x']) / 2
            txt.pose.position.y    = (det['y'] + best_gt['y']) / 2
            txt.pose.position.z    = 0.7
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.30
            txt.color.r = r; txt.color.g = g; txt.color.b = b; txt.color.a = 1.0
            txt.text = f"{best_dist:.2f}m"
            txt.lifetime.sec = txt.lifetime.nanosec = 0
            arr.markers.append(txt)

        if arr.markers:
            self.pub_viz.publish(arr)

        # ── Stats ─────────────────────────────────────────────────────────────
        if errors:
            mean_e = sum(errors) / len(errors)
            rmse   = math.sqrt(sum(e**2 for e in errors) / len(errors))
            stat.data = (
                f"Comparaison {len(errors)}/{n_det} cônes | "
                f"moy={mean_e:.3f}m | "
                f"RMSE={rmse:.3f}m | "
                f"min={min(errors):.3f}m max={max(errors):.3f}m | "
                f"fantômes={ghosts}"
            )
        else:
            stat.data = (
                f"Comparaison 0/{n_det} cônes | "
                f"moy=0.000m | RMSE=0.000m | min=0.000m max=0.000m | fantômes={ghosts}"
            )

        self.pub_stat.publish(stat)
        self.get_logger().info(stat.data, throttle_duration_sec=2)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', default=CFG['topics']['slam_lidar_map'])
    parsed, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = ConeEvaluatorNode(map_topic=parsed.map)
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
