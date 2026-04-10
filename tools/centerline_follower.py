#!/usr/bin/env python3
"""
centerline_follower.py — Pilotage autonome : suit le milieu de la piste.

Stratégie :
  1. Construit un chemin de waypoints (midpoints jaune↔bleu) depuis la carte
     SLAM complète (tous les cônes mémorisés, pas seulement ceux visibles).
  2. Localise la voiture sur ce chemin et cible le waypoint à LOOKAHEAD_DIST
     mètres en avant.
  3. Si le chemin global est insuffisant (1er tour en cours), fallback sur les
     midpoints localement visibles.

  → En virage serré avec un seul côté visible localement, le chemin mémorisé
    du tour précédent guide la voiture sans s'arrêter.

Stratégie vitesse : impulsions (comme pilotage manuel Z bref).
  - Vitesse lue depuis l'odométrie
  - Throttle seulement si en dessous de SPEED_TARGET
  - Frein fort en virage serré, frein léger si trop vite
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config_loader import CFG

import math
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                        DurabilityPolicy, qos_profile_sensor_data)
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand

# ── QoS ───────────────────────────────────────────────────────────────────────
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# ── Pure Pursuit ──────────────────────────────────────────────────────────────
LOOKAHEAD_DIST  = 4.0   # m — distance cible sur le chemin
WHEELBASE       = 1.5   # m — empattement FSDS
MAX_STEER_RAD   = 0.5   # rad → normalisé ±1 FSDS
MIN_AHEAD       = 0.8   # m — distance min pour considérer un point "devant"

# ── Vitesse ───────────────────────────────────────────────────────────────────
SPEED_TARGET    = 1.5   # m/s — cible
SPEED_MAX       = 2.0   # m/s — max absolu
THROTTLE_PULSE  = 0.35  # impulsion gaz
BRAKE_LIGHT     = 0.15  # frein léger (trop vite)
BRAKE_CORNER    = 0.55  # frein fort (virage serré)
CORNER_STEER    = 0.45  # |steering| → virage

# ── Sécurité ──────────────────────────────────────────────────────────────────
LOST_TIMEOUT_S  = 3.0   # s sans waypoint devant → arrêt d'urgence

# ── Géométrie piste ───────────────────────────────────────────────────────────
MAX_TRACK_WIDTH = 9.0   # m — écart max jaune↔bleu valide
TRACK_WIDTH     = 6.0   # m — largeur nominale de piste Formula Student
MIN_WAYPOINTS   = 2     # nb de waypoints min pour activer le suivi de chemin

_CONE_NS = {'cone_map', 'cone_map_lidar'}


# ── Utilitaires ───────────────────────────────────────────────────────────────

def _yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _world_to_car(px, py, car_x, car_y, yaw):
    dx = px - car_x
    dy = py - car_y
    return (dx * math.cos(yaw) + dy * math.sin(yaw),
           -dx * math.sin(yaw) + dy * math.cos(yaw))


# ── Nœud principal ────────────────────────────────────────────────────────────

class CenterlineFollower(Node):

    def __init__(self, map_topic: str):
        super().__init__('centerline_follower')

        self.car_x      = 0.0
        self.car_y      = 0.0
        self.car_yaw    = 0.0
        self.speed      = 0.0
        self._odom_ok   = False
        self._last_target_t = self.get_clock().now()

        # Cônes bruts (depuis la carte SLAM complète)
        self.yellow_cones: list = []
        self.blue_cones:   list = []
        self.gray_cones:   list = []

        # Chemin mémorisé : liste ordonnée de midpoints (world frame)
        self._waypoints: list = []

        _t = CFG['topics']
        self.create_subscription(Odometry, _t['odometry'],
                                 self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, map_topic,
                                 self._map_cb, LATCHED_QOS)
        self.pub_cmd = self.create_publisher(
            ControlCommand, _t['control_command'], 10)

        self.create_timer(0.05, self._control_loop)   # 20 Hz

        self.get_logger().info(
            f"CenterlineFollower — carte: {map_topic}  "
            f"target={SPEED_TARGET} m/s  lookahead={LOOKAHEAD_DIST} m")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        self.car_x   = p.position.x
        self.car_y   = p.position.y
        self.car_yaw = _yaw_from_quaternion(p.orientation)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed   = math.hypot(vx, vy)
        self._odom_ok = True

    def _map_cb(self, msg: MarkerArray):
        yellow, blue, gray = [], [], []
        for mk in msg.markers:
            if mk.ns not in _CONE_NS or mk.type != Marker.CYLINDER:
                continue
            r, g, b = mk.color.r, mk.color.g, mk.color.b
            pos = (mk.pose.position.x, mk.pose.position.y)
            if   r > 0.8 and g > 0.8 and b < 0.2:   yellow.append(pos)
            elif b > 0.7 and r < 0.3:                blue.append(pos)
            else:                                     gray.append(pos)

        # Fallback géométrique si LiDAR sans couleurs (tous gris)
        if not yellow and not blue and gray and self._odom_ok:
            for (px, py) in gray:
                xc, yc = _world_to_car(px, py, self.car_x, self.car_y, self.car_yaw)
                if xc > 0:
                    (blue if yc > 0 else yellow).append((px, py))

        self.yellow_cones = yellow
        self.blue_cones   = blue
        self.gray_cones   = gray

        # Reconstruction du chemin complet depuis la carte mémorisée
        mids = self._all_midpoints(yellow, blue)
        if len(mids) >= MIN_WAYPOINTS:
            self._waypoints = self._order_path(mids)

    # ── Calcul des midpoints ──────────────────────────────────────────────────

    def _all_midpoints(self, yellow, blue) -> list:
        """
        Midpoints entre TOUS les cônes jaunes et bleus stockés dans la carte.
        Chaque jaune est apparié au bleu le plus proche (< MAX_TRACK_WIDTH).
        """
        pts = []
        if not yellow or not blue:
            return pts
        for (yx, yy) in yellow:
            bd, bb = MAX_TRACK_WIDTH, None
            for (bx, by) in blue:
                d = math.hypot(yx - bx, yy - by)
                if d < bd:
                    bd, bb = d, (bx, by)
            if bb:
                pts.append(((yx + bb[0]) * 0.5, (yy + bb[1]) * 0.5))
        return pts

    def _order_path(self, midpoints: list) -> list:
        """
        Ordonne les midpoints en chemin circulaire par angle depuis leur centroïde.
        Fonctionne pour les circuits fermés ou semi-fermés (≥ 4 midpoints).
        """
        cx = sum(p[0] for p in midpoints) / len(midpoints)
        cy = sum(p[1] for p in midpoints) / len(midpoints)
        return sorted(midpoints, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))

    # ── Suivi de chemin ───────────────────────────────────────────────────────

    def _find_target(self) -> tuple | None:
        """
        Localise la voiture sur le chemin mémorisé et retourne le waypoint
        situé à LOOKAHEAD_DIST mètres en avant sur ce chemin.
        Retourne None si le chemin est vide.
        """
        path = self._waypoints
        n = len(path)
        if n == 0:
            return None

        # 1. Waypoint le plus proche de la voiture
        closest_idx = min(range(n),
                          key=lambda i: math.hypot(
                              path[i][0] - self.car_x,
                              path[i][1] - self.car_y))

        # 2. Direction de marche : comparer les deux voisins dans le repère voiture
        next_fwd = (closest_idx + 1) % n
        next_bwd = (closest_idx - 1) % n
        xc_fwd, _ = _world_to_car(path[next_fwd][0], path[next_fwd][1],
                                   self.car_x, self.car_y, self.car_yaw)
        xc_bwd, _ = _world_to_car(path[next_bwd][0], path[next_bwd][1],
                                   self.car_x, self.car_y, self.car_yaw)
        direction = 1 if xc_fwd >= xc_bwd else -1

        # 3. Marcher le long du chemin jusqu'à LOOKAHEAD_DIST cumulé
        accumulated = 0.0
        idx = closest_idx
        for _ in range(n):
            next_idx = (idx + direction) % n
            step = math.hypot(path[next_idx][0] - path[idx][0],
                              path[next_idx][1] - path[idx][1])
            accumulated += step
            idx = next_idx
            if accumulated >= LOOKAHEAD_DIST:
                return path[idx]

        # Chemin trop court → retourner le point opposé sur le cercle
        return path[(closest_idx + direction * max(1, n // 2)) % n]

    def _find_local_target(self) -> tuple | None:
        """
        Fallback local (1er tour ou carte insuffisante).
        Calcule la cible en repère voiture (toujours frais, pas de cache).

        Cas 1 — les deux côtés visibles : midpoint jaune↔bleu classique.
        Cas 2 — un seul côté visible : cible décalée vers le centre estimé
                 (TRACK_WIDTH/2 du côté manquant), sans créer de cônes virtuels
                 en mémoire (évite les données périmées quand la voiture avance).
        """
        yellow = self.yellow_cones
        blue   = self.blue_cones

        cx, cy, yaw = self.car_x, self.car_y, self.car_yaw

        # ── Cas 1 : les deux côtés disponibles ────────────────────────────────
        if yellow and blue:
            mids = self._all_midpoints(yellow, blue)
            ahead = []
            for (mx, my) in mids:
                xc, yc = _world_to_car(mx, my, cx, cy, yaw)
                if xc >= MIN_AHEAD:
                    ahead.append((math.hypot(xc, yc), xc, yc))
            if ahead:
                ahead.sort(key=lambda t: abs(t[0] - LOOKAHEAD_DIST))
                return (ahead[0][1], ahead[0][2])

        # ── Cas 2 : un seul côté — décalage vers le centre estimé ─────────────
        # Repère voiture : Y > 0 = gauche (bleu), Y < 0 = droite (jaune)
        # Si seulement jaune (droite) visible → centre à +TRACK_WIDTH/2 sur la gauche
        # Si seulement bleu  (gauche) visible → centre à -TRACK_WIDTH/2 sur la droite
        one_side = []
        y_offset = 0.0
        if yellow and not blue:
            one_side = yellow
            y_offset = +TRACK_WIDTH / 2.0   # décaler vers gauche depuis le jaune
        elif blue and not yellow:
            one_side = blue
            y_offset = -TRACK_WIDTH / 2.0   # décaler vers droite depuis le bleu

        if one_side:
            candidates = []
            for (px, py) in one_side:
                xc, yc = _world_to_car(px, py, cx, cy, yaw)
                if xc >= MIN_AHEAD:
                    # Cible = position du cône décalée vers le centre estimé
                    candidates.append((math.hypot(xc, yc), xc, yc + y_offset))
            if candidates:
                candidates.sort(key=lambda t: abs(t[0] - LOOKAHEAD_DIST))
                return (candidates[0][1], candidates[0][2])

        return None

    # ── Boucle de contrôle ────────────────────────────────────────────────────

    def _control_loop(self):
        cmd = ControlCommand()

        if not self._odom_ok:
            cmd.brake = 1.0
            self.pub_cmd.publish(cmd)
            return

        # ── Recherche de la cible ──────────────────────────────────────────────
        target_world = self._find_target()          # chemin mémorisé complet
        in_car_frame = False

        if target_world is not None:
            # Cible en repère monde → repère voiture
            tx, ty = _world_to_car(target_world[0], target_world[1],
                                   self.car_x, self.car_y, self.car_yaw)
            # Valider que la cible est globalement devant
            if tx < MIN_AHEAD * 0.5:
                target_world = None     # la cible est derrière → fallback

        if target_world is None:
            # Fallback : midpoints locaux (1er tour, chemin pas encore construit)
            local = self._find_local_target()
            if local is None:
                dt_lost = (self.get_clock().now() - self._last_target_t).nanoseconds * 1e-9
                if dt_lost > LOST_TIMEOUT_S:
                    cmd.brake = 1.0
                    self.get_logger().warn(
                        f"Piste perdue {dt_lost:.1f}s — ARRÊT",
                        throttle_duration_sec=1.0)
                else:
                    # Aucune cible mais délai pas dépassé : avancer lentement tout droit
                    # pour trouver de nouveaux cônes plutôt que bloquer sur place
                    if self.speed < 0.8:
                        cmd.throttle = THROTTLE_PULSE
                    cmd.steering = 0.0
                    self.get_logger().info(
                        f"Aucun cône visible — avance droit "
                        f"(J={len(self.yellow_cones)} B={len(self.blue_cones)} "
                        f"wp={len(self._waypoints)})",
                        throttle_duration_sec=1.0)
                self.pub_cmd.publish(cmd)
                return
            tx, ty = local
            in_car_frame = True
        else:
            in_car_frame = False

        self._last_target_t = self.get_clock().now()

        # ── Pure Pursuit ──────────────────────────────────────────────────────
        if not in_car_frame:
            tx, ty = _world_to_car(target_world[0], target_world[1],
                                   self.car_x, self.car_y, self.car_yaw)

        alpha     = math.atan2(ty, tx)
        ld        = math.hypot(tx, ty)
        steer_rad = math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld)
        steering  = max(-1.0, min(1.0, -steer_rad / MAX_STEER_RAD))

        # ── Vitesse (impulsions, jamais continu) ──────────────────────────────
        in_corner = abs(steering) > CORNER_STEER
        if in_corner:
            cmd.throttle = 0.0
            cmd.brake    = BRAKE_CORNER
        elif self.speed > SPEED_MAX:
            cmd.throttle = 0.0
            cmd.brake    = BRAKE_LIGHT
        elif self.speed < SPEED_TARGET:
            cmd.throttle = THROTTLE_PULSE
            cmd.brake    = 0.0
        else:
            cmd.throttle = 0.0
            cmd.brake    = 0.0

        cmd.steering = float(steering)
        self.pub_cmd.publish(cmd)

        mode = "PATH" if not in_car_frame else "LOCAL"
        self.get_logger().info(
            f"[{mode}] v={self.speed:.2f}m/s  steer={steering:+.2f}  "
            f"target=({tx:.1f},{ty:+.1f})  "
            f"{'CORNER' if in_corner else 'gaz' if cmd.throttle > 0 else 'inertie' if cmd.brake == 0 else 'frein'}  "
            f"waypoints={len(self._waypoints)}",
            throttle_duration_sec=0.5)


# ── Main ──────────────────────────────────────────────────────────────────────

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', default=CFG['topics']['slam_lidar_map'],
                        help='Topic MarkerArray de la carte SLAM')
    parsed, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = CenterlineFollower(map_topic=parsed.map)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = ControlCommand()
        stop.brake = 1.0
        node.pub_cmd.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
