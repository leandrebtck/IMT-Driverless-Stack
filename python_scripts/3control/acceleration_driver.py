#!/usr/bin/env python3
"""
acceleration_driver.py — Pilotage autonome épreuve d'accélération Formula Student.

Règle FSG : ligne droite de 75 m, départ arrêté, franchir la ligne d'arrivée le plus
vite possible puis s'arrêter avant la zone de sécurité.

Stratégie :
  1. ACCEL    — plein gaz, correction de cap pour rester droit (P sur le yaw).
  2. BRAKE    — freinage maximal dès que la distance franchie dépasse BRAKE_DIST
                ou que des cônes orange sont détectés à moins de ORANGE_STOP_DIST.
  3. STOP     — maintien frein, arrêt complet.

Paramètres clés :
  --dist     Distance totale de la zone d'accélération (défaut 75 m)
  --brake    Distance à partir de laquelle freiner avant la fin (défaut 10 m)
  --map      Topic MarkerArray SLAM (pour détection cônes orange à l'arrivée)
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from config_loader import CFG

import math
import enum
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                        DurabilityPolicy, qos_profile_sensor_data)
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand

LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# ── Distances ─────────────────────────────────────────────────────────────────
ACCEL_DIST       = 75.0   # m — longueur totale épreuve FSG
BRAKE_MARGIN     = 10.0   # m — freiner BRAKE_MARGIN mètres avant ACCEL_DIST
ORANGE_STOP_DIST  = 8.0    # m — arrêt si cônes orange détectés à cette distance
ORANGE_MIN_DIST   = 40.0   # m — ne pas chercher les orange avant cette distance (évite les cônes de départ)

# ── Vitesse / commandes ────────────────────────────────────────────────────────
THROTTLE_FULL   = 1.0     # plein gaz
BRAKE_FULL      = 1.0     # freinage maximal
STOP_SPEED      = 0.3     # m/s — en dessous : considéré arrêté

# ── Correction de cap ─────────────────────────────────────────────────────────
KP_YAW          = 0.8     # gain proportionnel yaw (rad d'écart → steering)
MAX_STEER_RAD   = 0.5     # rad → normalisé ±1 FSDS
WHEELBASE       = 1.5     # m

_CONE_NS = {'cone_map', 'cone_map_lidar',
            'cone_map_skidpad_lidar', 'cone_map_skidpad_stereo'}


class State(enum.Enum):
    ACCEL = 0
    BRAKE = 1
    STOP  = 2


def _yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _world_to_car(px, py, car_x, car_y, yaw):
    dx, dy = px - car_x, py - car_y
    return (dx * math.cos(yaw) + dy * math.sin(yaw),
           -dx * math.sin(yaw) + dy * math.cos(yaw))


def _angle_diff(a, b) -> float:
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


class AccelerationDriver(Node):

    def __init__(self, accel_dist: float, brake_margin: float, map_topic: str):
        super().__init__('acceleration_driver')

        self._accel_dist   = accel_dist
        self._brake_dist   = accel_dist - brake_margin  # distance à partir de laquelle freiner

        self.car_x   = 0.0
        self.car_y   = 0.0
        self.car_yaw = 0.0
        self.speed   = 0.0
        self._odom_ok     = False
        self._start_x: float | None = None
        self._start_y: float | None = None
        self._start_yaw: float | None = None   # cap initial à maintenir

        self._state = State.ACCEL
        self._orange_cones: list = []

        _t = CFG['topics']
        self.create_subscription(Odometry, _t['odometry'],
                                 self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, map_topic,
                                 self._map_cb, LATCHED_QOS)
        self.pub_cmd = self.create_publisher(
            ControlCommand, _t['control_command'], 10)

        self.create_timer(0.02, self._control_loop)   # 50 Hz
        self.get_logger().info(
            f"AccelerationDriver — dist={accel_dist}m  "
            f"frein à {self._brake_dist:.1f}m  map={map_topic}")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        self.car_x   = p.position.x
        self.car_y   = p.position.y
        self.car_yaw = _yaw_from_quaternion(p.orientation)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed   = math.hypot(vx, vy)
        if not self._odom_ok:
            self._start_x   = self.car_x
            self._start_y   = self.car_y
            self._start_yaw = self.car_yaw
            self._odom_ok   = True

    def _map_cb(self, msg: MarkerArray):
        """Collecte uniquement les cônes orange."""
        orange = []
        for mk in msg.markers:
            if mk.ns not in _CONE_NS or mk.type != Marker.CYLINDER:
                continue
            r, g, b = mk.color.r, mk.color.g, mk.color.b
            if r > 0.8 and 0.2 < g < 0.7 and b < 0.2:
                orange.append((mk.pose.position.x, mk.pose.position.y))
        self._orange_cones = orange

    # ── Détection cônes orange en avant ───────────────────────────────────────

    def _orange_dist_ahead(self) -> float | None:
        best = None
        for (px, py) in self._orange_cones:
            xc, yc = _world_to_car(px, py, self.car_x, self.car_y, self.car_yaw)
            if xc < 0.5 or abs(yc) > 5.0:
                continue
            d = math.hypot(xc, yc)
            if best is None or d < best:
                best = d
        return best

    # ── Correction de cap ─────────────────────────────────────────────────────

    def _heading_steering(self) -> float:
        """Steering proportionnel à l'écart entre le cap actuel et le cap initial."""
        if self._start_yaw is None:
            return 0.0
        err_yaw = _angle_diff(self._start_yaw, self.car_yaw)
        steer_rad = KP_YAW * err_yaw
        steer_rad = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, steer_rad))
        return -steer_rad / MAX_STEER_RAD   # normalisé ±1

    # ── Boucle de contrôle ────────────────────────────────────────────────────

    def _control_loop(self):
        cmd = ControlCommand()

        if not self._odom_ok:
            cmd.throttle = 0.3
            self.pub_cmd.publish(cmd)
            return

        dist = math.hypot(self.car_x - self._start_x,
                          self.car_y - self._start_y)

        # ── Transitions d'état ────────────────────────────────────────────────
        if self._state == State.ACCEL:
            if dist >= self._brake_dist:
                self._state = State.BRAKE
                self.get_logger().info(
                    f"ACCEL → BRAKE  (dist={dist:.1f}m, v={self.speed:.2f}m/s)")
            elif dist >= ORANGE_MIN_DIST:
                od = self._orange_dist_ahead()
                if od is not None and od < ORANGE_STOP_DIST:
                    self._state = State.BRAKE
                    self.get_logger().info(
                        f"ACCEL → BRAKE  (orange à {od:.1f}m, dist={dist:.1f}m)")

        if self._state == State.BRAKE:
            if self.speed < STOP_SPEED:
                self._state = State.STOP
                self.get_logger().info(
                    f"BRAKE → STOP  (v={self.speed:.2f}m/s, dist={dist:.1f}m)")

        # ── Commandes ─────────────────────────────────────────────────────────
        if self._state == State.STOP:
            cmd.throttle = 0.0
            cmd.brake    = BRAKE_FULL
            cmd.steering = 0.0
            self.pub_cmd.publish(cmd)
            self.get_logger().info(
                f"Arrêt complet — dist={dist:.1f}m", throttle_duration_sec=2.0)
            return

        if self._state == State.BRAKE:
            cmd.throttle = 0.0
            cmd.brake    = BRAKE_FULL
            cmd.steering = 0.0
            self.pub_cmd.publish(cmd)
            self.get_logger().info(
                f"[BRAKE] v={self.speed:.2f}m/s  dist={dist:.1f}m",
                throttle_duration_sec=0.2)
            return

        # ACCEL
        steering = self._heading_steering()
        cmd.throttle = THROTTLE_FULL
        cmd.brake    = 0.0
        cmd.steering = float(steering)
        self.pub_cmd.publish(cmd)
        self.get_logger().info(
            f"[ACCEL] v={self.speed:.2f}m/s  dist={dist:.1f}/{self._brake_dist:.0f}m  "
            f"steer={steering:+.2f}  yaw_err={math.degrees(_angle_diff(self._start_yaw, self.car_yaw)):.1f}°",
            throttle_duration_sec=0.2)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dist',  type=float, default=ACCEL_DIST,
                        help=f'Distance totale épreuve (défaut: {ACCEL_DIST}m)')
    parser.add_argument('--brake', type=float, default=BRAKE_MARGIN,
                        help=f'Marge de freinage avant la fin (défaut: {BRAKE_MARGIN}m)')
    parser.add_argument('--map',   default='/slam_lidar/cone_map',
                        help='Topic MarkerArray SLAM (pour cônes orange arrivée)')
    parsed, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = AccelerationDriver(
        accel_dist=parsed.dist,
        brake_margin=parsed.brake,
        map_topic=parsed.map,
    )
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
