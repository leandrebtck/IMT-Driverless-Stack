#!/usr/bin/env python3
"""
skidpad_driver.py — Pilotage autonome Skidpad Formula Student.

Stratégie (carte connue → géométrie pure, zéro dépendance au SLAM) :
  1. STRAIGHT  — avance jusqu'à STRAIGHT_DIST, calcule les centres analytiquement.
                 centre_droit = pos + R * (sin(yaw), -cos(yaw))
  2. RIGHT_1/2 — Pure Pursuit sur arc géométrique (sens horaire).
  3. CROSS     — Pure Pursuit vers le point d'entrée du cercle gauche.
  4. LEFT_1/2  — Pure Pursuit sur arc géométrique (sens antihoraire).
  5. EXIT_STR  — ligne droite de sortie jusqu'aux cônes orange SLAM.
  6. EXIT      — arrêt complet.

Paramètre clé : STRAIGHT_DIST (à calibrer selon la map FSDS).
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

# ── Géométrie Skidpad FSG ──────────────────────────────────────────────────────
DRIVING_RADIUS  = 9.125   # m
STRAIGHT_DIST   = 13.0    # m — à calibrer
LAP_ANGLE       = 2 * math.pi - 0.3
ON_CIRCLE_TOL   = 2.0     # m
ORANGE_STOP_DIST  = 5.0    # m — distance aux cônes orange pour s'arrêter
EXIT_MAX_DIST     = 25.0   # m — distance max en EXIT_STR avant arrêt forcé

# ── Pure Pursuit ──────────────────────────────────────────────────────────────
LOOKAHEAD_DIST  = 3.5
WHEELBASE       = 1.5
MAX_STEER_RAD   = 0.5

# ── Vitesse ───────────────────────────────────────────────────────────────────
SPEED_STRAIGHT   = 2.0
SPEED_CIRCLE     = 1.5
SPEED_MAX        = 3.0
THROTTLE_PULSE   = 0.4
BRAKE_LIGHT      = 0.3
BRAKE_CORNER     = 0.5
CORNER_STEER     = 0.5
SPEED_CORNER_MIN = 0.4

_CONE_NS = {'cone_map', 'cone_map_lidar',
            'cone_map_skidpad_lidar', 'cone_map_skidpad_stereo'}


class State(enum.Enum):
    STRAIGHT = 0
    RIGHT_1  = 1
    RIGHT_2  = 2
    CROSS    = 3
    LEFT_1   = 4
    LEFT_2   = 5
    EXIT_STR = 6
    EXIT     = 7


def _yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _world_to_car(px, py, car_x, car_y, yaw):
    dx = px - car_x
    dy = py - car_y
    return (dx * math.cos(yaw) + dy * math.sin(yaw),
           -dx * math.sin(yaw) + dy * math.cos(yaw))


def _angle_diff(a, b) -> float:
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


class SkidpadDriver(Node):

    def __init__(self, map_topic: str, straight_dist: float = STRAIGHT_DIST):
        super().__init__('skidpad_driver')
        self._straight_dist = straight_dist

        self.car_x    = 0.0
        self.car_y    = 0.0
        self.car_yaw  = 0.0
        self.speed    = 0.0
        self._odom_ok = False
        self._start_x: float | None = None
        self._start_y: float | None = None

        self._right_center: tuple | None = None
        self._left_center:  tuple | None = None

        self._state         = State.STRAIGHT
        self._lap_angle_acc = 0.0
        self._prev_angle:   float | None = None

        # Cônes orange pour la détection de sortie
        self._orange_cones: list = []
        # Position de début de EXIT_STR (fallback odomètre)
        self._exit_start_x: float | None = None
        self._exit_start_y: float | None = None

        _t = CFG['topics']
        self.create_subscription(Odometry, _t['odometry'],
                                 self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, map_topic,
                                 self._map_cb, LATCHED_QOS)
        self.pub_cmd = self.create_publisher(
            ControlCommand, _t['control_command'], 10)

        self.create_timer(0.05, self._control_loop)
        self.get_logger().info(
            f"SkidpadDriver — droite={straight_dist}m  R={DRIVING_RADIUS}m  "
            f"map={map_topic}")

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
            self._start_x = self.car_x
            self._start_y = self.car_y
            self._odom_ok = True

    def _map_cb(self, msg: MarkerArray):
        """Collecte uniquement les cônes orange pour la détection de sortie."""
        orange = []
        for mk in msg.markers:
            if mk.ns not in _CONE_NS or mk.type != Marker.CYLINDER:
                continue
            r, g, b = mk.color.r, mk.color.g, mk.color.b
            if r > 0.8 and 0.2 < g < 0.7 and b < 0.2:
                orange.append((mk.pose.position.x, mk.pose.position.y))
        self._orange_cones = orange

    # ── Calcul des centres (géométrie pure) ───────────────────────────────────

    def _compute_centers(self):
        yaw = self.car_yaw
        self._right_center = (self.car_x + DRIVING_RADIUS *  math.sin(yaw),
                              self.car_y - DRIVING_RADIUS *  math.cos(yaw))
        self._left_center  = (self.car_x - DRIVING_RADIUS *  math.sin(yaw),
                              self.car_y + DRIVING_RADIUS *  math.cos(yaw))
        self.get_logger().info(
            f"Centres — Droit: ({self._right_center[0]:.1f}, {self._right_center[1]:.1f})  "
            f"Gauche: ({self._left_center[0]:.1f}, {self._left_center[1]:.1f})")

    # ── Pure Pursuit ──────────────────────────────────────────────────────────

    def _circle_target(self, center: tuple, clockwise: bool) -> tuple:
        cx, cy  = center
        theta   = math.atan2(self.car_y - cy, self.car_x - cx)
        delta   = LOOKAHEAD_DIST / DRIVING_RADIUS
        theta_t = theta + (-delta if clockwise else +delta)
        return (cx + DRIVING_RADIUS * math.cos(theta_t),
                cy + DRIVING_RADIUS * math.sin(theta_t))

    def _approach_target(self, center: tuple) -> tuple:
        cx, cy = center
        theta  = math.atan2(self.car_y - cy, self.car_x - cx)
        return (cx + DRIVING_RADIUS * math.cos(theta),
                cy + DRIVING_RADIUS * math.sin(theta))

    def _steer_to(self, target_world: tuple) -> float:
        tx, ty    = _world_to_car(target_world[0], target_world[1],
                                  self.car_x, self.car_y, self.car_yaw)
        alpha     = math.atan2(ty, tx)
        ld        = math.hypot(tx, ty)
        steer_rad = math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld)
        return max(-1.0, min(1.0, -steer_rad / MAX_STEER_RAD))

    # ── Comptage des tours ────────────────────────────────────────────────────

    def _update_lap_angle(self, center: tuple, clockwise: bool) -> float:
        """Accumule uniquement les déplacements dans la direction attendue du cercle."""
        theta = math.atan2(self.car_y - center[1], self.car_x - center[0])
        if self._prev_angle is not None:
            diff = _angle_diff(theta, self._prev_angle)
            # CW : theta diminue → diff < 0 ; CCW : theta augmente → diff > 0
            if clockwise and diff < 0:
                self._lap_angle_acc += abs(diff)
            elif not clockwise and diff > 0:
                self._lap_angle_acc += diff
        self._prev_angle = theta
        return self._lap_angle_acc

    def _reset_lap(self):
        self._lap_angle_acc = 0.0
        self._prev_angle    = None

    # ── Détection sortie ──────────────────────────────────────────────────────

    def _orange_dist_ahead(self) -> float | None:
        best = None
        for (px, py) in self._orange_cones:
            xc, yc = _world_to_car(px, py, self.car_x, self.car_y, self.car_yaw)
            if xc < 0.5 or abs(yc) > 4.0:
                continue
            d = math.hypot(xc, yc)
            if best is None or d < best:
                best = d
        return best

    # ── Envoi commande ────────────────────────────────────────────────────────

    def _send(self, cmd: ControlCommand, steering: float,
              speed_target: float, in_circle: bool):
        in_corner = abs(steering) > CORNER_STEER
        if in_corner and in_circle and self.speed > SPEED_CORNER_MIN:
            cmd.throttle = 0.0
            cmd.brake    = BRAKE_CORNER
        elif self.speed > SPEED_MAX:
            cmd.throttle = 0.0
            cmd.brake    = BRAKE_LIGHT
        elif self.speed < speed_target:
            cmd.throttle = THROTTLE_PULSE
            cmd.brake    = 0.0
        else:
            cmd.throttle = 0.0
            cmd.brake    = 0.0
        cmd.steering = float(steering)
        self.pub_cmd.publish(cmd)
        self.get_logger().info(
            f"[{self._state.name}] v={self.speed:.2f}m/s  steer={steering:+.2f}  "
            f"lap={self._lap_angle_acc:.2f}rad  "
            f"{'CORNER' if in_corner and in_circle else 'gaz' if cmd.throttle > 0 else 'frein' if cmd.brake > 0 else 'inertie'}",
            throttle_duration_sec=0.5)

    # ── Boucle de contrôle ────────────────────────────────────────────────────

    def _control_loop(self):
        cmd = ControlCommand()
        if not self._odom_ok:
            cmd.throttle = 0.3
            self.pub_cmd.publish(cmd)
            return

        s = self._state

        if s == State.EXIT:
            cmd.throttle = 0.0
            cmd.brake    = 1.0
            cmd.steering = 0.0
            self.pub_cmd.publish(cmd)
            self.get_logger().info("Skidpad terminé.", throttle_duration_sec=2.0)
            return

        if s == State.EXIT_STR:
            if self._exit_start_x is None:
                self._exit_start_x = self.car_x
                self._exit_start_y = self.car_y
            exit_dist = math.hypot(self.car_x - self._exit_start_x,
                                   self.car_y - self._exit_start_y)
            od = self._orange_dist_ahead()
            if (od is not None and od < ORANGE_STOP_DIST) or exit_dist >= EXIT_MAX_DIST:
                reason = f"orange à {od:.1f}m" if od is not None else f"dist={exit_dist:.1f}m"
                self._state = State.EXIT
                self.get_logger().info(f"EXIT_STR → EXIT  ({reason})")
            cmd.throttle = THROTTLE_PULSE if self.speed < SPEED_STRAIGHT else 0.0
            cmd.brake    = 0.0
            cmd.steering = 0.0
            self.pub_cmd.publish(cmd)
            return

        if s == State.STRAIGHT:
            dist = math.hypot(self.car_x - self._start_x,
                              self.car_y - self._start_y)
            self.get_logger().info(
                f"[STRAIGHT] {dist:.1f}/{self._straight_dist}m  v={self.speed:.2f}m/s",
                throttle_duration_sec=0.5)
            if dist >= self._straight_dist:
                self._compute_centers()
                self._state = State.RIGHT_1
                self._reset_lap()
                self.get_logger().info(f"STRAIGHT → RIGHT_1  (dist={dist:.1f}m)")
            cmd.throttle = THROTTLE_PULSE if self.speed < SPEED_STRAIGHT else 0.0
            cmd.brake    = 0.0
            cmd.steering = 0.0
            self.pub_cmd.publish(cmd)
            return

        if s == State.CROSS:
            dist = math.hypot(self.car_x - self._left_center[0],
                              self.car_y - self._left_center[1])
            if abs(dist - DRIVING_RADIUS) < ON_CIRCLE_TOL:
                self._state = State.LEFT_1
                self._reset_lap()
                self.get_logger().info(f"CROSS → LEFT_1  (dist={dist:.2f}m)")
            target   = self._approach_target(self._left_center)
            steering = self._steer_to(target)
            self._send(cmd, steering, SPEED_CIRCLE, in_circle=False)
            return

        if s in (State.RIGHT_1, State.RIGHT_2):
            angle = self._update_lap_angle(self._right_center, clockwise=True)
            if angle >= LAP_ANGLE:
                if s == State.RIGHT_1:
                    self._state = State.RIGHT_2
                    self._reset_lap()
                    self.get_logger().info("RIGHT_1 → RIGHT_2")
                else:
                    self._state = State.CROSS
                    self._reset_lap()
                    self.get_logger().info("RIGHT_2 → CROSS")
            target   = self._circle_target(self._right_center, clockwise=True)
            steering = self._steer_to(target)
            self._send(cmd, steering, SPEED_CIRCLE, in_circle=True)
            return

        if s in (State.LEFT_1, State.LEFT_2):
            angle = self._update_lap_angle(self._left_center, clockwise=False)
            if angle >= LAP_ANGLE:
                if s == State.LEFT_1:
                    self._state = State.LEFT_2
                    self._reset_lap()
                    self.get_logger().info("LEFT_1 → LEFT_2")
                else:
                    self._state = State.EXIT_STR
                    self._exit_start_x = None
                    self.get_logger().info("LEFT_2 → EXIT_STR")
            target   = self._circle_target(self._left_center, clockwise=False)
            steering = self._steer_to(target)
            self._send(cmd, steering, SPEED_CIRCLE, in_circle=True)
            return


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', default='/slam_skidpad_stereo/cone_map',
                        help='Topic MarkerArray SLAM (pour détection cônes orange sortie)')
    parser.add_argument('--straight', type=float, default=STRAIGHT_DIST,
                        help=f'Distance ligne droite (défaut: {STRAIGHT_DIST}m)')
    parsed, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = SkidpadDriver(map_topic=parsed.map, straight_dist=parsed.straight)
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
