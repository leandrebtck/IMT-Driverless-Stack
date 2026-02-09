#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import math

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map_node')

        # --- Subscribers ---
        self.create_subscription(MarkerArray, '/cones_detected', self.cones_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(NavSatFix, '/gps', self.gps_callback, 10)  # Si GPS utilisé

        # --- Vehicle state ---
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0  # radians
        self.first_gps = None

        # --- Cones ---
        self.global_cones = []  # list of tuples: (x, y, color)
        self.cones_seen = set() # pour éviter duplication

        # --- PyQtGraph Window ---
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title="Circuit Map")
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)
        self.win.show()

        # Scatter plots
        self.vehicle_scatter = pg.ScatterPlotItem(size=10, brush='b')
        self.plot.addItem(self.vehicle_scatter)
        self.cones_scatter = pg.ScatterPlotItem(size=7)
        self.plot.addItem(self.cones_scatter)

        # Timer for refresh
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # update every 100 ms

    # ----------------------------
    # Odometry callback
    # ----------------------------
    def odom_callback(self, msg: Odometry):
        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y

        # Yaw extraction
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.vehicle_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ----------------------------
    # GPS callback
    # ----------------------------
    def gps_callback(self, msg: NavSatFix):
        if self.first_gps is None:
            self.first_gps = (msg.latitude, msg.longitude)
        # Conversion GPS -> local meters
        lat0, lon0 = self.first_gps
        dx = (msg.longitude - lon0) * 111320 * math.cos(math.radians(lat0))
        dy = (msg.latitude - lat0) * 110540
        self.vehicle_x = dx
        self.vehicle_y = dy

    # ----------------------------
    # Cones callback
    # ----------------------------
    def cones_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            # Créer un ID unique pour chaque cone selon sa position et couleur
            cone_id = (marker.pose.position.x, marker.pose.position.y, marker.color.r, marker.color.g, marker.color.b)
            if cone_id in self.cones_seen:
                continue
            self.cones_seen.add(cone_id)

            # Transformation locale -> globale selon véhicule
            lx = marker.pose.position.x
            ly = marker.pose.position.y

            # Rotation et translation
            gx = (lx * math.cos(self.vehicle_yaw) - ly * math.sin(self.vehicle_yaw)) + self.vehicle_x
            gy = (lx * math.sin(self.vehicle_yaw) + ly * math.cos(self.vehicle_yaw)) + self.vehicle_y

            color = (marker.color.r, marker.color.g, marker.color.b)
            self.global_cones.append((gx, gy, color))

    # ----------------------------
    # Update plot
    # ----------------------------
    def update_plot(self):
        if not self.global_cones:
            return

        # Cones
        x = [c[0] for c in self.global_cones]
        y = [c[1] for c in self.global_cones]
        brushes = [pg.mkBrush(int(c[2][0]*255), int(c[2][1]*255), int(c[2][2]*255)) for c in self.global_cones]
        self.cones_scatter.setData(x=x, y=y, brush=brushes)

        # Vehicle
        self.vehicle_scatter.setData([self.vehicle_x], [self.vehicle_y])

        # Auto zoom
        all_x = x + [self.vehicle_x]
        all_y = y + [self.vehicle_y]
        min_x, max_x = min(all_x)-5, max(all_x)+5
        min_y, max_y = min(all_y)-5, max(all_y)+5
        self.plot.setXRange(min_x, max_x, padding=0)
        self.plot.setYRange(min_y, max_y, padding=0)

# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CircuitMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
