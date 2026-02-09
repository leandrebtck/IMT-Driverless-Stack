#!/usr/bin/env python3
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from math import cos, sin, atan2

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map_node')

        # ROS Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(MarkerArray, '/cones_relative', self.cones_callback, 10)

        # Position de la voiture
        self.x_vehicle = 0.0
        self.y_vehicle = 0.0
        self.theta = 0.0

        # Stockage des cônes globaux pour éviter doublons
        self.global_cones = []

        # --- Setup PyQtGraph ---
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title="Circuit Map")
        self.win.resize(800, 600)
        self.win.setWindowTitle('Circuit Map - PyQtGraph')

        self.plot = self.win.addPlot()
        self.plot.showGrid(x=True, y=True)
        self.plot.setAspectLocked(True)  # Garder les axes proportionnels

        # Scatter plots
        self.cones_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255,0,0,200))
        self.vehicle_scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen('b'), brush=pg.mkBrush(0,0,255,200))

        self.plot.addItem(self.cones_scatter)
        self.plot.addItem(self.vehicle_scatter)

        self.win.show()

        # Timer pour update graphique
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)  # 20 Hz

    def odom_callback(self, msg: Odometry):
        # Position du véhicule
        self.x_vehicle = msg.pose.pose.position.x
        self.y_vehicle = msg.pose.pose.position.y

        # Orientation yaw 2D
        q = msg.pose.pose.orientation
        self.theta = 2 * atan2(q.z, q.w)

    def cones_callback(self, msg: MarkerArray):
        for m in msg.markers:
            # Transformation en coordonnées globales
            x_global = self.x_vehicle + cos(self.theta) * m.pose.position.x - sin(self.theta) * m.pose.position.y
            y_global = self.y_vehicle + sin(self.theta) * m.pose.position.x + cos(self.theta) * m.pose.position.y

            # Eviter doublons
            if not any(abs(c[0]-x_global)<0.2 and abs(c[1]-y_global)<0.2 for c in self.global_cones):
                self.global_cones.append((x_global, y_global))

    def update_plot(self):
        # Mettre à jour les positions
        if self.global_cones:
            cones_np = np.array(self.global_cones)
            self.cones_scatter.setData(cones_np[:,0], cones_np[:,1])

        self.vehicle_scatter.setData([self.x_vehicle], [self.y_vehicle])

        # Center plot autour de la voiture
        self.plot.setXRange(self.x_vehicle-10, self.x_vehicle+10, padding=0)
        self.plot.setYRange(self.y_vehicle-10, self.y_vehicle+10, padding=0)

def main(args=None):
    rclpy.init(args=args)
    node = CircuitMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        QtGui.QApplication.instance().quit()

if __name__ == "__main__":
    main()
