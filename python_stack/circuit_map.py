#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np
import sys

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map')

        # ---- Subscriptions ----
        self.cones_sub = self.create_subscription(
            MarkerArray,
            '/cones_relative',
            self.cones_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ---- PyQtGraph Setup ----
        self.app = QtWidgets.QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True, title="Circuit Map")
        self.win.resize(1000, 800)
        self.plot = self.win.addPlot(title="Map of cones and vehicle")
        self.plot.setAspectLocked(True)  # 1:1 aspect ratio
        self.plot.showGrid(x=True, y=True)

        # Scatter plots
        self.plot_points = {}  # points uniques des cônes
        self.cones_scatter = pg.ScatterPlotItem(pen=pg.mkPen(None), brush=pg.mkBrush(255,0,0), size=10)
        self.vehicle_scatter = pg.ScatterPlotItem(pen=pg.mkPen(None), brush=pg.mkBrush(0,0,255), size=12)
        self.plot.addItem(self.cones_scatter)
        self.plot.addItem(self.vehicle_scatter)

        # Vehicle position
        self.vehicle_pos = np.array([0.0, 0.0])

        # Fixed view range (juste assez large pour voir les cônes autour)
        self.view_range = 20  # mètres autour de la voiture

        # Timer pour refresh
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # refresh toutes les 100ms

        self.get_logger().info("Circuit map node started")

    def cones_callback(self, msg):
        """Stocke uniquement les nouveaux cones détectés"""
        for marker in msg.markers:
            x, y = marker.pose.position.x, marker.pose.position.y
            key = (round(x,2), round(y,2))
            if key not in self.plot_points:
                self.plot_points[key] = (x, y)

    def odom_callback(self, msg):
        """Met à jour la position de la voiture"""
        self.vehicle_pos[0] = msg.pose.pose.position.x
        self.vehicle_pos[1] = msg.pose.pose.position.y

    def update_plot(self):
        """Met à jour le plot dynamique avec une vue fixe autour de la voiture"""
        if self.plot_points:
            xy = np.array(list(self.plot_points.values()))
            self.cones_scatter.setData(x=xy[:,0], y=xy[:,1])
        self.vehicle_scatter.setData(x=[self.vehicle_pos[0]], y=[self.vehicle_pos[1]])

        # Fenêtre centrée sur la voiture avec une portée fixe
        self.plot.setXRange(self.vehicle_pos[0]-self.view_range, self.vehicle_pos[0]+self.view_range)
        self.plot.setYRange(self.vehicle_pos[1]-self.view_range, self.vehicle_pos[1]+self.view_range)

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
        sys.exit(node.app.exec_())

if __name__ == '__main__':
    main()
