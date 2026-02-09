#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import numpy as np

class CircuitMap(Node):
    def __init__(self, min_dist=0.8):
        super().__init__('circuit_map')

        # Subscribers ROS2
        self.create_subscription(MarkerArray, '/cones_relative', self.cones_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Stockage global
        self.cones_global = []
        self.car_pos = (0.0, 0.0)
        self.min_dist = min_dist  # distance min entre 2 cônes

        # ---- Setup PyQtGraph ----
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(show=True, title="Circuit Map")
        self.win.resize(800, 800)
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked(False)
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel('left', 'Y (m)')
        self.plot.setLabel('bottom', 'X (m)')

        # Scatter plots
        self.cones_scatter = pg.ScatterPlotItem(size=8, pen=pg.mkPen(None), brush=pg.mkBrush(255,0,0,200))
        self.car_scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen(None), brush=pg.mkBrush(0,0,255,255))
        self.plot.addItem(self.cones_scatter)
        self.plot.addItem(self.car_scatter)

        # Timer pour rafraîchir PyQtGraph et ROS
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)  # 20 Hz

        self.get_logger().info("CircuitMap node started")

    def cones_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            new_point = np.array([x, y])
            if self.is_new_cone(new_point):
                self.cones_global.append(new_point)

    def odom_callback(self, msg: Odometry):
        self.car_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def is_new_cone(self, point):
        """Retourne True si le point est assez éloigné de tous les points existants"""
        for existing in self.cones_global:
            if np.linalg.norm(existing - point) < self.min_dist:
                return False
        return True

    def update(self):
        rclpy.spin_once(self, timeout_sec=0)

        if self.cones_global:
            cones_array = np.array(self.cones_global)
            self.cones_scatter.setData(cones_array[:,0], cones_array[:,1])

        # Affichage voiture
        self.car_scatter.setData([self.car_pos[0]], [self.car_pos[1]])

        # Dézoom automatique
        all_x = [self.car_pos[0]] + [c[0] for c in self.cones_global]
        all_y = [self.car_pos[1]] + [c[1] for c in self.cones_global]
        if all_x and all_y:
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            margin_x = (max_x - min_x) * 0.2 + 1
            margin_y = (max_y - min_y) * 0.2 + 1
            self.plot.setXRange(min_x - margin_x, max_x + margin_x)
            self.plot.setYRange(min_y - margin_y, max_y + margin_y)

def main(args=None):
    rclpy.init(args=args)
    node = CircuitMap()
    try:
        node.app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
