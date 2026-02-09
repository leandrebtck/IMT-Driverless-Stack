#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import numpy as np

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map')

        # Subscribers ROS2
        self.create_subscription(MarkerArray, '/cones_relative', self.cones_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Stockage
        self.cones_global = []  # liste de tuples (x, y)
        self.car_pos = (0.0, 0.0)

        # ---- Setup PyQtGraph ----
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(show=True, title="Circuit Map")
        self.win.resize(800, 800)
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked(True)  # garder proportions réelles
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel('left', 'Y (m)')
        self.plot.setLabel('bottom', 'X (m)')

        # Scatter plots
        self.cones_scatter = pg.ScatterPlotItem(size=8, pen=pg.mkPen(None), brush=pg.mkBrush(255,0,0,200))
        self.car_scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen(None), brush=pg.mkBrush(0,0,255,255))
        self.plot.addItem(self.cones_scatter)
        self.plot.addItem(self.car_scatter)

        # Ligne pour relier les deux cônes les plus proches
        self.line_item = pg.PlotDataItem(pen=pg.mkPen('r', width=1))
        self.plot.addItem(self.line_item)

        # Timer pour rafraîchir PyQtGraph et ROS
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)  # 20 Hz

        self.get_logger().info("CircuitMap node started")

    def cones_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            key = (round(x,2), round(y,2))
            if key not in self.cones_global:
                self.cones_global.append(key)

    def odom_callback(self, msg: Odometry):
        self.car_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def update_plot(self):
        rclpy.spin_once(self, timeout_sec=0)

        # Affichage voiture
        self.car_scatter.setData([self.car_pos[0]], [self.car_pos[1]])

        # Affichage cônes
        if self.cones_global:
            cones_array = np.array(self.cones_global)
            self.cones_scatter.setData(cones_array[:,0], cones_array[:,1])

            # Relier les deux cônes les plus proches du véhicule
            dists = np.linalg.norm(cones_array - np.array(self.car_pos), axis=1)
            if len(dists) >= 2:
                idx = np.argsort(dists)[:2]
                pts = cones_array[idx]
                self.line_item.setData([pts[0,0], pts[1,0]], [pts[0,1], pts[1,1]])

        # Ajuster la vue autour de la voiture (fixe ±10 m)
        x_min = self.car_pos[0] - 10
        x_max = self.car_pos[0] + 10
        y_min = self.car_pos[1] - 10
        y_max = self.car_pos[1] + 10
        self.plot.setXRange(x_min, x_max)
        self.plot.setYRange(y_min, y_max)

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

if __name__ == "__main__":
    main()
