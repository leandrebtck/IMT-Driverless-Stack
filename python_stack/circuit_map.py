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

        # Stockage des cônes globaux et position voiture
        self.cones_global = {}  # {(x, y): couleur}
        self.car_pos = (0.0, 0.0)

        # ---- Setup PyQtGraph ----
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(show=True, title="Circuit Map")
        self.win.resize(800, 800)
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel('left', 'Y (m)')
        self.plot.setLabel('bottom', 'X (m)')

        # Scatter plots
        self.cones_scatter = pg.ScatterPlotItem(size=8, pen=pg.mkPen(None), brush=pg.mkBrush(255,0,0,200))
        self.car_scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen(None), brush=pg.mkBrush(0,0,255,255))
        self.plot.addItem(self.cones_scatter)
        self.plot.addItem(self.car_scatter)

        # Timer pour rafraîchir PyQtGraph
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
                self.cones_global[key] = (marker.color.r*255,
                                          marker.color.g*255,
                                          marker.color.b*255)

    def odom_callback(self, msg: Odometry):
        self.car_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def update_plot(self):
        rclpy.spin_once(self, timeout_sec=0)

        # Affichage voiture (toujours au centre)
        self.car_scatter.setData([0.0], [0.0])

        # Affichage des cônes globaux relatifs à la voiture
        if self.cones_global:
            coords = np.array([[x - self.car_pos[0], y - self.car_pos[1]] for x,y in self.cones_global.keys()])
            colors = [pg.mkBrush(r,g,b,200) for r,g,b in self.cones_global.values()]
            self.cones_scatter.setData(coords[:,0], coords[:,1], brush=colors)

        # Limites très larges pour ne pas tronquer (-1000 m à 1000 m)
        self.plot.setXRange(-100, 100)
        self.plot.setYRange(-100, 100)

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
