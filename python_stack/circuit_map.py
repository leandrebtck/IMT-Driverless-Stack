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

        # Stockage global
        self.cones_global = set()  # pour éviter les doublons
        self.car_pos = (0.0, 0.0)

        # ---- Setup PyQtGraph ----
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(show=True, title="Circuit Map")
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked(True)  # x=y
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel('left', 'Y (m)')
        self.plot.setLabel('bottom', 'X (m)')

        # Scatter plots
        self.cones_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255,0,0,200))
        self.car_scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen(None), brush=pg.mkBrush(0,0,255,200))
        self.plot.addItem(self.cones_scatter)
        self.plot.addItem(self.car_scatter)

        # Timer pour refresh
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # 10 Hz

        self.get_logger().info("CircuitMap node started")

    def cones_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            key = (round(x,2), round(y,2))  # éviter doublons
            if key not in self.cones_global:
                self.cones_global.add(key)

    def odom_callback(self, msg: Odometry):
        self.car_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def update_plot(self):
        # Convert sets to np.array
        if self.cones_global:
            cones_array = np.array(list(self.cones_global))
            self.cones_scatter.setData(cones_array[:,0], cones_array[:,1])
        # Voiture
        self.car_scatter.setData([self.car_pos[0]], [self.car_pos[1]])

        # Auto-range / zoom dynamique
        all_x = [self.car_pos[0]] + [c[0] for c in self.cones_global]
        all_y = [self.car_pos[1]] + [c[1] for c in self.cones_global]
        if all_x and all_y:
            self.plot.setXRange(min(all_x)-5, max(all_x)+5, padding=0)
            self.plot.setYRange(min(all_y)-5, max(all_y)+5, padding=0)

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

if __name__ == '__main__':
    main()
