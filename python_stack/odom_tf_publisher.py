#!/usr/bin/env python3
"""
odom_tf_publisher.py
Publie le TF dynamique fsds/map → fsds/FSCar à partir de l'odométrie FSDS.
C'est ce transform manquant qui empêche fsds/map d'exister dans le TF tree.

Sans ce nœud, rviz2 ne peut pas résoudre les positions dans fsds/map.

Subscription : /testing_only/odom  (nav_msgs/Odometry)
TF publié    : fsds/map → fsds/FSCar
Publication  : /slam/car_path  (nav_msgs/Path)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFPublisher(Node):

    def __init__(self):
        super().__init__('odom_tf_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.path = Path()
        self.path.header.frame_id = 'fsds/map'

        self.create_subscription(
            Odometry, '/testing_only/odom',
            self._odom_cb, qos_profile_sensor_data
        )

        self.pub_path = self.create_publisher(Path, '/slam/car_path', 10)

        self.get_logger().info(
            "OdomTFPublisher prêt — publie TF fsds/map → fsds/FSCar"
        )

    def _odom_cb(self, msg: Odometry):
        # ── Publie le TF dynamique fsds/map → fsds/FSCar ────────────────────
        t = TransformStamped()
        t.header          = msg.header          # frame_id = fsds/map
        t.child_frame_id  = 'fsds/FSCar'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # ── Chemin de la voiture pour RViz ───────────────────────────────────
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose   = msg.pose.pose
        self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp
        self.pub_path.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
