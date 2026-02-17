#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloPerceptionNode(Node):

    def __init__(self):
        super().__init__('yolo_perception_node')

        # Détection version ROS
        self.ros_distro = os.environ.get("ROS_DISTRO", "unknown").lower()
        self.get_logger().info(f"ROS Distro détectée : {self.ros_distro}")

        # Chargement modèle YOLO
        self.model = YOLO("best.pt")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.pose_publisher = self.create_publisher(
            Pose2D,
            '/detected_cone_pose',
            10
        )

    # ===============================
    # CALLBACK IMAGE
    # ===============================
    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame)

        for result in results:
            boxes = result.boxes

            for box in boxes:

                # Coordonnées
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                # Centre bbox
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # ===============================
                # NORMALISATION DU NOM
                # ===============================
                if "orange" in class_name.lower():
                    label = "ORANGE"
                    color = (0, 140, 255)  # orange en BGR
                elif "blue" in class_name.lower():
                    label = "BLUE"
                    color = (255, 0, 0)
                elif "yellow" in class_name.lower():
                    label = "YELLOW"
                    color = (0, 255, 255)
                else:
                    label = class_name.upper()
                    color = (0, 255, 0)

                display_text = f"{label} : {confidence:.2f}"

                # ===============================
                # DESSIN
                # ===============================
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # fond texte
                (w, h), _ = cv2.getTextSize(display_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (x1, y1 - h - 10), (x1 + w, y1), color, -1)

                # texte blanc
                cv2.putText(
                    frame,
                    display_text,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

                # ===============================
                # PUBLICATION POSE
                # ===============================
                pose_msg = Pose2D()

                if self.ros_distro == "iron":
                    # Iron → Pose2D simple
                    pose_msg.x = float(cx)
                    pose_msg.y = float(cy)
                    pose_msg.theta = 0.0

                elif self.ros_distro == "galactic":
                    # Galactic → Pose2D aussi simple (mais bug fréquent si mal importé)
                    pose_msg.x = float(cx)
                    pose_msg.y = float(cy)
                    pose_msg.theta = 0.0

                else:
                    # fallback
                    pose_msg.x = float(cx)
                    pose_msg.y = float(cy)
                    pose_msg.theta = 0.0

                self.pose_publisher.publish(pose_msg)

        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1)


# ===============================
# MAIN
# ===============================
def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
