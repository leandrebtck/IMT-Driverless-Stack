import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO
import os
import torch

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D


class YoloPerceptionNode(Node):

    def __init__(self):
        super().__init__('yolo_perception_node')

        # ---------------- CONFIGURATION ----------------
        script_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')

        self.get_logger().info(f"Dossier script : {script_dir}")
        self.get_logger().info(f"Chemin poids : {weights_path}")

        if not os.path.exists(weights_path):
            self.get_logger().error(f"FICHIER INTROUVABLE : {weights_path}")
            raise FileNotFoundError(weights_path)

        self.camera_topic = '/fsds/cam1/image_color'

        # ---------------- COULEURS & LABELS ----------------
        self.COLORS = {
            0: (0, 255, 255),   # Jaune
            1: (255, 0, 0),     # Bleu
            2: (0, 120, 255),   # Orange
        }

        self.CUSTOM_NAMES = {
            0: "JAUNE",
            1: "BLEU",
            2: "ORANGE",
        }

        # ---------------- CHARGEMENT YOLO ----------------
        try:
            self.model = YOLO(weights_path)
        except Exception as e:
            self.get_logger().error(f"Erreur chargement YOLO : {e}")
            raise e

        self.model.eval()

        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"YOLO lancé sur : {device}")

        self.bridge = CvBridge()

        # ---------------- SUBSCRIBER ----------------
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.listener_callback,
            qos_profile_sensor_data
        )

        # ---------------- PUBLISHER ----------------
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/perception/detections',
            10
        )

        self.get_logger().info("Perception prête (compatible ROS Iron).")


    def listener_callback(self, msg):

        try:
            # -------- Conversion Image --------
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = cv_image.copy()

            # -------- Inférence YOLO --------
            with torch.no_grad():
                results = self.model(cv_image, verbose=False, conf=0.5)

            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            if len(results) > 0:
                for box in results[0].boxes:

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])

                    # -------- AFFICHAGE --------
                    name_display = self.CUSTOM_NAMES.get(
                        cls_id,
                        self.model.names[cls_id]
                    )

                    label = f"{name_display} {conf:.2f}"
                    color = self.COLORS.get(cls_id, (255, 255, 255))

                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)

                    t_size, _ = cv2.getTextSize(
                        label,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        1
                    )

                    cv2.rectangle(
                        display_frame,
                        (x1, y1 - 20),
                        (x1 + t_size[0], y1),
                        color,
                        -1
                    )

                    cv2.putText(
                        display_frame,
                        label,
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 0),
                        1
                    )

                    # -------- DONNÉES ROS --------
                    w = float(x2 - x1)
                    h = float(y2 - y1)
                    cx = float(x1 + w / 2.0)
                    cy = float(y1 + h / 2.0)

                    detection = Detection2D()
                    detection.header = msg.header

                    # IMPORTANT : Compatible ROS Iron
                    detection.bbox = BoundingBox2D()
                    detection.bbox.center.position.x = cx
                    detection.bbox.center.position.y = cy
                    detection.bbox.center.theta = 0.0

                    detection.bbox.size_x = w
                    detection.bbox.size_y = h

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls_id)
                    hypothesis.hypothesis.score = conf

                    detection.results.append(hypothesis)
                    detections_msg.detections.append(detection)

            # -------- Publication --------
            self.publisher.publish(detections_msg)

            # -------- Affichage --------
            cv2.imshow("YOLO FINAL", display_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erreur Callback : {e}")


def main(args=None):

    rclpy.init(args=args)
    node = YoloPerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
