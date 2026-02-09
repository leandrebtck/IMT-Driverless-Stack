#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from ultralytics import YOLO
import cv2
import os
import torch

class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')

        # --- CONFIGURATION ---
        script_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')

        if not os.path.exists(weights_path):
            self.get_logger().error(f"❌ FICHIER INTROUVABLE ! Vérifier {weights_path}")
            raise FileNotFoundError(weights_path)
        else:
            self.get_logger().info(f"✅ Poids trouvés : {weights_path}")

        self.model = YOLO(weights_path)
        self.model.eval()
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"🚀 YOLO lancé sur : {device}")

        self.camera_topic = '/fsds/cam1/image_color'
        self.bridge = CvBridge()

        # Couleurs (BGR) pour markers RViz
        self.COLORS = {
            0: (0, 255, 255),   # JAUNE
            1: (255, 0, 0),     # BLEU
            2: (0, 255, 255),   # JAUNE
        }
        self.CUSTOM_NAMES = {
            0: "JAUNE",
            1: "BLEU",
            2: "JAUNE",
        }

        # Publisher MarkerArray
        self.cones_pub = self.create_publisher(MarkerArray, '/cones_detected', 10)

        # Subscriber image
        self.subscription = self.create_subscription(
            Image, self.camera_topic, self.listener_callback, qos_profile_sensor_data
        )
        self.get_logger().info("✅ YOLO perception lancée")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = cv_image.copy()

            with torch.no_grad():
                results = self.model(cv_image, verbose=False, conf=0.5)

            marker_array = MarkerArray()
            box_id = 0

            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                label = self.CUSTOM_NAMES.get(cls_id, str(cls_id))
                color_bgr = self.COLORS.get(cls_id, (255, 165, 0))  # ORANGE défaut

                # ---- Marker RViz ----
                marker = Marker()
                marker.id = box_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = 0.0  # position locale, cone_fusion fera transformation globale
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.0
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.5
                marker.color.a = 1.0
                marker.color.r = color_bgr[2]/255  # OpenCV BGR → RViz RGB
                marker.color.g = color_bgr[1]/255
                marker.color.b = color_bgr[0]/255
                marker.text = label
                marker_array.markers.append(marker)

                # ---- Affichage YOLO ----
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color_bgr, 2)
                t_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(display_frame, (x1, y1-20), (x1+t_size[0], y1), color_bgr, -1)
                cv2.putText(display_frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)

                box_id += 1

            # Publish MarkerArray
            self.cones_pub.publish(marker_array)

            # Show frame
            cv2.imshow("YOLO FINAL", display_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ Erreur YOLO callback : {e}")

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

if __name__ == "__main__":
    main()
