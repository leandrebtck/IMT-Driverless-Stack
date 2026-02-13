import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  #
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import sys
import torch
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')

        # --- SETUP ---
        script_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')

        self.model = YOLO(weights_path)
        self.model.to('cuda' if torch.cuda.is_available() else 'cpu')
        
        self.bridge = CvBridge()

        # 1. Abonnement Caméra (En mode "Sensor Data")
        self.subscription = self.create_subscription(
            Image,
            '/fsds/cam1/image_color',
            self.listener_callback,
            qos_profile_sensor_data
        )

        # 2. Publisher YOLO (En mode "Sensor Data" AUSSI !)
        # C'est ici que ça bloquait : on force la compatibilité
        self.publisher = self.create_publisher(
            Detection2DArray, 
            '/yolo/detections', 
            qos_profile_sensor_data 
        )
        
        self.get_logger().info("✅ YOLO (QoS Sensor Data) : Headless")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Inférence rapide (sans verbose)
            results = self.model(cv_image, verbose=False, conf=0.5)

            detections_msg = Detection2DArray()
            detections_msg.header = msg.header 

            if len(results) > 0:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])

                    w = float(x2 - x1)
                    h = float(y2 - y1)
                    cx = float(x1 + w / 2.0)
                    cy = float(y1 + h / 2.0)

                    detection = Detection2D()
                    detection.header = msg.header
                    detection.bbox.center.x = cx
                    detection.bbox.center.y = cy
                    detection.bbox.size_x = w
                    detection.bbox.size_y = h

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls_id)
                    hypothesis.hypothesis.score = conf
                    detection.results.append(hypothesis)
                    
                    detections_msg.detections.append(detection)

            # Publication forcée
            self.publisher.publish(detections_msg)
            # self.get_logger().info(f"📤 Envoi de {len(detections_msg.detections)} boîtes")

        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
