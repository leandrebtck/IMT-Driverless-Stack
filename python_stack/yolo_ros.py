import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
import torch

class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')

        # --- 1. CONFIGURATION ROBUSTE (Comme Script A) ---
        script_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')

        self.get_logger().info(f"📍 Chargement des poids depuis : {weights_path}")
        
        if not os.path.exists(weights_path):
            self.get_logger().error("❌ FICHIER POIDS INTROUVABLE ! Arrêt du noeud.")
            # On pourrait raise une erreur ici, mais on laisse planter proprement plus bas
        
        self.model_path = weights_path
        
        # Charger le modèle
        try:
            self.model = YOLO(self.model_path)
            self.model.to('cuda' if torch.cuda.is_available() else 'cpu')
        except Exception as e:
            self.get_logger().error(f"❌ Impossible de charger YOLO : {e}")
            raise e

        self.bridge = CvBridge()

        # --- 2. SETUP ROS (Correction de l'erreur Script B) ---
        
        # Publisher pour la fusion (Output)
        self.publisher = self.create_publisher(
            Detection2DArray, 
            '/perception/detections', 
            10  # Queue size standard
        )

        # Subscriber Image (Input)
        # ⚠️ CRUCIAL : On assigne bien 'self.subscription' pour ne pas qu'il soit supprimé
        self.subscription = self.create_subscription(
            Image,
            '/fsds/cam1/image_color',
            self.listener_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("✅ YOLO Node Prêt -> Publie sur /perception/detections")

    def listener_callback(self, msg):
        try:
            # 1. Conversion Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 2. Inférence (Avec no_grad pour la vitesse, comme Script A)
            with torch.no_grad():
                results = self.model(cv_image, verbose=False, conf=0.5)

            # 3. Préparation du message ROS
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header # Très important pour la fusion (synchronisation)

            # S'il y a des détections
            if len(results) > 0:
                for box in results[0].boxes:
                    # Récupération des données YOLO
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])

                    # Création de l'objet Detection2D
                    detection = Detection2D()
                    detection.header = msg.header

                    # --- Remplissage de la Bounding Box ---
                    # Centre X, Centre Y
                    detection.bbox.center.position.x = float(x1 + x2) / 2.0
                    detection.bbox.center.position.y = float(y1 + y2) / 2.0
                    # Taille X, Taille Y
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)

                    # --- Remplissage de l'hypothèse (Classe + Score) ---
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls_id) # Doit être un string
                    hypothesis.hypothesis.score = conf
                    
                    detection.results.append(hypothesis)
                    detections_msg.detections.append(detection)

                    # (Optionnel) Debug visuel dans le terminal
                    # print(f"Class: {cls_id}, Conf: {conf:.2f}")

            # 4. Publication
            self.publisher.publish(detections_msg)

            # 5. (Optionnel) Visualisation pour débugger comme Script A
            # display_frame = results[0].plot()
            # cv2.imshow("YOLO DEBUG", display_frame)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ Erreur Callback : {e}")

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
