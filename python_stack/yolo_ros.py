import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
import torch

class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')

        # --- OPTIMISATION 1 : CONFIGURATION SKIP FRAME ---
        self.frame_count = 0
        self.SKIP_RATE = 2  # Ne traite que 1 image sur (SKIP_RATE + 1). 
                            # 2 = Traite 1 image sur 3.
        
        # --- CONFIGURATION CHEMIN ---
        import os
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.model_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')
        
        # --- CONFIGURATION GPU ---
        # Vérification computationnelle du device
        self.device = '0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"⚙️ Périphérique d'inférence : {self.device.upper()}")

        # COULEURS (Visuel)
        self.COLORS = {
            0: (0, 255, 255),  # Jaune
            1: (255, 0, 0),    # Bleu
            2: (0, 165, 255),  # Orange
        }

        self.get_logger().info(f"🔧 Chargement modèle : {self.model_path}")
        
        try:
            # Chargement du modèle
            self.model = YOLO(self.model_path)
        except Exception as e:
            self.get_logger().error(f"❌ Erreur modèle : {e}")
            raise e

        self.bridge = CvBridge()

        # QoS
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic_callback, # J'ai renommé pour la clarté
            self.listener_callback,
            qos_profile_sensor_data
        )
        # Définition du topic (vérifie que c'est le bon pour ton simu)
        self.camera_topic = '/fsds/cam1/image_color' 

        self.get_logger().info("✅ Perception OPTIMISÉE lancée (Skip Frame activé).")

    # Petit hack pour définir le topic dynamiquement si besoin, sinon tu peux le mettre en dur
    @property
    def camera_topic_callback(self):
        return '/fsds/cam1/image_color'

    def listener_callback(self, msg):
        # --- OPTIMISATION 1 : FILTRAGE TEMPOREL ---
        self.frame_count += 1
        # Si le reste de la division n'est pas 0, on saute cette image
        if self.frame_count % (self.SKIP_RATE + 1) != 0:
            return 

        try:
            # 1. Conversion
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = cv_image.copy()

            # 2. Inférence (Avec paramètres d'optimisation)
            # - device=self.device : Force le GPU
            # - imgsz=640 : Évite que le modèle upscale inutilement
            # - half=False : Parfois True accélère, mais peut bugger sur certaines cartes, False est stable
            results = self.model(cv_image, verbose=False, conf=0.5, device=self.device, imgsz=640)

            # 3. Traitement
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                name_display = self.model.names[cls_id]
                label = f"{name_display} {conf:.2f}"
                color = self.COLORS.get(cls_id, (255, 255, 255))

                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                t_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(display_frame, (x1, y1 - 20), (x1 + t_size[0], y1), color, -1)
                cv2.putText(display_frame, label, (x1, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

            # 4. Affichage
            cv2.imshow("YOLO OPTIMIZED", display_frame)
            
            # --- OPTIMISATION 2 : GESTION GUI ---
            # WaitKey(1) est nécessaire pour rafraîchir la fenêtre
            key = cv2.waitKey(1) 
            if key == 27: # Touche Echap pour fermer proprement si besoin
                cv2.destroyAllWindows()

        except Exception as e:
            self.get_logger().error(f"Erreur Display : {e}")

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
