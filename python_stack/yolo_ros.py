import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os 
import sys

class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')

        # --- CONFIGURATION ---
        script_dir = os.path.dirname(os.path.abspath(__file__))

        #On construit le chemin vers le poids
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')


        self.get_logger().info(f"📍 Dossier du script : {script_dir}")
        self.get_logger().info(f"⚖️ Chemin calculé du .pt : {weights_path}")

        # Vérification 
        if not os.path.exists(weights_path):
            self.get_logger().error(f" FICHIER INTROUVABLE ! Vérifier que le dossier 'weights' est bien dans {script_dir}")
        else:
            self.get_logger().info("✅ Fichier trouvé ! Chargement...")

        self.model_path = weights_path
        self.camera_topic = '/fsds/cam1/image_color'
        
        # 1. CORRECTION DES COULEURS (Pour le cadre)
        self.COLORS = {
            0: (0, 255, 255),  # ID 0 -> Cadre JAUNE (BGR)
            1: (255, 0, 0),    # ID 1 -> Cadre BLEU  (BGR)
            2: (0, 255, 255),  # ID 2 -> Cadre JAUNE (au cas où)
        }

        # 2. CORRECTION DES NOMS (Pour le texte)
        # On remplace le nom interne du modèle par le vrai nom visuel
        self.CUSTOM_NAMES = {
            0: "JAUNE",        # Le modèle dit "blue_cone", on force "JAUNE"
            1: "BLEU",         # Le modèle dit "seg_blue", on force "BLEU"
            2: "JAUNE",        # Le modèle dit "yellow", on garde "JAUNE"
        }

        self.get_logger().info(f"🔧 Chargement modèle : {self.model_path}")
        
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            self.get_logger().error(f"❌ Erreur modèle : {e}")
            raise e

        self.bridge = CvBridge()

        # QoS Robustesse
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.listener_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info(" Perception lancée. Noms et Couleurs corrigés.")

    def listener_callback(self, msg):
        try:
            # 1. Réception
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = cv_image.copy()

            # 2. Inférence
            results = self.model(cv_image, verbose=False, conf=0.5)

            # 3. Traitement
            for box in results[0].boxes:
                # Infos brutes
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                # --- PARTIE TEXTE ---
                # Si l'ID est dans notre liste personnalisée, on prend notre nom
                if cls_id in self.CUSTOM_NAMES:
                    name_display = self.CUSTOM_NAMES[cls_id]
                else:
                    # Sinon on prend le nom par défaut du modèle
                    name_display = self.model.names[cls_id]

                label = f"{name_display} {conf:.2f}"
                # ----------------------------------

                # Choix Couleur
                color = self.COLORS.get(cls_id, (255, 255, 255))

                # Dessin Cadre
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                
                # Dessin Texte
                t_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(display_frame, (x1, y1 - 20), (x1 + t_size[0], y1), color, -1)
                cv2.putText(display_frame, label, (x1, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

            # 4. Affichage
            cv2.imshow("YOLO FINAL", display_frame)
            cv2.waitKey(1)

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
