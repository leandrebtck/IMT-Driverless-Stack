import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
import sys
import torch  
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose #(fusion)


class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')

        # --- CONFIGURATION ---
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # On construit le chemin vers le poids
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')

        self.get_logger().info(f"📍 Dossier du script : {script_dir}")
        self.get_logger().info(f"⚖️ Chemin calculé du .pt : {weights_path}")

        # Vérification 
        if not os.path.exists(weights_path):
            self.get_logger().error(
                f"❌ FICHIER INTROUVABLE ! Vérifier que le dossier 'weights' est bien dans {script_dir}"
            )
        else:
            self.get_logger().info("✅ Fichier trouvé ! Chargement...")

        self.model_path = weights_path
        self.camera_topic = '/fsds/cam1/image_color'

        # Couleurs (BGR)
        self.COLORS = {
            0: (0, 255, 255),
            1: (255, 0, 0),
            2: (0, 255, 255),
        }

        # Noms personnalisés
        self.CUSTOM_NAMES = {
            0: "JAUNE",
            1: "BLEU",
            2: "JAUNE",
        }

        self.get_logger().info(f"🔧 Chargement modèle : {self.model_path}")
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            self.get_logger().error(f"❌ Erreur modèle : {e}")
            raise e

        # 🔹 MODE INFÉRENCE
        self.model.eval()

        # 🔹 CHOIX DU DEVICE
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"🚀 YOLO lancé sur : {device}")

        self.bridge = CvBridge()

        # QoS Robustesse
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.listener_callback,
            qos_profile_sensor_data
        )

        # AJOUT FUSION:Publisher 
        # On crée le canal pour envoyer les détections au script de fusion
        self.publisher = self.create_publisher(
            Detection2DArray, 
            '/yolo/detections', 
            10
        )

        self.get_logger().info("✅ Perception lancée. Noms et couleurs corrigés.")

    def listener_callback(self, msg):
        try:
            # 1. Réception
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = cv_image.copy()

            # 2. Inférence
            with torch.no_grad():   # 🔹 AJOUT
                results = self.model(cv_image, verbose=False, conf=0.5)

            #AJOUT FUSION: Préparation du message global 
            # On prépare le paquet vide qui va contenir toutes les boîtes
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header  # Très important : on garde la même heure que l'image
    
            # 3. Traitement
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                name_display = self.CUSTOM_NAMES.get(
                    cls_id, self.model.names[cls_id]
                )

                label = f"{name_display} {conf:.2f}"
                color = self.COLORS.get(cls_id, (255, 255, 255))

                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)

                t_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(
                    display_frame, (x1, y1 - 20),
                    (x1 + t_size[0], y1), color, -1
                )
                cv2.putText(
                    display_frame, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1
                )

                #AJOUT FUSION : Remplissage du message ROS 
                # Conversion Coin Haut-Gauche (x1,y1) -> Centre (cx, cy) pour ROS
                w = float(x2 - x1)
                h = float(y2 - y1)
                cx = float(x1 + w / 2.0)
                cy = float(y1 + h / 2.0)

                # Création de l'objet détection unique
                detection = Detection2D()
                detection.header = msg.header
                
                # Géométrie de la boîte
                detection.bbox.center.x = cx
                detection.bbox.center.y = cy
                detection.bbox.size_x = w
                detection.bbox.size_y = h

                # Ce qu'il y a dedans (Classe + Score)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(cls_id) # "0" ou "1"
                hypothesis.hypothesis.score = conf
                
                detection.results.append(hypothesis)
                
                # On ajoute cette boîte au paquet global
                detections_msg.detections.append(detection)
        

            #AJOUT FUSION: Envoi final
            # On publie le paquet complet sur le réseau ROS
            self.publisher.publish(detections_msg)
    
            # 4. Affichage
            cv2.imshow("YOLO FINAL", display_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ Erreur Display : {e}")

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
