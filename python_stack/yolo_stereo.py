#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from ultralytics import YOLO
import os
import torch

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

FSG_CONE_REAL_HEIGHT_M = 0.325

class YoloStereoNode(Node):
    def __init__(self):
        super().__init__('yolo_stereo_node')
        self.bridge = CvBridge()

        self.baseline = 0.32
        self.focal_length = 208.0
        self.fx = None

        script_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')

        self.get_logger().info(f"Dossier du script : {script_dir}")
        self.get_logger().info(f"Chemin des poids : {weights_path}")

        if not os.path.exists(weights_path):
            self.get_logger().error(f"FICHIER INTROUVABLE : {weights_path}")
            if os.path.exists(os.path.join(script_dir, 'best_FINAL.pt')):
                weights_path = os.path.join(script_dir, 'best_FINAL.pt')
                self.get_logger().warn(f"Trouve a la racine, utilisation de {weights_path}")
            else:
                raise FileNotFoundError(f"Impossible de trouver best_FINAL.pt dans {weights_path}")

        self.model = YOLO(weights_path)
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"YOLO lance sur : {device}")

        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

        self.left_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=5,
            P1=8 * 3 * 5 ** 2,
            P2=32 * 3 * 5 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=5,
            speckleWindowSize=100,
            speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        self.COLORS = {0: (0, 255, 255), 1: (255, 0, 0), 2: (0, 140, 255)}
        self.CUSTOM_NAMES = {0: "JAUNE", 1: "BLEU", 2: "ORANGE"}

        self.sub_cam_info = self.create_subscription(
            CameraInfo,
            '/fsds/cam1/camera_info',
            self.camera_info_cb,
            10
        )
        self.sub_left = message_filters.Subscriber(
            self, Image, '/fsds/cam1/image_color',
            qos_profile=qos_profile_sensor_data
        )
        self.sub_right = message_filters.Subscriber(
            self, Image, '/fsds/cam2/image_color',
            qos_profile=qos_profile_sensor_data
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right], 10, 0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.publisher = self.create_publisher(
            Detection2DArray, '/perception/stereo_detections', 10
        )

        self.get_logger().info("Perception Stereo lancee.")

    def camera_info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.focal_length = self.fx
            self.get_logger().info(f"Focale recue via camera_info : {self.focal_length:.2f} px")

    def compute_disparity(self, gray_left, gray_right):
        disp = self.left_matcher.compute(gray_left, gray_right)
        return disp.astype(np.float32) / 16.0

    def estimate_depth_stereo(self, disparity_f32, x1, y1, x2, y2):
        roi_x1 = max(0, x1)
        roi_y1 = max(0, y1)
        roi_x2 = min(disparity_f32.shape[1], x2)
        roi_y2 = min(disparity_f32.shape[0], y2)

        roi_disp = disparity_f32[roi_y1:roi_y2, roi_x1:roi_x2]
        valid_disp = roi_disp[roi_disp > 1.0]

        if len(valid_disp) < 3:
            return None, None

        d_obj = np.percentile(valid_disp, 85)
        if d_obj < 1.0:
            return None, None

        z = (self.focal_length * self.baseline) / d_obj

        STEREO_Z_MAX = 15.0
        if z > STEREO_Z_MAX:
            return None, None

        return z, 'STEREO'

    def estimate_depth_monocular(self, bbox_height_px):
        if bbox_height_px < 5:
            return None, None

        z = (self.focal_length * FSG_CONE_REAL_HEIGHT_M) / bbox_height_px
        return z, 'MONO'

    def compute_confidence(self, method, z, valid_count=0, disp_mean=0.0):
        if method is None or z is None or z <= 0:
            return 0.0

        if method == 'STEREO':
            STEREO_Z_MAX = 15.0
            valid_score = min(valid_count / 100.0, 1.0)
            disp_score  = min(disp_mean / 80.0, 1.0) if disp_mean > 0 else 0.0
            z_score     = max(0.0, (STEREO_Z_MAX - z) / STEREO_Z_MAX)
            conf = (0.35 * valid_score + 0.40 * disp_score + 0.25 * z_score) * 100.0

        elif method == 'MONO':
            bbox_h_est = (self.focal_length * FSG_CONE_REAL_HEIGHT_M) / z
            conf = min(bbox_h_est / 30.0, 1.0) * 100.0

        else:
            conf = 0.0

        return round(conf, 1)

    def sync_callback(self, msg_left, msg_right):
        try:
            img_left  = self.bridge.imgmsg_to_cv2(msg_left,  "bgr8")
            img_right = self.bridge.imgmsg_to_cv2(msg_right, "bgr8")
            display_frame = img_left.copy()

            with torch.no_grad():
                results = self.model(img_left, verbose=False, conf=0.5)

            lab_left  = cv2.cvtColor(img_left,  cv2.COLOR_BGR2LAB)
            lab_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2LAB)
            lab_left[:, :, 0]  = self.clahe.apply(lab_left[:, :, 0])
            lab_right[:, :, 0] = self.clahe.apply(lab_right[:, :, 0])

            bgr_left_clahe  = cv2.cvtColor(lab_left,  cv2.COLOR_LAB2BGR)
            bgr_right_clahe = cv2.cvtColor(lab_right, cv2.COLOR_LAB2BGR)

            gray_left  = cv2.cvtColor(bgr_left_clahe,  cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(bgr_right_clahe, cv2.COLOR_BGR2GRAY)

            disparity_f32 = self.compute_disparity(gray_left, gray_right)

            detections_msg = Detection2DArray()
            detections_msg.header = msg_left.header

            if len(results) > 0:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id     = int(box.cls[0])
                    yolo_conf  = float(box.conf[0])
                    class_name = self.model.names[cls_id].lower()

                    if "orange" in class_name or "large" in class_name:
                        key_id = 2
                    elif "blue" in class_name:
                        key_id = 1
                    elif "yellow" in class_name:
                        key_id = 0
                    else:
                        key_id = cls_id

                    w_box = x2 - x1
                    h_box = y2 - y1

                    z_obj, method = self.estimate_depth_stereo(
                        disparity_f32, x1, y1, x2, y2
                    )
                    if z_obj is None:
                        z_obj, method = self.estimate_depth_monocular(h_box)

                    roi_disp_dbg = disparity_f32[
                        max(0, y1):min(disparity_f32.shape[0], y2),
                        max(0, x1):min(disparity_f32.shape[1], x2)
                    ]
                    valid_dbg    = roi_disp_dbg[roi_disp_dbg > 1.0]
                    disp_mean_val = float(roi_disp_dbg.mean()) if roi_disp_dbg.size > 0 else 0.0

                    depth_conf = self.compute_confidence(
                        method, z_obj,
                        valid_count=len(valid_dbg),
                        disp_mean=disp_mean_val
                    )

                    if z_obj is not None:
                        tag    = '[S]' if method == 'STEREO' else '[M]'
                        text_z = f"Z:{z_obj:.2f}m{tag} {depth_conf:.0f}%"
                    else:
                        text_z     = "Z:N/A"
                        depth_conf = 0.0

                    border_color = self.COLORS.get(key_id, (0, 255, 0))

                    label_text = self.CUSTOM_NAMES.get(key_id, class_name.upper())
                    full_label = f"{label_text} {yolo_conf:.2f} {text_z}"

                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), border_color, 2)
                    (w_text, h_text), _ = cv2.getTextSize(
                        full_label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                    )
                    cv2.rectangle(
                        display_frame, (x1, y1 - 18), (x1 + w_text, y1), border_color, -1
                    )
                    cv2.putText(
                        display_frame, full_label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1
                    )

                    w  = float(w_box)
                    h  = float(h_box)
                    cx = float(x1 + w / 2.0)
                    cy = float(y1 + h / 2.0)

                    detection = Detection2D()
                    detection.header = msg_left.header

                    try:
                        detection.bbox.center.x     = cx
                        detection.bbox.center.y     = cy
                        detection.bbox.center.theta = 0.0
                    except AttributeError:
                        try:
                            detection.bbox.center.position.x = cx
                            detection.bbox.center.position.y = cy
                        except AttributeError:
                            pass

                    detection.bbox.size_x = w
                    detection.bbox.size_y = h

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(key_id)
                    if method == 'STEREO':
                        hyp.hypothesis.score =  depth_conf / 100.0
                    elif method == 'MONO':
                        hyp.hypothesis.score = -(depth_conf / 100.0)
                    else:
                        hyp.hypothesis.score = 0.0

                    detection.results.append(hyp)
                    detections_msg.detections.append(detection)

            self.publisher.publish(detections_msg)

            cv2.imshow("STEREO FUSION FINAL", display_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erreur Callback : {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = YoloStereoNode()
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
