#!/usr/bin/env python3
"""
YOLO Object Detection Service for MIE443 Contest 2
Detects objects using YOLOv8/v13 and returns the highest confidence detection.
"""

import rclpy
from rclpy.node import Node
from mie443_contest2.srv import DetectObject
import cv2
import numpy as np
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLO model
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded')
        
        # Confidence threshold
        self.confidence_threshold = 0.6

        # Allowed classes for contest logic (only these are considered valid)
        self.allowed_classes = {
            "clock",
            "cup",
            "bottle",
            "motorcycle",
            "potted plant",
        }
        
        # Create service
        self.service = self.create_service(
            DetectObject,
            'detect_object',
            self.detect_callback
        )
        
        self.get_logger().info('YOLO Detector Service ready')

    def detect_callback(self, request, response):
        """Process image and return highest confidence detection."""
        
        # Decode compressed image
        np_arr = np.frombuffer(request.image.data, np.uint8)
        save_detected_image = request.save_detected_image
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if image is None:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "Failed to decode image"
            return response
        
        # Run YOLO inference
        results = self.model(image, verbose=False, device='cpu')
        boxes = results[0].boxes

        ### YOUR CODE HERE ###

        # No object detected because of no bounding boxes
        if boxes is None or len(boxes) == 0:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "No objects detected"
            return response

        # Filter by confidence threshold
        conf = boxes.conf.cpu().numpy()
        conf_mask = conf >= self.confidence_threshold
        if not conf_mask.any():
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = f"No detections above {self.confidence_threshold} confidence"
            return response

        # Filter by allowed classes
        class_ids = boxes.cls.cpu().numpy().astype(int)
        class_names = [self.model.names[i] for i in class_ids]
        allowed_mask = np.array([name in self.allowed_classes for name in class_names], dtype=bool)
        final_mask = conf_mask & allowed_mask
        if not final_mask.any():
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "No allowed objects detected above confidence threshold"
            return response

        # Detection successful (highest confidence among allowed)
        allowed_idxs = np.where(final_mask)[0]
        best_idx = allowed_idxs[np.argmax(conf[allowed_idxs])]
        class_id = int(class_ids[best_idx])
        confidence = float(conf[best_idx])
        class_name = class_names[best_idx]

        response.success = True
        response.class_id = class_id
        response.class_name = class_name
        response.confidence = confidence
        response.message = f"Detected {class_name}"

        # Save image with only allowed detections drawn
        if save_detected_image:
            filename = f"/home/nicolas-rebollo/YOLO Images/{class_name}.jpg"
            annotated_image = image.copy()
            boxes_xyxy = boxes.xyxy.cpu().numpy()
            for i in allowed_idxs:
                x1, y1, x2, y2 = boxes_xyxy[i].astype(int)
                label = f"{class_names[i]} {conf[i]:.2f}"
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated_image,
                    label,
                    (x1, max(0, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
            cv2.imwrite(filename, annotated_image)
            self.get_logger().info(f"Saved detection image to {filename}")

        self.get_logger().info(f"Detected: {class_name} ({confidence:.2f})")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
