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
        self.confidence_threshold = 0.2
        
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
        mask = boxes.conf >= self.confidence_threshold
        if not mask.any():
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = f"No detections above {self.confidence_threshold} confidence"
            return response

        # Detection successful
        # Get highest confidence detection
        best_idx = boxes.conf.argmax()
        class_id = int(boxes.cls[best_idx])
        confidence = float(boxes.conf[best_idx])
        class_name = self.model.names[class_id]

        response.success = True
        response.class_id = class_id
        response.class_name = class_name
        response.confidence = confidence
        response.message = f"Detected {class_name}"

        # Save image of detected object
        if save_detected_image:
            filename = f"/home/nicolas-rebollo/YOLO Images/{class_name}.jpg"
            annotated_image = results[0].plot()
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
