import rclpy
from rclpy.node import Node
from roboflowoak import RoboflowOak
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import String
from pyzbar.pyzbar import decode  # QR Code reader

class QRCodeDetectionNode(Node):
    def __init__(self):
        super().__init__('qr_code_detection_node')

        # ROS 2 Publishers
        self.qr_location_pub = self.create_publisher(Detection2DArray, '/vertical_classification', 10)
        self.annotated_image_pub = self.create_publisher(CompressedImage, '/annotated_image/compressed', 10)
        self.qr_string_pub = self.create_publisher(String, '/qr_code_string', 10)

        # Roboflow Model Configuration
        self.rf = RoboflowOak(
            model="qr-code-80bee", confidence=0.5, overlap=0.5,
            version="1", api_key="FdbPQV6UvRCTuTrFHVpY", rgb=True,
            depth=False, device=None, blocking=True
        )

        self.timer = self.create_timer(0.1, self.detect_qr_codes)  # 10 Hz processing

    def detect_qr_codes(self):
        result, frame, raw_frame, _ = self.rf.detect()

        # Predictions from Roboflow
        predictions = result["predictions"]

        # Annotate and process predictions
        detections_msg = Detection2DArray()
        annotated_frame = frame.copy()

        if predictions:
            for prediction in predictions:
                x_center = prediction.x / frame.shape[1]
                y_center = prediction.y / frame.shape[0]
                width = prediction.width / frame.shape[1]
                height = prediction.height / frame.shape[0]

                # Add bounding box to detections
                detection = Detection2D()
                detection.bbox = BoundingBox2D()
                detection.bbox.center.position.x = x_center
                detection.bbox.center.position.y = y_center
                detection.bbox.size_x = width
                detection.bbox.size_y = height
                detections_msg.detections.append(detection)

                # Draw bounding box on the frame
                start_point = (int(prediction.x - prediction.width / 2), int(prediction.y - prediction.height / 2))
                end_point = (int(prediction.x + prediction.width / 2), int(prediction.y + prediction.height / 2))
                cv2.rectangle(annotated_frame, start_point, end_point, (0, 255, 0), 2)
                cv2.putText(annotated_frame, prediction.class_name, start_point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            # No detections: publish a default bounding box with zeros
            default_detection = Detection2D()
            default_detection.bbox = BoundingBox2D()
            default_detection.bbox.center.position.x = 0.000
            default_detection.bbox.center.position.y = 0.000
            default_detection.bbox.size_x = 0.000
            default_detection.bbox.size_y = 0.000
            detections_msg.detections.append(default_detection)

        # Publish detections
        self.qr_location_pub.publish(detections_msg)

        # Publish the annotated image
        compressed_image_msg = CompressedImage()
        compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = np.array(cv2.imencode('.jpg', annotated_frame)[1]).tobytes()
        self.annotated_image_pub.publish(compressed_image_msg)

        # Decode QR codes for string content
        decoded_strings = decode(raw_frame)

        if decoded_strings:
            for qr in decoded_strings:
                qr_string_msg = String()
                qr_string_msg.data = qr.data.decode('utf-8')
                self.qr_string_pub.publish(qr_string_msg)
        else:
            # Publish an empty string if no QR code is detected
            qr_string_msg = String()
            qr_string_msg.data = ""
            self.qr_string_pub.publish(qr_string_msg)


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
