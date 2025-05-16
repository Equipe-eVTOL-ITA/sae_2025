import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from custom_msgs.msg import BarCode, MultiBarCode
from cv_bridge import CvBridge
from roboflowoak import RoboflowOak
import cv2
import numpy as np
import time

class BarcodeSubscriber(Node):
    def __init__(self):
        super().__init__('barcode_subscriber')
        
        # Initialize RoboflowOak model
        self.rf = RoboflowOak(
            model="barcode_detection-sgwcz",
            confidence=0.5,
            overlap=0.5,
            version="2",
            api_key="FdbPQV6UvRCTuTrFHVpY",
            rgb=True,
            depth=True,
            device=None,
            blocking=True
        )

        # Publisher for the annotated image
        self.image_publisher = self.create_publisher(CompressedImage, 'camera/annotated/image/compressed', 10)
        
        # Publisher for aggregated bounding box info
        self.multi_bbox_publisher = self.create_publisher(MultiBarCode, 'barcode/bounding_boxes', 10)
        
        self.bridge = CvBridge()

    def start_detection_loop(self):
        while rclpy.ok():
            # Run the inference and get the results
            t0 = time.time()
            result, frame, raw_frame, depth = self.rf.detect()
            predictions = result["predictions"]

            # Annotate the frame with bounding boxes
            annotated_frame = self.annotate_frame(frame, predictions)

            # Create and publish MultiBarCode message with all detections
            multi_bbox_msg = self.create_multi_bounding_box_message(predictions, frame.shape)
            self.multi_bbox_publisher.publish(multi_bbox_msg)

            # Convert annotated frame to a compressed ROS image message
            annotated_msg = self.bridge.cv2_to_compressed_imgmsg(annotated_frame)
            
            # Publish the annotated image
            self.image_publisher.publish(annotated_msg)

            # Display inference time for benchmarking
            print(f"INFERENCE TIME IN MS: {1/(time.time() - t0)}")

    def annotate_frame(self, frame, predictions):
        for pred in predictions:
            # Access attributes directly from the Prediction object
            x, y, width, height = pred.x, pred.y, pred.width, pred.height
            start_point = (int(x - width / 2), int(y - height / 2))
            end_point = (int(x + width / 2), int(y + height / 2))
            cv2.rectangle(frame, start_point, end_point, (0, 255, 0), 2)
        return frame

    def create_multi_bounding_box_message(self, predictions, frame_shape):
        # Create MultiBarCode message
        multi_bbox_msg = MultiBarCode()

        # Populate MultiBarCode with individual BarCode messages
        for prediction in predictions:
            # Normalize center coordinates, width, and height
            center_x = prediction.x / frame_shape[1]
            center_y = prediction.y / frame_shape[0]
            width = prediction.width / frame_shape[1]
            height = prediction.height / frame_shape[0]

            # Create individual BarCode message
            bbox_msg = BarCode()
            bbox_msg.center_x = float(center_x)
            bbox_msg.center_y = float(center_y)
            bbox_msg.width = float(width)
            bbox_msg.height = float(height)
            
            # Append to the MultiBarCode array
            multi_bbox_msg.barcodes.append(bbox_msg)
        
        return multi_bbox_msg

def main(args=None):
    rclpy.init(args=args)
    barcode_subscriber = BarcodeSubscriber()
    barcode_subscriber.start_detection_loop()
    barcode_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
