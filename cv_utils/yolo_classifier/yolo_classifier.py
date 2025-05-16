import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D
import cv2
import os
from ultralytics import YOLO

class YoloClassifierNode(Node):
    def __init__(self):
        super().__init__('yolo_classifier')

        # Declare parameters for the model name and image topic
        self.declare_parameter('model', 'yolo_v1')
        self.declare_parameter('image_topic', '/vertical_camera/compressed')
        
        # Get parameter values
        model_name = self.get_parameter('model').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # Get the directory of the current script
        self.script_dir = os.path.dirname(os.path.realpath(__file__))

        # Construct the path to the model file
        self.model_path = os.path.join(self.script_dir, '../models', f'{model_name}.pt')

        # Load the model
        self.model = YOLO(self.model_path)

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Setup subscriber, publisher, and timer
        self.bridge = CvBridge()
        self.latest_image_msg = None  # To store the latest image message
        self.subscriber = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile
        )

        # Set dynamic topic names based on the model name
        classification_topic = '/vertical_classification'
        image_topic = f'/{model_name}/image/compressed'

        self.classification_publisher_ = self.create_publisher(Detection2DArray, classification_topic, 10)
        self.image_publisher_ = self.create_publisher(CompressedImage, image_topic, 10)

        # Set up a 10 Hz timer for processing images
        self.timer = self.create_timer(0.1, self.process_image_callback)

    def image_callback(self, msg):
        # Store the latest image message
        self.latest_image_msg = msg

    def process_image_callback(self):
        # Only process if a new image has been received
        if self.latest_image_msg is None:
            return

        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.bridge.compressed_imgmsg_to_cv2(self.latest_image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return
        
        # Convert the frame to a format suitable for the model
        input_image = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

        # Perform the prediction
        results = self.model(input_image)

        detections = Detection2DArray()

        # Process outputs
        for result in results:
            for box in result.boxes:
                
                # CLASSIFICATION POINTS
                detection = Detection2D()
                detection.bbox = BoundingBox2D()
                x_center, y_center, width, height = box.xywhn[0].cpu().numpy()
                detection.bbox.center.position.x = float(x_center)
                detection.bbox.center.position.y = float(y_center)
                detection.bbox.size_x = float(width)
                detection.bbox.size_y = float(height)
                if box.conf[0] > 0.3:
                    detections.detections.append(detection)

                # ANNOTATED IMAGES
                x1, y1, x2, y2 = box.xyxy[0]
                confidence = box.conf[0]

                # Draw the bounding box
                cv2.rectangle(current_frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)

                # Display the confidence
                label = f'{confidence:.2f}'
                label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                top = max(int(y1), label_size[1])
                cv2.rectangle(current_frame, (int(x1), top - label_size[1]), (int(x1) + label_size[0], top + base_line), (255, 0, 0), cv2.FILLED)
                cv2.putText(current_frame, label, (int(x1), top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        self.classification_publisher_.publish(detections)

        # Convert the modified frame back to an Image message
        try:
            annotated_msg = self.bridge.cv2_to_compressed_imgmsg(current_frame)
            self.get_logger().info("Publishing annotated image.")
            self.image_publisher_.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert annotated image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloClassifierNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
