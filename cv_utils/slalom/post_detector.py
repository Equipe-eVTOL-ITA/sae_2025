
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
import cv2
import numpy as np
from .color_segmentation import ColorSegmentation

class PostDetector(Node):
    def __init__(self):
        super().__init__('post_detector')

        # Declare parameters
        self.declare_parameter('image_topic', '/vertical_camera/compressed')
        self.declare_parameter('min_area', 500)
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('morph_iterations', 2)
        self.declare_parameter('min_aspect_ratio', 0.1)
        self.declare_parameter('max_aspect_ratio', 2.0)
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('processing_frequency', 10.0)  # Hz
        
        # Color range parameters for HSV segmentation
        # Preto (Black)
        self.declare_parameter('preto_lower_h', 0)
        self.declare_parameter('preto_lower_s', 0)
        self.declare_parameter('preto_lower_v', 0)
        self.declare_parameter('preto_upper_h', 180)
        self.declare_parameter('preto_upper_s', 255)
        self.declare_parameter('preto_upper_v', 60)
        
        # Azul (Blue)
        self.declare_parameter('azul_lower_h', 100)
        self.declare_parameter('azul_lower_s', 50)
        self.declare_parameter('azul_lower_v', 50)
        self.declare_parameter('azul_upper_h', 130)
        self.declare_parameter('azul_upper_s', 255)
        self.declare_parameter('azul_upper_v', 255)
        
        # Vermelho (Red) - Two ranges due to HSV wrap-around
        self.declare_parameter('vermelho_lower1_h', 0)
        self.declare_parameter('vermelho_lower1_s', 50)
        self.declare_parameter('vermelho_lower1_v', 50)
        self.declare_parameter('vermelho_upper1_h', 10)
        self.declare_parameter('vermelho_upper1_s', 255)
        self.declare_parameter('vermelho_upper1_v', 255)
        self.declare_parameter('vermelho_lower2_h', 170)
        self.declare_parameter('vermelho_lower2_s', 50)
        self.declare_parameter('vermelho_lower2_v', 50)
        self.declare_parameter('vermelho_upper2_h', 180)
        self.declare_parameter('vermelho_upper2_s', 255)
        self.declare_parameter('vermelho_upper2_v', 255)
        
        # Rosa (Pink)
        self.declare_parameter('rosa_lower_h', 140)
        self.declare_parameter('rosa_lower_s', 50)
        self.declare_parameter('rosa_lower_v', 50)
        self.declare_parameter('rosa_upper_h', 170)
        self.declare_parameter('rosa_upper_s', 255)
        self.declare_parameter('rosa_upper_v', 255)
        
        # Get parameter values
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        processing_freq = self.get_parameter('processing_frequency').get_parameter_value().double_value
        
        # Initialize color segmentation with parameters
        self.color_segmentation = ColorSegmentation()
        self._update_segmentation_parameters()

        # Setup QoS profile
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Setup subscriber, publishers, and timer
        self.bridge = CvBridge()
        self.latest_image_msg = None
        
        self.subscriber = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile
        )

        # Create dynamic topic names based on image topic
        base_topic = self.image_topic.replace('/compressed', '').replace('/', '')
        if base_topic.startswith('_'):
            base_topic = base_topic[1:]
        
        classification_topic = '/slalom'
        annotated_image_topic = f'/post_detector/{base_topic}/image'

        self.classification_publisher = self.create_publisher(Detection2DArray, classification_topic, 10)
        self.image_publisher = self.create_publisher(Image, annotated_image_topic, 10)

        # Set up timer for processing images
        timer_period = 1.0 / processing_freq
        self.timer = self.create_timer(timer_period, self.process_image_callback)
        
        self.get_logger().info(f'Post Detector initialized')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info(f'Publishing detections to: {classification_topic}')
        self.get_logger().info(f'Publishing annotated images to: {annotated_image_topic}')

    def _update_segmentation_parameters(self):
        """Update color segmentation parameters from ROS parameters"""
        
        # Update basic parameters
        min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        morph_kernel_size = self.get_parameter('morph_kernel_size').get_parameter_value().integer_value
        morph_iterations = self.get_parameter('morph_iterations').get_parameter_value().integer_value
        min_aspect_ratio = self.get_parameter('min_aspect_ratio').get_parameter_value().double_value
        max_aspect_ratio = self.get_parameter('max_aspect_ratio').get_parameter_value().double_value
        
        self.color_segmentation.update_parameters(
            min_area=min_area,
            morph_kernel_size=morph_kernel_size,
            morph_iterations=morph_iterations,
            min_aspect_ratio=min_aspect_ratio,
            max_aspect_ratio=max_aspect_ratio
        )
        
        # Update color ranges
        color_ranges = {
            'Preto': {
                'lower': np.array([
                    self.get_parameter('preto_lower_h').get_parameter_value().integer_value,
                    self.get_parameter('preto_lower_s').get_parameter_value().integer_value,
                    self.get_parameter('preto_lower_v').get_parameter_value().integer_value
                ]),
                'upper': np.array([
                    self.get_parameter('preto_upper_h').get_parameter_value().integer_value,
                    self.get_parameter('preto_upper_s').get_parameter_value().integer_value,
                    self.get_parameter('preto_upper_v').get_parameter_value().integer_value
                ])
            },
            'Azul': {
                'lower': np.array([
                    self.get_parameter('azul_lower_h').get_parameter_value().integer_value,
                    self.get_parameter('azul_lower_s').get_parameter_value().integer_value,
                    self.get_parameter('azul_lower_v').get_parameter_value().integer_value
                ]),
                'upper': np.array([
                    self.get_parameter('azul_upper_h').get_parameter_value().integer_value,
                    self.get_parameter('azul_upper_s').get_parameter_value().integer_value,
                    self.get_parameter('azul_upper_v').get_parameter_value().integer_value
                ])
            },
            'Vermelho': {
                'lower1': np.array([
                    self.get_parameter('vermelho_lower1_h').get_parameter_value().integer_value,
                    self.get_parameter('vermelho_lower1_s').get_parameter_value().integer_value,
                    self.get_parameter('vermelho_lower1_v').get_parameter_value().integer_value
                ]),
                'upper1': np.array([
                    self.get_parameter('vermelho_upper1_h').get_parameter_value().integer_value,
                    self.get_parameter('vermelho_upper1_s').get_parameter_value().integer_value,
                    self.get_parameter('vermelho_upper1_v').get_parameter_value().integer_value
                ]),
                'lower2': np.array([
                    self.get_parameter('vermelho_lower2_h').get_parameter_value().integer_value,
                    self.get_parameter('vermelho_lower2_s').get_parameter_value().integer_value,
                    self.get_parameter('vermelho_lower2_v').get_parameter_value().integer_value
                ]),
                'upper2': np.array([
                    self.get_parameter('vermelho_upper2_h').get_parameter_value().integer_value,
                    self.get_parameter('vermelho_upper2_s').get_parameter_value().integer_value,
                    self.get_parameter('vermelho_upper2_v').get_parameter_value().integer_value
                ])
            },
            'Rosa': {
                'lower': np.array([
                    self.get_parameter('rosa_lower_h').get_parameter_value().integer_value,
                    self.get_parameter('rosa_lower_s').get_parameter_value().integer_value,
                    self.get_parameter('rosa_lower_v').get_parameter_value().integer_value
                ]),
                'upper': np.array([
                    self.get_parameter('rosa_upper_h').get_parameter_value().integer_value,
                    self.get_parameter('rosa_upper_s').get_parameter_value().integer_value,
                    self.get_parameter('rosa_upper_v').get_parameter_value().integer_value
                ])
            }
        }
        
        self.color_segmentation.update_color_ranges(color_ranges)

    def image_callback(self, msg):
        """Store the latest image message"""
        self.latest_image_msg = msg

    def process_image_callback(self):
        """Process the latest image for post detection"""
        if self.latest_image_msg is None:
            return

        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.bridge.compressed_imgmsg_to_cv2(self.latest_image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return

        # Perform color segmentation and detection
        detections, visualization_mask = self.color_segmentation.segment_all_colors(current_frame)

        # Create Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header = self.latest_image_msg.header

        # Create annotated image
        annotated_frame = current_frame.copy()
        
        # Apply transparent mask overlay (alpha controls mask transparency)
        alpha = 0.3  # Lower value = more transparent mask, higher value = more opaque mask
        annotated_frame = cv2.addWeighted(annotated_frame, 1 - alpha, visualization_mask, alpha, 0)

        for detection in detections:
            class_name = detection['class']
            bbox = detection['bbox']
            area = detection['area']
            x, y, w, h = bbox
            
            # Calculate confidence based on multiple factors
            # 1. Area factor: Larger detections are more confident
            area_factor = min(1.0, area / 2000.0)  # Normalize to reasonable post area
            
            # 2. Aspect ratio factor: Posts should be vertical (height >> width)
            aspect_ratio = w / h if h > 0 else 0
            # Ideal aspect ratio for posts is ~0.04 (0.1m / 2.5m), allow range 0.02-0.2
            if 0.02 <= aspect_ratio <= 0.1:
                aspect_factor = 1.0  # Perfect vertical post
            elif 0.1 < aspect_ratio <= 0.2:
                aspect_factor = 0.8  # Still good vertical post
            elif 0.2 < aspect_ratio <= 0.5:
                aspect_factor = 0.6  # Less vertical but acceptable
            else:
                aspect_factor = 0.3  # Not very post-like
            
            # 3. Height factor: Posts should have significant height
            height_factor = min(1.0, h / 100.0)  # Normalize to reasonable post height in pixels
            
            # 4. Width factor: Posts shouldn't be too wide
            width_factor = max(0.5, min(1.0, 50.0 / max(w, 1)))  # Penalize very wide detections
            
            # Combine factors for final confidence
            confidence = min(0.99, max(0.3, area_factor * aspect_factor * height_factor * width_factor))
            
            # Filter by confidence threshold
            if confidence < self.confidence_threshold:
                continue

            # Create Detection2D message
            det_msg = Detection2D()
            det_msg.header = self.latest_image_msg.header
            
            # Normalize bounding box
            x_center, y_center, norm_w, norm_h = self.color_segmentation.normalize_bbox(
                bbox, current_frame.shape[:2]
            )
            
            det_msg.bbox = BoundingBox2D()
            det_msg.bbox.center.position.x = x_center
            det_msg.bbox.center.position.y = y_center
            det_msg.bbox.size_x = norm_w
            det_msg.bbox.size_y = norm_h

            # Create hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = confidence

            det_msg.results.append(hypothesis)
            detection_array.detections.append(det_msg)

            # Draw bounding box on annotated image
            x, y, w, h = bbox
            color_bgr = self.color_segmentation._get_color_bgr(class_name)
            
            # Draw rectangle
            cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), color_bgr, 3)
            
            # Draw label
            label = f'{class_name}: {confidence:.2f}'
            label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            
            # Background rectangle for text
            cv2.rectangle(annotated_frame, 
                         (x, y - label_size[1] - baseline - 5), 
                         (x + label_size[0], y), 
                         color_bgr, cv2.FILLED)
            
            # Text
            cv2.putText(annotated_frame, label, 
                       (x, y - baseline - 2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Publish detections
        self.classification_publisher.publish(detection_array)

        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            annotated_msg.header = self.latest_image_msg.header
            self.image_publisher.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert annotated image: {str(e)}")

        # Log detection info
        if len(detection_array.detections) > 0:
            detected_classes = [det.results[0].hypothesis.class_id for det in detection_array.detections]
            self.get_logger().info(f"Detected posts: {detected_classes}")


def main(args=None):
    rclpy.init(args=args)
    node = PostDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()