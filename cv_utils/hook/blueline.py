#!/usr/bin/env python3
"""
Blue Line Detector for Line Following

This ROS 2 node detects a blue fabric line (25cm width) using the vertical camera
and publishes line guidance information for the drone to follow the line.

The detector uses HSV color segmentation to identify blue pixels, then calculates
the line's centroid and orientation to guide the drone's movement. The region of
interest is vertical (left-right), and angles are published relative to vertical
lines with clockwise being positive, in radians, ranging from -π/2 to π/2.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PointStamped, Vector3Stamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from typing import Tuple, Optional
import math

class BlueLineDetector(Node):
    def __init__(self):
        super().__init__('blueline_detector')

        # Declare parameters
        self.declare_parameter('image_topic', '/vertical_camera/compressed')
        self.declare_parameter('processing_frequency', 10.0)  # Hz
        self.declare_parameter('confidence_threshold', 0.5)
        
        # Blue line detection parameters
        self.declare_parameter('min_line_area', 200)  # Minimum area for valid line detection
        self.declare_parameter('morph_kernel_size', 7)
        self.declare_parameter('morph_iterations', 3)
        
        # Blue line HSV parameters (navy blue fabric)
        self.declare_parameter('blue_lower_h', 80)
        self.declare_parameter('blue_lower_s', 70)
        self.declare_parameter('blue_lower_v', 25)
        self.declare_parameter('blue_upper_h', 140)
        self.declare_parameter('blue_upper_s', 255)
        self.declare_parameter('blue_upper_v', 255)
        
        # Line following parameters
        self.declare_parameter('roi_left', 0.2)     # ROI left as fraction of image width
        self.declare_parameter('roi_right', 0.8)    # ROI right as fraction of image width
        self.declare_parameter('min_line_length', 20)  # Minimum pixels for valid line segment
        self.declare_parameter('max_line_gap', 50)     # Maximum gap to connect line segments
        
        # Get parameter values
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        processing_freq = self.get_parameter('processing_frequency').get_parameter_value().double_value
        
        # Line detection parameters
        self.min_line_area = self.get_parameter('min_line_area').get_parameter_value().integer_value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').get_parameter_value().integer_value
        self.morph_iterations = self.get_parameter('morph_iterations').get_parameter_value().integer_value
        
        # HSV color range for blue line
        self.blue_lower = np.array([
            self.get_parameter('blue_lower_h').get_parameter_value().integer_value,
            self.get_parameter('blue_lower_s').get_parameter_value().integer_value,
            self.get_parameter('blue_lower_v').get_parameter_value().integer_value
        ])
        self.blue_upper = np.array([
            self.get_parameter('blue_upper_h').get_parameter_value().integer_value,
            self.get_parameter('blue_upper_s').get_parameter_value().integer_value,
            self.get_parameter('blue_upper_v').get_parameter_value().integer_value
        ])
        
        # ROI parameters
        self.roi_left = self.get_parameter('roi_left').get_parameter_value().double_value
        self.roi_right = self.get_parameter('roi_right').get_parameter_value().double_value
        self.min_line_length = self.get_parameter('min_line_length').get_parameter_value().integer_value
        self.max_line_gap = self.get_parameter('max_line_gap').get_parameter_value().integer_value

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

        # Create dynamic topic names
        base_topic = self.image_topic.replace('/compressed', '').replace('/', '')
        if base_topic.startswith('_'):
            base_topic = base_topic[1:]
        
        # Publishers
        self.line_centroid_publisher = self.create_publisher(PointStamped, '/blueline/centroid', 10)
        self.line_direction_publisher = self.create_publisher(Vector3Stamped, '/blueline/direction', 10)
        self.image_publisher = self.create_publisher(Image, f'/blueline_detector/{base_topic}/image', 10)

        # Set up timer for processing images
        timer_period = 1.0 / processing_freq
        self.timer = self.create_timer(timer_period, self.process_image_callback)
        
        self.get_logger().info(f'Blue Line Detector initialized')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info(f'Publishing centroid to: /blueline/centroid')
        self.get_logger().info(f'Publishing direction to: /blueline/direction')
        self.get_logger().info(f'Blue HSV Range: H={self.blue_lower[0]}-{self.blue_upper[0]}, S={self.blue_lower[1]}-{self.blue_upper[1]}, V={self.blue_lower[2]}-{self.blue_upper[2]}')

    def image_callback(self, msg):
        """Store the latest image message"""
        self.latest_image_msg = msg

    def segment_blue_line(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Segment blue line from the image
        
        Args:
            image: Input BGR image
            
        Returns:
            mask: Binary mask of blue line pixels
            colored_mask: Colored mask for visualization
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for blue color
        mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        
        # Apply ROI (focus on middle portion of image)
        height, width = mask.shape
        roi_start = int(width * self.roi_left)
        roi_end = int(width * self.roi_right)
        
        # Zero out areas outside ROI
        mask[:, :roi_start] = 0
        mask[:, roi_end:] = 0
        
        # Apply morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                         (self.morph_kernel_size, self.morph_kernel_size))
        
        # Close to connect broken line segments
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.morph_iterations)
        
        # Use a vertical kernel to connect vertical line segments better
        vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, self.morph_kernel_size * 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, vertical_kernel, iterations=1)
        
        # Open to remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Create colored mask for visualization (cyan for blue line)
        colored_mask = np.zeros_like(image)
        colored_mask[mask > 0] = (255, 255, 0)  # Cyan in BGR
        
        return mask, colored_mask

    def analyze_line(self, mask: np.ndarray, image_shape: Tuple[int, int]) -> Tuple[Optional[Tuple[float, float]], Optional[float], float]:
        """
        Analyze the line mask to extract centroid and direction
        
        Args:
            mask: Binary mask of the line
            image_shape: (height, width) of the original image
            
        Returns:
            centroid: (x, y) normalized coordinates of line centroid
            angle: Line direction angle in radians relative to vertical (clockwise positive, range: -π/2 to π/2)
            confidence: Detection confidence [0, 1]
        """
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None, 0.0
        
        # Filter contours by area
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= self.min_line_area:
                valid_contours.append(contour)
        
        if not valid_contours:
            return None, None, 0.0
        
        # Combine all valid contours
        all_points = np.vstack(valid_contours)
        
        # Calculate moments for centroid
        moments = cv2.moments(all_points)
        
        if moments['m00'] == 0:
            return None, None, 0.0
        
        # Calculate centroid
        cx = moments['m10'] / moments['m00']
        cy = moments['m01'] / moments['m00']
        
        # Normalize coordinates
        height, width = image_shape
        norm_cx = cx / width
        norm_cy = cy / height
        
        # Fit line to get direction
        [vx, vy, x0, y0] = cv2.fitLine(all_points, cv2.DIST_L2, 0, 0.01, 0.01)
        
        # Calculate angle relative to vertical in radians
        # atan2(vx, vy) gives angle from vertical (clockwise positive)
        # Range is limited to -π/2 to π/2 (-90° to 90°)
        angle_radians = math.atan2(vx, vy)
        
        # Ensure angle is within -π/2 to π/2 range
        if angle_radians > math.pi/2:
            angle_radians -= math.pi
        elif angle_radians < -math.pi/2:
            angle_radians += math.pi
        
        # Calculate confidence based on line area relative to ROI
        roi_width = int(width * (self.roi_right - self.roi_left))
        roi_area = height * roi_width
        line_area = np.sum(mask > 0)
        
        # Confidence based on how much of the ROI is covered by the line
        # A good line should cover 5-20% of the ROI area
        area_ratio = line_area / roi_area
        if 0.05 <= area_ratio <= 0.2:
            confidence = min(1.0, area_ratio * 5.0)  # Scale 0.05-0.2 to 0.25-1.0
        else:
            confidence = max(0.1, min(0.8, area_ratio * 2.0))  # Lower confidence for unusual ratios
        
        return (norm_cx, norm_cy), angle_radians, confidence

    def process_image_callback(self):
        """Process the latest image for line detection"""
        if self.latest_image_msg is None:
            return

        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.bridge.compressed_imgmsg_to_cv2(self.latest_image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return

        # Segment blue line
        mask, colored_mask = self.segment_blue_line(current_frame)
        
        # Analyze line
        centroid, angle, confidence = self.analyze_line(mask, current_frame.shape[:2])
        
        # Create annotated image
        annotated_frame = current_frame.copy()
        
        # Apply transparent mask overlay
        alpha = 0.4
        annotated_frame = cv2.addWeighted(annotated_frame, 1 - alpha, colored_mask, alpha, 0)
        
        # Draw ROI rectangle
        height, width = current_frame.shape[:2]
        roi_start = int(width * self.roi_left)
        roi_end = int(width * self.roi_right)
        cv2.rectangle(annotated_frame, (roi_start, 0), (roi_end, height), (255, 255, 255), 2)
        
        # If line detected, draw visualization and publish data
        if centroid is not None and confidence >= self.confidence_threshold:
            # Convert normalized centroid back to pixel coordinates
            pixel_cx = int(centroid[0] * width)
            pixel_cy = int(centroid[1] * height)
            
            # Draw centroid
            cv2.circle(annotated_frame, (pixel_cx, pixel_cy), 8, (0, 255, 0), -1)
            
            # Draw direction line
            line_length = 100
            # For vertical reference: angle=0 means vertical, positive clockwise
            # Convert to standard math coordinates for drawing
            end_x = int(pixel_cx + line_length * math.sin(angle))
            end_y = int(pixel_cy + line_length * math.cos(angle))
            cv2.arrowedLine(annotated_frame, (pixel_cx, pixel_cy), (end_x, end_y), (0, 255, 255), 3)
            
            # Add text information
            angle_degrees = math.degrees(angle)
            info_text = f'Line: conf={confidence:.2f}, angle={angle_degrees:.1f}° ({angle:.3f}rad)'
            cv2.putText(annotated_frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            position_text = f'Position: ({centroid[0]:.2f}, {centroid[1]:.2f})'
            cv2.putText(annotated_frame, position_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Publish centroid
            centroid_msg = PointStamped()
            centroid_msg.header = self.latest_image_msg.header
            centroid_msg.point.x = centroid[0]  # Normalized x
            centroid_msg.point.y = centroid[1]  # Normalized y
            centroid_msg.point.z = confidence   # Confidence as z
            self.line_centroid_publisher.publish(centroid_msg)
            
            # Publish direction
            direction_msg = Vector3Stamped()
            direction_msg.header = self.latest_image_msg.header
            direction_msg.vector.x = math.sin(angle)   # Unit vector x (for vertical reference)
            direction_msg.vector.y = math.cos(angle)   # Unit vector y (for vertical reference)
            direction_msg.vector.z = angle             # Angle in radians
            self.line_direction_publisher.publish(direction_msg)
            
            # Log every 10th detection to avoid spam
            if hasattr(self, '_detection_count'):
                self._detection_count += 1
            else:
                self._detection_count = 1
                
            if self._detection_count % 10 == 0:
                angle_degrees = math.degrees(angle)
                self.get_logger().info(f"Line detected: pos=({centroid[0]:.2f}, {centroid[1]:.2f}), angle={angle_degrees:.1f}° ({angle:.3f}rad), conf={confidence:.2f}")
        else:
            cv2.putText(annotated_frame, 'No line detected', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Add parameter information
        cv2.putText(annotated_frame, f'Conf threshold: {self.confidence_threshold:.2f}', 
                   (10, height - 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                   
        cv2.putText(annotated_frame, f'Min area: {self.min_line_area}', 
                   (10, height - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                   
        cv2.putText(annotated_frame, f'ROI: {self.roi_left:.1f}-{self.roi_right:.1f}', 
                   (10, height - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            annotated_msg.header = self.latest_image_msg.header
            self.image_publisher.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert annotated image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = BlueLineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()