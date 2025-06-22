#!/usr/bin/env python3
"""
Blue Base Detector for Landing

This ROS 2 node detects a blue circular landing base using the vertical camera
and publishes bounding box information for the drone to land on it.

The detector uses HSV color segmentation to identify blue pixels, then finds
circular contours that match the landing base characteristics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
import math

class BlueBaseDetector(Node):
    def __init__(self):
        super().__init__('blue_detector')

        # Declare parameters
        self.declare_parameter('image_topic', '/vertical_camera/compressed')
        self.declare_parameter('processing_frequency', 10.0)  # Hz
        self.declare_parameter('confidence_threshold', 0.1)
        
        # Blue base detection parameters
        self.declare_parameter('min_base_area', 100)  # Minimum area for valid base detection
        self.declare_parameter('morph_kernel_size', 7)
        self.declare_parameter('morph_iterations', 3)
        
        # Circle detection parameters
        self.declare_parameter('circle_dp', 1.0)  # Inverse ratio of accumulator resolution
        self.declare_parameter('circle_min_dist', 30)  # Minimum distance between circle centers
        self.declare_parameter('circle_param1', 50)  # Upper threshold for edge detection
        self.declare_parameter('circle_param2', 30)  # Accumulator threshold for circle centers
        self.declare_parameter('circle_min_radius', 10)  # Minimum circle radius
        self.declare_parameter('circle_max_radius', 200)  # Maximum circle radius
        
        # Blue base HSV parameters (navy blue base)
        self.declare_parameter('blue_lower_h', 70)
        self.declare_parameter('blue_lower_s', 60)
        self.declare_parameter('blue_lower_v', 25)
        self.declare_parameter('blue_upper_h', 150)
        self.declare_parameter('blue_upper_s', 255)
        self.declare_parameter('blue_upper_v', 255)
        
        # Get parameter values
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        processing_freq = self.get_parameter('processing_frequency').get_parameter_value().double_value
        
        # Base detection parameters
        self.min_base_area = self.get_parameter('min_base_area').get_parameter_value().integer_value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').get_parameter_value().integer_value
        self.morph_iterations = self.get_parameter('morph_iterations').get_parameter_value().integer_value
        
        # Circle detection parameters
        self.circle_dp = self.get_parameter('circle_dp').get_parameter_value().double_value
        self.circle_min_dist = self.get_parameter('circle_min_dist').get_parameter_value().integer_value
        self.circle_param1 = self.get_parameter('circle_param1').get_parameter_value().integer_value
        self.circle_param2 = self.get_parameter('circle_param2').get_parameter_value().integer_value
        self.circle_min_radius = self.get_parameter('circle_min_radius').get_parameter_value().integer_value
        self.circle_max_radius = self.get_parameter('circle_max_radius').get_parameter_value().integer_value
        
        # HSV color range for blue base
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
        self.detection_publisher = self.create_publisher(Detection2DArray, '/blue_base/detections', 10)
        self.image_publisher = self.create_publisher(Image, f'/blue_detector/{base_topic}/image', 10)

        # Set up timer for processing images
        timer_period = 1.0 / processing_freq
        self.timer = self.create_timer(timer_period, self.process_image_callback)
        
        self.get_logger().info(f'Blue Base Detector initialized')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info(f'Publishing detections to: /blue_base/detections')
        self.get_logger().info(f'Blue HSV Range: H={self.blue_lower[0]}-{self.blue_upper[0]}, S={self.blue_lower[1]}-{self.blue_upper[1]}, V={self.blue_lower[2]}-{self.blue_upper[2]}')

    def image_callback(self, msg):
        """Store the latest image message"""
        self.latest_image_msg = msg

    def segment_blue_base(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Segment blue base from the image using HSV color segmentation
        
        Args:
            image: Input BGR image
            
        Returns:
            mask: Binary mask of blue base pixels
            colored_mask: Colored mask for visualization
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for blue color range
        mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        
        # Morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                         (self.morph_kernel_size, self.morph_kernel_size))
        
        # Close to fill gaps in the base
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.morph_iterations)
        
        # Open to remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Create colored mask for visualization (blue for base)
        colored_mask = np.zeros_like(image)
        colored_mask[mask > 0] = [255, 0, 0]  # Blue color
        
        return mask, colored_mask

    def detect_circular_base(self, mask: np.ndarray, image_shape: Tuple[int, int]) -> List[Dict]:
        """
        Detect circular landing base using HoughCircles on the blue mask
        
        Args:
            mask: Binary mask of blue pixels
            image_shape: Shape of original image (height, width)
            
        Returns:
            List of detected circular bases with position and confidence
        """
        height, width = image_shape
        
        # Apply Gaussian blur to reduce noise for circle detection
        blurred_mask = cv2.GaussianBlur(mask, (9, 9), 2)
        
        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(
            blurred_mask,
            cv2.HOUGH_GRADIENT,
            dp=self.circle_dp,
            minDist=self.circle_min_dist,
            param1=self.circle_param1,
            param2=self.circle_param2,
            minRadius=self.circle_min_radius,
            maxRadius=self.circle_max_radius
        )
        
        detections = []
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            
            for (x, y, r) in circles:
                # Ensure circle is within image bounds
                if x - r < 0 or x + r >= width or y - r < 0 or y + r >= height:
                    continue
                
                # Calculate area and coverage
                area = np.pi * r * r
                if area < self.min_base_area:
                    continue
                
                # Calculate how much of the circle is actually blue pixels
                circle_mask = np.zeros((height, width), dtype=np.uint8)
                cv2.circle(circle_mask, (x, y), r, 255, -1)
                
                # Count blue pixels within the circle
                blue_pixels_in_circle = cv2.bitwise_and(mask, circle_mask)
                blue_pixel_count = np.sum(blue_pixels_in_circle > 0)
                circle_pixel_count = np.sum(circle_mask > 0)
                
                # Calculate fill ratio (how much of the circle is blue)
                fill_ratio = blue_pixel_count / circle_pixel_count if circle_pixel_count > 0 else 0
                
                # Normalize coordinates to [0, 1]
                center_x = x / width
                center_y = y / height
                size_x = (2 * r) / width
                size_y = (2 * r) / height
                
                # Calculate confidence based on fill ratio and size
                area_coverage = min(area / (width * height), 0.3) / 0.3
                confidence = 0.6 * fill_ratio + 0.4 * area_coverage
                confidence = max(0.0, min(1.0, confidence))
                
                if confidence >= self.confidence_threshold and fill_ratio > 0.5:
                    detections.append({
                        'center_x': center_x,
                        'center_y': center_y,
                        'size_x': size_x,
                        'size_y': size_y,
                        'confidence': confidence,
                        'area': area,
                        'radius': r,
                        'fill_ratio': fill_ratio,
                        'detection_method': 'circle'
                    })
        
        # Sort by confidence (highest first)
        detections.sort(key=lambda x: x['confidence'], reverse=True)
        
        return detections

    def analyze_base_contours(self, mask: np.ndarray, image_shape: Tuple[int, int]) -> List[Dict]:
        """
        Find and analyze contours to detect circular blue base (fallback method)
        This method tries to find the most circular part of irregular contours
        
        Args:
            mask: Binary mask of blue pixels
            image_shape: Shape of original image (height, width)
            
        Returns:
            List of detected bases with position and confidence
        """
        height, width = image_shape
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by minimum area
            if area < self.min_base_area:
                continue
            
            # Try to fit a circle to the contour to find the best circular center
            (circle_x, circle_y), radius = cv2.minEnclosingCircle(contour)
            circle_center = (int(circle_x), int(circle_y))
            circle_radius = int(radius)
            
            # Calculate how much of the fitted circle overlaps with the actual contour
            circle_mask = np.zeros((height, width), dtype=np.uint8)
            cv2.circle(circle_mask, circle_center, circle_radius, 255, -1)
            
            # Create contour mask
            contour_mask = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(contour_mask, [contour], 255)
            
            # Calculate overlap
            overlap = cv2.bitwise_and(circle_mask, contour_mask)
            overlap_area = np.sum(overlap > 0)
            circle_area = np.pi * circle_radius * circle_radius
            
            # Calculate how well the contour fits a circle
            circle_fit_ratio = overlap_area / circle_area if circle_area > 0 else 0
            
            # If the contour fits well to a circle, use the circle center
            if circle_fit_ratio > 0.6:
                center_x = circle_x / width
                center_y = circle_y / height
                size_x = (2 * circle_radius) / width
                size_y = (2 * circle_radius) / height
            else:
                # Fallback to bounding rectangle but try to find the most square part
                x, y, w, h = cv2.boundingRect(contour)
                center_x = (x + w/2) / width
                center_y = (y + h/2) / height
                size_x = w / width
                size_y = h / height
            
            # Calculate circularity to prefer circular shapes
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
            else:
                circularity = 0
            
            # Calculate aspect ratio (should be close to 1 for circles)
            aspect_ratio = min(size_x, size_y) / max(size_x, size_y) if max(size_x, size_y) > 0 else 0
            
            # Combined confidence based on circularity, aspect ratio, and circle fit
            area_coverage = min(area / (width * height), 0.3) / 0.3  # Normalize to max 30% coverage
            confidence = (0.3 * area_coverage + 0.4 * circularity + 0.2 * aspect_ratio + 0.1 * circle_fit_ratio)
            confidence = max(0.0, min(1.0, confidence))
            
            if confidence >= self.confidence_threshold:
                detections.append({
                    'center_x': center_x,
                    'center_y': center_y,
                    'size_x': size_x,
                    'size_y': size_y,
                    'confidence': confidence,
                    'area': area,
                    'circularity': circularity,
                    'circle_fit_ratio': circle_fit_ratio,
                    'detection_method': 'contour'
                })
        
        # Sort by confidence (highest first)
        detections.sort(key=lambda x: x['confidence'], reverse=True)
        
        return detections

    def process_image_callback(self):
        """Process the latest image for base detection"""
        if self.latest_image_msg is None:
            return

        try:
            # Convert ROS compressed image message to OpenCV image
            current_frame = self.bridge.compressed_imgmsg_to_cv2(self.latest_image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return

        # Segment blue base
        mask, colored_mask = self.segment_blue_base(current_frame)
        
        # Try circle detection first (more accurate for circular bases)
        detections = self.detect_circular_base(mask, current_frame.shape[:2])
        
        # If no good circle detections, fall back to contour analysis
        if not detections:
            detections = self.analyze_base_contours(mask, current_frame.shape[:2])
        
        # Combine both methods if needed and remove duplicates
        circle_detections = self.detect_circular_base(mask, current_frame.shape[:2])
        contour_detections = self.analyze_base_contours(mask, current_frame.shape[:2])
        
        # Prioritize circle detections, but include contour detections if they're significantly better
        all_detections = circle_detections.copy()
        
        for contour_det in contour_detections:
            # Check if this contour detection is far from any circle detection
            is_duplicate = False
            for circle_det in circle_detections:
                distance = np.sqrt((contour_det['center_x'] - circle_det['center_x'])**2 + 
                                 (contour_det['center_y'] - circle_det['center_y'])**2)
                if distance < 0.1:  # If centers are close (within 10% of image), consider duplicate
                    is_duplicate = True
                    break
            
            # Add contour detection if it's not a duplicate or if it has much higher confidence
            if not is_duplicate or (contour_det['confidence'] > max([d['confidence'] for d in circle_detections], default=0) + 0.2):
                all_detections.append(contour_det)
        
        # Sort all detections by confidence
        detections = sorted(all_detections, key=lambda x: x['confidence'], reverse=True)
        
        # Log detection statistics occasionally
        if detections:
            circle_count = len([d for d in detections if d.get('detection_method') == 'circle'])
            contour_count = len([d for d in detections if d.get('detection_method') == 'contour'])
            if hasattr(self, '_detection_log_counter'):
                self._detection_log_counter += 1
            else:
                self._detection_log_counter = 1
            
            if self._detection_log_counter % 30 == 0:  # Log every 30 detections
                best_detection = detections[0]
                self.get_logger().info(f"Detections: {circle_count} circles, {contour_count} contours. "
                                     f"Best: {best_detection.get('detection_method', 'unknown')} "
                                     f"confidence={best_detection['confidence']:.3f}")
        
        # Create annotated image
        annotated_frame = current_frame.copy()
        
        # Apply transparent mask overlay
        alpha = 0.3
        annotated_frame = cv2.addWeighted(annotated_frame, 1 - alpha, colored_mask, alpha, 0)
        
        # Create Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = "camera_frame"
        
        for detection in detections:
            # Draw detection on annotated frame
            height, width = current_frame.shape[:2]
            x1 = int((detection['center_x'] - detection['size_x']/2) * width)
            y1 = int((detection['center_y'] - detection['size_y']/2) * height)
            x2 = int((detection['center_x'] + detection['size_x']/2) * width)
            y2 = int((detection['center_y'] + detection['size_y']/2) * height)
            
            # Choose color based on detection method
            if detection.get('detection_method') == 'circle':
                bbox_color = (0, 255, 0)  # Green for circle detection
                center_color = (0, 0, 255)  # Red center
            else:
                bbox_color = (255, 255, 0)  # Cyan for contour detection
                center_color = (255, 0, 255)  # Magenta center
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), bbox_color, 2)
            
            # If it's a circle detection, also draw the actual circle
            if detection.get('detection_method') == 'circle' and 'radius' in detection:
                center_px = (int(detection['center_x'] * width), int(detection['center_y'] * height))
                radius_px = int(detection['radius'])
                cv2.circle(annotated_frame, center_px, radius_px, (0, 255, 255), 2)  # Yellow circle
            
            # Draw center point
            center_px = (int(detection['center_x'] * width), int(detection['center_y'] * height))
            cv2.circle(annotated_frame, center_px, 5, center_color, -1)
            
            # Draw confidence and method text
            method = detection.get('detection_method', 'unknown')
            confidence_text = f"{detection['confidence']:.2f} ({method})"
            cv2.putText(annotated_frame, confidence_text, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, bbox_color, 2)
            
            # Create Detection2D message
            det_msg = Detection2D()
            det_msg.header = detection_array.header
            
            # Set bounding box
            det_msg.bbox.center.position.x = detection['center_x']
            det_msg.bbox.center.position.y = detection['center_y']
            det_msg.bbox.size_x = detection['size_x']
            det_msg.bbox.size_y = detection['size_y']
            
            # Set hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = "blue_base"
            hypothesis.hypothesis.score = detection['confidence']
            det_msg.results.append(hypothesis)
            
            detection_array.detections.append(det_msg)
        
        # Publish detection results
        self.detection_publisher.publish(detection_array)
        
        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            annotated_msg.header = detection_array.header
            self.image_publisher.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert annotated image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    detector = BlueBaseDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('Blue base detector interrupted')
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()