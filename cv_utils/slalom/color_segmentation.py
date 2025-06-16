import cv2
import numpy as np
from typing import Dict, List, Tuple, Optional

class ColorSegmentation:
    """Class for color-based segmentation of posts using HSV color space"""
    
    def __init__(self):
        # Define HSV ranges for each color (adjustable parameters)
        self.color_ranges = {
            'Preto': {
                'lower': np.array([0, 0, 0]),
                'upper': np.array([180, 255, 60])  # Low value for black/dark colors
            },
            'Azul': {
                'lower': np.array([100, 50, 50]),
                'upper': np.array([130, 255, 255])
            },
            'Vermelho': {
                'lower1': np.array([0, 50, 50]),    # Red wraps around in HSV
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 50, 50]),
                'upper2': np.array([180, 255, 255])
            },
            'Rosa': {
                'lower': np.array([140, 50, 50]),
                'upper': np.array([170, 255, 255])
            }
        }
        
        # Morphological operations parameters
        self.morph_kernel_size = 5
        self.morph_iterations = 2
        
        # Minimum area for valid detection
        self.min_area = 500
        
        # Aspect ratio constraints for post detection (width/height)
        # Posts are vertical: 0.1m diameter, 2.5m height = aspect ratio ~0.04
        # Allow some tolerance for perspective and detection noise
        self.min_aspect_ratio = 0.02  # Very thin vertical objects
        self.max_aspect_ratio = 0.8   # Still vertical, but allow some perspective distortion
    
    def update_color_ranges(self, color_ranges: Dict):
        """Update color ranges with new parameters"""
        self.color_ranges.update(color_ranges)
    
    def update_parameters(self, min_area: int = None, morph_kernel_size: int = None, 
                         morph_iterations: int = None, min_aspect_ratio: float = None,
                         max_aspect_ratio: float = None):
        """Update segmentation parameters"""
        if min_area is not None:
            self.min_area = min_area
        if morph_kernel_size is not None:
            self.morph_kernel_size = morph_kernel_size
        if morph_iterations is not None:
            self.morph_iterations = morph_iterations
        if min_aspect_ratio is not None:
            self.min_aspect_ratio = min_aspect_ratio
        if max_aspect_ratio is not None:
            self.max_aspect_ratio = max_aspect_ratio
    
    def segment_color(self, image: np.ndarray, color_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Segment a specific color from the image
        
        Args:
            image: Input BGR image
            color_name: Name of the color to segment ('Preto', 'Azul', 'Vermelho', 'Rosa')
            
        Returns:
            mask: Binary mask of segmented pixels
            colored_mask: 3-channel colored mask for visualization
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if color_name not in self.color_ranges:
            return np.zeros(hsv.shape[:2], dtype=np.uint8), np.zeros_like(image)
        
        color_range = self.color_ranges[color_name]
        
        # Create mask based on color
        if color_name == 'Vermelho':
            # Red color wraps around in HSV, so we need two ranges
            mask1 = cv2.inRange(hsv, color_range['lower1'], color_range['upper1'])
            mask2 = cv2.inRange(hsv, color_range['lower2'], color_range['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
        
        # Apply morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                         (self.morph_kernel_size, self.morph_kernel_size))
        
        # First close to fill gaps in posts
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.morph_iterations)
        
        # Use a vertical kernel to connect vertical segments better for posts
        vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, self.morph_kernel_size * 2))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, vertical_kernel, iterations=1)
        
        # Then open to remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Create colored mask for visualization
        colored_mask = np.zeros_like(image)
        colored_mask[mask > 0] = self._get_color_bgr(color_name)
        
        return mask, colored_mask
    
    def _get_color_bgr(self, color_name: str) -> Tuple[int, int, int]:
        """Get BGR color values for visualization"""
        color_map = {
            'Preto': (50, 50, 50),      # Dark gray for better visibility
            'Azul': (255, 0, 0),        # Blue in BGR
            'Vermelho': (0, 0, 255),    # Red in BGR
            'Rosa': (180, 105, 255)     # Pink in BGR
        }
        return color_map.get(color_name, (255, 255, 255))
    
    def find_contours_and_bboxes(self, mask: np.ndarray, color_name: str) -> List[Dict]:
        """
        Find contours in the mask and create bounding boxes
        
        Args:
            mask: Binary mask
            color_name: Name of the detected color
            
        Returns:
            List of detection dictionaries containing bbox info and class
        """
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by minimum area
            if area < self.min_area:
                continue
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Filter by aspect ratio (to avoid detecting non-post objects)
            aspect_ratio = w / h if h > 0 else 0
            if not (self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio):
                continue
            
            # Additional filtering for vertical posts
            # Posts should have significant height relative to their width
            if h < w * 2:  # Height should be at least twice the width
                continue
                
            # Filter out very small detections that are likely noise
            if w < 5 or h < 10:  # Minimum pixel dimensions
                continue
            
            detection = {
                'class': color_name,
                'bbox': (x, y, w, h),
                'area': area,
                'contour': contour
            }
            
            detections.append(detection)
        
        return detections
    
    def merge_nearby_detections(self, detections: List[Dict], merge_distance: int = 50) -> List[Dict]:
        """
        Merge detections that are close to each other and of the same class
        
        Args:
            detections: List of detection dictionaries
            merge_distance: Maximum distance between detections to merge them
            
        Returns:
            List of merged detections
        """
        if len(detections) <= 1:
            return detections
        
        merged_detections = []
        used_indices = set()
        
        for i, detection1 in enumerate(detections):
            if i in used_indices:
                continue
                
            # Start a new group with this detection
            group = [detection1]
            used_indices.add(i)
            
            # Find nearby detections of the same class
            for j, detection2 in enumerate(detections):
                if j in used_indices or j <= i:
                    continue
                
                if detection1['class'] != detection2['class']:
                    continue
                
                # Calculate distance between bounding box centers
                x1, y1, w1, h1 = detection1['bbox']
                x2, y2, w2, h2 = detection2['bbox']
                
                center1_x, center1_y = x1 + w1/2, y1 + h1/2
                center2_x, center2_y = x2 + w2/2, y2 + h2/2
                
                distance = ((center1_x - center2_x)**2 + (center1_y - center2_y)**2)**0.5
                
                if distance < merge_distance:
                    group.append(detection2)
                    used_indices.add(j)
            
            # Merge the group into a single detection
            if len(group) == 1:
                merged_detections.append(group[0])
            else:
                merged_detection = self._merge_detection_group(group)
                merged_detections.append(merged_detection)
        
        return merged_detections
    
    def _merge_detection_group(self, group: List[Dict]) -> Dict:
        """Merge a group of detections into a single detection"""
        if len(group) == 1:
            return group[0]
        
        # Find the bounding box that encompasses all detections
        min_x = min(det['bbox'][0] for det in group)
        min_y = min(det['bbox'][1] for det in group)
        max_x = max(det['bbox'][0] + det['bbox'][2] for det in group)
        max_y = max(det['bbox'][1] + det['bbox'][3] for det in group)
        
        merged_bbox = (min_x, min_y, max_x - min_x, max_y - min_y)
        merged_area = sum(det['area'] for det in group)
        
        return {
            'class': group[0]['class'],  # All should have the same class
            'bbox': merged_bbox,
            'area': merged_area,
            'contour': group[0]['contour']  # Keep the first contour
        }
    
    def segment_all_colors(self, image: np.ndarray) -> Tuple[List[Dict], np.ndarray]:
        """
        Segment all colors and return detections and combined visualization mask
        
        Args:
            image: Input BGR image
            
        Returns:
            detections: List of all detections
            combined_mask: Combined colored mask for visualization (green for all detected pixels)
        """
        all_detections = []
        combined_mask = np.zeros_like(image)
        combined_binary_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        
        for color_name in self.color_ranges.keys():
            mask, _ = self.segment_color(image, color_name)
            detections = self.find_contours_and_bboxes(mask, color_name)
            
            all_detections.extend(detections)
            
            # Combine all binary masks
            combined_binary_mask = cv2.bitwise_or(combined_binary_mask, mask)
        
        # Merge nearby detections of the same class
        all_detections = self.merge_nearby_detections(all_detections, merge_distance=80)
        
        # Create green mask for all detected pixels
        combined_mask[combined_binary_mask > 0] = (0, 255, 0)  # Green in BGR
        
        return all_detections, combined_mask
    
    def normalize_bbox(self, bbox: Tuple[int, int, int, int], 
                      image_shape: Tuple[int, int]) -> Tuple[float, float, float, float]:
        """
        Normalize bounding box coordinates to [0, 1] range
        
        Args:
            bbox: (x, y, w, h) in pixels
            image_shape: (height, width) of the image
            
        Returns:
            Normalized (x_center, y_center, width, height)
        """
        x, y, w, h = bbox
        img_h, img_w = image_shape
        
        # Convert to center coordinates and normalize
        x_center = (x + w / 2) / img_w
        y_center = (y + h / 2) / img_h
        norm_w = w / img_w
        norm_h = h / img_h
        
        return x_center, y_center, norm_w, norm_h
