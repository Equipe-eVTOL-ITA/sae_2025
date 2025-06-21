#!/usr/bin/env python3
"""
Single Image HSV Extractor
Extracts HSV values from the center region (50x400 pixels) of a single input image.
Based on the hsv_calibration.py tool but simplified for single image analysis.

Usage:
    python3 extract_hsv_single.py <image_path>
    
Example:
    python3 extract_hsv_single.py ./imagens_crosshair/crosshair_1750427106173832288.png
"""

import cv2
import numpy as np
import sys
import os
import argparse

class HSVExtractor:
    def __init__(self, region_width=50, region_height=400):
        self.region_width = region_width
        self.region_height = region_height
        
    def extract_hsv_from_image(self, image_path):
        """
        Extract HSV values from the center region of an image.
        
        Args:
            image_path (str): Path to the input image
            
        Returns:
            dict: Contains mean HSV, std HSV, and region info
        """
        # Check if file exists
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Image file not found: {image_path}")
            
        # Load the image
        bgr_image = cv2.imread(image_path)
        if bgr_image is None:
            raise ValueError(f"Could not load image: {image_path}")
            
        print(f"📷 Loaded image: {image_path}")
        print(f"   Image dimensions: {bgr_image.shape[1]}x{bgr_image.shape[0]} (WxH)")
        
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        
        # Get image dimensions
        height, width = hsv_image.shape[:2]
        
        # Calculate center region coordinates
        center_x = width // 2
        center_y = height // 2
        
        # Calculate region boundaries
        x1 = max(0, center_x - self.region_width // 2)
        x2 = min(width, center_x + self.region_width // 2)
        y1 = max(0, center_y - self.region_height // 2)
        y2 = min(height, center_y + self.region_height // 2)
        
        # Adjust actual dimensions based on image boundaries
        actual_width = x2 - x1
        actual_height = y2 - y1
        
        print(f"🎯 Extraction region:")
        print(f"   Requested: {self.region_width}x{self.region_height} pixels")
        print(f"   Actual: {actual_width}x{actual_height} pixels")
        print(f"   Coordinates: ({x1}, {y1}) to ({x2}, {y2})")
        
        # Extract the region of interest
        roi = hsv_image[y1:y2, x1:x2]
        
        if roi.size == 0:
            raise ValueError("Region of interest is empty")
            
        # Calculate mean and standard deviation for the HSV region
        mean_hsv = cv2.mean(roi)[:3]  # Get H, S, V means
        _, stddev_hsv = cv2.meanStdDev(roi)
        stddev_hsv = stddev_hsv.flatten()[:3]  # Get H, S, V standard deviations
        
        # Convert to numpy arrays
        mean_hsv = np.array(mean_hsv, dtype=np.uint8)
        
        # Create a visualization image
        vis_image = self.create_visualization(bgr_image, x1, y1, x2, y2, mean_hsv, stddev_hsv)
        
        # Prepare results
        results = {
            'image_path': image_path,
            'image_size': (width, height),
            'region_coords': (x1, y1, x2, y2),
            'region_size': (actual_width, actual_height),
            'mean_hsv': mean_hsv,
            'stddev_hsv': stddev_hsv,
            'roi_pixels': roi.shape[0] * roi.shape[1],
            'visualization': vis_image
        }
        
        return results
    
    def create_visualization(self, bgr_image, x1, y1, x2, y2, mean_hsv, stddev_hsv):
        """Create a visualization showing the extracted region and HSV values."""
        vis_image = bgr_image.copy()
        
        # Draw the extraction region
        cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        # Add crosshairs at center
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        cross_size = 10
        cv2.line(vis_image, (center_x - cross_size, center_y), (center_x + cross_size, center_y), (0, 255, 0), 2)
        cv2.line(vis_image, (center_x, center_y - cross_size), (center_x, center_y + cross_size), (0, 255, 0), 2)
        
        # Add text overlay with HSV information
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_thickness = 2
        
        # Background rectangle for text
        text_bg_height = 120
        cv2.rectangle(vis_image, (10, 10), (400, text_bg_height), (0, 0, 0), -1)
        cv2.rectangle(vis_image, (10, 10), (400, text_bg_height), (255, 255, 255), 2)
        
        # HSV information text
        y_offset = 30
        cv2.putText(vis_image, f"HSV Mean: H={mean_hsv[0]} S={mean_hsv[1]} V={mean_hsv[2]}", 
                   (15, y_offset), font, font_scale, (255, 255, 255), font_thickness)
        
        y_offset += 25
        cv2.putText(vis_image, f"HSV Std:  H={stddev_hsv[0]:.1f} S={stddev_hsv[1]:.1f} V={stddev_hsv[2]:.1f}", 
                   (15, y_offset), font, font_scale, (255, 255, 255), font_thickness)
        
        y_offset += 25
        cv2.putText(vis_image, f"Region: {x2-x1}x{y2-y1} pixels", 
                   (15, y_offset), font, font_scale, (255, 255, 255), font_thickness)
        
        y_offset += 25
        cv2.putText(vis_image, f"Center: ({(x1+x2)//2}, {(y1+y2)//2})", 
                   (15, y_offset), font, font_scale, (255, 255, 255), font_thickness)
        
        return vis_image
    
    def print_results(self, results):
        """Print the extraction results in a formatted way."""
        print(f"\n📊 HSV Extraction Results:")
        print(f"=" * 50)
        
        mean_hsv = results['mean_hsv']
        stddev_hsv = results['stddev_hsv']
        
        print(f"Image: {os.path.basename(results['image_path'])}")
        print(f"Image size: {results['image_size'][0]}x{results['image_size'][1]} pixels")
        print(f"Region size: {results['region_size'][0]}x{results['region_size'][1]} pixels")
        print(f"Region coordinates: {results['region_coords']}")
        print(f"Total pixels analyzed: {results['roi_pixels']}")
        
        print(f"\n🎨 HSV Values:")
        print(f"  Mean HSV: H={mean_hsv[0]}, S={mean_hsv[1]}, V={mean_hsv[2]}")
        print(f"  Std HSV:  H={stddev_hsv[0]:.1f}, S={stddev_hsv[1]:.1f}, V={stddev_hsv[2]:.1f}")
        
        # Quality assessment
        max_std = max(stddev_hsv)
        if max_std < 10:
            quality = "🟢 Excellent (very uniform)"
        elif max_std < 20:
            quality = "🟡 Good (mostly uniform)"
        elif max_std < 40:
            quality = "🟠 Fair (some variation)"
        else:
            quality = "🔴 Poor (high variation)"
            
        print(f"  Quality: {quality}")
        
        # Generate HSV ranges using different strategies
        print(f"\n📏 Suggested HSV Ranges:")
        
        # Conservative range (1 std dev)
        h_range_1std = [max(0, mean_hsv[0] - stddev_hsv[0]), min(179, mean_hsv[0] + stddev_hsv[0])]
        s_range_1std = [max(0, mean_hsv[1] - stddev_hsv[1]), min(255, mean_hsv[1] + stddev_hsv[1])]
        v_range_1std = [max(0, mean_hsv[2] - stddev_hsv[2]), min(255, mean_hsv[2] + stddev_hsv[2])]
        
        print(f"  Conservative (1σ):")
        print(f"    lower_hsv = np.array([{int(h_range_1std[0])}, {int(s_range_1std[0])}, {int(v_range_1std[0])}])")
        print(f"    upper_hsv = np.array([{int(h_range_1std[1])}, {int(s_range_1std[1])}, {int(v_range_1std[1])}])")
        
        # Moderate range (2 std dev)
        h_range_2std = [max(0, mean_hsv[0] - 2*stddev_hsv[0]), min(179, mean_hsv[0] + 2*stddev_hsv[0])]
        s_range_2std = [max(0, mean_hsv[1] - 2*stddev_hsv[1]), min(255, mean_hsv[1] + 2*stddev_hsv[1])]
        v_range_2std = [max(0, mean_hsv[2] - 2*stddev_hsv[2]), min(255, mean_hsv[2] + 2*stddev_hsv[2])]
        
        print(f"  Moderate (2σ):")
        print(f"    lower_hsv = np.array([{int(h_range_2std[0])}, {int(s_range_2std[0])}, {int(v_range_2std[0])}])")
        print(f"    upper_hsv = np.array([{int(h_range_2std[1])}, {int(s_range_2std[1])}, {int(v_range_2std[1])}])")
        
        # Fixed range (useful minimum ranges)
        h_fixed = [max(0, mean_hsv[0] - 15), min(179, mean_hsv[0] + 15)]
        s_fixed = [max(0, mean_hsv[1] - 30), min(255, mean_hsv[1] + 30)]
        v_fixed = [max(0, mean_hsv[2] - 30), min(255, mean_hsv[2] + 30)]
        
        print(f"  Fixed ranges:")
        print(f"    lower_hsv = np.array([{int(h_fixed[0])}, {int(s_fixed[0])}, {int(v_fixed[0])}])")
        print(f"    upper_hsv = np.array([{int(h_fixed[1])}, {int(s_fixed[1])}, {int(v_fixed[1])}])")
    
    def save_results(self, results, output_dir="./"):
        """Save the results to files."""
        base_name = os.path.splitext(os.path.basename(results['image_path']))[0]
        
        # Save visualization image
        vis_path = os.path.join(output_dir, f"{base_name}_hsv_analysis.png")
        cv2.imwrite(vis_path, results['visualization'])
        print(f"\n💾 Visualization saved: {vis_path}")
        
        # Save text results
        txt_path = os.path.join(output_dir, f"{base_name}_hsv_results.txt")
        mean_hsv = results['mean_hsv']
        stddev_hsv = results['stddev_hsv']
        
        with open(txt_path, 'w') as f:
            f.write("=== HSV Single Image Extraction Results ===\n\n")
            f.write(f"Input image: {results['image_path']}\n")
            f.write(f"Image size: {results['image_size'][0]}x{results['image_size'][1]} pixels\n")
            f.write(f"Extraction region: {results['region_size'][0]}x{results['region_size'][1]} pixels\n")
            f.write(f"Region coordinates: {results['region_coords']}\n")
            f.write(f"Total pixels analyzed: {results['roi_pixels']}\n\n")
            
            f.write(f"HSV Statistics:\n")
            f.write(f"  Mean: H={mean_hsv[0]}, S={mean_hsv[1]}, V={mean_hsv[2]}\n")
            f.write(f"  Std:  H={stddev_hsv[0]:.1f}, S={stddev_hsv[1]:.1f}, V={stddev_hsv[2]:.1f}\n\n")
            
            # Write different range suggestions
            f.write("Suggested HSV Ranges:\n\n")
            
            f.write("Conservative (1 standard deviation):\n")
            h_range = [max(0, mean_hsv[0] - stddev_hsv[0]), min(179, mean_hsv[0] + stddev_hsv[0])]
            s_range = [max(0, mean_hsv[1] - stddev_hsv[1]), min(255, mean_hsv[1] + stddev_hsv[1])]
            v_range = [max(0, mean_hsv[2] - stddev_hsv[2]), min(255, mean_hsv[2] + stddev_hsv[2])]
            f.write(f"  lower_hsv = np.array([{int(h_range[0])}, {int(s_range[0])}, {int(v_range[0])}])\n")
            f.write(f"  upper_hsv = np.array([{int(h_range[1])}, {int(s_range[1])}, {int(v_range[1])}])\n\n")
            
            f.write("Moderate (2 standard deviations):\n")
            h_range = [max(0, mean_hsv[0] - 2*stddev_hsv[0]), min(179, mean_hsv[0] + 2*stddev_hsv[0])]
            s_range = [max(0, mean_hsv[1] - 2*stddev_hsv[1]), min(255, mean_hsv[1] + 2*stddev_hsv[1])]
            v_range = [max(0, mean_hsv[2] - 2*stddev_hsv[2]), min(255, mean_hsv[2] + 2*stddev_hsv[2])]
            f.write(f"  lower_hsv = np.array([{int(h_range[0])}, {int(s_range[0])}, {int(v_range[0])}])\n")
            f.write(f"  upper_hsv = np.array([{int(h_range[1])}, {int(s_range[1])}, {int(v_range[1])}])\n")
        
        print(f"💾 Results saved: {txt_path}")
        
        return vis_path, txt_path

def main():
    parser = argparse.ArgumentParser(description='Extract HSV values from center region of an image')
    parser.add_argument('image_path', help='Path to the input image')
    parser.add_argument('--width', type=int, default=50, help='Width of extraction region (default: 50)')
    parser.add_argument('--height', type=int, default=400, help='Height of extraction region (default: 400)')
    parser.add_argument('--save', action='store_true', help='Save visualization and results to files')
    parser.add_argument('--show', action='store_true', help='Display the visualization image')
    parser.add_argument('--output-dir', default='./', help='Output directory for saved files (default: current directory)')
    
    args = parser.parse_args()
    
    try:
        # Create extractor
        extractor = HSVExtractor(region_width=args.width, region_height=args.height)
        
        # Extract HSV values
        results = extractor.extract_hsv_from_image(args.image_path)
        
        # Print results
        extractor.print_results(results)
        
        # Save results if requested
        if args.save:
            os.makedirs(args.output_dir, exist_ok=True)
            extractor.save_results(results, args.output_dir)
        
        # Show visualization if requested
        if args.show:
            cv2.imshow('HSV Analysis', results['visualization'])
            print(f"\n👁️  Press any key to close the visualization window...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
