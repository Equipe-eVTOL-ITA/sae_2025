#!/usr/bin/env python3
"""
HSV Color Space Debug Tool for Post Detection

This script generates visual representations of the HSV color ranges
used for post detection to help with field debugging and parameter tuning.

Usage:
    python3 hsv_color_debug.py [--params /path/to/params.yaml]
"""

import cv2
import numpy as np
import yaml
import argparse
import os
from typing import Dict, Tuple
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import hsv_to_rgb

class HSVColorDebugger:
    def __init__(self, params_file=None):
        """Initialize the HSV color debugger with parameters"""
        self.script_dir = os.path.dirname(os.path.realpath(__file__))
        
        # Default HSV parameters (same as in params.yaml)
        self.color_params = {
            'Preto': {
                'lower_h': 0, 'lower_s': 0, 'lower_v': 0,
                'upper_h': 180, 'upper_s': 255, 'upper_v': 50
            },
            'Azul': {
                'lower_h': 105, 'lower_s': 110, 'lower_v': 85,
                'upper_h': 125, 'upper_s': 255, 'upper_v': 255
            },
            'Vermelho': {
                'lower1_h': 0, 'lower1_s': 50, 'lower1_v': 50,
                'upper1_h': 10, 'upper1_s': 255, 'upper1_v': 255,
                'lower2_h': 170, 'lower2_s': 50, 'lower2_v': 50,
                'upper2_h': 180, 'upper2_s': 255, 'upper2_v': 255
            },
            'Rosa': {
                'lower_h': 140, 'lower_s': 50, 'lower_v': 50,
                'upper_h': 170, 'upper_s': 255, 'upper_v': 255
            }
        }
        
        # Load parameters from file if provided
        if params_file:
            self.load_params_from_file(params_file)
    
    def load_params_from_file(self, params_file):
        """Load HSV parameters from YAML file"""
        try:
            with open(params_file, 'r') as f:
                params = yaml.safe_load(f)
            
            post_params = params.get('post_detector', {}).get('ros__parameters', {})
            
            # Update color parameters from file
            for color in ['preto', 'azul', 'rosa']:
                color_name = color.capitalize()
                if color_name == 'Preto':
                    color_name = 'Preto'
                elif color_name == 'Azul':
                    color_name = 'Azul'
                elif color_name == 'Rosa':
                    color_name = 'Rosa'
                
                # Standard colors (not red)
                if color in ['preto', 'azul', 'rosa']:
                    self.color_params[color_name]['lower_h'] = post_params.get(f'{color}_lower_h', self.color_params[color_name]['lower_h'])
                    self.color_params[color_name]['lower_s'] = post_params.get(f'{color}_lower_s', self.color_params[color_name]['lower_s'])
                    self.color_params[color_name]['lower_v'] = post_params.get(f'{color}_lower_v', self.color_params[color_name]['lower_v'])
                    self.color_params[color_name]['upper_h'] = post_params.get(f'{color}_upper_h', self.color_params[color_name]['upper_h'])
                    self.color_params[color_name]['upper_s'] = post_params.get(f'{color}_upper_s', self.color_params[color_name]['upper_s'])
                    self.color_params[color_name]['upper_v'] = post_params.get(f'{color}_upper_v', self.color_params[color_name]['upper_v'])
            
            # Special handling for red (vermelho)
            red_params = ['vermelho_lower1_h', 'vermelho_lower1_s', 'vermelho_lower1_v',
                         'vermelho_upper1_h', 'vermelho_upper1_s', 'vermelho_upper1_v',
                         'vermelho_lower2_h', 'vermelho_lower2_s', 'vermelho_lower2_v',
                         'vermelho_upper2_h', 'vermelho_upper2_s', 'vermelho_upper2_v']
            
            for param in red_params:
                if param in post_params:
                    key = param.replace('vermelho_', '')
                    self.color_params['Vermelho'][key] = post_params[param]
            
            print(f"Loaded parameters from {params_file}")
            
        except Exception as e:
            print(f"Warning: Could not load parameters from {params_file}: {e}")
            print("Using default parameters")
    
    def create_hsv_wheel(self, size=400):
        """Create an HSV color wheel showing hue and saturation"""
        # Create coordinate arrays
        y, x = np.ogrid[:size, :size]
        center = size // 2
        
        # Calculate distance from center and angle
        dx = x - center
        dy = y - center
        distance = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx)
        
        # Convert angle to hue (0-180 for OpenCV HSV)
        hue = (angle + np.pi) / (2 * np.pi) * 180
        
        # Saturation based on distance from center
        saturation = np.clip(distance / (center * 0.9) * 255, 0, 255)
        
        # Full value (brightness)
        value = np.ones_like(hue) * 255
        
        # Create HSV image
        hsv = np.dstack([hue, saturation, value]).astype(np.uint8)
        
        # Mask out pixels outside the circle
        mask = distance <= center * 0.9
        hsv[~mask] = [0, 0, 255]  # White background
        
        # Convert to BGR for display
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        bgr[~mask] = [255, 255, 255]  # White background
        
        return bgr, mask
    
    def create_value_strip(self, hue, saturation, width=50, height=400):
        """Create a vertical strip showing value range for given hue and saturation"""
        strip = np.zeros((height, width, 3), dtype=np.uint8)
        
        for i in range(height):
            value = int((height - i - 1) / height * 255)
            strip[i, :] = [hue, saturation, value]
        
        # Convert to BGR
        bgr = cv2.cvtColor(strip, cv2.COLOR_HSV2BGR)
        return bgr
    
    def draw_hsv_range_on_wheel(self, wheel, mask, color_name, color_params, color_bgr):
        """Draw the HSV range for a color on the wheel"""
        size = wheel.shape[0]
        center = size // 2
        
        # Create range mask
        if color_name == 'Vermelho':
            # Special handling for red (two ranges)
            range_mask1 = self.create_range_mask(size, center, 
                                               color_params['lower1_h'], color_params['upper1_h'],
                                               color_params['lower1_s'], color_params['upper1_s'])
            range_mask2 = self.create_range_mask(size, center, 
                                               color_params['lower2_h'], color_params['upper2_h'],
                                               color_params['lower2_s'], color_params['upper2_s'])
            range_mask = range_mask1 | range_mask2
        else:
            range_mask = self.create_range_mask(size, center,
                                              color_params['lower_h'], color_params['upper_h'],
                                              color_params['lower_s'], color_params['upper_s'])
        
        # Apply the range mask with transparency
        overlay = wheel.copy()
        overlay[range_mask & mask] = color_bgr
        
        # Blend with original
        alpha = 0.7
        wheel[range_mask & mask] = cv2.addWeighted(
            wheel[range_mask & mask], 1 - alpha,
            overlay[range_mask & mask], alpha, 0
        )
        
        return wheel
    
    def create_range_mask(self, size, center, h_min, h_max, s_min, s_max):
        """Create a mask for the HSV range on the wheel"""
        y, x = np.ogrid[:size, :size]
        dx = x - center
        dy = y - center
        distance = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx)
        
        # Convert angle to hue (0-180)
        hue = (angle + np.pi) / (2 * np.pi) * 180
        
        # Saturation based on distance
        saturation = np.clip(distance / (center * 0.9) * 255, 0, 255)
        
        # Create mask for this color range
        if h_min <= h_max:
            hue_mask = (hue >= h_min) & (hue <= h_max)
        else:
            # Handle wrap-around (like red)
            hue_mask = (hue >= h_min) | (hue <= h_max)
        
        sat_mask = (saturation >= s_min) & (saturation <= s_max)
        
        return hue_mask & sat_mask
    
    def create_color_samples(self):
        """Create sample color patches for each color range"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Post Color HSV Ranges - Sample Colors', fontsize=16, fontweight='bold')
        
        colors = ['Preto', 'Azul', 'Vermelho', 'Rosa']
        bgr_colors = [(50, 50, 50), (255, 0, 0), (0, 0, 255), (180, 105, 255)]
        
        for i, (color_name, bgr_color) in enumerate(zip(colors, bgr_colors)):
            row = i // 2
            col = i % 2
            ax = axes[row, col]
            
            params = self.color_params[color_name]
            
            # Create sample patches
            if color_name == 'Vermelho':
                # Two ranges for red
                samples1 = self.generate_color_samples(
                    params['lower1_h'], params['upper1_h'],
                    params['lower1_s'], params['upper1_s'],
                    params['lower1_v'], params['upper1_v']
                )
                samples2 = self.generate_color_samples(
                    params['lower2_h'], params['upper2_h'],
                    params['lower2_s'], params['upper2_s'],
                    params['lower2_v'], params['upper2_v']
                )
                samples = np.vstack([samples1, samples2])
            else:
                samples = self.generate_color_samples(
                    params['lower_h'], params['upper_h'],
                    params['lower_s'], params['upper_s'],
                    params['lower_v'], params['upper_v']
                )
            
            # Display samples
            ax.imshow(samples)
            ax.set_title(f'{color_name}\nHSV Range Samples', fontweight='bold')
            ax.set_xticks([])
            ax.set_yticks([])
            
            # Add parameter text
            if color_name == 'Vermelho':
                param_text = f"Range 1: H:{params['lower1_h']}-{params['upper1_h']}, S:{params['lower1_s']}-{params['upper1_s']}, V:{params['lower1_v']}-{params['upper1_v']}\n"
                param_text += f"Range 2: H:{params['lower2_h']}-{params['upper2_h']}, S:{params['lower2_s']}-{params['upper2_s']}, V:{params['lower2_v']}-{params['upper2_v']}"
            else:
                param_text = f"H: {params['lower_h']}-{params['upper_h']}\nS: {params['lower_s']}-{params['upper_s']}\nV: {params['lower_v']}-{params['upper_v']}"
            
            ax.text(0.02, 0.98, param_text, transform=ax.transAxes, 
                   verticalalignment='top', fontsize=8, 
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        plt.tight_layout()
        return fig
    
    def generate_color_samples(self, h_min, h_max, s_min, s_max, v_min, v_max, samples_per_dim=8):
        """Generate a grid of color samples within the HSV range"""
        # Create sample grid
        if h_min <= h_max:
            h_samples = np.linspace(h_min, h_max, samples_per_dim)
        else:
            # Handle wrap-around
            h_samples1 = np.linspace(h_min, 180, samples_per_dim//2)
            h_samples2 = np.linspace(0, h_max, samples_per_dim//2)
            h_samples = np.concatenate([h_samples1, h_samples2])
        
        s_samples = np.linspace(s_min, s_max, samples_per_dim)
        v_samples = np.linspace(v_min, v_max, samples_per_dim)
        
        # Create sample image
        sample_height = len(v_samples) * 20
        sample_width = len(h_samples) * len(s_samples) * 10
        
        sample_image = np.zeros((sample_height, sample_width, 3), dtype=np.uint8)
        
        for i, v in enumerate(v_samples):
            y_start = i * 20
            y_end = (i + 1) * 20
            
            x_pos = 0
            for h in h_samples:
                for s in s_samples:
                    x_end = x_pos + 10
                    sample_image[y_start:y_end, x_pos:x_end] = [h, s, v]
                    x_pos = x_end
        
        # Convert to RGB for matplotlib
        rgb_image = cv2.cvtColor(sample_image, cv2.COLOR_HSV2RGB)
        return rgb_image
    
    def create_hsv_visualization(self):
        """Create comprehensive HSV visualization"""
        # Create HSV wheel
        wheel, mask = self.create_hsv_wheel(600)
        
        # Define colors for visualization
        color_bgrs = {
            'Preto': (128, 128, 128),    # Gray for visibility
            'Azul': (255, 0, 0),         # Blue
            'Vermelho': (0, 0, 255),     # Red
            'Rosa': (255, 0, 255)        # Magenta
        }
        
        # Draw ranges on wheel
        for color_name, color_bgr in color_bgrs.items():
            wheel = self.draw_hsv_range_on_wheel(wheel, mask, color_name, 
                                               self.color_params[color_name], color_bgr)
        
        # Add labels and legend
        cv2.putText(wheel, 'HSV Color Wheel - Post Detection Ranges', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        
        # Add legend
        legend_y = 60
        for color_name, color_bgr in color_bgrs.items():
            cv2.circle(wheel, (20, legend_y), 10, color_bgr, -1)
            cv2.putText(wheel, color_name, (40, legend_y + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
            legend_y += 30
        
        return wheel
    
    def save_debug_images(self):
        """Generate and save all debug images"""
        output_dir = self.script_dir
        
        print("Generating HSV color debug images...")
        
        # 1. HSV wheel visualization
        print("Creating HSV wheel visualization...")
        wheel = self.create_hsv_visualization()
        wheel_path = os.path.join(output_dir, 'hsv_color_wheel.png')
        cv2.imwrite(wheel_path, wheel)
        print(f"Saved HSV wheel: {wheel_path}")
        
        # 2. Color samples
        print("Creating color sample grids...")
        fig = self.create_color_samples()
        samples_path = os.path.join(output_dir, 'hsv_color_samples.png')
        fig.savefig(samples_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved color samples: {samples_path}")
        
        # 3. Parameter summary
        print("Creating parameter summary...")
        self.create_parameter_summary()
        
        print(f"\nDebug images generated successfully in: {output_dir}")
        print("Files created:")
        print("  - hsv_color_wheel.png: HSV wheel with detection ranges highlighted")
        print("  - hsv_color_samples.png: Sample colors for each detection range")
        print("  - hsv_parameters.txt: Text summary of all HSV parameters")
    
    def create_parameter_summary(self):
        """Create a text summary of all HSV parameters"""
        summary_path = os.path.join(self.script_dir, 'hsv_parameters.txt')
        
        with open(summary_path, 'w') as f:
            f.write("HSV Color Parameters for Post Detection\n")
            f.write("=" * 50 + "\n\n")
            
            for color_name, params in self.color_params.items():
                f.write(f"{color_name} Post:\n")
                f.write("-" * 20 + "\n")
                
                if color_name == 'Vermelho':
                    f.write(f"Range 1: H={params['lower1_h']}-{params['upper1_h']}, ")
                    f.write(f"S={params['lower1_s']}-{params['upper1_s']}, ")
                    f.write(f"V={params['lower1_v']}-{params['upper1_v']}\n")
                    f.write(f"Range 2: H={params['lower2_h']}-{params['upper2_h']}, ")
                    f.write(f"S={params['lower2_s']}-{params['upper2_s']}, ")
                    f.write(f"V={params['lower2_v']}-{params['upper2_v']}\n")
                else:
                    f.write(f"H: {params['lower_h']}-{params['upper_h']}\n")
                    f.write(f"S: {params['lower_s']}-{params['upper_s']}\n")
                    f.write(f"V: {params['lower_v']}-{params['upper_v']}\n")
                
                f.write("\n")
            
            f.write("Notes:\n")
            f.write("- H (Hue): 0-180 in OpenCV HSV\n")
            f.write("- S (Saturation): 0-255\n")
            f.write("- V (Value/Brightness): 0-255\n")
            f.write("- Red uses two ranges due to hue wrap-around at 0/180\n")
        
        print(f"Saved parameter summary: {summary_path}")

def main():
    parser = argparse.ArgumentParser(description='Generate HSV color debug visualizations for post detection')
    parser.add_argument('--params', type=str, 
                       default='/home/ceccon/frtl_2025_ws/src/sae_2025/fase1/launch/params.yaml',
                       help='Path to parameters YAML file')
    
    args = parser.parse_args()
    
    # Create debugger and generate images
    debugger = HSVColorDebugger(args.params)
    debugger.save_debug_images()

if __name__ == '__main__':
    main()
