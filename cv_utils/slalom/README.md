# Post Detector for Slalom Mission

## Overview

This computer vision system detects colored posts (Preto/Black, Azul/Blue, Vermelho/Red, Rosa/Pink) for the drone slalom mission. The system uses HSV color space segmentation combined with morphological operations and geometric filtering to identify vertical posts while avoiding false positives from sky, shadows, and other objects.

## How It Works

### 1. Color Segmentation
- Converts input BGR image to HSV color space for better color separation
- Applies color range thresholds for each target color (Black, Blue, Red, Pink)
- Red color uses two HSV ranges due to hue wrap-around (0-10° and 170-180°)

### 2. Morphological Operations
- **Closing**: Fills gaps within detected post pixels
- **Vertical kernel closing**: Connects vertical segments better using a tall, narrow kernel
- **Opening**: Removes small noise pixels

### 3. Contour Detection & Filtering
- Finds contours in the segmented masks
- Filters by minimum area (300 pixels by default)
- **Aspect ratio filtering**: Posts should be vertical (width/height ratio between 0.02-0.8)
- **Geometric constraints**: Height must be at least 2x width, minimum dimensions 5x10 pixels

### 4. Detection Merging
- Merges nearby detections of the same class within 80 pixels distance
- Prevents multiple detections for the same post

### 5. Confidence Calculation
Confidence is calculated based on multiple factors:
- **Area factor**: Larger detections are more confident (normalized by 2000 pixels)
- **Aspect ratio factor**: Posts with ideal vertical proportions get higher confidence
- **Height factor**: Taller detections are more confident (normalized by 100 pixels)
- **Width factor**: Very wide detections are penalized

## Real World Deployment Checklist

### 1. Lighting Conditions
- **Overcast days**: May need to reduce value (V) thresholds for all colors
- **Bright sunlight**: May need to increase saturation (S) thresholds to avoid washed-out colors
- **Shadows**: Monitor for false "Preto" detections from post shadows

### 2. Camera Positioning
- Ensure camera is level and pointing forward
- Check that posts appear vertical in the image
- Verify that post diameter appears as expected thin vertical lines

### 3. HSV Calibration
- Use a color picker tool to sample actual post colors in your lighting conditions
- Adjust HSV ranges in `params.yaml` if detection is poor
- Test with known post colors before flight

### 4. Common Issues & Solutions

| Issue | Likely Cause | Solution |
|-------|--------------|----------|
| Sky detected as blue | Low saturation threshold | Increase `azul_lower_s` and `azul_lower_v` |
| Post shadows detected as black | High value threshold for black | Reduce `preto_upper_v` |
| Posts not detected | HSV range too narrow | Widen HSV ranges for specific colors |
| Multiple detections per post | Low merge distance | Increase merge distance or reduce `min_area` |
| Small noise detections | Low area threshold | Increase `min_area` parameter |

### 5. Parameter Tuning Order
1. **Start with color ranges**: Get basic color detection working
2. **Adjust area threshold**: Filter out noise while keeping posts
3. **Fine-tune aspect ratios**: Ensure only vertical objects are detected
4. **Adjust confidence threshold**: Set appropriate detection sensitivity

## Parameters Description

### Image Processing Parameters
```yaml
image_topic: '/horizontal_camera/compressed'  # Input image topic
processing_frequency: 10.0                    # Processing rate in Hz
confidence_threshold: 0.3                     # Minimum confidence to publish detection
```

### Morphological Operations
```yaml
min_area: 300                    # Minimum contour area in pixels
morph_kernel_size: 5            # Size of morphological operation kernel
morph_iterations: 2             # Number of morphological operation iterations
```

### Geometric Constraints
```yaml
min_aspect_ratio: 0.02          # Minimum width/height ratio (very thin objects)
max_aspect_ratio: 0.8           # Maximum width/height ratio (still vertical)
```

### HSV Color Ranges

#### Black (Preto)
```yaml
preto_lower_h: 0        # Hue: 0° (covers all hues)
preto_lower_s: 0        # Saturation: any
preto_lower_v: 0        # Value: dark colors
preto_upper_h: 180      # Hue: 180° (covers all hues)
preto_upper_s: 255      # Saturation: any
preto_upper_v: 50       # Value: up to dark gray
```

#### Blue (Azul)
```yaml
azul_lower_h: 105       # Hue: blue range start
azul_lower_s: 100       # Saturation: avoid light blue/sky
azul_lower_v: 80        # Value: avoid very light blues
azul_upper_h: 125       # Hue: blue range end
azul_upper_s: 255       # Saturation: maximum
azul_upper_v: 255       # Value: maximum
```

#### Red (Vermelho)
Due to HSV hue wrap-around, red uses two ranges:
```yaml
# Range 1: 0-10°
vermelho_lower1_h: 0
vermelho_upper1_h: 10
# Range 2: 170-180°
vermelho_lower2_h: 170
vermelho_upper2_h: 180
# Common S,V values for both ranges
vermelho_lower_s: 50
vermelho_lower_v: 50
vermelho_upper_s: 255
vermelho_upper_v: 255
```

#### Pink (Rosa)
```yaml
rosa_lower_h: 140       # Hue: pink/magenta range
rosa_lower_s: 50        # Saturation: moderate
rosa_lower_v: 50        # Value: moderate
rosa_upper_h: 170       # Hue: pink range end
rosa_upper_s: 255       # Saturation: maximum
rosa_upper_v: 255       # Value: maximum
```

## Output Topics

- **Detections**: `/slalom` (Detection2DArray) - Normalized bounding boxes with class and confidence
- **Annotated Image**: `/post_detector/{camera_name}/image` (Image) - Image with green mask overlay and bounding boxes

## Usage

```bash
# Run the detector with default parameters
ros2 run sae_cv_utils post_detector

# Run with custom image topic
ros2 run sae_cv_utils post_detector --ros-args -p image_topic:=/my_camera/compressed

# Run with parameter file
ros2 run sae_cv_utils post_detector --ros-args --params-file /path/to/params.yaml
```

## Visualization

The annotated image shows:
- **Green transparent overlay**: All pixels detected as any post color
- **Colored bounding boxes**: Each detection with its specific color
- **Labels**: Class name and confidence score

## Performance Notes

- Processing at 10 Hz is recommended for real-time performance
- Higher resolution images may require increased `min_area` parameter
- Lower resolution images may need reduced geometric constraints
- The system is optimized for post diameter ~0.1m and height ~2.5m viewed from typical drone distances
