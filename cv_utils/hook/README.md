# Hook Detection Module

This module contains computer vision detectors for the "hook" mission in the SAE competition, which involves following a blue line and detecting a red hose.

## Overview

The hook mission requires the drone to:
1. **Follow a blue fabric line** (25cm width) using the vertical camera
2. **Detect a red hose** (1.25cm diameter) using the horizontal camera

Both detectors use HSV color segmentation and publish detection results for mission guidance.

## Detectors

### 1. Blue Line Detector (`blueline.py`)

**Purpose**: Detect and follow a blue fabric line using the vertical camera.

**Method**:
- HSV color segmentation to isolate blue pixels
- Morphological operations to clean up the mask
- Contour detection and area filtering
- Centroid calculation using image moments
- Direction estimation using `cv2.fitLine()`

**Key Features**:
- ROI processing to focus on relevant image areas
- Confidence calculation based on line area and visibility
- Real-time line direction and position tracking
- Robust to lighting variations

**Published Topics**:
- `/blueline/centroid` (PointStamped): Line center position in image coordinates
- `/blueline/direction` (Vector3Stamped): Line direction vector for navigation
- `/blueline/debug_image` (Image): Annotated debug visualization

### 2. Red Hose Detector (`mangueira.py`)

**Purpose**: Detect a thin red hose using the horizontal camera.

**Method**:
- HSV color segmentation with dual red ranges (0-10° and 170-180°)
- Morphological operations for noise reduction
- Contour analysis with geometric constraints
- Aspect ratio filtering for elongated objects
- Position and orientation estimation

**Key Features**:
- Specialized for very thin, elongated objects
- Dual HSV ranges to handle red hue wrap-around
- Geometric validation (aspect ratio 3-50, length/width constraints)
- Confidence scoring based on shape characteristics

**Published Topics**:
- `/mangueira/position` (PointStamped): Hose center position in image coordinates
- `/mangueira/detections` (Detection2DArray): Complete detection data with confidence
- `/mangueira/debug_image` (Image): Annotated debug visualization

## Specialized Algorithms for Thin Hose Detection

### Challenge: Detecting 1.25cm Hose at 0.1-0.8m Distance

The red hose presents unique challenges:
- **Extremely thin appearance**: 1.25cm diameter at 0.1-0.8m distance = 2-15 pixels width
- **Varying texture**: Red hose may have inconsistent coloration
- **Fragmentation**: Thin objects often appear broken in the image
- **Orientation variability**: Hose can be at any angle

### Optimized Detection Approach

#### 1. **Specialized Morphological Operations**
```python
# Directional kernels for connecting broken segments
horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 1))
vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 7))
diagonal_kernels = [[1,0,0],[0,1,0],[0,0,1]] and [[0,0,1],[0,1,0],[1,0,0]]
```
- Uses multiple directional kernels to connect broken hose segments
- Preserves thin structures while connecting gaps
- Handles all orientations (horizontal, vertical, diagonal)

#### 2. **Advanced Confidence Scoring**
```python
confidence = 0.25 * red_pixel_ratio +     # HSV color consistency
             0.20 * aspect_score +         # Shape elongation
             0.20 * size_score +           # Appropriate size
             0.15 * solidity_score +       # Shape regularity
             0.10 * compactness_score +    # Perimeter/area ratio
             0.10 * position_score         # Center preference
```
- Multi-factor confidence based on shape characteristics
- Optimized for very thin, elongated objects
- Robust to lighting and texture variations

#### 3. **Relaxed Parameter Constraints**
```yaml
min_hose_area: 50          # Very small minimum (was 200)
max_hose_area: 1000        # Reasonable maximum (was 5000)
min_aspect_ratio: 0.0      # No minimum constraint (was 3.0)
max_aspect_ratio: 200.0    # Very high maximum (was 50.0)
min_hose_length: 1         # Minimal length requirement (was 30)
max_hose_width: 800        # Very relaxed width (was 25)
```
- Extremely permissive geometric constraints
- Prioritizes detection over false negative reduction
- Relies on confidence scoring for quality assessment

#### 4. **Dual Red HSV Ranges**
```yaml
# Range 1: Lower red spectrum (0-10 degrees)
red_lower1_h: 0, red_upper1_h: 10

# Range 2: Upper red spectrum (170-200 degrees)  
red_lower2_h: 170, red_upper2_h: 200
```
- Handles red hue wrap-around in HSV space
- Extended upper range (170-200) for better coverage
- Lowered saturation thresholds (50 vs 70) for faded red

### Real-World Deployment Considerations

#### Distance-Based Scaling
- **0.1m distance**: Hose ≈ 12-15 pixels wide, very visible
- **0.3m distance**: Hose ≈ 4-6 pixels wide, challenging
- **0.8m distance**: Hose ≈ 1-3 pixels wide, extremely difficult

#### Expected Performance
- **Close range (0.1-0.3m)**: High confidence detection (0.7-0.9)
- **Medium range (0.3-0.5m)**: Moderate confidence (0.5-0.7)
- **Far range (0.5-0.8m)**: Low confidence (0.3-0.5), may require multiple frames

#### Lighting Adaptations
```yaml
# Bright conditions - increase saturation
red_lower1_s: 70, red_lower2_s: 70

# Dim conditions - decrease saturation  
red_lower1_s: 30, red_lower2_s: 30

# Varying lighting - decrease value threshold
red_lower1_v: 20, red_lower2_v: 20
```

## Configuration Parameters

### Blue Line Detector Parameters

#### HSV Color Range (Navy Blue Fabric)
```yaml
blue_lower_h: 80     # Hue lower bound (updated for better detection)
blue_lower_s: 70     # Saturation lower bound  
blue_lower_v: 25     # Value lower bound (lowered for darker conditions)
blue_upper_h: 140    # Hue upper bound (extended range)
blue_upper_s: 255    # Saturation upper bound
blue_upper_v: 255    # Value upper bound
```

#### Detection Parameters
```yaml
min_line_area: 200         # Reduced for better sensitivity
morph_kernel_size: 7       # Morphological operation kernel size
morph_iterations: 3        # Number of morphological iterations
roi_top: 0.2              # ROI top as fraction of image height
roi_bottom: 0.8           # ROI bottom as fraction of image height
min_line_length: 20       # Reduced minimum pixels for valid line segment
max_line_gap: 50          # Increased maximum gap to connect line segments
```

### Red Hose Detector Parameters

#### HSV Color Range (Red Hose - Optimized for Thin Detection)
```yaml
# Range 1: 0-10 degrees (red lower bound)
red_lower1_h: 0
red_lower1_s: 50    # Reduced for faded/weathered red
red_lower1_v: 30    # Reduced for shadows/dim lighting
red_upper1_h: 10
red_upper1_s: 255
red_upper1_v: 255

# Range 2: 170-200 degrees (red upper bound - extended)
red_lower2_h: 170
red_lower2_s: 50    # Reduced for faded/weathered red
red_lower2_v: 30    # Reduced for shadows/dim lighting
red_upper2_h: 200   # Extended beyond standard 180 for better coverage
red_upper2_s: 255
red_upper2_v: 255
```

#### Detection Parameters (Extremely Relaxed for Thin Objects)
```yaml
min_hose_area: 50          # Very small minimum for thin hose
max_hose_area: 1000        # Reasonable maximum to avoid large objects
morph_kernel_size: 3       # Small kernel to preserve thin structures
morph_iterations: 2        # Limited iterations to avoid over-processing
min_aspect_ratio: 0.0      # No minimum constraint (any shape allowed)
max_aspect_ratio: 200.0    # Very high maximum for extremely thin objects
min_hose_length: 1         # Minimal length requirement
max_hose_width: 800        # Very relaxed width constraint
```

## Usage

### Running the Detectors

**Blue Line Detector**:
```bash
ros2 run sae_cv_utils blueline_detector
```

**Red Hose Detector**:
```bash
ros2 run sae_cv_utils mangueira_detector
```

### With Parameters:
```bash
ros2 run sae_cv_utils blueline_detector --ros-args --params-file config/params.yaml
ros2 run sae_cv_utils mangueira_detector --ros-args --params-file config/params.yaml
```

### Monitoring Detection Results

**Blue Line Following**:
```bash
# Monitor line centroid
ros2 topic echo /blueline/centroid

# Monitor line direction
ros2 topic echo /blueline/direction

# View debug visualization
ros2 run rqt_image_view rqt_image_view /blueline/debug_image
```

**Red Hose Detection**:
```bash
# Monitor hose position
ros2 topic echo /mangueira/position

# Monitor detection array
ros2 topic echo /mangueira/detections

# View debug visualization
ros2 run rqt_image_view rqt_image_view /mangueira/debug_image
```

## Real-World Deployment Tips

### Blue Line Detection

1. **Lighting Conditions**:
   - Adjust HSV ranges based on ambient lighting
   - Navy blue fabric may appear darker in shadows
   - Consider auto-exposure camera settings

2. **Line Visibility**:
   - Ensure minimum line area is visible in frame
   - Adjust ROI parameters based on flight altitude
   - Monitor confidence values for detection quality

3. **Flight Guidance**:
   - Use centroid for lateral positioning
   - Use direction vector for heading alignment
   - Implement smooth control to avoid oscillations

### Red Hose Detection

1. **Hose Characteristics**:
   - Very thin object requires careful aspect ratio tuning
   - Red color may vary under different lighting
   - Consider shadows and reflections

2. **Detection Reliability**:
   - Monitor confidence scores for valid detections
   - Use multiple frames for stable detection
   - Implement temporal filtering for smooth tracking

3. **Camera Position**:
   - Horizontal camera angle affects hose visibility
   - Adjust detection parameters based on viewing angle
   - Consider hose orientation relative to camera

## Color Space Considerations

### HSV vs RGB
- HSV is more robust to lighting changes
- Hue channel isolates color information
- Saturation and Value can be adjusted for conditions

### Red Color Detection
- Red hue wraps around (0° and 360° are both red)
- Use dual ranges: [0-10°] and [170-180°]
- Combine both masks for complete red detection

### Blue Color Detection
- Blue hue is stable around 120° (H=100-130)
- Navy blue has lower saturation and value
- Adjust S,V ranges based on fabric material

## Troubleshooting

### Common Issues

1. **No Line/Hose Detected**:
   - Check HSV parameter ranges
   - Verify camera topic is publishing
   - Ensure adequate lighting
   - Check minimum area thresholds

2. **False Positives**:
   - Tighten geometric constraints
   - Increase confidence thresholds
   - Add more specific shape filters
   - Improve lighting conditions

3. **Unstable Detections**:
   - Implement temporal filtering
   - Adjust morphological parameters
   - Increase processing frequency
   - Use multiple validation criteria

### Debug Visualization

Both detectors publish debug images showing:
- Original image with detections overlaid
- HSV mask visualization
- Contour detection results
- Confidence scores and measurements
- Coordinate system indicators

Use `rqt_image_view` to monitor these debug topics for tuning and troubleshooting.

## Integration with Mission Control

The hook detectors integrate with the mission control system by:

1. **Publishing standardized messages** (PointStamped, Detection2DArray)
2. **Providing confidence scores** for detection quality assessment
3. **Offering configurable parameters** for field tuning
4. **Supporting real-time performance** (10Hz processing frequency)

The detection results can be used by higher-level control nodes to:
- Navigate along the blue line
- Approach and align with the red hose
- Execute the hook mission objectives
