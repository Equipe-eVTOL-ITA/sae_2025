# Slalom Mission Implementation

This project implements a finite state machine for a drone slalom mission where the drone must traverse 4 poles of specific colors in a configurable order.

## Mission Overview

The drone performs the following sequence:
1. **Initial Takeoff** - Takes off to specified height and saves middle position
2. **Search Pole** - Rotates to search for target pole by color using post detection
3. **Approach Pole** - Centers and approaches pole using PID control
4. **Around Pole** - Navigates in semicircle around pole (alternating left/right)
5. **Check Next Pole** - Determines if mission continues or completes
6. **Return Middle** - Returns to starting position after all poles
7. **Final Landing** - Lands at middle position

## Color-Based Pole Detection

The system uses computer vision to detect poles by their colors:
- **Rosa** (Pink) - HSV range configurable in post_detector parameters
- **Vermelho** (Red) - HSV range with wrap-around handling  
- **Preto** (Black) - Low value range for dark objects
- **Azul** (Blue) - Blue hue range avoiding sky detection

### Configurable Traversal Order

The pole traversal order is configured via parameters:
- `color_1`: First pole color to search for
- `color_2`: Second pole color to search for  
- `color_3`: Third pole color to search for
- `color_4`: Fourth pole color to search for

Default sequence: Rosa → Vermelho → Preto → Azul

## State Machine States

### SearchPoleState
- Rotates drone to search for target pole by specific color
- Uses post detection from `/slalom` topic with color filtering
- Finds most centered pole detection of target color
- Returns "POLE_FOUND" or "POLE_NOT_FOUND"

### ApproachPoleState  
- Uses PID controllers for yaw (centering) and position (distance)
- Approaches pole of target color until target width is achieved
- Maintains pole in center of image with color verification
- Handles loss of detection with 10-second timeout
- Returns "APPROACH_COMPLETE", "APPROACH_TIMEOUT", or "POLE_LOST"

### AroundPoleState
- Performs semicircular navigation around pole
- Alternates direction: pole 1&3=right, pole 2&4=left
- Uses circular trajectory with configurable radius
- Returns "NAVIGATION_COMPLETE" or "NAVIGATION_TIMEOUT"

### ReturnMiddleState
- Returns drone to starting middle position
- Uses straight-line navigation
- Returns "RETURN_COMPLETE" or "RETURN_TIMEOUT"

### CheckNextPoleState
- Increments pole counter and checks mission status
- Returns "CONTINUE_MISSION" or "MISSION_COMPLETE"

## Configuration Parameters

Located in `launch/params.yaml` under `fase1_fsm`:

```yaml
# Basic flight parameters
takeoff_height: -1.7          # Takeoff altitude (negative in NED)
max_vertical_velocity: 1.0    # Max vertical speed (m/s)
max_horizontal_velocity: 1.0  # Max horizontal speed (m/s)

# Slalom mission parameters
total_poles: 4                # Number of poles to traverse
rotation_speed: 0.5           # Search rotation speed (rad/s)
approach_speed: 0.8           # Pole approach speed (m/s)
navigation_radius: 1.0        # Circular navigation radius (m)
angular_velocity: 0.3         # Navigation angular velocity (rad/s)
return_speed: 1.0             # Return to middle speed (m/s)

# Pole color sequence (colors to traverse in order)
color_1: "Rosa"               # First pole color
color_2: "Vermelho"           # Second pole color  
color_3: "Preto"              # Third pole color
color_4: "Azul"               # Fourth pole color

# Search parameters
max_search_time: 30.0         # Max search time per pole (seconds)

# Approach parameters
goal_width: 0.3               # Target pole width in image (0.0-1.0)
max_approach_time: 20.0       # Max approach time (seconds)
centering_tolerance: 0.05     # Pole centering tolerance (5%)

# PID parameters for approach
pid_yaw_kp: 1.0               # PID proportional gain for yaw control
pid_yaw_ki: 0.1               # PID integral gain for yaw control
pid_yaw_kd: 0.05              # PID derivative gain for yaw control
pid_width_kp: 0.5             # PID proportional gain for width control
pid_width_ki: 0.1             # PID integral gain for width control
pid_width_kd: 0.02            # PID derivative gain for width control

# Navigation parameters
max_navigation_time: 45.0     # Max navigation time around pole (seconds)

# Return parameters
position_tolerance: 0.3       # Return position tolerance (meters)
max_return_time: 30.0         # Max return time (seconds)
```

## Post Detection

The system uses the `post_detector` node from `sae_cv_utils` package:
- Subscribes to `/horizontal_camera/compressed`
- Publishes detections to `/slalom` topic
- Configurable HSV color ranges for different pole colors
- Morphological filtering and aspect ratio constraints

## Usage

### Launch the Mission
```bash
ros2 launch sae_fase1 fase1.launch.py
```

### Parameters Override
```bash
ros2 launch sae_fase1 fase1.launch.py \
  -p takeoff_height:=-2.5 \
  -p color_1:="Azul" \
  -p color_2:="Rosa" \
  -p navigation_radius:=2.0
```

### Color Detection Tuning
Edit the HSV color ranges in `params.yaml` under `post_detector` section to adjust color detection sensitivity for different lighting conditions.

## State Transitions

```
INITIAL TAKEOFF → SEARCH POLE
SEARCH POLE → APPROACH POLE (pole found) | RETURN MIDDLE (not found)
APPROACH POLE → AROUND POLE (complete) | SEARCH POLE (pole lost)
AROUND POLE → CHECK NEXT POLE
CHECK NEXT POLE → SEARCH POLE (continue) | RETURN MIDDLE (complete)
RETURN MIDDLE → FINAL LANDING
FINAL LANDING → FINISHED
```

## Key Features

- **Color-Based Detection**: Searches for specific pole colors in configurable sequence
- **Alternating Traversal**: Poles 1&3 traversed clockwise, poles 2&4 counter-clockwise
- **Robust Error Handling**: Timeouts and fallback transitions with detection loss handling
- **PID Control**: Smooth pole approach and centering with color verification
- **Post Detection Integration**: Real-time pole detection using computer vision with HSV filtering
- **Configurable Parameters**: Easy tuning through launch parameters including color sequence
- **Performance Optimized**: Selective subscription management for efficiency

## Files Structure

```
fase1/
├── include/fase1/
│   ├── search_pole_state.hpp      # Pole search state
│   ├── approach_pole_state.hpp    # Pole approach state  
│   ├── around_pole_state.hpp      # Pole navigation state
│   ├── return_middle_state.hpp    # Return to middle state
│   └── check_next_pole_state.hpp  # Mission progress check
├── src/
│   └── fase1.cpp                  # Main FSM and node implementation
└── launch/
    ├── fase1.launch.py            # Launch file with post detector
    └── params.yaml                # Mission and detection parameters
```
