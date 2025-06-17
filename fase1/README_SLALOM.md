# Slalom Mission Implementation

This project implements a finite state machine for a drone slalom mission where the drone must traverse 4 poles in alternating sides.

## Mission Overview

The drone performs the following sequence:
1. **Initial Takeoff** - Takes off to specified height and saves middle position
2. **Search Pole** - Rotates to search for target pole using post detection
3. **Approach Pole** - Centers and approaches pole using PID control
4. **Around Pole** - Navigates in semicircle around pole (alternating left/right)
5. **Check Next Pole** - Determines if mission continues or completes
6. **Return Middle** - Returns to starting position after all poles
7. **Final Landing** - Lands at middle position

## State Machine States

### SearchPoleState
- Rotates drone to search for target pole
- Uses post detection from `/slalom` topic
- Finds most centered pole detection
- Returns "POLE_FOUND" or "POLE_NOT_FOUND"

### ApproachPoleState  
- Uses PID controllers for yaw (centering) and position (distance)
- Approaches pole until target width is achieved
- Maintains pole in center of image
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
takeoff_height: -3.0          # Takeoff altitude (negative in NED)
max_vertical_velocity: 1.0    # Max vertical speed (m/s)
max_horizontal_velocity: 1.0  # Max horizontal speed (m/s)

# Slalom mission parameters
total_poles: 4                # Number of poles to traverse
rotation_speed: 0.5           # Search rotation speed (rad/s)
approach_speed: 0.8           # Pole approach speed (m/s)
navigation_radius: 1.5        # Circular navigation radius (m)
angular_velocity: 0.3         # Navigation angular velocity (rad/s)
return_speed: 1.0             # Return to middle speed (m/s)
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
  -p total_poles:=3 \
  -p navigation_radius:=2.0
```

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

- **Alternating Traversal**: Poles 1&3 traversed clockwise, poles 2&4 counter-clockwise
- **Robust Error Handling**: Timeouts and fallback transitions
- **PID Control**: Smooth pole approach and centering
- **Post Detection Integration**: Real-time pole detection using computer vision
- **Configurable Parameters**: Easy tuning through launch parameters
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
