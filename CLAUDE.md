# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS 2 (Robot Operating System) Python package called `ai_driver` that implements obstacle detection using LiDAR and AI-assisted navigation decisions. The robot performs a multi-stage turning maneuver (forward, spin, alternate direction) to navigate around obstacles.

## Architecture

### Single-File Node Design

The entire ROS 2 node is implemented in `ai_driver_node.py` with these core components:

**AIDriveNode class** - Single responsibility class with these subsystems:

1. **ROS 2 Subscribers**:
   - `/scan` (LaserScan at 30Hz) - Obstacle detection from LiDAR
   
2. **ROS 2 Publishers**:
   - `cmd_vel` (Twist) - Velocity commands to robot

3. **ROS 2 Services** (Trigger messages):
   - `/ai_drive/start` - Starts forward movement sequence, resets to FORWARD_1 state
   - `/ai_drive/reset` - Resets entire state machine and clears velocity commands
   
4. **Timer Callback** (100ms polling):
   - Checks state transitions and obstacle avoidance logic

5. **State Machine**:
   ```
   WAIT → [FORWARD_1] → TURN_SPIN → FORWARD_2 → FINISH_SPIN → DONE
                              ↓ OBSTACLE_AVOID (emergency branch)
              └──────────────┘
   ```
   
   - State machine is driven by duration-based transitions in the 100ms timer callback
   - Emergency state `OBSTACLE_AVOID` activated when obstacle ≤ stop_distance

### Key Design Patterns

**Duration-Based State Transitions** (avoid infinite loops):
```python
duration = self.move_distance / self.linear_speed
if self.action_start_time and (now - self.action_start_time >= duration):
    self.state = next_state  # Transition to next stage
```

**Obstacle Avoidance Override**:
All motion commands check for `OBSTACLE_AVOID` state first:
```python
def move_forward(self, speed=None):
    if self.state == 'OBSTACLE_AVOID':
        return True  # Stay stopped and waiting
    # ... normal motion logic
```

**AI Agent Integration**:
- Optional AI calls via HTTPS POST to `{model_url}/api/v1/chat`
- Response parsing order (first match wins):
  1. JSON with `{"path_clear": bool, "spin_direction": str, "spin_angle_degrees": float}`
  2. Fallback keyword matching for path clearance
  3. Default fallback spin commands if AI not configured

**LIDAR Payload Construction**:
The `_prepare_lidar_payload()` method samples 4 directions (-90, -45, 45, 90°):
```python
for target_angle in [-180, -135, -90, -45, 0, 45, 90, 135, 180]:
    # Calculate LIDAR index from angle
    idx = int((target_angle - angle_min) / angle_increment)
    distance = ranges[idx]
```

## Parameters

### Drive Parameters
- `linear_speed` (float, default 0.2) - Forward velocity in m/s
- `angular_speed` (float, default 0.5) - Rotation velocity in rad/s
- `move_distance` (float, default 1.25) - Distance to travel per stage in meters

### Obstacle Avoidance Parameters
- `stop_distance` (float, default 0.5) - Threshold for obstacle detection
- `spin_speed` (float, default 1.0) - Speed when spinning/avoiding obstacles
- `max_spin_attempts` (int, default 4) - Max LIDAR spin attempts before giving up
- `attempt_timeout` (float, default 2.5) - Time between spin attempts

### AI Parameters
- `model_url` (string, empty) - AI API endpoint URL
- `model_name` (string, default 'qwen') - Model name to use

## Dependencies

**ROS 2 packages**:
- `rclpy` - ROS 2 Python client
- `sensor_msgs` - Contains LaserScan message
- `geometry_msgs` - Contains Twist message
- `std_msgs` - Base messages
- `std_srvs` - Trigger service messages

**Python**:
- `requests` - HTTP API calls to AI agent
- `json`, `math`, `re`, `threading` - Standard library

## Commands

### Build from source
```bash
colcon build --packages-select ai_driver
```

### Install in development mode
```bash
source /opt/ros/<distro>/setup.bash
python3 -m pip install -e .
```

### Run tests
```bash
# Run all tests
colcon test --packages-select ai_driver

# Run a single test file (pytest)
pytest test/test_flake8.py

# Run pytest with verbose output
pytest -v
```

### Run the node
```bash
source /opt/ros/<distro>/setup.bash
ros2 launch ai_driver/launch_ai_drive_node.py
```

## Testing Notes

Test files use ament validation:
- `test_copyright.py` - License header check
- `test_flake8.py` - Code style verification
- `test_pep257.py` - Docstring checks

Run tests individually with pytest for verbose output and to isolate failures.

## State Machine Flow

**Normal Operation**:
1. Service call triggers `/ai_drive/start` → state = FORWARD_1
2. Move forward at linear_speed for move_distance
3. Transition to TURN_SPIN (backward motion)
4. Transition to FINISH_SPIN (right turn 180°)
5. State = DONE, velocity = (0, 0)

**Obstacle Avoidance Flow**:
When obstacle detected in FORWARD_1:
1. State → OBSTACLE_AVOID (emergency stop)
2. Call AI with LIDAR data from 4 directions
3. Parse AI response for path_clear and spin_angle
4. If path_clear + verified by local sensors → return to FORWARD_1
5. If no clear path → continue spinning at reduced speed
6. Spin attempts limited by max_spin_attempts

**AI Response Parsing Strategy**:
The node uses multi-level fallback:
1. Extract JSON object with structured response
2. Fallback to keyword patterns for "clear path", "no obstacle"
3. Extract spin direction from "left"/"right" keywords
4. Use default values (direction=LEFT, angle=45°) if parsing fails

## Clock Timing Notes

- LiDAR subscription at 30Hz for responsive obstacle detection
- Timer callback at 100ms (`timer_period`) - main state machine drive loop
- Use `self.clock.now().nanoseconds / 1e9` for wall-clock time comparisons
- Duration-based transitions prevent infinite loops (e.g., duration = distance / speed)
