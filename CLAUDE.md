# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS 2 (Robot Operating System) Python package called `ai_driver` that implements obstacle detection using LiDAR and AI-assisted navigation decisions. The robot performs a multi-stage turning maneuver (forward, spin, alternate direction) to navigate around obstacles.

## Common Commands

### Build from source
```bash
colcon build --packages-select ai_driver
```

### Install in development mode
```bash
colcon build --packages-select ai_driver --event-on-request DEBUG --install-ros-package-deps
source /opt/ros/<distro>/setup.bash
python3 -m pip install -e .
```

### Run tests
```bash
# Run all tests
colcon test --packages-select ai_driver

# Run a single test file
pytest test/test_flake8.py

# Run pytest with verbose output
pytest -v
```

### Run the node
```bash
# Start ROS 2 environment
source /opt/ros/<distro>/setup.bash

# Launch the node (ROS 2 terminal or source)
ros2 launch ai_driver/launch_ai_drive_node.py
```

## Architecture

### Package Structure
- `ai_driver/ai_driver_node.py` - Main ROS 2 Node implementing AIDriveNode class
- `resource/` - ROS 2 resource files for package installation
- `test/` - Python test files (ament_copyright, flake8, pep257)
- `setup.py` - Python packaging configuration

### Key Components

**AIDriveNode** (`ai_driver_node.py`) - The main class with these responsibilities:

1. **ROS 2 Subscribers/Publishers**:
   - `/scan` (LaserScan message at 30Hz) - LiDAR obstacle detection
   - `cmd_vel` (Twist message) - Robot velocity control

2. **ROS 2 Services**:
   - `/ai_drive/start` - Starts forward movement sequence
   - `/ai_drive/reset` - Resets state machine

3. **State Machine** - Manages robot navigation states:
   - `WAIT` → `FORWARD_1` → `TURN_SPIN` → `FORWARD_2` → `FINISH_SPIN` → `DONE`
   - Emergency state: `OBSTACLE_AVOID` (activated when obstacle ≤ stop_distance)

4. **AI Integration**:
   - Optional AI agent calls via HTTPS API (`/api/v1/chat`)
   - Models configured via `model_url` and `model_name` parameters
   - Response parsing extracts `linear.x` and `angular.z` values
   - Falls back to predefined commands: FORWARD, STOP, SPIN

### Parameters (declare_parameter)

**Drive parameters**:
- `linear_speed` (float, default 0.2) - Forward velocity in m/s
- `angular_speed` (float, default 0.5) - Rotation velocity in rad/s
- `move_distance` (float, default 1.25) - Distance to travel per stage in meters

**Obstacle avoidance parameters**:
- `stop_distance` (float, default 0.5) - Threshold distance for obstacle detection
- `spin_speed` (float, default 1.0) - Speed when spinning/avoiding obstacles
- `max_spin_attempts` (int, default 4) - Maximum LIDAR spin attempts before giving up
- `attempt_timeout` (float, default 2.5) - Time between spin attempts

**AI parameters**:
- `model_url` (string, empty) - AI API endpoint URL
- `model_name` (string, default 'qwen') - Model name to use

## ROS 2 Dependencies

**Required packages** (in package.xml):
- `rclpy` - ROS 2 Python client library
- `geometry_msgs` - Contains Twist message
- `sensor_msgs` - Contains LaserScan message
- `std_msgs` - Base messages
- `std_srvs` - Trigger service messages

## Testing Notes

The test files use ament validation:
- `test_copyright.py` - Checks for license headers
- `test_flake8.py` - Code style checking
- `test_pep257.py` - Docstring verification

Run with colcon to get ROS 2-compliant test output:
```bash
colcon test --packages-select ai_driver --event-handlers console_default+
```

## AI Response Handling

The node parses AI responses in this order (first match wins):
1. JSON object with `{"linear.x": value, "angular.z": value}`
2. Plain JSON strings directly returning output
3. Pattern-based regex extraction for `linear.x` and `angular.z`
4. State-based commands: FORWARD/STOP/SPIN text

Fallback behavior: If no AI URL configured, or if parsing fails, uses predefined velocity values based on the command words detected.

## Clock Timing Notes

- LiDAR subscription at 30Hz for obstacle detection (responsive)
- Timer callback at 100ms (`timer_period`) - checks state when moving
- Use `clock.now().nanoseconds / 1e9` for wall-clock time comparisons
- Duration-based state transitions prevent infinite loops
