#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
import math
import requests
import json
import re
import threading

class AIDriveNode(Node):
    def __init__(self):
        super().__init__('ai_drive')

        # Parameters
        self.linear_speed = float(self.declare_parameter('linear_speed', 0.2).value)
        self.angular_speed = float(self.declare_parameter('angular_speed', 0.5).value)
        self.move_distance = float(self.declare_parameter('move_distance', 1.25).value)

        # Obstacle avoidance parameters with defaults
        self.stop_distance = float(self.declare_parameter('stop_distance', 0.5).value)
        self.spin_speed = float(self.declare_parameter('spin_speed', 1.0).value)
        self.max_spin_attempts = int(self.declare_parameter('max_spin_attempts', 4).value)
        self.attempt_timeout = float(self.declare_parameter('attempt_timeout', 2.5).value)

        # AI agent parameters
        self.model_url = self.declare_parameter('model_url', '').value
        self.model_name = self.declare_parameter('model_name', 'qwen').value

        # Publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

        # State machine variables
        self.state = 'WAIT'
        self.drive_duration = None
        self.action_start_time = None
        self.wait_timeout = 2.0

        # LIDAR subscription for obstacle avoidance (30Hz polling for responsiveness)
        self.scan_message = None
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Clock and timer
        self.clock = self.get_clock()
        self.timer_period = 0.1  # 100ms polling - increased frequency for faster response
        self.timer = self.create_timer(self.timer_period, self.drive_callback)

        # Create services (MUST be instance methods to access self)
        self.reset_service = self.create_service(Trigger, '/ai_driver/reset', self.handle_reset)
        self.start_service = self.create_service(Trigger, '/ai_driver/start', self.handle_start)

        # AI-based spin tracking (for OBSTACLE_AVOID state)
        self.ai_spin_request_count = 0
        self.spin_retry_count = 0
        self.last_spin_angular = 0.0
        self.ai_request_count = None  # For rate-limited logging
        self.last_ai_request_time = None  # Timestamp of last request
        self.last_request_success = True
        self.fallback_spin_start_time = None
        self.fallback_spin_completed = False

        # Background AI Request Handling
        self.ai_request_in_progress = False
        self.ai_response_data = None
        self.ai_lock = threading.Lock()

        self.get_logger().info(
            f"Simple Drive Node initialized. "
            f"Linear speed: {self.linear_speed} m/s, Angular speed: {self.angular_speed} rad/s, "
            f"Move distance: {self.move_distance}m, Stop distance: {self.stop_distance}m, "
            f"Spin speed: {self.spin_speed} rad/s"
        )

    def _prepare_lidar_payload(self):
        """Prepare LIDAR scan data as JSON payload for AI request."""
        if self.scan_message is None or len(self.scan_message.ranges) == 0:
            return {"error": "No valid LIDAR data"}

        ranges = self.scan_message.ranges
        range_min = self.scan_message.range_min
        range_max = self.scan_message.range_max
        angle_min = self.scan_message.angle_min
        angle_increment = self.scan_message.angle_increment

        self.get_logger().info(f"DEBUG LIDAR metrics: ranges_len={len(ranges)}, range_min={range_min}, range_max={range_max}, angle_min={angle_min}, angle_max={self.scan_message.angle_max}, angle_increment={angle_increment}")

        # Sample 9 points between full left and full right
        target_angles_deg = [-180, -135, -90, -45, 0, 45, 90, 135, 180]
        obstacle_readings = []

        for target_angle in target_angles_deg:
            # Find the index in the LIDAR scan that is closest to target_angle
            # angle = angle_min + idx * angle_increment
            idx = int((target_angle - angle_min) / angle_increment)
            # Clamp idx to valid range
            idx = max(0, min(idx, len(ranges) - 1))
            distance = ranges[idx]

            angle = angle_min + idx * angle_increment
            is_valid = (distance < range_max * 0.95) and (distance >= range_min)
            if is_valid:
                obstacle_readings.append({
                    "angle_deg": math.degrees(angle),
                    "distance": distance
                })

        if obstacle_readings:
            angles = [r["angle_deg"] for r in obstacle_readings]
            self.get_logger().info(
                f"DEBUG: {len(obstacle_readings)} valid readings at target angles. "
                f"Angles: {[f'{a:.2f}' for a in angles]}"
            )

        min_distance = float('inf')
        for reading in obstacle_readings:
            if reading["distance"] < min_distance:
                min_distance = reading["distance"]

        payload = {
            "scan_type": "obstacle_avoid",
            "timestamp": str(self.clock.now().nanoseconds / 1e9),
            "range_min": float(range_min),
            "range_max": float(range_max),
            "num_sensors": len(ranges),
            "closest_obstacle_distance": float(min_distance) if min_distance < float('inf') else None,
            "obstacle_readings": obstacle_readings,
            "request": "I have gathered LIDAR data for 9 directions: -180, -135,-90, -45, 0, 45, 90, 135 and 180 degrees. Analyze this data and determine: 1) Is there a clear path in any of these directions? 2) If an obstacle exists, should we spin to avoid it? How many degrees? 3) If a clear path is found, what is the best direction to resume forward motion? Return in JSON format with path_clear (boolean), spin_direction (\"left\" or \"right\"), spin_angle_degrees (float), and suggested_heading_degrees (float). If path_clear is true, suggested_heading_degrees should be the angle of the clear path."
        }

        return payload


    def _send_ai_request(self, payload=None):
        """Send sensor data to AI agent and parse response."""
        # Only log request if we haven't logged it in last 5 seconds (reduce spam)
        if self.ai_request_count is None or (self.clock.now().nanoseconds / 1e9 - self.last_ai_request_time >= 5.0):
            self.get_logger().info(f"--- [AI REQUEST] Attempting API call to {self.model_url}/api/v1/chat ---")

        if not self.model_url:
            self.get_logger().warning("No model URL configured. Skipping AI request.")
            return None

        try:
            # Use provided payload or generate LIDAR-based payload
            actual_payload = payload or self._prepare_lidar_payload()

            # Only log full payload info on first request in OBSTACLE_AVOID (reduce spam)
            if self.state == 'OBSTACLE_AVOID' and self.ai_request_count is None:
                self.get_logger().info(f"[AI] Payload type: {type(actual_payload).__name__}")

            # Record timing for rate-limiting logs
            self.ai_request_count = (self.ai_request_count or 0) + 1
            self.last_ai_request_time = self.clock.now().nanoseconds / 1e9

            response = requests.post(
                f"{self.model_url}/api/v1/chat",
                json={
                    "model": self.model_name,
                    "input": json.dumps(actual_payload),
                    "stream": False
                },
                headers={"Content-Type": "application/json"},
                timeout=240  # Increased timeout for slow AI server (240s)
            )

            response.raise_for_status()
            result = response.json()
            ai_response = ""
            if isinstance(result.get('output'), list):
                for item in result['output']:
                    if isinstance(item, dict) and 'content' in item:
                        ai_response += item['content'] + "\n"
            elif isinstance(result.get('output'), str):
                ai_response = result['output']
            elif isinstance(result, dict) and 'choices' in result and result['choices']:
                ai_response = result['choices'][0].get('message', {}).get('content', '')

            self.get_logger().info(f"[AI] Response length: {len(ai_response)} chars")
            self.get_logger().info(f"[AI] Raw response preview:\n{ai_response[:300]}...")  # Log first 300 chars for debugging

            # 1. Try to find a JSON block in the response
            path_clear = False
            spin_direction = None
            spin_angle_degrees = 0.0

            json_data = None
            try:
                # Look for a JSON block in the response
                # This regex looks for a string starting with { and ending with }
                json_match = re.search(r'(\{.*\})', ai_response, re.DOTALL)
                if json_match:
                    json_data = json.loads(json_match.group(1))
                    if isinstance(json_data, dict):
                        path_clear = json_data.get('path_clear', False)
                        spin_direction = json_data.get('spin_direction')
                        spin_angle_degrees = json_data.get('spin_angle_degrees', 0.0)
                        self.get_logger().info(f"[AI] Parsed JSON: path_clear={path_clear}, spin_dir={spin_direction}, angle={spin_angle_degrees}")
            except Exception as e:
                self.get_logger().debug(f"[AI] JSON parsing failed: {e}")

            # 2. If JSON parsing didn't give us a clear result, fallback to very conservative keyword matching
            if not path_clear:
                # Only use keywords if the JSON block was missing or didn't contain path_clear
                # We also check for more explicit "clearance" phrases
                explicit_clear_keywords = ["clear path", "no obstacle", "obstacle cleared", "path is clear"]
                if any(kw in ai_response.lower() for kw in explicit_clear_keywords):
                    path_clear = True
                    self.get_logger().info(f"[AI] Detected clear path via explicit keywords")
                # We REMOVED "safe", "go ahead", "proceed" as they cause too many false positives

            # 3. Extract spin_direction if not found in JSON
            if spin_direction is None:
                if "left" in ai_response.lower():
                    spin_direction = "left"
                elif "right" in ai_response.lower():
                    spin_direction = "right"
                else:
                    spin_direction = "right" # Default

            # 4. Extract spin_angle_degrees if not found in JSON
            if spin_angle_degrees == 0.0:
                angle_match = re.search(r'"spin_angle_degrees"\s*[:\s]*(\d+(?:\.\d+)?)', ai_response)
                if not angle_match:
                    angle_match_alt = re.search(r'"spin_angle"\s*[:\s]*(\d+(?:\.\d+)?)', ai_response)
                    if angle_match_alt:
                        try:
                            spin_angle_degrees = float(angle_match_alt.group(1))
                        except (ValueError, TypeError):
                            pass
                else:
                    try:
                        spin_angle_degrees = float(angle_match.group(1))
                    except (ValueError, TypeError):
                        pass

            # 5. Handle "FORWARD", "STOP", etc. (Secondary verification)
            if spin_direction is None or spin_angle_degrees == 0.0:
                if re.search(r'(\b(FORWARD|CLEAR PATH|PATH CLEAR|GO)\b)', ai_response):
                    path_clear = True
                    self.get_logger().info(f"[AI] Detected clear path command")
                elif re.search(r'(\b(STOP|HOLD|WAIT)\b)', ai_response):
                    if not path_clear:
                        if spin_direction is None:
                            spin_direction = "left"
                        if spin_angle_degrees == 0.0:
                            spin_angle_degrees = 20.0  # Small reduction

            # 6. Final safety fallbacks
            if not path_clear:
                if spin_direction is None:
                    spin_direction = "left"
                    self.get_logger().info(f"[AI] Using default direction: LEFT")

                if spin_angle_degrees == 0.0:
                    spin_angle_degrees = 45.0
                    self.get_logger().warning(f"[AI] Using default spin angle: 45° (AI did not specify an angle)")

            # Calculate angular velocity
            angular_velocity = 0.0
            if not path_clear and spin_angle_degrees != 0.0:
                angular_radians = math.radians(spin_angle_degrees)
                # Divide by polling period (0.1s) gives radians covered per second at this step rate
                angular_velocity = -angular_radians / self.timer_period  # Negative for consistent spin direction with fallback
                self.get_logger().info(f"[AI CALC] spin_angle={spin_angle_degrees}° -> {angular_velocity:.4f} rad/s")

            if path_clear:
                # Clear path detected - verify with local sensors before transitioning
                actual_dist = self.find_closest_obstacle()
                if actual_dist is None or actual_dist > self.stop_distance:
                    # Clear path detected - transition back to forward movement state
                    self.state = 'FORWARD_1'
                    self.action_start_time = self.clock.now().nanoseconds / 1e9
                    self.get_logger().info(f"[AI] Path cleared! Transitioning from OBSTACLE_AVOID back to FORWARD_1")
                else:
                    self.get_logger().warning(f"[AI] Path reported clear, but obstacle detected at {actual_dist:.2f}m. Staying in OBSTACLE_AVOID.")
                    self.twist.angular.z = -self.spin_speed
                    self.publisher_.publish(self.twist)

            # APPLY VELOCITY COMMAND based on AI decision
            self.twist.linear.x = 0.0  # Always zero linear in OBSTACLE_AVOID until path is clear
            self.twist.angular.z = angular_velocity
            self.get_logger().info(f"[AI APPLIED] Setting angular_z={angular_velocity:.4f} rad/s")

            self.publisher_.publish(self.twist)
        except Exception as e:
            self.last_request_success = False
            self.get_logger().error(f"[AI] Error in _send_ai_request: {e}")

    def handle_reset(self, request, response):
        """Handle reset service calls - MUST be at class level"""
        self.get_logger().info("--- [RESET SERVICE] Resetting robot state ---")

        # Reset ALL state including AI spin request tracking
        self.state = 'WAIT'
        self.action_start_time = None
        self.drive_duration = None
        self.scan_message = None
        self.ai_spin_request_count = 0
        self.spin_retry_count = 0
        self.last_spin_angular = 0.0
        self.ai_request_count = None  # Reset to force logging on next request
        self.last_ai_request_time = None
        self.last_request_success = True
        self.fallback_spin_start_time = None
        self.fallback_spin_completed = False

        # Clear any pending velocity commands immediately
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

        response.success = True
        response.message = "Robot reset successfully"
        return response

    def scan_callback(self, msg):
        """Callback when new LIDAR scan data is received."""
        self.scan_message = msg

    def find_closest_obstacle(self):
        """Find closest obstacle in forward direction (all angles)."""
        if self.scan_message is None or len(self.scan_message.ranges) == 0:
            return None

        ranges = self.scan_message.ranges
        max_range = self.scan_message.range_max
        angle_min = self.scan_message.angle_min
        angle_increment = self.scan_message.angle_increment

        # Collect all valid readings (use actual LIDAR FOV from message)
        forward_readings = []
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            # Include readings that are valid (not noise, not beyond max range)
            is_valid = (distance < max_range * 0.95) and (distance >= self.scan_message.range_min)
            if is_valid:
                forward_readings.append((i, angle, distance))

        # Find closest obstacle (valid readings only)
        min_distance = float('inf')
        for i, angle, distance in forward_readings:
            if distance < min_distance:
                min_distance = distance

        return min_distance if min_distance < float('inf') else None

    def handle_start(self, request, response):
        """Handle start service calls - immediately starts forward movement"""
        # Reset state machine and transition to FORWARD_1
        self.state = 'FORWARD_1'
        self.action_start_time = self.clock.now().nanoseconds / 1e9
        self.drive_duration = None

        # Clear any pending velocity commands
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

        # Reset obstacle avoidance tracking (for AI-based spin decisions)
        self.scan_message = None

        response.success = True
        response.message = "Starting movement sequence"
        return response

    def drive_callback(self):
        now = self.clock.now().nanoseconds / 1e9

        # Check for obstacle in moving states (FORWARD_1, TURN_SPIN, FORWARD_2, FINISH_SPIN)
        obstacle_distance = None
        if self.state in ['FORWARD_1', 'TURN_SPIN', 'FORWARD_2', 'FINISH_SPIN']:
            obstacle_distance = self.find_closest_obstacle()

        # Wait state
        if self.state == 'WAIT':
            return

        elif self.state == 'FORWARD_1':
            # Obstacle avoidance - emergency stop if obstacle detected
            if obstacle_distance is not None and obstacle_distance <= self.stop_distance:
                self.state = 'OBSTACLE_AVOID'
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)
            else:
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)

                duration = self.move_distance / self.linear_speed
                if self.action_start_time is not None and (now - self.action_start_time >= duration):
                    self.state = 'TURN_SPIN'
                    self.action_start_time = now

        elif self.state == 'OBSTACLE_AVOID':
            obstacle_dist = self.find_closest_obstacle()

            # Check for recent API errors - used to decide fallback behavior
            has_recent_error = not self.last_request_success

            if obstacle_dist is None:
                # No LIDAR data available - spin ONLY if recent AI calls succeeded or we haven't exhausted retries
                if self.spin_retry_count < self.max_spin_attempts and has_recent_error:
                    # Spin to find path while retrying (but don't spin indefinitely on error)
                    current_angular_vel = -self.spin_speed
                    self.get_logger().info(f"[NO LIDAR + RETRYING] Spinning to find path (attempt {self.spin_retry_count})")
                elif not has_recent_error and self.ai_request_count is None:
                    # First request, no AI response yet - don't spin aggressively
                    current_angular_vel = 0.0
                    self.get_logger().info("[NO LIDAR + FIRST REQUEST] Waiting for LIDAR data or AI response")

            elif (obstacle_dist <= self.stop_distance and self.spin_retry_count < self.max_spin_attempts):
                # Obstacle still within stop distance - try to find clear path
                if abs(self.twist.angular.z) >= 1e-6:
                    # AI is responding with spin commands, use them
                    current_angular_vel = self.twist.angular.z
                elif has_recent_error:
                    # AI errored recently but we need to keep trying - spin with reduced speed
                    current_angular_vel = self.spin_speed * 0.5
                    self.get_logger().warning(f"[AI ERROR + FALLBACK] Obstacle at {obstacle_dist:.2f}m, using reduced fallback spin")
                else:
                    # No valid spin command from AI - use fallback only if retry count allows
                    current_angular_vel = self.spin_speed * 0.5
                    self.get_logger().warning(f"[AI NO SPIN COMMAND] Obstacle at {obstacle_dist:.2f}m, using reduced fallback spin (retry {self.spin_retry_count})")

            elif obstacle_dist is not None and obstacle_dist > self.stop_distance:
                # Obstacle is beyond stop distance - path is clear, resume forward motion
                current_angular_vel = 0.0
                self.get_logger().info(f"[OBSTACLE CLEAR] Path open at {obstacle_dist:.2f}m, angular vel={current_angular_vel:.4f}")

                # Transition back to FORWARD_1 after a brief pause (allows robot to verify clear path)
                duration = self.move_distance / self.linear_speed
                if self.action_start_time is not None and (now - self.action_start_time >= duration):
                    self.state = 'FORWARD_1'
                    self.spin_retry_count = 0
                    self.action_start_time = now
                    self.get_logger().info(f"[OBSTACLE_AVOID] Path verified clear, transitioning to FORWARD_1")

            # Apply velocity: zero linear, angular handles spin/forward decisions
            self.twist.linear.x = 0.0
            self.twist.angular.z = current_angular_vel

            # Track for retry logic
            self.last_spin_angular = current_angular_vel

            self.publisher_.publish(self.twist)

        elif self.state == 'TURN_SPIN':
            # Obstacle avoidance check
            if obstacle_distance is not None and obstacle_distance <= self.stop_distance:
                self.state = 'OBSTACLE_AVOID'
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)
            else:
                speed = self.linear_speed  # moving backward relative to original heading
                self.twist.linear.x = speed
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)

                duration = self.move_distance / abs(speed)
                if self.action_start_time is not None and (now - self.action_start_time >= duration):
                    self.state = 'FINISH_SPIN'
                    self.action_start_time = now

        elif self.state == 'FINISH_SPIN':
            # Obstacle avoidance check
            if obstacle_distance is not None and obstacle_distance <= self.stop_distance:
                self.state = 'OBSTACLE_AVOID'
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.angular_speed  # Right turn for 180°
                self.publisher_.publish(self.twist)

                angle_to_spin = math.pi  # 180° in radians
                spin_duration = angle_to_spin / self.angular_speed

                if self.action_start_time is not None and (now - self.action_start_time >= spin_duration):
                    self.state = 'DONE'
                    self.action_start_time = now

        elif self.state == 'DONE':
            # Keep publishing stopped velocity in DONE state
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init()
    node = AIDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
