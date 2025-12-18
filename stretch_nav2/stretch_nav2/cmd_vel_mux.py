#!/usr/bin/env python3

"""
Cmd Vel Mux Node

This node sits between Nav2's controller_server and the stretch_driver,
providing directional control over navigation.

Control Input (Float32MultiArray with 4 values: [forward, left, right, back]):
  - forward > 0: Scale Nav2's cmd_vel linear.x by this value, follow the planned path
  - left > 0: Add left turning (positive angular.z) while scaling forward velocity
  - right > 0: Add right turning (negative angular.z) while scaling forward velocity
  - back > 0: Override with backward movement at this speed (ignores Nav2)
  - All zeros: Stop and cancel the Nav2 goal

The mux automatically resumes the Nav2 goal when any direction becomes non-zero.

Topics:
    Subscribes:
        /nav2/cmd_vel (Twist) - Nav2 velocity commands
        /nav_control (Float32MultiArray) - Directional control [forward, left, right, back]
    Publishes:
        /stretch/cmd_vel (Twist) - Output to robot driver

Actions:
    Client: /navigate_to_pose (NavigateToPose) - To cancel/resend goals
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID

# TODO: nav2 seems to not face along the global planned path (about 20 degrees off) and then just runs forward, unless it actually reaches a >0 cost in the costmap

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # Use reentrant callback group to allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('input_topic', '/nav2/cmd_vel')
        self.declare_parameter('output_topic', '/stretch/cmd_vel')
        self.declare_parameter('control_topic', '/nav_control')
        self.declare_parameter('stow_on_goal', True)
        self.declare_parameter('nav2_linear_scale', 5.0)  # multiplier for Nav2's linear velocity
        self.declare_parameter('nav2_angular_scale', 5.0)  # multiplier for Nav2's angular velocity
        self.declare_parameter('max_linear_acceleration', 1.0)  # m/s^2
        self.declare_parameter('max_linear_braking', 2.0)  # m/s^2
        self.declare_parameter('max_angular_acceleration', 2.0)  # rad/s^2
        self.declare_parameter('max_angular_braking', 4.0)  # rad/s^2

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        self.stow_on_goal = self.get_parameter('stow_on_goal').get_parameter_value().bool_value
        self.nav2_linear_scale = self.get_parameter('nav2_linear_scale').get_parameter_value().double_value
        self.nav2_angular_scale = self.get_parameter('nav2_angular_scale').get_parameter_value().double_value
        self.max_linear_acceleration = self.get_parameter('max_linear_acceleration').get_parameter_value().double_value
        self.max_linear_braking = self.get_parameter('max_linear_braking').get_parameter_value().double_value
        self.max_angular_acceleration = self.get_parameter('max_angular_acceleration').get_parameter_value().double_value
        self.max_angular_braking = self.get_parameter('max_angular_braking').get_parameter_value().double_value

        # Current velocity state for acceleration limiting
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.last_cmd_time = None

        # Control state: [forward, left, right, back]
        self.control = [0.0, 0.0, 0.0, 0.0]
        self.is_stopped = True  # Track if we're in stopped state (all zeros)
        self.last_control_time = None  # None means no control received yet
        self.control_timeout = 0.5  # seconds

        # Latest Nav2 cmd_vel
        self.nav2_cmd_vel = Twist()

        # Saved goal for resume functionality
        self.saved_goal_pose = None
        self.behavior_tree = ''
        self.last_goal_send_time = None  # Track when we last sent a goal
        self.goal_resend_interval = 1.0  # seconds

        # Robot mode tracking
        self.robot_mode = None
        self.required_mode = 'room_navigation'
        self.has_stowed_for_session = False  # Track if we've stowed since entering room_navigation

        # Subscriber for robot mode
        self.mode_sub = self.create_subscription(
            String,
            'mode',
            self.mode_callback,
            10
        )

        # Subscriber for Nav2 cmd_vel
        self.nav2_sub = self.create_subscription(
            Twist,
            input_topic,
            self.nav2_cmd_vel_callback,
            10
        )

        # Subscriber for directional control
        self.control_sub = self.create_subscription(
            Float32MultiArray,
            control_topic,
            self.control_callback,
            10
        )

        # Publisher to stretch_driver
        self.pub = self.create_publisher(Twist, output_topic, 10)

        # Subscribe to navigation goals with matching QoS
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            goal_qos
        )

        # Action client for NavigateToPose
        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Service client to cancel all goals
        self.cancel_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
            callback_group=self.callback_group
        )

        # Service client to stow the robot
        self.stow_client = self.create_client(
            Trigger,
            '/stow_the_robot',
            callback_group=self.callback_group
        )

        # Service clients for camera direction
        self.camera_along_base_client = self.create_client(
            Trigger,
            '/camera_along_base',
            callback_group=self.callback_group
        )
        self.camera_along_base_backward_client = self.create_client(
            Trigger,
            '/camera_along_base_backward',
            callback_group=self.callback_group
        )

        # Camera direction tracking ('forward', 'backward', or None)
        self.current_camera_direction = None

        # Lidar-based slowdown (use BEST_EFFORT QoS to match the lidar publisher)
        lidar_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan_filtered", self.lidar_callback, lidar_qos)
        self.avoid_width = 1
        self.avoid_extent = self.avoid_width / 2.0
        self.avoid_slowdown = 0.3
        self.front_vel_multiplier = 1.0
        self.back_vel_multiplier = 1.0

        # Timer to publish cmd_vel at regular rate
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)  # 20 Hz

        self.get_logger().info(
            f'Cmd Vel Mux started: {input_topic} -> {output_topic}'
        )
        self.get_logger().info(
            f'Control topic: {control_topic} [forward, left, right, back]'
        )

    def mode_callback(self, msg: String):
        """Track the current robot mode."""
        if self.robot_mode != msg.data:
            self.get_logger().info(f'Robot mode changed: {self.robot_mode} -> {msg.data}')
            # Reset stow flag when leaving room_navigation mode
            if (self.robot_mode == self.required_mode and msg.data != self.required_mode) and msg.data != "stowing":
                self.has_stowed_for_session = False
        self.robot_mode = msg.data

    def nav2_cmd_vel_callback(self, msg: Twist):
        """Store the latest Nav2 cmd_vel."""
        self.nav2_cmd_vel = msg

    def lidar_callback(self, msg: LaserScan) -> None:
        """Process lidar data to compute front/back velocity multipliers for obstacle slowdown.

        Lidar frame is rotated 180° from base_link, so:
        - Angle 0 in lidar frame = backward direction in robot frame
        - Angle ±π in lidar frame = forward direction in robot frame

        Handles both -π to π (real robot) and 0 to 2π (simulation) angle ranges
        by normalizing to -π to π.
        """
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # Normalize angles to -π to π range (handles both real robot and simulation)
        # Real robot: already -π to π, no change
        # Simulation: 0 to 2π, angles > π get converted to negative equivalent
        angles = np.where(angles > math.pi, angles - 2*math.pi, angles)

        # Filter out invalid readings (negative values like -1.0 mean no return)
        ranges = np.array(msg.ranges)
        ranges = np.where(ranges <= 0, np.inf, ranges)

        # Front of robot: around ±π (theta < -2.50 or theta > 2.50)
        # We compute lateral offset (y) to filter for obstacles in robot's path
        front_y = [r * math.sin(theta) if (theta < -2.50 or theta > 2.50) else np.inf for r, theta in zip(ranges, angles)]
        front_ranges = [r if abs(y) < self.avoid_extent else np.inf for r, y in zip(ranges, front_y)]

        # Back of robot: around 0 (theta > -0.64 and theta < 0.64)
        back_y = [r * math.sin(theta) if (theta > -0.64 and theta < 0.64) else np.inf for r, theta in zip(ranges, angles)]
        back_ranges = [r if abs(y) < self.avoid_extent else np.inf for r, y in zip(ranges, back_y)]

        min_front = min(front_ranges)
        min_back = min(back_ranges)

        lidar_to_front_of_robot = 0.05
        lidar_to_back_of_robot = 0.23

        # Smoothly interpolate multiplier: 1.0 at avoid_slowdown distance, 0.0 at robot edge
        front_distance = min_front - lidar_to_front_of_robot
        if front_distance < self.avoid_slowdown:
            self.front_vel_multiplier = max(0.0, min(1.0, front_distance / self.avoid_slowdown))
            self.get_logger().info(f"Slowing down, front vel multiplier: {self.front_vel_multiplier:.2f}", throttle_duration_sec=1.0)
        else:
            self.front_vel_multiplier = 1.0

        back_distance = min_back - lidar_to_back_of_robot
        if back_distance < self.avoid_slowdown:
            self.back_vel_multiplier = max(0.0, min(1.0, back_distance / self.avoid_slowdown))
            self.get_logger().info(f"Slowing down, back vel multiplier: {self.back_vel_multiplier:.2f}", throttle_duration_sec=1.0)
        else:
            self.back_vel_multiplier = 1.0

    def control_callback(self, msg: Float32MultiArray):
        """Handle directional control input."""
        if len(msg.data) < 4:
            self.get_logger().warn(f'Expected 4 values [forward, left, right, back], got {len(msg.data)}')
            return

        forward, left, right, back = msg.data[0], msg.data[1], msg.data[2], msg.data[3]

        # Check if all zeros (stop command)
        all_zero = (forward == 0.0 and left == 0.0 and right == 0.0 and back == 0.0)

        if all_zero and not self.is_stopped:
            # Transition to stopped state
            self.get_logger().info('Stopping: cancelling Nav2 goal')
            self.cancel_nav2_goal()
            self.is_stopped = True
        elif not all_zero and self.is_stopped:
            # Transition from stopped to moving
            self.is_stopped = False
            # Only stow once when first entering room_navigation mode
            if self.stow_on_goal and not self.has_stowed_for_session:
                self.stow_robot()
                self.has_stowed_for_session = True
            if self.saved_goal_pose is not None:
                self.get_logger().info('Resuming: resending Nav2 goal')
                self.send_nav2_goal(self.saved_goal_pose)
            else:
                self.get_logger().info('Resuming (no saved goal)')

        self.control = [forward, left, right, back]
        self.last_control_time = self.get_clock().now()

    def publish_cmd_vel(self):
        """Publish cmd_vel based on current control state."""
        # Only process commands if in room_navigation mode
        if self.robot_mode != self.required_mode:
            return

        # Check for timeout - only if we have received a control message before
        if self.last_control_time is not None:
            time_since_last = (self.get_clock().now() - self.last_control_time).nanoseconds / 1e9
            if time_since_last > self.control_timeout:
                if not self.is_stopped:
                    self.get_logger().info('Control timeout: stopping')
                    self.cancel_nav2_goal()
                    self.is_stopped = True
                self.control = [0.0, 0.0, 0.0, 0.0]

        # Resend goal every 1 second while moving to reset progress checker
        if not self.is_stopped and self.saved_goal_pose is not None:
            now = self.get_clock().now()
            if self.last_goal_send_time is None or \
               (now - self.last_goal_send_time).nanoseconds / 1e9 > self.goal_resend_interval:
                self.send_nav2_goal(self.saved_goal_pose)
                self.last_goal_send_time = now

        forward, left, right, back = self.control

        # Update camera direction based on movement
        self.update_camera_direction(forward > 0.0, back > 0.0)

        output = Twist()

        if back > 0.0:
            # Back overrides everything - back value is already the desired velocity
            output.linear.x = -back
            output.angular.z = 0.0
        elif forward > 0.0 or left > 0.0 or right > 0.0:
            # Check if Nav2 is providing velocity commands
            nav2_has_velocity = abs(self.nav2_cmd_vel.linear.x) > 0.001 or abs(self.nav2_cmd_vel.angular.z) > 0.001

            if nav2_has_velocity:
                # Compute user's desired angular velocity
                user_angular = 0.0
                if left > 0.0:
                    user_angular += left
                if right > 0.0:
                    user_angular -= right

                # Determine forward scaling based on user input
                if forward > 0.0:
                    # User pressing forward - use full forward scale
                    forward_scale = forward
                elif left > 0.0 or right > 0.0:
                    # User only pressing left/right - reduce forward motion significantly
                    forward_scale = max(left, right) * 0.3
                else:
                    forward_scale = 0.0

                output.linear.x = self.nav2_cmd_vel.linear.x * forward_scale * self.nav2_linear_scale

                # Get Nav2's angular command (scaled)
                nav2_angular = self.nav2_cmd_vel.angular.z * forward_scale * self.nav2_angular_scale

                # If user and Nav2 want opposite directions, user takes over completely
                if user_angular != 0.0 and nav2_angular != 0.0 and np.sign(user_angular) != np.sign(nav2_angular):
                    output.angular.z = user_angular
                else:
                    # Same direction or no conflict - combine them
                    output.angular.z = nav2_angular + user_angular
            else:
                # No Nav2 goal - use forward value directly as velocity
                if forward > 0.0:
                    output.linear.x = forward
                output.angular.z = 0.0

                # User turning without Nav2
                if left > 0.0:
                    output.angular.z += left
                if right > 0.0:
                    output.angular.z -= right
        else:
            # All zeros - ramp down to stop smoothly
            output.linear.x = 0.0
            output.angular.z = 0.0

        # Apply lidar slowdown before publishing
        if output.linear.x > 0:
            output.linear.x *= self.front_vel_multiplier
        elif output.linear.x < 0:
            output.linear.x *= self.back_vel_multiplier

        # Apply acceleration limiting
        output.linear.x, output.angular.z = self.apply_acceleration_limit(
            output.linear.x, output.angular.z)

        self.pub.publish(output)

    def goal_pose_callback(self, msg: PoseStamped):
        """Track the current navigation goal."""
        self.saved_goal_pose = msg
        self.get_logger().info(f'Saved goal pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        # Send the goal to Nav2 if we're not stopped
        if not self.is_stopped:
            self.get_logger().info('Sending new goal to Nav2')
            self.send_nav2_goal(msg)

        # Only stow once when first entering room_navigation mode
        if self.stow_on_goal and not self.has_stowed_for_session:
            self.stow_robot()
            self.has_stowed_for_session = True

    def stow_robot(self):
        """Call the stow_the_robot service."""
        if not self.stow_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/stow_the_robot service not available')
            return

        self.get_logger().info('Stowing robot before navigation...')
        request = Trigger.Request()
        future = self.stow_client.call_async(request)
        future.add_done_callback(self.stow_done_callback)

    def stow_done_callback(self, future):
        """Handle the result of stowing the robot."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Robot stowed successfully')
            else:
                self.get_logger().warn(f'Failed to stow robot: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Stow service call failed: {e}')

    def cancel_nav2_goal(self):
        """Cancel all current Nav2 navigation goals."""
        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Cancel service not available, cannot cancel goal')
            return

        cancel_request = CancelGoal.Request()
        cancel_request.goal_info.goal_id = UUID()

        self.get_logger().info('Cancelling all Nav2 goals...')
        future = self.cancel_client.call_async(cancel_request)
        future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        """Handle the result of cancelling goals."""
        try:
            result = future.result()
            if len(result.goals_canceling) > 0:
                self.get_logger().info(f'Successfully cancelled {len(result.goals_canceling)} goal(s)')
            else:
                self.get_logger().info('No active goals to cancel')
        except Exception as e:
            self.get_logger().error(f'Failed to cancel goals: {e}')

    def send_nav2_goal(self, pose: PoseStamped):
        """Send a navigation goal to Nav2."""
        if not self.nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose action server not available, cannot send goal')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = self.behavior_tree

        self.get_logger().info(
            f'Sending Nav2 goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Handle response from sending a goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal was rejected')
            return
        self.get_logger().info('Nav2 goal accepted')

        # Get the result to detect when goal is reached
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_goal_result_callback)

    def nav_goal_result_callback(self, future):
        """Handle result when navigation goal completes."""
        try:
            result = future.result()
            status = result.status
            # Status 4 = SUCCEEDED (from action_msgs/GoalStatus)
            if status == 4:
                self.get_logger().info('Navigation goal reached, clearing saved goal')
                self.saved_goal_pose = None
            elif status == 5:  # CANCELED
                self.get_logger().info('Navigation goal was canceled')
            elif status == 6:  # ABORTED
                self.get_logger().warn('Navigation goal aborted')
        except Exception as e:
            self.get_logger().error(f'Failed to get navigation result: {e}')

    def apply_acceleration_limit(self, target_linear: float, target_angular: float) -> tuple:
        """Apply acceleration limiting to velocity commands.

        Uses asymmetric limits - slower acceleration, faster braking.
        """
        now = self.get_clock().now()
        if self.last_cmd_time is None:
            dt = 0.05  # Assume 20Hz on first call
        else:
            dt = (now - self.last_cmd_time).nanoseconds / 1e9
        self.last_cmd_time = now

        # Apply acceleration limit to linear velocity
        self.current_linear_vel = self._limit_velocity(
            self.current_linear_vel, target_linear, dt,
            self.max_linear_acceleration, self.max_linear_braking)

        # Apply acceleration limit to angular velocity
        self.current_angular_vel = self._limit_velocity(
            self.current_angular_vel, target_angular, dt,
            self.max_angular_acceleration, self.max_angular_braking)

        return self.current_linear_vel, self.current_angular_vel

    def _limit_velocity(self, current: float, target: float, dt: float,
                        max_accel: float, max_brake: float) -> float:
        """Apply acceleration/braking limit to a single velocity component."""
        if dt <= 0.0:
            return target

        delta = target - current
        if abs(delta) < 0.001:
            return target

        # Determine if we're speeding up or slowing down
        same_direction = (np.sign(current) == np.sign(target)) or (current == 0.0) or (target == 0.0)
        speeding_up = (abs(target) > abs(current)) and same_direction

        # Use acceleration limit when speeding up, braking limit when slowing down
        max_change = (max_accel if speeding_up else max_brake) * dt

        if abs(delta) <= max_change:
            return target
        else:
            return current + np.sign(delta) * max_change

    def update_camera_direction(self, is_forward: bool, is_backward: bool):
        """Update camera direction based on movement direction.

        Uses /camera_along_base service for forward direction.
        Uses /camera_along_base_backward service for backward direction.
        """
        if is_forward and self.current_camera_direction != 'forward':
            # Point camera forward
            if self.camera_along_base_client.wait_for_service(timeout_sec=0.1):
                request = Trigger.Request()
                future = self.camera_along_base_client.call_async(request)
                future.add_done_callback(self.camera_direction_callback)
                self.current_camera_direction = 'forward'
                self.get_logger().info('Switching camera to forward direction', throttle_duration_sec=1.0)
        elif is_backward and self.current_camera_direction != 'backward':
            # Point camera backward
            if self.camera_along_base_backward_client.wait_for_service(timeout_sec=0.1):
                request = Trigger.Request()
                future = self.camera_along_base_backward_client.call_async(request)
                future.add_done_callback(self.camera_direction_callback)
                self.current_camera_direction = 'backward'
                self.get_logger().info('Switching camera to backward direction', throttle_duration_sec=1.0)

    def camera_direction_callback(self, future):
        """Handle result of camera direction service call."""
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn(f'Camera direction change failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Camera direction service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()

    # Use multi-threaded executor for concurrent callbacks
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
