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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
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
        self.declare_parameter('backup_speed', 0.1)  # m/s for backward movement
        self.declare_parameter('turn_speed', 0.5)    # rad/s for left/right turning
        self.declare_parameter('nav2_angular_scale', 2.0)  # multiplier for Nav2's angular velocity

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        self.stow_on_goal = self.get_parameter('stow_on_goal').get_parameter_value().bool_value
        self.backup_speed = self.get_parameter('backup_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.nav2_angular_scale = self.get_parameter('nav2_angular_scale').get_parameter_value().double_value

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
        self.robot_mode = msg.data

    def nav2_cmd_vel_callback(self, msg: Twist):
        """Store the latest Nav2 cmd_vel."""
        self.nav2_cmd_vel = msg

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
            # Transition from stopped to moving - resend goal
            self.is_stopped = False
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

        output = Twist()

        if back > 0.0:
            # Back overrides everything - just go backward
            output.linear.x = -self.backup_speed * back
            output.angular.z = 0.0
        elif forward > 0.0 or left > 0.0 or right > 0.0:
            # Scale Nav2's linear velocity by forward (or by turn magnitude)
            forward_scale = forward if forward > 0.0 else max(0.3, 1.0 - (left + right) * 0.5)
            output.linear.x = self.nav2_cmd_vel.linear.x * forward_scale

            # Start with Nav2's angular velocity scaled (with additional nav2_angular_scale multiplier)
            output.angular.z = self.nav2_cmd_vel.angular.z * forward_scale * self.nav2_angular_scale

            # Add turning from left/right
            if left > 0.0:
                output.angular.z += self.turn_speed * left
            if right > 0.0:
                output.angular.z -= self.turn_speed * right
        else:
            # All zeros - don't publish anything (robot should stop from Nav2 being cancelled)
            return

        self.pub.publish(output)

    def goal_pose_callback(self, msg: PoseStamped):
        """Track the current navigation goal."""
        self.saved_goal_pose = msg
        self.get_logger().info(f'Saved goal pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        # Send the goal to Nav2 if we're not stopped
        if not self.is_stopped:
            self.get_logger().info('Sending new goal to Nav2')
            self.send_nav2_goal(msg)

        # Stow the robot when a new goal is received
        if self.stow_on_goal:
            self.stow_robot()

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
