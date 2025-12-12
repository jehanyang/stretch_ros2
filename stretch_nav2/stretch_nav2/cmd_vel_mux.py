#!/usr/bin/env python3

"""
Cmd Vel Mux Node

This node sits between Nav2's controller_server and the stretch_driver,
allowing you to pause/resume Nav2 velocity commands via a service.

When disabled (paused):
  - Stops forwarding cmd_vel messages
  - Cancels the current Nav2 goal
  - Saves the goal pose for later

When enabled (resumed):
  - Resumes forwarding cmd_vel messages
  - Re-sends the saved goal pose to Nav2

When a new goal is received:
  - Calls /stow_the_robot service to stow the arm

Topics:
    Subscribes: /nav2/cmd_vel (Twist)
    Publishes:  /stretch/cmd_vel (Twist)

Services:
    /cmd_vel_mux/enable (SetBool) - Enable/disable forwarding of Nav2 commands

Actions:
    Client: /navigate_to_pose (NavigateToPose) - To cancel/resend goals
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import SetBool, Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatusArray
from unique_identifier_msgs.msg import UUID


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # Use reentrant callback group to allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('input_topic', '/nav2/cmd_vel')
        self.declare_parameter('output_topic', '/stretch/cmd_vel')
        self.declare_parameter('enabled', True)
        self.declare_parameter('stow_on_goal', True)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        self.stow_on_goal = self.get_parameter('stow_on_goal').get_parameter_value().bool_value

        # Saved goal for resume functionality
        self.saved_goal_pose = None
        self.behavior_tree = ''

        # Subscriber for Nav2 cmd_vel
        self.sub = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_vel_callback,
            10
        )

        # Publisher to stretch_driver
        self.pub = self.create_publisher(Twist, output_topic, 10)

        # Subscribe to navigation goals from RViz with matching QoS
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

        # Also subscribe to the action status to detect active goals
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.goal_status_callback,
            10
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

        # Service to enable/disable forwarding
        self.srv = self.create_service(
            SetBool,
            '~/enable',
            self.enable_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            f'Cmd Vel Mux started: {input_topic} -> {output_topic} (enabled={self.enabled})'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Forward cmd_vel if enabled, otherwise drop the message."""
        if self.enabled:
            self.pub.publish(msg)

    def goal_pose_callback(self, msg: PoseStamped):
        """Track the current navigation goal from RViz or other sources."""
        self.saved_goal_pose = msg
        self.get_logger().info(f'Saved goal pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        # Stow the robot when a new goal is received
        if self.stow_on_goal:
            self.stow_robot()

    def goal_status_callback(self, msg: GoalStatusArray):
        """Track goal status to know if there's an active goal."""
        # This is informational - we could use this to track active goals
        pass

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

    def enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Enable or disable cmd_vel forwarding."""
        was_enabled = self.enabled
        self.enabled = request.data

        if was_enabled and not self.enabled:
            # Transitioning from enabled to disabled - cancel current goal
            self.cancel_nav2_goal()
            response.message = 'Nav2 cmd_vel forwarding disabled, goal cancelled'
        elif not was_enabled and self.enabled:
            # Transitioning from disabled to enabled - resend saved goal
            if self.saved_goal_pose is not None:
                self.send_nav2_goal(self.saved_goal_pose)
                response.message = 'Nav2 cmd_vel forwarding enabled, goal resent'
            else:
                response.message = 'Nav2 cmd_vel forwarding enabled (no saved goal to resend)'
        else:
            response.message = f'Nav2 cmd_vel forwarding already {"enabled" if self.enabled else "disabled"}'

        response.success = True
        self.get_logger().info(response.message)
        return response

    def cancel_nav2_goal(self):
        """Cancel all current Nav2 navigation goals."""
        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Cancel service not available, cannot cancel goal')
            return

        # Cancel all goals by sending an empty goal_info (cancels all)
        cancel_request = CancelGoal.Request()
        # Empty UUID and zero timestamp = cancel all goals
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
