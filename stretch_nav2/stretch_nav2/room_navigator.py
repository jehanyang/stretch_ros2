#! /usr/bin/env python3

"""
Room Navigator

A simple node that provides services to navigate to predefined room locations.

Services:
    /go_to_kitchen (Trigger) - Navigate to the kitchen
    /go_to_bedroom (Trigger) - Navigate to the bedroom

The node uses BasicNavigator which publishes to /goal_pose, allowing
cmd_vel_mux to track the current goal for pause/resume functionality.
"""

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading


class RoomNavigator(Node):
    def __init__(self):
        super().__init__('room_navigator')

        # Define room locations with orientations (quaternion z, w components)
        # TODO: Update these coordinates based on your map
        # Orientation: (z=0, w=1) = face +X, (z=0.707, w=0.707) = face +Y, (z=-0.707, w=0.707) = face -Y
        self.rooms = {
            'kitchen': {'x': 0.73, 'y': -1.40, 'qz': -0.707, 'qw': 0.707},   # Face -Y
            'bedroom': {'x': -0.80, 'y': 6.933, 'qz': 0.707, 'qw': 0.707},  # Face +Y
        }

        # Use reentrant callback group for concurrent service calls
        self.callback_group = ReentrantCallbackGroup()

        # Track navigation state
        self.is_navigating = False
        self.nav_lock = threading.Lock()
        self.current_goal_handle = None
        self.navigation_result = None
        self.navigation_complete = threading.Event()

        # Action client for NavigateToPose
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Publisher for goal_pose (for cmd_vel_mux to track)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Create services for each room
        self.kitchen_srv = self.create_service(
            Trigger,
            'go_to_kitchen',
            self.go_to_kitchen_callback,
            callback_group=self.callback_group
        )

        self.bedroom_srv = self.create_service(
            Trigger,
            'go_to_bedroom',
            self.go_to_bedroom_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Room Navigator started')
        self.get_logger().info(f'  Kitchen: ({self.rooms["kitchen"]["x"]}, {self.rooms["kitchen"]["y"]})')
        self.get_logger().info(f'  Bedroom: ({self.rooms["bedroom"]["x"]}, {self.rooms["bedroom"]["y"]})')

    def create_pose(self, x: float, y: float, qz: float = 0.0, qw: float = 1.0) -> PoseStamped:
        """Create a PoseStamped message for the given coordinates and orientation."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected')
            self.navigation_result = 'rejected'
        else:
            self.get_logger().info('Goal accepted')
            self.current_goal_handle = goal_handle
            self.navigation_result = 'accepted'

        self.navigation_complete.set()

    def navigate_to_room(self, room_name: str) -> tuple:
        """Navigate to a room. Returns (success, message)."""
        with self.nav_lock:
            if self.is_navigating:
                return False, 'Already navigating to another location'
            self.is_navigating = True

        try:
            if room_name not in self.rooms:
                return False, f'Unknown room: {room_name}'

            # Wait for action server
            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                return False, 'NavigateToPose action server not available'

            room = self.rooms[room_name]
            pose = self.create_pose(room['x'], room['y'], room.get('qz', 0.0), room.get('qw', 1.0))

            # Publish goal pose for cmd_vel_mux to track
            self.goal_pose_pub.publish(pose)

            self.get_logger().info(f'Navigating to {room_name} at ({room["x"]}, {room["y"]})')

            # Reset state
            self.navigation_complete.clear()
            self.navigation_result = None

            # Send the goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)

            # Wait for goal to be accepted or rejected
            self.navigation_complete.wait()

            if self.navigation_result == 'accepted':
                return True, f'Navigation to {room_name} started'
            else:
                return False, f'Goal to {room_name} was rejected'

        finally:
            with self.nav_lock:
                self.is_navigating = False

    def go_to_kitchen_callback(self, request, response):
        """Service callback to navigate to the kitchen."""
        success, message = self.navigate_to_room('kitchen')
        response.success = success
        response.message = message
        self.get_logger().info(message)
        return response

    def go_to_bedroom_callback(self, request, response):
        """Service callback to navigate to the bedroom."""
        success, message = self.navigate_to_room('bedroom')
        response.success = success
        response.message = message
        self.get_logger().info(message)
        return response


def main(args=None):
    rclpy.init(args=args)

    room_navigator = RoomNavigator()

    # Use multi-threaded executor to handle concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(room_navigator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        room_navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
