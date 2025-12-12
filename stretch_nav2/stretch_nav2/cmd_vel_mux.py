#!/usr/bin/env python3

"""
Cmd Vel Mux Node

This node sits between Nav2's controller_server and the stretch_driver,
allowing you to pause/resume Nav2 velocity commands via a service.

Topics:
    Subscribes: /nav2/cmd_vel (Twist)
    Publishes:  /stretch/cmd_vel (Twist)

Services:
    /cmd_vel_mux/enable (SetBool) - Enable/disable forwarding of Nav2 commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # Declare parameters
        self.declare_parameter('input_topic', '/nav2/cmd_vel')
        self.declare_parameter('output_topic', '/stretch/cmd_vel')
        self.declare_parameter('enabled', True)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value

        # Subscriber for Nav2 cmd_vel
        self.sub = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_vel_callback,
            10
        )

        # Publisher to stretch_driver
        self.pub = self.create_publisher(Twist, output_topic, 10)

        # Service to enable/disable forwarding
        self.srv = self.create_service(
            SetBool,
            '~/enable',
            self.enable_callback
        )

        self.get_logger().info(
            f'Cmd Vel Mux started: {input_topic} -> {output_topic} (enabled={self.enabled})'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Forward cmd_vel if enabled, otherwise drop the message."""
        if self.enabled:
            self.pub.publish(msg)

    def enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Enable or disable cmd_vel forwarding."""
        self.enabled = request.data
        response.success = True
        response.message = f'Nav2 cmd_vel forwarding {"enabled" if self.enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
