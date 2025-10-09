#!/usr/bin/env python3
"""
Command Velocity Relay Node

This node relays cmd_vel messages from Nav2 (/cmd_vel) to the Husky robot (/husky/cmd_vel).
This is needed because Nav2 publishes to /cmd_vel but the Husky simulation expects /husky/cmd_vel.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscribe to Nav2's cmd_vel output
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish to Husky's expected cmd_vel topic
        self.publisher = self.create_publisher(
            Twist,
            '/husky/cmd_vel',
            10
        )
        
        self.get_logger().info('CMD Vel Relay: /cmd_vel -> /husky/cmd_vel')
    
    def cmd_vel_callback(self, msg):
        """Relay cmd_vel messages from Nav2 to Husky"""
        # Simply forward the message to the Husky namespace
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        relay_node = CmdVelRelay()
        rclpy.spin(relay_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'relay_node' in locals():
            relay_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

