#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class DualRobotPositionLogger(Node):
    def __init__(self):
        super().__init__('dual_robot_position_logger')\
        
        # Declare and get parameters for topic names
        self.declare_parameter('robot1_topic', '/husky/odometry')
        self.declare_parameter('robot2_topic', '/drone/odometry')
        
        robot1_topic = self.get_parameter('robot1_topic').get_parameter_value().string_value
        robot2_topic = self.get_parameter('robot2_topic').get_parameter_value().string_value
        
        # Store latest positions
        self.robot1_position = None
        self.robot2_position = None
        
        # Create timer to print every 4 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Create subscriptions for both robots
        self.subscription_robot1 = self.create_subscription(
            Odometry,
            robot1_topic,
            self.robot1_callback,
            10)
        
        self.subscription_robot2 = self.create_subscription(
            Odometry,
            robot2_topic,
            self.robot2_callback,
            10)
        
        self.get_logger().info(f'Subscribed to Robot 1: {robot1_topic}')
        self.get_logger().info(f'Subscribed to Robot 2: {robot2_topic}')
        
    def robot1_callback(self, msg):
        # Store the latest position
        self.robot1_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
    
    def robot2_callback(self, msg):
        # Store the latest position
        self.robot2_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
    
    def timer_callback(self):
        # Print Robot 1 position if available
        if self.robot1_position is not None:
            pos = self.robot1_position
            self.get_logger().info(
                f'Husky Position - X: {pos["x"]:.3f}, Y: {pos["y"]:.3f}, Z: {pos["z"]:.3f}'
            )
        else:
            self.get_logger().warn('Robot 1: No position data received yet')
        
        # Print Robot 2 position if available
        if self.robot2_position is not None:
            pos = self.robot2_position
            self.get_logger().info(
                f'Drone Position - X: {pos["x"]:.3f}, Y: {pos["y"]:.3f}, Z: {pos["z"]:.3f}'
            )
        else:
            self.get_logger().warn('Robot 2: No position data received yet')

def main(args=None):
    rclpy.init(args=args)
    
    dual_robot_position_logger = DualRobotPositionLogger()
    
    try:
        rclpy.spin(dual_robot_position_logger)
    except KeyboardInterrupt:
        pass
    
    dual_robot_position_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()