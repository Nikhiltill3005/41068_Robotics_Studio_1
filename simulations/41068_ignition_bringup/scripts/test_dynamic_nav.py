#!/usr/bin/env python3
"""
Test Dynamic Navigation Script

This script helps test the dynamic navigation capabilities by:
1. Moving the robot to build an initial map
2. Setting navigation goals that require obstacle avoidance
3. Demonstrating mapping while navigating

Usage:
    ros2 run 41068_ignition_bringup test_dynamic_nav.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import time


class DynamicNavTester(Node):
    def __init__(self):
        super().__init__('dynamic_nav_tester')
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher for manual movement (to build initial map)
        self.cmd_vel_pub = self.create_publisher(Twist, '/husky/cmd_vel', 10)
        
        # Subscription to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/husky/odometry',
            self.odom_callback,
            10
        )
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.get_logger().info('Dynamic Navigation Tester initialized')
        
    def odom_callback(self, msg):
        """Update robot position"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def move_robot_manual(self, linear_x, angular_z, duration):
        """Move robot manually to build initial map"""
        self.get_logger().info(f'Moving robot: linear={linear_x}, angular={angular_z} for {duration}s')
        
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('Manual movement completed')
    
    def send_nav_goal(self, x, y, yaw=0.0):
        """Send navigation goal"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header = Header()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f'Sending navigation goal: ({x:.2f}, {y:.2f}, {yaw:.2f})')
        
        future = self.nav_client.send_goal_async(goal_msg)
        return True
    
    def build_initial_map(self):
        """Build initial map by moving robot around"""
        self.get_logger().info('Building initial map...')
        
        # Move forward
        self.move_robot_manual(0.5, 0.0, 3.0)
        time.sleep(1.0)
        
        # Turn right
        self.move_robot_manual(0.0, -0.5, 2.0)
        time.sleep(1.0)
        
        # Move forward
        self.move_robot_manual(0.5, 0.0, 3.0)
        time.sleep(1.0)
        
        # Turn left
        self.move_robot_manual(0.0, 0.5, 2.0)
        time.sleep(1.0)
        
        # Move forward
        self.move_robot_manual(0.5, 0.0, 2.0)
        time.sleep(1.0)
        
        self.get_logger().info('Initial map building completed')
    
    def test_dynamic_navigation(self):
        """Test dynamic navigation with obstacle avoidance"""
        self.get_logger().info('Starting dynamic navigation test...')
        
        # Wait a bit for systems to initialize
        time.sleep(2.0)
        
        # Step 1: Build initial map
        self.build_initial_map()
        
        # Step 2: Test navigation goals
        goals = [
            (2.0, 0.0, 0.0),    # Forward
            (2.0, 2.0, 1.57),   # Right turn
            (0.0, 2.0, 3.14),   # Backward
            (0.0, 0.0, 0.0),    # Return to origin
        ]
        
        for i, (x, y, yaw) in enumerate(goals):
            self.get_logger().info(f'Navigation test {i+1}/4: Going to ({x}, {y})')
            
            if self.send_nav_goal(x, y, yaw):
                # Wait for navigation to complete
                time.sleep(10.0)  # Adjust based on distance
            
            time.sleep(2.0)  # Brief pause between goals
        
        self.get_logger().info('Dynamic navigation test completed!')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = DynamicNavTester()
        
        # Start the test
        tester.test_dynamic_navigation()
        
        # Keep node alive
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'tester' in locals():
            tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

