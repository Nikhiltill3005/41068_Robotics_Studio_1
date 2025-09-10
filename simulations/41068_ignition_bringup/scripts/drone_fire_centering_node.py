#!/usr/bin/env python3

"""
Drone Fire Centering Node

This node subscribes to fire detection results and automatically controls the drone
to center the detected fire in the camera view.

Subscribed Topics:
    /drone/fire_detection/location (geometry_msgs/PointStamped): Fire pixel coordinates
    /drone/ir_camera/camera_info (sensor_msgs/CameraInfo): Camera parameters

Published Topics:
    /drone/cmd_vel (geometry_msgs/Twist): Drone movement commands

Parameters:
    max_linear_speed (double, default: 1.0): Maximum linear velocity in m/s
    max_angular_speed (double, default: 1.0): Maximum angular velocity in rad/s  
    deadzone_pixels (int, default: 20): Pixel radius considered as "centered"

Usage:
    # Launch both fire detection and centering:
    ros2 launch 41068_ignition_bringup 41068_fire_centering.launch.py
    
    # Or run individually:
    ros2 run 41068_ignition_bringup drone_fire_centering_node.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import CameraInfo
import math

class DroneFireCenteringNode(Node):
    def __init__(self):
        super().__init__('drone_fire_centering_node')
        
        # Subscribers
        self.fire_location_sub = self.create_subscription(
            PointStamped,
            '/drone/fire_detection/location',
            self.fire_location_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/drone/ir_camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publisher for drone control
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )
        
        # Camera parameters
        self.camera_width = 640  # Default values, will be updated from camera_info
        self.camera_height = 480
        self.camera_center_x = self.camera_width / 2
        self.camera_center_y = self.camera_height / 2
        
        # Declare parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0) 
        self.declare_parameter('deadzone_pixels', 20)
        
        # Control parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.deadzone_pixels = self.get_parameter('deadzone_pixels').get_parameter_value().integer_value
        
        # PID-like control gains
        self.kp_linear = 0.002  # Proportional gain for linear movement
        self.kp_angular = 0.003  # Proportional gain for angular movement
        
        # State tracking
        self.last_fire_time = None
        self.control_timeout = 2.0  # Stop controlling after this many seconds without fire detection
        
        # Timer for stopping motion when no fire detected
        self.control_timer = self.create_timer(0.1, self.control_timeout_check)
        
        self.get_logger().info('Drone Fire Centering Node started')
        self.get_logger().info(f'Camera resolution: {self.camera_width}x{self.camera_height}')
        self.get_logger().info(f'Control gains - Linear: {self.kp_linear}, Angular: {self.kp_angular}')

    def camera_info_callback(self, msg):
        """Update camera parameters from camera info"""
        self.camera_width = msg.width
        self.camera_height = msg.height
        self.camera_center_x = self.camera_width / 2
        self.camera_center_y = self.camera_height / 2
        
        self.get_logger().info(f'Updated camera resolution: {self.camera_width}x{self.camera_height}')

    def fire_location_callback(self, msg):
        """Process fire detection and generate control commands"""
        self.last_fire_time = self.get_clock().now()
        
        # Extract fire location in pixels
        fire_x = msg.point.x
        fire_y = msg.point.y
        fire_area = msg.point.z
        
        # Calculate error from image center
        error_x = fire_x - self.camera_center_x  # Positive = fire is to the right
        error_y = fire_y - self.camera_center_y  # Positive = fire is below center
        
        # Calculate distance from center
        distance_from_center = math.sqrt(error_x**2 + error_y**2)
        
        self.get_logger().info(f'Fire detected at ({fire_x:.1f}, {fire_y:.1f}), '
                              f'error: ({error_x:.1f}, {error_y:.1f}), '
                              f'distance: {distance_from_center:.1f}px, area: {fire_area:.1f}px')
        
        # Create control command
        twist = Twist()
        
        # Only control if fire is outside deadzone
        if distance_from_center > self.deadzone_pixels:
            # Since the IR camera now faces down, we need to map pixel errors to drone movement:
            # - error_x (horizontal in image) maps to drone's lateral movement (twist.linear.y)
            # - error_y (vertical in image) maps to drone's forward/backward movement (twist.linear.x)
            
            # Proportional control for lateral movement (left/right)
            # Positive error_x means fire is to the right, so drone should move right (negative linear.y)
            twist.linear.y = -self.kp_linear * error_x
            
            # Proportional control for forward/backward movement
            # Positive error_y means fire is below center, so drone should move forward (positive linear.x)
            twist.linear.x = self.kp_linear * error_y
            
            # Limit speeds
            twist.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, twist.linear.x))
            twist.linear.y = max(-self.max_linear_speed, min(self.max_linear_speed, twist.linear.y))
            
            # Optional: Add altitude adjustment based on fire size
            # If fire appears small, move closer (down), if large, move away (up)
            target_area = 2000  # Target fire area in pixels
            area_error = target_area - fire_area
            twist.linear.z = self.kp_linear * area_error * 0.0005  # Very gentle altitude adjustment
            twist.linear.z = max(-0.3, min(0.3, twist.linear.z))  # Limit altitude speed
            
            self.get_logger().info(f'Control command - X: {twist.linear.x:.3f}, '
                                  f'Y: {twist.linear.y:.3f}, Z: {twist.linear.z:.3f}')
        else:
            # Fire is centered, stop movement
            self.get_logger().info('Fire is centered - stopping movement')
        
        # Publish control command
        self.cmd_vel_pub.publish(twist)

    def control_timeout_check(self):
        """Stop drone if no fire detected for timeout period"""
        if self.last_fire_time is not None:
            current_time = self.get_clock().now()
            time_since_fire = (current_time - self.last_fire_time).nanoseconds / 1e9
            
            if time_since_fire > self.control_timeout:
                # Send stop command
                stop_twist = Twist()
                self.cmd_vel_pub.publish(stop_twist)
                
                # Log timeout (throttled to avoid spam)
                self.get_logger().info('No fire detected - stopping drone movement', 
                                     throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    
    drone_centering_node = DroneFireCenteringNode()
    
    try:
        rclpy.spin(drone_centering_node)
    except KeyboardInterrupt:
        drone_centering_node.get_logger().info('Drone Fire Centering Node shutting down')
    
    drone_centering_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
