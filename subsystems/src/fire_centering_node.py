#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class FireCenteringNode(Node):
    def __init__(self):
        super().__init__('fire_centering_node')
        
        # Initialize CV bridge for debug visualization
        self.bridge = CvBridge()
        
        # Camera parameters (from IR camera config)
        self.camera_width = 640
        self.camera_height = 480
        self.camera_center_x = self.camera_width // 2
        self.camera_center_y = self.camera_height // 2
        
        # Control parameters
        self.declare_parameter('enabled', False)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('kp_linear', 0.5)  # Proportional gain for linear movement
        self.declare_parameter('kp_angular', 1.0)  # Proportional gain for angular movement
        self.declare_parameter('tolerance_pixels', 20)  # Pixel tolerance for centering
        self.declare_parameter('hover_z_velocity', 0.1)  # Hover velocity when no fire detected
        self.declare_parameter('fire_timeout', 2.0)  # Timeout for fire detection in seconds
        self.declare_parameter('target_height', 7.5)  # Target height in meters
        self.declare_parameter('height_tolerance', 0.5)  # Height tolerance in meters
        self.declare_parameter('kp_height', 2.0)  # Proportional gain for height control
        self.declare_parameter('kp_xy_fast', 4.0)  # Fast proportional gain for x,y centering
        
        # Read parameters
        self.enabled = self.get_parameter('enabled').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.tolerance_pixels = self.get_parameter('tolerance_pixels').value
        self.hover_z_velocity = self.get_parameter('hover_z_velocity').value
        self.fire_timeout = self.get_parameter('fire_timeout').value
        self.target_height = self.get_parameter('target_height').value
        self.height_tolerance = self.get_parameter('height_tolerance').value
        self.kp_height = self.get_parameter('kp_height').value
        self.kp_xy_fast = self.get_parameter('kp_xy_fast').value
        
        # State variables
        self.current_fire_location = None
        self.last_fire_time = None
        self.drone_position = None
        self.drone_orientation = None
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.debug_image_pub = self.create_publisher(Image, '/drone/fire_centering/debug_image', 10)
        
        # Subscribers
        self.fire_location_sub = self.create_subscription(
            PointStamped,
            '/drone/fire_detection/location',
            self.fire_location_callback,
            10
        )
        
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/drone/odometry',
            self.odometry_callback,
            10
        )
        
        # Timer for control loop (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Timer for status updates (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Fire Centering Node started')
        self.get_logger().info(f'Enabled: {self.enabled}')
        self.get_logger().info(f'Camera center: ({self.camera_center_x}, {self.camera_center_y})')
        self.get_logger().info(f'Tolerance: {self.tolerance_pixels} pixels')
        
    def fire_location_callback(self, msg):
        """Handle fire detection messages"""
        self.current_fire_location = (msg.point.x, msg.point.y, msg.point.z)
        self.last_fire_time = self.get_clock().now()
        
        self.get_logger().debug(f'Fire detected at pixel ({msg.point.x:.1f}, {msg.point.y:.1f}) with area {msg.point.z:.1f}')
        
    def odometry_callback(self, msg):
        """Handle drone odometry messages"""
        self.drone_position = msg.pose.pose.position
        self.drone_orientation = msg.pose.pose.orientation
        
    def control_loop(self):
        """Main control loop for fire centering"""
        if not self.enabled:
            return
            
        current_time = self.get_clock().now()
        
        # Check if we have recent fire detection
        fire_detected = False
        if (self.current_fire_location is not None and 
            self.last_fire_time is not None):
            time_since_fire = (current_time - self.last_fire_time).nanoseconds / 1e9
            if time_since_fire < self.fire_timeout:
                fire_detected = True
        
        cmd_vel = Twist()
        
        # Get current drone height
        current_height = 0.0
        if self.drone_position is not None:
            current_height = self.drone_position.z
        
        # Check if we're at target height
        height_error = self.target_height - current_height
        at_target_height = abs(height_error) < self.height_tolerance
        
        if fire_detected:
            # Calculate error from center
            fire_x, fire_y, fire_area = self.current_fire_location
            error_x = fire_x - self.camera_center_x
            error_y = fire_y - self.camera_center_y
            
            if not at_target_height:
                # Phase 1: Fly to target height first
                linear_x = 0.0  # No forward/backward movement
                linear_y = 0.0  # No left/right movement
                linear_z = height_error * self.kp_height  # Height control
                angular_z = 0.0  # No rotation
                
                self.get_logger().info(f'Flying to target height: current={current_height:.1f}m, target={self.target_height:.1f}m, error={height_error:.1f}m')
                
            else:
                # Phase 2: At target height - do fast horizontal centering only
                # Note: Camera coordinates: x=right, y=down
                # Drone coordinates: x=forward, y=left, z=up
                # So we map: camera_x -> drone_y, camera_y -> drone_z
                
                # Linear velocities (x,y movement for centering, keep z fixed)
                linear_x = -error_y * self.kp_xy_fast / 100.0  # Fast vertical centering (forward/backward)
                linear_y = -error_x * self.kp_xy_fast / 100.0  # Fast horizontal centering (left/right)
                linear_z = 0.0  # Keep height fixed at target
                
                # Angular velocity for fine-tuning
                angular_z = -error_x * self.kp_angular / 100.0 * 0.1  # Small angular correction
                
                # Apply limits
                linear_x = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_x))
                linear_y = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_y))
                angular_z = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_z))
                
                # Check if we're close enough to center
                distance_from_center = math.sqrt(error_x**2 + error_y**2)
                if distance_from_center < self.tolerance_pixels:
                    self.get_logger().info('Fire centered!')
                    # Still apply small corrections to maintain position
                    linear_x *= 0.1
                    linear_y *= 0.1
                    angular_z *= 0.1
                else:
                    self.get_logger().info(f'Fast centering fire: error=({error_x:.1f}, {error_y:.1f}), distance={distance_from_center:.1f}, cmd=({linear_x:.3f}, {linear_y:.3f}, {linear_z:.3f}, {angular_z:.3f})')
            
            cmd_vel.linear.x = linear_x
            cmd_vel.linear.y = linear_y
            cmd_vel.linear.z = linear_z
            cmd_vel.angular.z = angular_z
                
        else:
            # No fire detected - maintain target height or hover
            if not at_target_height:
                # Fly to target height even without fire
                linear_x = 0.0
                linear_y = 0.0
                linear_z = height_error * self.kp_height
                angular_z = 0.0
                self.get_logger().info(f'No fire - flying to target height: current={current_height:.1f}m, target={self.target_height:.1f}m')
            else:
                # At target height, hover in place
                linear_x = 0.0
                linear_y = 0.0
                linear_z = self.hover_z_velocity  # Small upward velocity to maintain altitude
                angular_z = 0.0
                self.get_logger().debug('No fire detected - hovering at target height')
            
            cmd_vel.linear.x = linear_x
            cmd_vel.linear.y = linear_y
            cmd_vel.linear.z = linear_z
            cmd_vel.angular.z = angular_z
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Create and publish debug image
        self.publish_debug_image(fire_detected)
        
    def publish_debug_image(self, fire_detected):
        """Create and publish debug visualization"""
        # Create a blank image
        debug_image = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
        
        # Draw camera center crosshair
        cv2.drawMarker(debug_image, (self.camera_center_x, self.camera_center_y), 
                      (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        
        # Draw tolerance circle
        cv2.circle(debug_image, (self.camera_center_x, self.camera_center_y), 
                  self.tolerance_pixels, (0, 255, 255), 1)
        
        if fire_detected and self.current_fire_location is not None:
            fire_x, fire_y, fire_area = self.current_fire_location
            fire_pixel = (int(fire_x), int(fire_y))
            
            # Draw fire location
            cv2.circle(debug_image, fire_pixel, 10, (0, 0, 255), -1)
            cv2.drawMarker(debug_image, fire_pixel, (255, 255, 255), cv2.MARKER_CROSS, 15, 2)
            
            # Draw line from center to fire
            cv2.line(debug_image, (self.camera_center_x, self.camera_center_y), 
                    fire_pixel, (255, 0, 0), 2)
            
            # Add text
            error_x = fire_x - self.camera_center_x
            error_y = fire_y - self.camera_center_y
            distance = math.sqrt(error_x**2 + error_y**2)
            
            cv2.putText(debug_image, f'Fire: ({fire_x:.0f}, {fire_y:.0f})', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_image, f'Error: ({error_x:.0f}, {error_y:.0f})', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_image, f'Distance: {distance:.0f}px', 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            if distance < self.tolerance_pixels:
                cv2.putText(debug_image, 'CENTERED!', 
                           (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(debug_image, 'No fire detected', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Add status text
        status_text = "ENABLED" if self.enabled else "DISABLED"
        cv2.putText(debug_image, f'Fire Centering: {status_text}', 
                   (10, self.camera_height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Convert and publish
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = "ir_camera_link"
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing debug image: {str(e)}')
            
    def publish_status(self):
        """Publish status information"""
        if self.enabled:
            if self.current_fire_location is not None:
                fire_x, fire_y, fire_area = self.current_fire_location
                error_x = fire_x - self.camera_center_x
                error_y = fire_y - self.camera_center_y
                distance = math.sqrt(error_x**2 + error_y**2)
                
                if distance < self.tolerance_pixels:
                    self.get_logger().info('Status: Fire centered')
                else:
                    self.get_logger().info(f'Status: Centering fire (error: {distance:.1f}px)')
            else:
                self.get_logger().info('Status: No fire detected - hovering')
        else:
            self.get_logger().info('Status: Fire centering disabled')

def main(args=None):
    rclpy.init(args=args)
    
    fire_centering_node = FireCenteringNode()
    
    try:
        rclpy.spin(fire_centering_node)
    except KeyboardInterrupt:
        fire_centering_node.get_logger().info('Fire centering node shutting down')
    
    fire_centering_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
