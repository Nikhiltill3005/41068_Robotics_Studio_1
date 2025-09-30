#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class FireScanNode(Node):
    def __init__(self):
        super().__init__('fire_scan_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera parameters (calculated from Gazebo xacro)
        self.camera_width = 640
        self.camera_height = 480
        
        # Camera intrinsics from xacro: horizontal_fov=1.047 rad, 640x480
        self.fx = 554.3  # Calculated from FOV
        self.fy = 554.3
        self.cx = 320.0
        self.cy = 240.0
        
        # Control parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('scan_speed', 3.5)
        self.declare_parameter('target_height', 14.0)
        self.declare_parameter('height_tolerance', 0.5)
        
        # Fire detection parameters
        self.declare_parameter('brightness_threshold', 200)
        self.declare_parameter('min_fire_area', 50)
        self.declare_parameter('max_fire_area', 10000)
        
        # Search pattern parameters
        self.declare_parameter('search_grid_spacing', 10.0)
        self.declare_parameter('search_hold_time', 2.0)
        
        # Read parameters
        self.enabled = self.get_parameter('enabled').value
        self.scan_speed = self.get_parameter('scan_speed').value
        self.target_height = self.get_parameter('target_height').value
        self.height_tolerance = self.get_parameter('height_tolerance').value
        
        self.brightness_threshold = self.get_parameter('brightness_threshold').value
        self.min_fire_area = self.get_parameter('min_fire_area').value
        self.max_fire_area = self.get_parameter('max_fire_area').value
        
        self.search_grid_spacing = self.get_parameter('search_grid_spacing').value
        self.search_hold_time = self.get_parameter('search_hold_time').value
        
        # SIMPLIFIED STATE: just SCAN
        self.drone_position = None
        self.search_waypoints = []
        self.current_waypoint = 0
        self.waypoint_arrival_time = None
        
        # Fire logging - much simpler
        self.detected_fires = []  # List of world positions
        self.duplicate_threshold = 8.0  # Increased from 3.0 meters - more forgiving for testing
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.fire_position_pub = self.create_publisher(PointStamped, '/drone/fire_scan/fire_position', 10)
        self.fire_positions_pub = self.create_publisher(PoseArray, '/drone/fire_scan/fire_positions', 10)
        self.debug_image_pub = self.create_publisher(Image, '/drone/fire_scan/debug_image', 10)
        
        # Subscribers
        self.thermal_sub = self.create_subscription(Image, '/drone/ir_camera/image_raw', self.thermal_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/drone/odometry', self.odom_callback, 10)
        
        # Timers
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.status_timer = self.create_timer(2.0, self.status_update)   # Every 2 seconds
        self.fire_positions_timer = self.create_timer(1.0, self.publish_all_fire_positions)  # Publish fire positions every second
        
        # Initialize
        self.generate_search_pattern()
        self.frame_count = 0
        
        self.get_logger().info(f'Fire Scan Node started')
        self.get_logger().info(f'Target height: {self.target_height}m, Scan speed: {self.scan_speed}m/s')
        self.get_logger().info(f'Grid spacing: {self.search_grid_spacing}m, Fire detection threshold: {self.brightness_threshold}')
        self.get_logger().info('Publishing individual fires to: /drone/fire_scan/fire_position')
        self.get_logger().info('Publishing all fire positions to: /drone/fire_scan/fire_positions')

    def thermal_callback(self, msg):
        """Process thermal images and detect fires - log immediately without centering"""
        self.frame_count += 1
        if self.frame_count % 3 != 0:  # Process every 3rd frame to reduce CPU load
            return
            
        if self.drone_position is None:
            return  # Can't calculate world positions without drone position
            
        try:
            # Convert image
            if msg.encoding == 'mono8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
                threshold = self.brightness_threshold
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                threshold = self.brightness_threshold
            
            # Detect fires
            fires = self.detect_fires(cv_image, threshold)
            
            # Process each detected fire immediately
            for fire_x, fire_y, fire_area in fires:
                fire_world = self.estimate_fire_world_position(fire_x, fire_y)
                if fire_world and not self.is_duplicate_fire(fire_world):
                    self.log_fire_immediately(fire_world, fire_x, fire_y, fire_area)
            
            # Create debug image
            if fires:
                debug_image = self.create_debug_image(cv_image, fires)
                self.publish_debug_image(debug_image, msg.header)
        
        except Exception as e:
            self.get_logger().error(f'Thermal processing error: {e}')

    def detect_fires(self, image, threshold):
        """Simple fire detection"""
        _, thresh = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        fires = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_fire_area < area < self.max_fire_area:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    x = int(M["m10"] / M["m00"])
                    y = int(M["m01"] / M["m00"])
                    fires.append((x, y, area))
        return fires

    def log_fire_immediately(self, fire_world, pixel_x, pixel_y, area):
        """Log fire position immediately when detected"""
        self.detected_fires.append(fire_world)
        
        # Publish fire location
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = fire_world[0]
        msg.point.y = fire_world[1] 
        msg.point.z = fire_world[2]
        self.fire_position_pub.publish(msg)
        
        self.get_logger().info(f'FIRE LOGGED #{len(self.detected_fires)}: World({fire_world[0]:.1f}, {fire_world[1]:.1f}) Pixel({pixel_x}, {pixel_y}) Area({area:.0f}px)')
        
        # Publish all detected fires
        self.publish_all_fire_positions()

    def publish_all_fire_positions(self):
        """Publish all detected fire positions as a PoseArray"""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"
        
        for fire_world in self.detected_fires:
            pose = Pose()
            pose.position.x = fire_world[0]
            pose.position.y = fire_world[1]
            pose.position.z = fire_world[2]
            
            # Set default orientation (no rotation)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            
            pose_array.poses.append(pose)
        
        self.fire_positions_pub.publish(pose_array)

    def odom_callback(self, msg):
        """Update drone position"""
        was_none = self.drone_position is None
        self.drone_position = msg.pose.pose.position
        
        # Generate search pattern once we have position
        if was_none and not self.search_waypoints:
            self.generate_search_pattern()

    def control_loop(self):
        """SIMPLE CONTROL: Just fly the search pattern at target height"""
        if not self.enabled or self.drone_position is None:
            return
            
        cmd = Twist()
        
        # Get height info
        height = self.drone_position.z
        height_error = self.target_height - height
        at_target_height = abs(height_error) < self.height_tolerance
        
        # Always prioritize getting to target height
        if not at_target_height:
            cmd.linear.z = height_error * 2.0
            
        # Execute search pattern if at correct height
        if at_target_height and self.search_waypoints:
            search_cmd = self.execute_search_pattern()
            cmd.linear.x = search_cmd.linear.x
            cmd.linear.y = search_cmd.linear.y
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)

    def execute_search_pattern(self):
        """Navigate through search waypoints"""
        cmd = Twist()
        
        if self.current_waypoint >= len(self.search_waypoints):
            # Pattern complete
            self.get_logger().info(f"Search pattern completed! Found {len(self.detected_fires)} fires total.")
            return cmd
        
        # Navigate to current waypoint
        target_x, target_y = self.search_waypoints[self.current_waypoint]
        dx = target_x - self.drone_position.x
        dy = target_y - self.drone_position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 2.0:  # Reached waypoint
            current_time = self.get_clock().now()
            if self.waypoint_arrival_time is None:
                self.waypoint_arrival_time = current_time
                self.get_logger().info(f"Reached waypoint {self.current_waypoint + 1}/{len(self.search_waypoints)} - scanning...")
            
            # Hold at waypoint for scanning
            hold_duration = (current_time - self.waypoint_arrival_time).nanoseconds / 1e9
            if hold_duration > self.search_hold_time:
                # Move to next waypoint
                self.current_waypoint += 1
                self.waypoint_arrival_time = None
                if self.current_waypoint < len(self.search_waypoints):
                    self.get_logger().info(f"Moving to waypoint {self.current_waypoint + 1}/{len(self.search_waypoints)}")
        else:
            # Move toward waypoint
            cmd.linear.x = (dx / distance) * self.scan_speed
            cmd.linear.y = (dy / distance) * self.scan_speed
            
            # Apply velocity limits
            max_vel = 2.0
            cmd.linear.x = max(-max_vel, min(max_vel, cmd.linear.x))
            cmd.linear.y = max(-max_vel, min(max_vel, cmd.linear.y))
        
        return cmd

    def estimate_fire_world_position(self, fire_x, fire_y):
        """Calculate fire world position"""
        if self.drone_position is None:
            return None
            
        dx = fire_x - self.cx
        dy = fire_y - self.cy
        
        angle_x = math.atan2(dx, self.fx)
        angle_y = math.atan2(dy, self.fy)
        
        drone_z = self.drone_position.z
        offset_x = drone_z * math.tan(angle_x)
        offset_y = drone_z * math.tan(angle_y)
        
        world_x = self.drone_position.x + offset_x
        world_y = self.drone_position.y + offset_y
        
        return (world_x, world_y, 0.0)

    def is_duplicate_fire(self, fire_world):
        """Check if fire is too close to a previously detected fire"""
        for detected in self.detected_fires:
            dist = math.sqrt((fire_world[0] - detected[0])**2 + (fire_world[1] - detected[1])**2)
            if dist < self.duplicate_threshold:
                return True
        return False

    def generate_search_pattern(self):
        """Generate simple grid search pattern"""
        if self.drone_position is None:
            return
            
        # Create waypoints in 50x50m area from -25 to +25
        waypoints = []
        
        # Lawn mower pattern for systematic coverage
        x_points = list(range(-25, 26, int(self.search_grid_spacing)))
        y_points = list(range(-25, 26, int(self.search_grid_spacing)))
        
        for i, x in enumerate(x_points):
            if i % 2 == 0:  # Even rows: bottom to top
                for y in y_points:
                    waypoints.append((float(x), float(y)))
            else:  # Odd rows: top to bottom
                for y in reversed(y_points):
                    waypoints.append((float(x), float(y)))
        
        self.search_waypoints = waypoints
        self.current_waypoint = 0
        self.get_logger().info(f"Generated {len(waypoints)} search waypoints in lawn-mower pattern")

    def create_debug_image(self, original_image, fire_locations):
        """Create debug image with fire locations marked"""
        # Convert to color for visualization
        debug_image = cv2.cvtColor(original_image, cv2.COLOR_GRAY2BGR)
        
        # Draw detection overlay
        for fire_x, fire_y, area in fire_locations:
            center = (fire_x, fire_y)
            radius = max(int(np.sqrt(area / np.pi)), 10)
            
            # Draw red circle around fire
            cv2.circle(debug_image, center, radius, (0, 0, 255), 2)
            # Draw center crosshair
            cv2.drawMarker(debug_image, center, (0, 255, 0), cv2.MARKER_CROSS, 10, 2)
            # Add text with info
            cv2.putText(debug_image, f'FIRE {area:.0f}px', 
                       (fire_x + 15, fire_y - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return debug_image

    def publish_debug_image(self, debug_image, header):
        """Publish debug image"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing debug image: {e}')

    def status_update(self):
        """Print status information"""
        if not self.enabled:
            return
            
        height = self.drone_position.z if self.drone_position else 0.0
        waypoint_progress = f"{self.current_waypoint + 1}/{len(self.search_waypoints)}" if self.search_waypoints else "0/0"
        
        self.get_logger().info(f'Scanning - Height: {height:.1f}m, Waypoint: {waypoint_progress}, Fires Found: {len(self.detected_fires)}')

def main(args=None):
    rclpy.init(args=args)
    node = FireScanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Fire scan completed')
        if node.detected_fires:
            node.get_logger().info(f'FINAL RESULTS: Found {len(node.detected_fires)} fires:')
            for i, fire in enumerate(node.detected_fires):
                node.get_logger().info(f'  Fire {i+1}: ({fire[0]:.1f}, {fire[1]:.1f})')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()