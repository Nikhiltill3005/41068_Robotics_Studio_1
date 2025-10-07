#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty
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
        self.declare_parameter('scan_speed', 2.5)  # Reduced from 3.5 for smoother motion
        self.declare_parameter('target_height', 14.0)
        self.declare_parameter('height_tolerance', 0.5)
        self.declare_parameter('auto_search', False)  # Start search automatically or wait for trigger
        self.declare_parameter('debug_image_rate', 6)  # Publish debug image every Nth frame
        
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
        self.auto_search = self.get_parameter('auto_search').value
        self.debug_image_rate = self.get_parameter('debug_image_rate').value
        
        self.brightness_threshold = self.get_parameter('brightness_threshold').value
        self.min_fire_area = self.get_parameter('min_fire_area').value
        self.max_fire_area = self.get_parameter('max_fire_area').value
        
        self.search_grid_spacing = self.get_parameter('search_grid_spacing').value
        self.search_hold_time = self.get_parameter('search_hold_time').value
        
        # SIMPLIFIED STATE: just SCAN
        self.drone_position = None
        self.drone_orientation = None
        self.search_waypoints = []
        self.current_waypoint = 0
        self.waypoint_arrival_time = None
        self.search_active = self.auto_search  # Only search if auto_search enabled
        
        # Fire logging - much simpler
        self.detected_fires = []  # List of world positions
        self.duplicate_threshold = 15.0  # Increased threshold to prevent multiple detections of same fire
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.fire_positions_pub = self.create_publisher(PoseArray, '/drone/fire_scan/fire_positions', 10)
        self.debug_image_pub = self.create_publisher(Image, '/drone/fire_scan/debug_image', 10)
        
        # Subscribers
        self.thermal_sub = self.create_subscription(Image, '/drone/ir_camera/image_raw', self.thermal_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/drone/odometry', self.odom_callback, 10)
        self.search_trigger_sub = self.create_subscription(Bool, '/drone/fire_scan/start_search', self.search_trigger_callback, 10)
        
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
        self.get_logger().info(f'Performance: Debug images every {self.debug_image_rate} frames, fire detection every 3 frames')
        self.get_logger().info('Publishing all fire positions to: /drone/fire_scan/fire_positions')
        self.get_logger().info('Publishing debug images to: /drone/fire_scan/debug_image')
        if self.auto_search:
            self.get_logger().info('Auto-search ENABLED - will start searching automatically')
        else:
            self.get_logger().info('Auto-search DISABLED - send True to /drone/fire_scan/start_search to begin')

    def search_trigger_callback(self, msg):
        """Toggle search pattern execution"""
        if msg.data and not self.search_active:
            self.search_active = True
            self.get_logger().info('SEARCH ACTIVATED - Starting search pattern')
            # Reset to beginning if already completed
            if self.current_waypoint >= len(self.search_waypoints):
                self.current_waypoint = 0
                self.waypoint_arrival_time = None
        elif not msg.data and self.search_active:
            self.search_active = False
            self.get_logger().info('SEARCH PAUSED - Debug visualization continues')

    def thermal_callback(self, msg):
        """Process thermal images and detect fires - log immediately without centering"""
        try:
            # Convert image
            if msg.encoding == 'mono8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
                threshold = self.brightness_threshold
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                threshold = self.brightness_threshold
            
            # Process fire detection every 3rd frame to reduce CPU load
            fires_with_world_pos = []
            self.frame_count += 1
            if self.frame_count % 3 == 0:
                # Detect fires
                fires = self.detect_fires(cv_image, threshold)
                
                # Calculate world positions for all detected fires
                for fire_x, fire_y, fire_area in fires:
                    fire_world = self.estimate_fire_world_position(fire_x, fire_y)
                    fires_with_world_pos.append((fire_x, fire_y, fire_area, fire_world))
                    
                    # Process each detected fire immediately
                    if fire_world and not self.is_duplicate_fire(fire_world):
                        self.log_fire_immediately(fire_world, fire_x, fire_y, fire_area)
            
            # Publish debug image at reduced rate to save CPU
            # This reduces debug image publishing while still detecting fires frequently
            if self.frame_count % self.debug_image_rate == 0:
                debug_image = self.create_debug_image(cv_image, fires_with_world_pos)
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
        
        # Note: Individual fire positions are published via the PoseArray publisher
        # in the publish_all_fire_positions method
        
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
        """Update drone position and orientation"""
        was_none = self.drone_position is None
        self.drone_position = msg.pose.pose.position
        self.drone_orientation = msg.pose.pose.orientation
        
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
        
        # Always prioritize getting to target height (only if search is active)
        if self.search_active and not at_target_height:
            # Smoother height control with softer gains
            cmd.linear.z = max(-1.0, min(1.0, height_error * 1.5))
            
        # Execute search pattern if active and at correct height
        if self.search_active and at_target_height and self.search_waypoints:
            search_cmd = self.execute_search_pattern()
            cmd.linear.x = search_cmd.linear.x
            cmd.linear.y = search_cmd.linear.y
        
        # Publish command (will be zero if search not active)
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
            # Smooth velocity control with proportional gain
            # Velocity reduces as we get closer to waypoint
            speed_factor = min(1.0, distance / 5.0)  # Slow down within 5m
            cmd.linear.x = (dx / distance) * self.scan_speed * speed_factor
            cmd.linear.y = (dy / distance) * self.scan_speed * speed_factor
            
            # Apply velocity limits (reduced for smoother motion)
            max_vel = 1.5
            cmd.linear.x = max(-max_vel, min(max_vel, cmd.linear.x))
            cmd.linear.y = max(-max_vel, min(max_vel, cmd.linear.y))
        
        return cmd

    def estimate_fire_world_position(self, fire_x, fire_y):
        """Calculate fire world position accounting for downward-facing camera"""
        if self.drone_position is None or self.drone_orientation is None:
            return None
            
        # Camera pixel coordinates relative to center
        dx = fire_x - self.cx
        dy = fire_y - self.cy
        
        # Calculate angles in camera frame
        angle_x = math.atan2(dx, self.fx)
        angle_y = math.atan2(dy, self.fy)
        
        drone_height = self.drone_position.z
        if drone_height <= 0.1:  # Avoid division by very small numbers
            return None
            
        # Calculate ground projection distances
        # For a downward-facing camera, the ground distance is height * tan(angle)
        ground_dist_x = drone_height * math.tan(angle_x)
        ground_dist_y = drone_height * math.tan(angle_y)
        
        # Get drone's yaw angle from quaternion (for future rotation support)
        q = self.drone_orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # Camera-to-World coordinate transformation
        # For this downward-facing camera configuration (rpy="0 1.57 0"):
        # - Camera right (+dx in image) maps to World -Y
        # - Camera down (+dy in image) maps to World -X
        # This was empirically determined to be correct (Version 7 from testing)
        world_x = self.drone_position.x - ground_dist_y
        world_y = self.drone_position.y - ground_dist_x
        
        # TODO: When drone rotation is implemented, apply yaw rotation here:
        # world_offset_x = -ground_dist_y * cos(yaw) + ground_dist_x * sin(yaw)
        # world_offset_y = -ground_dist_y * sin(yaw) - ground_dist_x * cos(yaw)
        
        # Optional debug logging (only occasionally to avoid spam)
        if self.frame_count % 90 == 0:  # Log every 90th processed frame
            self.get_logger().info(
                f'Fire Detection: Pixel({fire_x}, {fire_y}) -> '
                f'World({world_x:.2f}, {world_y:.2f}) | '
                f'Drone({self.drone_position.x:.2f}, {self.drone_position.y:.2f}, {drone_height:.2f}m) '
                f'Yaw:{math.degrees(yaw):.1f}°'
            )
        
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

    def create_debug_image(self, original_image, fires_with_world_pos):
        """Create debug image with fire locations and world coordinates marked"""
        # Convert to color for visualization
        debug_image = cv2.cvtColor(original_image, cv2.COLOR_GRAY2BGR)
        
        # Add drone position info at top of image
        if self.drone_position:
            info_text = f'Drone: X={self.drone_position.x:.1f} Y={self.drone_position.y:.1f} Z={self.drone_position.z:.1f}m'
            cv2.putText(debug_image, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Add drone yaw angle
            if self.drone_orientation:
                q = self.drone_orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                yaw_deg = math.degrees(yaw)
                yaw_text = f'Yaw: {yaw_deg:.1f} deg'
                cv2.putText(debug_image, yaw_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            total_fires_text = f'Total Fires: {len(self.detected_fires)}'
            cv2.putText(debug_image, total_fires_text, (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add search status
            search_status = "SEARCH: ACTIVE" if self.search_active else "SEARCH: PAUSED"
            search_color = (0, 255, 0) if self.search_active else (0, 165, 255)  # Green if active, orange if paused
            cv2.putText(debug_image, search_status, (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, search_color, 2)
        
        # Add status text when no fires detected
        if len(fires_with_world_pos) == 0:
            cv2.putText(debug_image, 'No fires detected in view', (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Draw fire detections with world coordinates
        for fire_x, fire_y, area, fire_world in fires_with_world_pos:
            center = (fire_x, fire_y)
            radius = max(int(np.sqrt(area / np.pi)), 10)
            
            # Draw red circle around fire
            cv2.circle(debug_image, center, radius, (0, 0, 255), 3)
            # Draw center crosshair
            cv2.drawMarker(debug_image, center, (0, 255, 0), cv2.MARKER_CROSS, 15, 3)
            
            # Calculate text position (to the right of the fire)
            text_x = fire_x + radius + 10
            text_y = fire_y
            
            # Add background rectangles for text readability
            if fire_world:
                # World coordinate text - LARGER AND MORE VISIBLE
                world_text = f'World: ({fire_world[0]:.1f}, {fire_world[1]:.1f})'
                (w, h), _ = cv2.getTextSize(world_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(debug_image, (text_x - 2, text_y - h - 5), 
                            (text_x + w + 2, text_y + 5), (0, 0, 0), -1)
                cv2.putText(debug_image, world_text, (text_x, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                # Pixel area text below
                pixel_text = f'Area: {area:.0f}px'
                (w2, h2), _ = cv2.getTextSize(pixel_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(debug_image, (text_x - 2, text_y + 10), 
                            (text_x + w2 + 2, text_y + 10 + h2 + 5), (0, 0, 0), -1)
                cv2.putText(debug_image, pixel_text, (text_x, text_y + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Draw a line from fire center to text for clarity
                cv2.line(debug_image, center, (text_x, text_y), (0, 255, 255), 2)
            else:
                no_pos_text = 'World: N/A (no drone pos)'
                (w, h), _ = cv2.getTextSize(no_pos_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(debug_image, (text_x - 2, text_y - h - 5), 
                            (text_x + w + 2, text_y + 5), (0, 0, 0), -1)
                cv2.putText(debug_image, no_pos_text, (text_x, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Add camera center crosshair for reference
        cv2.drawMarker(debug_image, (int(self.cx), int(self.cy)), 
                      (255, 255, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(debug_image, 'Camera Center', 
                   (int(self.cx) + 10, int(self.cy) - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
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
        search_status = "ACTIVE" if self.search_active else "PAUSED"
        
        self.get_logger().info(f'Status: {search_status} - Height: {height:.1f}m, Waypoint: {waypoint_progress}, Fires: {len(self.detected_fires)}')

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