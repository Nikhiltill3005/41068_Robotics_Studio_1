#!/usr/bin/env python3

"""
Drone Search Pattern Node

This node implements a grid search pattern for fire detection missions.
The drone will hover at a specified altitude and systematically search an area
using a lawn-mower/grid pattern while the fire detection node monitors for fires.

Subscribed Topics:
    /drone/fire_detection/location (geometry_msgs/PointStamped): Fire detection results
    /drone/odom (nav_msgs/Odometry): Drone position feedback

Published Topics:
    /drone/cmd_vel (geometry_msgs/Twist): Drone movement commands

Parameters:
    search_altitude (double, default: 5.0): Altitude to maintain during search
    search_speed (double, default: 1.0): Speed during search pattern
    pattern_width (double, default: 20.0): Width of search area
    pattern_length (double, default: 30.0): Length of search area  
    lane_spacing (double, default: 3.0): Distance between search lanes
    waypoint_tolerance (double, default: 0.5): Distance tolerance for waypoint arrival
    fire_investigation_time (double, default: 15.0): Time to hover over detected fire
    fire_hover_altitude (double, default: 2.0): Altitude when investigating fire
    auto_start (bool, default: true): Start search automatically

Usage:
    ros2 run 41068_ignition_bringup drone_search_pattern_node.py
    
    Or with custom parameters:
    ros2 run 41068_ignition_bringup drone_search_pattern_node.py --ros-args \
        -p search_altitude:=5.0 -p pattern_width:=25.0 -p lane_spacing:=2.5
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time
from enum import Enum

class SearchState(Enum):
    IDLE = "idle"
    ASCENDING = "ascending" 
    SEARCHING = "searching"
    INVESTIGATING_FIRE = "investigating_fire"
    COMPLETED = "completed"

class DroneSearchPatternNode(Node):
    def __init__(self):
        super().__init__('drone_search_pattern_node')
        
        # Declare parameters
        self.declare_parameter('search_altitude', 5.0)
        self.declare_parameter('search_speed', 1.0)
        self.declare_parameter('pattern_width', 20.0)
        self.declare_parameter('pattern_length', 30.0)
        self.declare_parameter('lane_spacing', 3.0)
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('fire_investigation_time', 15.0)
        self.declare_parameter('fire_hover_altitude', 2.0)
        self.declare_parameter('auto_start', True)
        
        # Get parameters
        self.search_altitude = self.get_parameter('search_altitude').get_parameter_value().double_value
        self.search_speed = self.get_parameter('search_speed').get_parameter_value().double_value
        self.pattern_width = self.get_parameter('pattern_width').get_parameter_value().double_value
        self.pattern_length = self.get_parameter('pattern_length').get_parameter_value().double_value
        self.lane_spacing = self.get_parameter('lane_spacing').get_parameter_value().double_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.fire_investigation_time = self.get_parameter('fire_investigation_time').get_parameter_value().double_value
        self.fire_hover_altitude = self.get_parameter('fire_hover_altitude').get_parameter_value().double_value
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        
        # Subscribers
        self.fire_location_sub = self.create_subscription(
            PointStamped,
            '/drone/fire_detection/location',
            self.fire_detection_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/drone/search_status',
            10
        )
        
        # State variables
        self.current_state = SearchState.IDLE
        self.current_position = Point()
        self.start_position = Point()
        self.current_waypoint_index = 0
        self.waypoints = []
        self.fire_investigation_start_time = None
        self.fire_location = None
        self.last_fire_detection_time = None
        
        # Control variables
        self.max_altitude_speed = 1.0
        self.altitude_tolerance = 0.2
        
        # Timer for main control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info(f'Drone Search Pattern Node started')
        self.get_logger().info(f'Search area: {self.pattern_width}m x {self.pattern_length}m')
        self.get_logger().info(f'Search altitude: {self.search_altitude}m, Lane spacing: {self.lane_spacing}m')
        
        # Auto-start if enabled
        if self.auto_start:
            self.get_logger().info('Auto-start enabled - will begin search when drone position is received')

    def odom_callback(self, msg):
        """Update current drone position from odometry"""
        self.current_position = msg.pose.pose.position
        
        # If this is the first position reading and auto_start is enabled, start the search
        if self.current_state == SearchState.IDLE and self.auto_start:
            self.start_search()

    def fire_detection_callback(self, msg):
        """Handle fire detection - pause search and investigate"""
        self.last_fire_detection_time = self.get_clock().now()
        
        # If we're searching and not already investigating a fire, switch to investigation mode
        if self.current_state == SearchState.SEARCHING:
            self.fire_location = self.current_position  # Record current position as fire location
            self.current_state = SearchState.INVESTIGATING_FIRE
            self.fire_investigation_start_time = self.get_clock().now()
            
            self.get_logger().info(f'FIRE DETECTED! Investigating at position: '
                                 f'({self.fire_location.x:.2f}, {self.fire_location.y:.2f})')
            self.publish_status(f'Fire detected! Investigating at ({self.fire_location.x:.1f}, {self.fire_location.y:.1f})')

    def start_search(self):
        """Initialize and start the search pattern"""
        if self.current_state != SearchState.IDLE:
            self.get_logger().warn('Search already in progress')
            return
            
        # Record starting position
        self.start_position = Point()
        self.start_position.x = self.current_position.x
        self.start_position.y = self.current_position.y
        self.start_position.z = self.current_position.z
        
        # Generate waypoints for grid search pattern
        self.generate_search_waypoints()
        
        # Start by ascending to search altitude
        self.current_state = SearchState.ASCENDING
        self.current_waypoint_index = 0
        
        self.get_logger().info(f'Starting search pattern from position: '
                             f'({self.start_position.x:.2f}, {self.start_position.y:.2f}, {self.start_position.z:.2f})')
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints for search pattern')
        self.publish_status(f'Starting search - {len(self.waypoints)} waypoints generated')

    def generate_search_waypoints(self):
        """Generate waypoints for a lawn-mower/grid search pattern"""
        self.waypoints = []
        
        # Calculate number of lanes needed
        num_lanes = int(self.pattern_width / self.lane_spacing) + 1
        
        # Generate waypoints in a lawn-mower pattern
        for lane in range(num_lanes):
            y_offset = lane * self.lane_spacing - (self.pattern_width / 2)
            
            if lane % 2 == 0:  # Even lanes: go forward
                start_x = -self.pattern_length / 2
                end_x = self.pattern_length / 2
            else:  # Odd lanes: go backward  
                start_x = self.pattern_length / 2
                end_x = -self.pattern_length / 2
            
            # Add waypoints along this lane
            num_points_per_lane = max(2, int(self.pattern_length / 2))  # At least 2 points per lane
            for i in range(num_points_per_lane):
                if num_points_per_lane == 1:
                    x_offset = start_x
                else:
                    progress = i / (num_points_per_lane - 1)
                    x_offset = start_x + progress * (end_x - start_x)
                
                waypoint = Point()
                waypoint.x = self.start_position.x + x_offset
                waypoint.y = self.start_position.y + y_offset  
                waypoint.z = self.search_altitude
                
                self.waypoints.append(waypoint)

    def control_loop(self):
        """Main control loop - called at 10 Hz"""
        if self.current_state == SearchState.IDLE:
            # Send zero velocity when idle
            self.publish_zero_velocity()
            
        elif self.current_state == SearchState.ASCENDING:
            self.handle_ascending()
            
        elif self.current_state == SearchState.SEARCHING:
            self.handle_searching()
            
        elif self.current_state == SearchState.INVESTIGATING_FIRE:
            self.handle_fire_investigation()
            
        elif self.current_state == SearchState.COMPLETED:
            self.handle_search_completed()

    def handle_ascending(self):
        """Handle ascending to search altitude"""
        altitude_error = self.search_altitude - self.current_position.z
        
        if abs(altitude_error) < self.altitude_tolerance:
            # Reached search altitude, start searching
            self.current_state = SearchState.SEARCHING
            self.get_logger().info(f'Reached search altitude ({self.search_altitude}m), starting grid search')
            self.publish_status(f'At search altitude ({self.search_altitude}m) - starting grid search')
        else:
            # Continue ascending
            twist = Twist()
            twist.linear.z = min(self.max_altitude_speed, max(-self.max_altitude_speed, altitude_error * 1.0))
            self.cmd_vel_pub.publish(twist)

    def handle_searching(self):
        """Handle grid search pattern navigation"""
        if self.current_waypoint_index >= len(self.waypoints):
            # Completed all waypoints
            self.current_state = SearchState.COMPLETED
            self.get_logger().info('Grid search pattern completed!')
            self.publish_status('Search pattern completed')
            return
        
        # Get current target waypoint
        target = self.waypoints[self.current_waypoint_index]
        
        # Calculate distance to target
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y
        dz = target.z - self.current_position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if distance < self.waypoint_tolerance:
            # Reached waypoint, move to next one
            self.current_waypoint_index += 1
            progress = self.current_waypoint_index / len(self.waypoints) * 100
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}/{len(self.waypoints)} '
                                 f'({progress:.1f}% complete)')
            self.publish_status(f'Progress: {self.current_waypoint_index}/{len(self.waypoints)} waypoints ({progress:.0f}%)')
        else:
            # Move toward target waypoint
            twist = Twist()
            
            # Normalize direction vector
            if distance > 0:
                twist.linear.x = (dx / distance) * self.search_speed
                twist.linear.y = (dy / distance) * self.search_speed
                twist.linear.z = (dz / distance) * self.max_altitude_speed * 0.5  # Gentler altitude control
            
            self.cmd_vel_pub.publish(twist)

    def handle_fire_investigation(self):
        """Handle fire investigation mode"""
        current_time = self.get_clock().now()
        investigation_elapsed = (current_time - self.fire_investigation_start_time).nanoseconds / 1e9
        
        # Move to investigation altitude
        altitude_error = self.fire_hover_altitude - self.current_position.z
        
        twist = Twist()
        if abs(altitude_error) > self.altitude_tolerance:
            twist.linear.z = min(self.max_altitude_speed * 0.5, 
                               max(-self.max_altitude_speed * 0.5, altitude_error * 1.0))
        
        # Check if investigation time is complete
        if investigation_elapsed >= self.fire_investigation_time:
            # Return to search altitude and continue search
            self.current_state = SearchState.SEARCHING
            self.get_logger().info(f'Fire investigation complete after {investigation_elapsed:.1f}s, resuming search')
            self.publish_status('Fire investigation complete - resuming search')
        else:
            # Continue hovering and investigating
            remaining_time = self.fire_investigation_time - investigation_elapsed
            self.publish_status(f'Investigating fire - {remaining_time:.1f}s remaining')
        
        self.cmd_vel_pub.publish(twist)

    def handle_search_completed(self):
        """Handle search completion"""
        # Send zero velocity and status
        self.publish_zero_velocity()
        # Only publish status occasionally to avoid spam
        if hasattr(self, '_last_completion_status_time'):
            if (self.get_clock().now() - self._last_completion_status_time).nanoseconds / 1e9 > 5.0:
                self.publish_status('Search pattern completed - mission finished')
                self._last_completion_status_time = self.get_clock().now()
        else:
            self.publish_status('Search pattern completed - mission finished')
            self._last_completion_status_time = self.get_clock().now()

    def publish_zero_velocity(self):
        """Publish zero velocity command"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def publish_status(self, message):
        """Publish status message"""
        status_msg = String()
        status_msg.data = f"[{self.current_state.value}] {message}"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    search_node = DroneSearchPatternNode()
    
    try:
        rclpy.spin(search_node)
    except KeyboardInterrupt:
        search_node.get_logger().info('Drone search pattern node shutting down')
    
    search_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

