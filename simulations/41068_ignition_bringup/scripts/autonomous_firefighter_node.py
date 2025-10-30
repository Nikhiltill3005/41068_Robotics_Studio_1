#!/usr/bin/env python3
"""
Autonomous Firefighter Node for Husky Robot

This node implements a state machine that:
- Scans for test_fire entities in the world at startup
- Subscribes to fire locations from the overhead drone
- Autonomously navigates to each fire location
- Approaches fires to within 1.5m
- Extinguishes fires with particle animation
- Deletes fire entities from the simulation
- Handles dynamic fire list that populates over time

States:
- IDLE: Waiting for fires to be detected
- PLANNING: Sorting and prioritizing fires
- NAVIGATING: Moving to next fire location (Nav2 handles approach)
- EXTINGUISHING: Applying extinguishing action
- BACKING_UP: Recovery maneuver (if enabled)
- COMPLETED: All known fires extinguished

Usage:
    ros2 run 41068_ignition_bringup autonomous_firefighter_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import math
from enum import Enum
import time
import re
import subprocess
import json


class FirefighterState(Enum):
    """State machine states for autonomous firefighting"""
    IDLE = 0              # Waiting for fires
    PLANNING = 1          # Sorting/prioritizing fires
    NAVIGATING = 2        # Moving to fire location
    EXTINGUISHING = 3     # Extinguishing fire (APPROACHING removed - trust Nav2)
    COMPLETED = 4         # All fires handled
    BACKING_UP = 5        # Recovery: backing up to clear obstacles


class AutonomousFirefighter(Node):
    def __init__(self):
        super().__init__('autonomous_firefighter')

        # Parameters
        self.declare_parameter('target_distance', 0.75)  # Target distance from fire (meters) - increased for safety
        self.declare_parameter('distance_tolerance', 0.4)  # Tolerance for positioning
        self.declare_parameter('extinguish_duration', 3.0)  # How long to extinguish (seconds)
        self.declare_parameter('auto_start', True)  # Auto-start firefighting
        self.declare_parameter('min_wait_time', 5.0)  # Wait time in IDLE before checking again
        self.declare_parameter('stuck_timeout', 8.0)  # Time stuck before giving up (seconds)
        self.declare_parameter('stuck_velocity_threshold', 0.05)  # Velocity threshold for stuck detection (m/s)
        self.declare_parameter('fallback_distance', 2.5)  # Distance to fire to consider "close enough" (meters)
        self.declare_parameter('fire_match_tolerance', 1.0)  # Distance tolerance for matching fire positions (meters)
        self.declare_parameter('enable_multi_angle_approach', False)  # Enable trying multiple approach angles
        self.declare_parameter('enable_backup_recovery', False)  # Enable backup recovery maneuver
        self.last_skip_time = 0.0  # Cooldown for skipping fires
        self.skip_cooldown = 2.0  # Seconds to wait after skipping a fire


        self.target_distance = self.get_parameter('target_distance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.extinguish_duration = self.get_parameter('extinguish_duration').value
        self.auto_start = self.get_parameter('auto_start').value
        self.min_wait_time = self.get_parameter('min_wait_time').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_velocity_threshold = self.get_parameter('stuck_velocity_threshold').value
        self.fallback_distance = self.get_parameter('fallback_distance').value
        self.fire_match_tolerance = self.get_parameter('fire_match_tolerance').value
        self.enable_multi_angle_approach = self.get_parameter('enable_multi_angle_approach').value
        self.enable_backup_recovery = self.get_parameter('enable_backup_recovery').value

        # State machine
        self.state = FirefighterState.IDLE
        self.previous_state = None

        # Fire tracking
        self.detected_fires = []  # List of fire Poses from drone
        self.extinguished_fires = []  # List of extinguished fire positions
        self.current_target_fire = None
        self.fire_queue = []  # Ordered list of fires to visit
        self.approach_attempts = 0  # Track approach angle attempts
        self.max_approach_attempts = 8  # Try 8 different angles around fire
        self.backup_attempts = 0  # Track backup recovery attempts
        self.max_backup_attempts = 2  # Try backing up twice before giving up

        # Fire entity tracking (name -> position mapping)
        self.fire_entities = {}  # {name: (x, y, z)}

        # Robot state
        self.robot_position = None
        self.robot_orientation = None
        self.robot_velocity = 0.0  # Linear velocity magnitude
        self.robot_linear_x = 0.0
        self.robot_linear_y = 0.0

        # Navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.navigation_result = None

        # Timing
        self.state_start_time = time.time()
        self.last_fire_update_time = time.time()
        self.extinguish_start_time = None

        # Stuck detection
        self.stuck_start_time = None
        self.is_stuck = False

        # Subscriptions
        self.fire_sub = self.create_subscription(
            PoseArray,
            '/drone/fire_scan/fire_positions',
            self.fire_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/husky/odometry',
            self.odom_callback,
            10
        )

        # Publishers
        self.extinguish_marker_pub = self.create_publisher(
            MarkerArray,
            '/firefighter/extinguish_effect',
            10
        )

        self.status_marker_pub = self.create_publisher(
            Marker,
            '/firefighter/status',
            10
        )

        # cmd_vel publisher for backup recovery (only used if enable_backup_recovery=True)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/husky/cmd_vel',
            10
        )

        # State machine timer
        self.state_machine_timer = self.create_timer(0.1, self.state_machine_update)
        self.status_timer = self.create_timer(2.0, self.publish_status)

        self.get_logger().info('=== Autonomous Firefighter Initialized ===')
        self.get_logger().info(f'Target distance: {self.target_distance}m')
        self.get_logger().info(f'Extinguish duration: {self.extinguish_duration}s')
        self.get_logger().info(f'Auto-start: {self.auto_start}')
        self.get_logger().info(f'Multi-angle approach: {self.enable_multi_angle_approach}')
        self.get_logger().info(f'Backup recovery: {self.enable_backup_recovery}')
        
        # Scan world for fire entities
        self.scan_world_for_fires()
        
        self.get_logger().info('Waiting for fire locations from drone...')

    def scan_world_for_fires(self):
        """Scan the world for test_fire entities and store their positions using ign service"""
        self.get_logger().info('=== Scanning world for fire entities ===')
        
        try:
            # Call ign service to get scene info
            cmd = [
                'ign', 'service',
                '-s', '/world/bushland_world/scene/info',
                '--reqtype', 'ignition.msgs.Empty',
                '--reptype', 'ignition.msgs.Scene',
                '--timeout', '5000',
                '--req', ''
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode != 0:
                self.get_logger().error(f'Failed to get scene info, return code: {result.returncode}')
                self.get_logger().warn('Fire entities will not be deleted from simulation.')
                return
            
            output = result.stdout
            
            # Pattern to match test_fire with or without numbers
            fire_pattern = re.compile(r'test_fire\d*')
            
            # Find all unique fire names
            fire_names = list(set(fire_pattern.findall(output)))
            
            if not fire_names:
                self.get_logger().warn('No test_fire entities found in scene info')
                return
            
            self.get_logger().info(f'Found {len(fire_names)} fire entities')
            
            # Parse positions for each fire
            for fire_name in fire_names:
                # Find the model block for this fire
                # Pattern: model { ... name: "fire_name" ... pose { position { x: ... y: ... z: ... } } }
                model_pattern = re.compile(
                    r'model\s*\{[^}]*?name:\s*"' + re.escape(fire_name) + r'"[^}]*?'
                    r'pose\s*\{[^}]*?position\s*\{[^}]*?'
                    r'x:\s*([-\d.]+)[^}]*?'
                    r'y:\s*([-\d.]+)[^}]*?'
                    r'(?:z:\s*([-\d.]+))?',
                    re.DOTALL
                )
                
                match = model_pattern.search(output)
                
                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    z = float(match.group(3)) if match.group(3) else 0.0
                    
                    self.fire_entities[fire_name] = (x, y, z)
                    self.get_logger().info(
                        f'  {fire_name}: position ({x:.2f}, {y:.2f}, {z:.2f})'
                    )
                else:
                    # Store with None position if we can't parse it
                    self.fire_entities[fire_name] = None
                    self.get_logger().warn(f'  {fire_name}: could not parse position')
            
            self.get_logger().info(f'=== Successfully registered {len(self.fire_entities)} fire entities ===')
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Timeout while scanning for fires')
            self.get_logger().warn('The ign service command timed out. Fire entities will not be deleted.')
        except Exception as e:
            self.get_logger().error(f'Error scanning for fires: {str(e)}')

    def scan_fires_alternative(self):
        """Alternative method: scan for fires by checking model names"""
        try:
            # Get list of topics to find fire models
            cmd = ['ign', 'topic', '-l']
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            fire_pattern = re.compile(r'test_fire\d+')
            fire_names = fire_pattern.findall(result.stdout)
            
            if fire_names:
                for name in set(fire_names):
                    if name not in self.fire_entities:
                        self.fire_entities[name] = None
                        self.get_logger().info(f'  Found via topics: {name}')
        except Exception as e:
            self.get_logger().warn(f'Alternative scan method failed: {str(e)}')

    def find_fire_entity_name(self, fire_position):
        """Find the fire entity name that matches the given position"""
        if not self.fire_entities:
            self.get_logger().warn('No fire entities registered. Cannot match fire to entity.')
            return None
        
        # If we have positions stored, use distance matching
        closest_name = None
        closest_dist = float('inf')
        
        for name, pos in self.fire_entities.items():
            if pos is None:
                # No position stored yet, update it with current fire position
                # and assume this is the fire we're looking for
                self.fire_entities[name] = (fire_position.x, fire_position.y, fire_position.z)
                self.get_logger().info(
                    f'Associated {name} with position '
                    f'({fire_position.x:.2f}, {fire_position.y:.2f})'
                )
                return name
            
            # Calculate distance
            dist = math.sqrt(
                (fire_position.x - pos[0])**2 +
                (fire_position.y - pos[1])**2
            )
            
            if dist < closest_dist and dist < self.fire_match_tolerance:
                closest_dist = dist
                closest_name = name
        
        if closest_name:
            self.get_logger().info(
                f'Matched fire at ({fire_position.x:.2f}, {fire_position.y:.2f}) '
                f'to entity "{closest_name}" (distance: {closest_dist:.2f}m)'
            )
        else:
            # If no match found, try to find an unassociated fire entity
            for name, pos in self.fire_entities.items():
                if pos is None:
                    self.fire_entities[name] = (fire_position.x, fire_position.y, fire_position.z)
                    self.get_logger().info(
                        f'Associated {name} with position '
                        f'({fire_position.x:.2f}, {fire_position.y:.2f}) (no close match)'
                    )
                    return name
            
            self.get_logger().warn(
                f'No fire entity found matching position '
                f'({fire_position.x:.2f}, {fire_position.y:.2f}) '
                f'within {self.fire_match_tolerance}m tolerance'
            )
        
        return closest_name

    def delete_fire_entity(self, fire_name):
        """Delete a fire entity from the simulation using ign service"""
        if not fire_name:
            return False
        
        self.get_logger().info(f'Attempting to delete fire entity: {fire_name}')
        
        try:
            # Call ign service to remove the entity
            cmd = [
                'ign', 'service',
                '-s', '/world/bushland_world/remove',
                '--reqtype', 'ignition.msgs.Entity',
                '--reptype', 'ignition.msgs.Boolean',
                '--req', f'name: "{fire_name}" type: 2',
                '--timeout', '2000'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0 and 'data: true' in result.stdout.lower():
                self.get_logger().info(f'✓ Successfully deleted {fire_name} from simulation!')
                # Remove from our tracking dictionary
                if fire_name in self.fire_entities:
                    del self.fire_entities[fire_name]
                return True
            else:
                self.get_logger().error(f'✗ Failed to delete {fire_name} from simulation')
                self.get_logger().error(f'  Output: {result.stdout}')
                self.get_logger().error(f'  Error: {result.stderr}')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Timeout while trying to delete {fire_name}')
            return False
        except Exception as e:
            self.get_logger().error(f'Exception while deleting {fire_name}: {str(e)}')
            return False

    def fire_callback(self, msg):
        """Update list of detected fires from drone"""
        self.last_fire_update_time = time.time()

        # Update fire list (replace with new data)
        new_fires = []
        for pose in msg.poses:
            fire_pos = (pose.position.x, pose.position.y, pose.position.z)
            # Only add if not already extinguished
            if not self.is_fire_extinguished(fire_pos):
                new_fires.append(pose)

        # Check for new fires
        if len(new_fires) > len(self.detected_fires):
            num_new = len(new_fires) - len(self.detected_fires)
            self.get_logger().info(
                f'Received {num_new} NEW fire(s)! Total: {len(new_fires)} '
                f'[Current state: {self.state.name}]'
            )

        self.detected_fires = new_fires

        # If we're IDLE and have fires, transition to PLANNING
        if self.state == FirefighterState.IDLE and len(self.detected_fires) > 0:
            if self.auto_start:
                self.get_logger().info('Fires detected! Starting firefighting operations.')
                self.transition_to_state(FirefighterState.PLANNING)

    def odom_callback(self, msg):
        """Update robot position and velocity"""
        self.robot_position = msg.pose.pose.position
        self.robot_orientation = msg.pose.pose.orientation

        # Extract velocity
        self.robot_linear_x = msg.twist.twist.linear.x
        self.robot_linear_y = msg.twist.twist.linear.y
        self.robot_velocity = math.sqrt(self.robot_linear_x**2 + self.robot_linear_y**2)

    def is_fire_extinguished(self, fire_pos):
        """Check if a fire has already been extinguished"""
        for ext_fire in self.extinguished_fires:
            dist = math.sqrt(
                (fire_pos[0] - ext_fire[0])**2 +
                (fire_pos[1] - ext_fire[1])**2
            )
            if dist < 0.5:  # Within 0.5m = same fire
                return True
        return False

    def check_stuck_near_fire(self):
        """Check if robot is stuck (low velocity) but close enough to fire to proceed"""
        if self.current_target_fire is None or self.robot_position is None:
            return False

        # Grace period: Don't check stuck detection immediately after state transition
        time_in_state = time.time() - self.state_start_time
        if time_in_state < 3.0:  # Give robot 3 seconds to start moving
            return False

        # Calculate distance to fire
        dist_to_fire = math.sqrt(
            (self.current_target_fire.position.x - self.robot_position.x)**2 +
            (self.current_target_fire.position.y - self.robot_position.y)**2
        )

        # Check if velocity is low
        if self.robot_velocity < self.stuck_velocity_threshold:
            # Start stuck timer if not already started
            if self.stuck_start_time is None:
                self.stuck_start_time = time.time()
                self.get_logger().info(
                    f'Robot appears stuck (vel={self.robot_velocity:.3f} m/s). '
                    f'Monitoring for {self.stuck_timeout}s...'
                )

            # Check if stuck for long enough
            stuck_duration = time.time() - self.stuck_start_time

            if stuck_duration > self.stuck_timeout:
                # Check if we're close enough to the fire
                if dist_to_fire < self.fallback_distance:
                    self.get_logger().info(
                        f'Stuck but close enough! Distance to fire: {dist_to_fire:.2f}m '
                        f'(threshold: {self.fallback_distance}m)'
                    )
                    self.stuck_start_time = None
                    return True
                else:
                    self.get_logger().warn(
                        f'Stuck and too far! Distance: {dist_to_fire:.2f}m, '
                        f'threshold: {self.fallback_distance}m. Giving up on this fire.'
                    )
                    self.stuck_start_time = None
                    # Return special code to skip this fire
                    return 'skip_fire'
        else:
            # Robot is moving, reset stuck timer
            if self.stuck_start_time is not None:
                self.get_logger().info(f'Robot moving again (vel={self.robot_velocity:.3f} m/s)')
            self.stuck_start_time = None

        return False

    def transition_to_state(self, new_state):
        """Transition to a new state"""
        if new_state != self.state:
            self.previous_state = self.state
            self.state = new_state
            self.state_start_time = time.time()

            # Reset stuck detection on state transitions
            self.stuck_start_time = None
            
            # Clear navigation state when leaving navigation state
            if self.previous_state == FirefighterState.NAVIGATING:
                if new_state != FirefighterState.NAVIGATING:
                    self.navigation_result = None

            self.get_logger().info(f'STATE TRANSITION: {self.previous_state.name} -> {self.state.name}')

    def state_machine_update(self):
        """Main state machine update loop"""
        if self.robot_position is None:
            return  # Wait for robot position

        # State machine logic
        if self.state == FirefighterState.IDLE:
            self.handle_idle_state()
        elif self.state == FirefighterState.PLANNING:
            self.handle_planning_state()
        elif self.state == FirefighterState.NAVIGATING:
            self.handle_navigating_state()
        elif self.state == FirefighterState.EXTINGUISHING:
            self.handle_extinguishing_state()
        elif self.state == FirefighterState.COMPLETED:
            self.handle_completed_state()
        elif self.state == FirefighterState.BACKING_UP:
            self.handle_backing_up_state()

    def handle_idle_state(self):
        """IDLE state: Wait for fires to appear"""
        # Check if enough time has passed and new fires available
        time_in_state = time.time() - self.state_start_time

        if len(self.detected_fires) > 0 and time_in_state > self.min_wait_time:
            if self.auto_start:
                self.transition_to_state(FirefighterState.PLANNING)

    def handle_planning_state(self):
        """PLANNING state: Sort and prioritize fires"""
        # Check cooldown after skipping a fire
        time_since_skip = time.time() - self.last_skip_time
        if time_since_skip < self.skip_cooldown:
            return  # Wait before replanning
        
        if len(self.detected_fires) == 0:
            self.get_logger().info('No fires to extinguish. Returning to IDLE.')
            self.transition_to_state(FirefighterState.IDLE)
            return

        # Sort fires by distance from robot (closest first)
        fires_with_dist = []
        for fire in self.detected_fires:
            dist = math.sqrt(
                (fire.position.x - self.robot_position.x)**2 +
                (fire.position.y - self.robot_position.y)**2
            )
            fires_with_dist.append((dist, fire))

        fires_with_dist.sort(key=lambda x: x[0])
        self.fire_queue = [fire for _, fire in fires_with_dist]

        self.get_logger().info(f'Planned route to {len(self.fire_queue)} fire(s)')
        for i, fire in enumerate(self.fire_queue[:3]):  # Show first 3
            dist = fires_with_dist[i][0]
            self.get_logger().info(
                f'  Fire {i+1}: ({fire.position.x:.1f}, {fire.position.y:.1f}) - {dist:.1f}m away'
            )

        # Select first fire and navigate
        if len(self.fire_queue) > 0:
            self.current_target_fire = self.fire_queue.pop(0)
            self.approach_attempts = 0  # Reset approach attempts for new fire
            self.backup_attempts = 0  # Reset backup attempts for new fire
            self.transition_to_state(FirefighterState.NAVIGATING)


    def handle_navigating_state(self):
        """NAVIGATING state: Move to fire location"""
        if self.current_target_fire is None:
            self.transition_to_state(FirefighterState.PLANNING)
            return

        # Check if stuck but close enough to fire
        stuck_check = self.check_stuck_near_fire()
        if stuck_check == True:
            # Close enough to proceed
            self.get_logger().info('Stuck detection: Close enough to fire, proceeding to extinguish!')
            # Cancel current goal if active
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            self.navigation_result = None
            self.transition_to_state(FirefighterState.EXTINGUISHING)
            return
        elif stuck_check == 'skip_fire':
            # Too far, skip this fire and try next
            self.get_logger().warn('Skipping unreachable fire, moving to next target.')
            self.last_skip_time = time.time()
            # Cancel current goal if active
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            self.navigation_result = None
            self.current_target_fire = None
            self.transition_to_state(FirefighterState.PLANNING)
            return

        # Check if we need to send a goal
        if self.current_goal_handle is None and self.navigation_result is None:
            # Calculate approach position
            approach_pose = self.calculate_approach_pose(self.current_target_fire)
            self.send_navigation_goal(approach_pose)
            return

        # Check navigation status - only if result is set
        if self.navigation_result is not None:
            if self.navigation_result:
                # Trust Nav2's goal checker - go directly to EXTINGUISHING
                dist_to_fire = math.sqrt(
                    (self.current_target_fire.position.x - self.robot_position.x)**2 +
                    (self.current_target_fire.position.y - self.robot_position.y)**2
                )
                self.get_logger().info(
                    f'Navigation successful! Distance to fire: {dist_to_fire:.2f}m. Starting extinguish!'
                )
                self.approach_attempts = 0  # Reset for next fire
                self.backup_attempts = 0  # Reset backup attempts
                self.transition_to_state(FirefighterState.EXTINGUISHING)
            else:
                # Navigation failed - try alternative approach angle if enabled
                if self.enable_multi_angle_approach:
                    self.approach_attempts += 1
                    if self.approach_attempts < self.max_approach_attempts:
                        self.get_logger().warn(
                            f'Navigation failed! Trying alternative approach angle '
                            f'({self.approach_attempts}/{self.max_approach_attempts})'
                        )
                        # Reset navigation state and try again
                        self.current_goal_handle = None
                        self.navigation_result = None
                        return  # Will retry on next update

                # Multi-angle disabled or all angles exhausted - try backup or skip
                should_try_backup = False
                skip_reason = ""

                if self.enable_multi_angle_approach and self.approach_attempts >= self.max_approach_attempts:
                    # All angles tried
                    should_try_backup = True
                    skip_reason = f'All {self.max_approach_attempts} approach angles'
                elif not self.enable_multi_angle_approach:
                    # Multi-angle disabled, first failure
                    should_try_backup = True
                    skip_reason = 'Navigation'

                if should_try_backup:
                    # Try backing up if enabled
                    if self.enable_backup_recovery and self.backup_attempts < self.max_backup_attempts:
                        self.get_logger().warn(
                            f'{skip_reason} failed! '
                            f'Attempting backup recovery ({self.backup_attempts + 1}/{self.max_backup_attempts})...'
                        )
                        self.approach_attempts = 0  # Reset for retry after backup
                        self.transition_to_state(FirefighterState.BACKING_UP)
                    else:
                        # Skip this fire
                        if self.enable_backup_recovery:
                            self.get_logger().warn(
                                f'{skip_reason} and {self.max_backup_attempts} backup attempts failed! '
                                f'Skipping this fire.'
                            )
                        elif self.enable_multi_angle_approach:
                            self.get_logger().warn(
                                f'{skip_reason} failed! (Backup recovery disabled) Skipping this fire.'
                            )
                        else:
                            self.get_logger().warn(
                                f'{skip_reason} failed! (Multi-angle and backup disabled) Skipping this fire.'
                            )
                        self.last_skip_time = time.time()
                        self.current_target_fire = None
                        self.approach_attempts = 0
                        self.backup_attempts = 0
                        self.transition_to_state(FirefighterState.PLANNING)

            # Reset for next goal
            self.current_goal_handle = None
            self.navigation_result = None

    def handle_extinguishing_state(self):
        """EXTINGUISHING state: Apply extinguishing action"""
        # Start extinguishing on first entry
        if self.extinguish_start_time is None:
            self.extinguish_start_time = time.time()
            fire_pos = self.current_target_fire.position
            self.get_logger().info(
                f'EXTINGUISHING FIRE at ({fire_pos.x:.1f}, {fire_pos.y:.1f})...'
            )

        # Publish extinguishing particle effect
        self.publish_extinguish_effect()

        # Check if extinguishing complete
        elapsed = time.time() - self.extinguish_start_time
        if elapsed >= self.extinguish_duration:
            # Mark fire as extinguished
            fire_pos = (
                self.current_target_fire.position.x,
                self.current_target_fire.position.y,
                self.current_target_fire.position.z
            )
            self.extinguished_fires.append(fire_pos)

            self.get_logger().info(
                f'Fire EXTINGUISHED! Total: {len(self.extinguished_fires)}'
            )

            # Find and delete the fire entity from simulation
            fire_name = self.find_fire_entity_name(self.current_target_fire.position)
            if fire_name:
                self.delete_fire_entity(fire_name)

                # Move robot slightly to force costmap update with new laser scans
                self.get_logger().info('Moving to refresh costmap (fire deleted)...')
                refresh_cmd = Twist()
                refresh_cmd.linear.x = -0.15  # Small backward movement
                refresh_cmd.angular.z = 0.0

                # Publish movement for 0.5 seconds
                for _ in range(5):
                    self.cmd_vel_pub.publish(refresh_cmd)
                    time.sleep(0.1)

                # Stop
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)

                # Give costmap a moment to process the new scans
                time.sleep(0.5)
                self.get_logger().info('Costmap refreshed')
            else:
                self.get_logger().warn('Could not find fire entity name to delete')

            # Reset extinguish timer
            self.extinguish_start_time = None
            self.current_target_fire = None

            # Check if more fires to handle
            if len(self.fire_queue) > 0:
                self.transition_to_state(FirefighterState.PLANNING)
            elif len(self.detected_fires) > len(self.extinguished_fires):
                # New fires detected, replan
                self.transition_to_state(FirefighterState.PLANNING)
            else:
                self.transition_to_state(FirefighterState.COMPLETED)

    def handle_completed_state(self):
        """COMPLETED state: All known fires extinguished"""
        # Check periodically if new fires have appeared
        time_in_state = time.time() - self.state_start_time

        if time_in_state > 5.0:  # Check every 5 seconds
            unhandled_fires = len(self.detected_fires) - len(self.extinguished_fires)
            if unhandled_fires > 0:
                self.get_logger().info(f'New fire(s) detected! Resuming operations.')
                self.transition_to_state(FirefighterState.PLANNING)
            else:
                # Still completed, reset timer
                self.state_start_time = time.time()

    def handle_backing_up_state(self):
        """BACKING_UP state: Recovery maneuver to clear obstacles"""
        time_in_state = time.time() - self.state_start_time
        backup_duration = 2.0  # Back up for 2 seconds

        if time_in_state < backup_duration:
            # Publish backup velocity command
            cmd = Twist()
            cmd.linear.x = -0.3  # Reverse at 0.3 m/s
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

            if time_in_state < 0.5:  # Log only once at start
                self.get_logger().info('Executing backup maneuver to clear obstacles...')
        else:
            # Stop the robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

            # Increment backup counter
            self.backup_attempts += 1

            self.get_logger().info(
                f'Backup complete. Retrying navigation (backup attempt {self.backup_attempts}/{self.max_backup_attempts})...'
            )

            # Return to navigating state to retry
            self.transition_to_state(FirefighterState.NAVIGATING)

    def calculate_approach_pose(self, fire_pose):
        """Calculate position to approach fire from (at target distance)

        Uses approach_attempts to try different angles around the fire
        if previous attempts failed.
        """
        # Calculate direction from robot to fire
        dx = fire_pose.position.x - self.robot_position.x
        dy = fire_pose.position.y - self.robot_position.y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 0.01:
            dist = 0.01  # Avoid division by zero

        # Base angle from robot to fire
        base_angle = math.atan2(dy, dx)

        # Calculate approach angle based on multi-angle setting
        if self.enable_multi_angle_approach:
            # Add offset based on attempt number to try different approach angles
            # Spread attempts around the fire at 45-degree intervals
            angle_offset = (self.approach_attempts * 2 * math.pi / self.max_approach_attempts)
            approach_angle = base_angle + math.pi + angle_offset  # Start from opposite side
        else:
            # Direct approach from current position
            approach_angle = base_angle + math.pi

        # Position at target_distance from fire at the calculated angle
        approach_x = fire_pose.position.x + math.cos(approach_angle) * self.target_distance
        approach_y = fire_pose.position.y + math.sin(approach_angle) * self.target_distance

        # Create approach pose facing the fire
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = 'map'
        approach_pose.header.stamp = self.get_clock().now().to_msg()
        approach_pose.pose.position.x = approach_x
        approach_pose.pose.position.y = approach_y
        approach_pose.pose.position.z = 0.0

        # Orientation should face the fire
        yaw_to_fire = math.atan2(
            fire_pose.position.y - approach_y,
            fire_pose.position.x - approach_x
        )

        approach_pose.pose.orientation.z = math.sin(yaw_to_fire / 2.0)
        approach_pose.pose.orientation.w = math.cos(yaw_to_fire / 2.0)

        # Log alternative angle attempts only if multi-angle is enabled
        if self.enable_multi_angle_approach and self.approach_attempts > 0:
            self.get_logger().info(
                f'Trying alternative approach angle {self.approach_attempts + 1}/{self.max_approach_attempts} '
                f'at ({approach_x:.2f}, {approach_y:.2f})'
            )

        return approach_pose

    def send_navigation_goal(self, pose_stamped):
        """Send navigation goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.get_logger().info(
            f'Sending goal: ({pose_stamped.pose.position.x:.2f}, '
            f'{pose_stamped.pose.position.y:.2f})'
        )

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        # Only process if we're still in NAVIGATING state
        if self.state != FirefighterState.NAVIGATING:
            self.get_logger().info('Ignoring stale navigation response from previous goal')
            return

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal was REJECTED by Nav2!')
            self.navigation_result = False
            return

        self.get_logger().info('Navigation goal ACCEPTED, waiting for result...')
        self.current_goal_handle = goal_handle

        # Now wait for the actual result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation completion"""
        # Only process if we're still in NAVIGATING state
        if self.state != FirefighterState.NAVIGATING:
            self.get_logger().info('Ignoring stale navigation result from previous goal')
            return

        result = future.result()
        self.navigation_result = (result.status == 4)  # 4 = SUCCEEDED

        if self.navigation_result:
            self.get_logger().info('Navigation goal reached!')
        else:
            self.get_logger().warn(f'Navigation goal failed with status: {result.status}')

    def publish_extinguish_effect(self):
        """Publish particle effect for extinguishing"""
        if self.current_target_fire is None or self.robot_position is None:
            return

        marker_array = MarkerArray()

        # Create spray cone particles - MORE VISIBLE
        num_particles = 50
        for i in range(num_particles):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'extinguish_particles'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Calculate particle position (spray from robot to fire)
            t = i / num_particles  # 0 to 1
            dx = self.current_target_fire.position.x - self.robot_position.x
            dy = self.current_target_fire.position.y - self.robot_position.y

            # Add animated spread based on time
            current_time = time.time()
            spread = 0.5
            offset_x = math.sin(current_time * 3 + i * 0.5) * spread * t
            offset_y = math.cos(current_time * 3 + i * 0.5) * spread * t

            marker.pose.position.x = self.robot_position.x + dx * t + offset_x
            marker.pose.position.y = self.robot_position.y + dy * t + offset_y
            marker.pose.position.z = 0.5 + math.sin(t * 3.14) * 0.8  # Arc upward

            marker.pose.orientation.w = 1.0

            # Larger particle size
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            # Bright blue water color
            marker.color.r = 0.1
            marker.color.g = 0.6
            marker.color.b = 1.0
            marker.color.a = 0.9

            # Longer lifetime for visibility
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000  # 0.5 seconds

            marker_array.markers.append(marker)

        # Add fire suppression marker at fire location - BIGGER AND MORE VISIBLE
        fire_marker = Marker()
        fire_marker.header.frame_id = 'map'
        fire_marker.header.stamp = self.get_clock().now().to_msg()
        fire_marker.ns = 'fire_suppression'
        fire_marker.id = 999
        fire_marker.type = Marker.CYLINDER
        fire_marker.action = Marker.ADD

        fire_marker.pose.position = self.current_target_fire.position
        fire_marker.pose.position.z = 1.0
        fire_marker.pose.orientation.w = 1.0

        # Animated pulsing effect
        pulse = 1.0 + 0.3 * math.sin(time.time() * 5)
        fire_marker.scale.x = 1.5 * pulse
        fire_marker.scale.y = 1.5 * pulse
        fire_marker.scale.z = 2.0

        # Bright white/blue suppression effect
        fire_marker.color.r = 0.9
        fire_marker.color.g = 0.95
        fire_marker.color.b = 1.0
        fire_marker.color.a = 0.7

        fire_marker.lifetime.sec = 0
        fire_marker.lifetime.nanosec = 500000000  # 0.5 seconds

        marker_array.markers.append(fire_marker)

        # Add a ground splash effect
        splash_marker = Marker()
        splash_marker.header.frame_id = 'map'
        splash_marker.header.stamp = self.get_clock().now().to_msg()
        splash_marker.ns = 'fire_suppression'
        splash_marker.id = 1000
        splash_marker.type = Marker.CYLINDER
        splash_marker.action = Marker.ADD

        splash_marker.pose.position = self.current_target_fire.position
        splash_marker.pose.position.z = 0.05
        splash_marker.pose.orientation.w = 1.0

        # Ground splash
        splash_marker.scale.x = 2.0
        splash_marker.scale.y = 2.0
        splash_marker.scale.z = 0.1

        # Blue water on ground
        splash_marker.color.r = 0.2
        splash_marker.color.g = 0.5
        splash_marker.color.b = 0.9
        splash_marker.color.a = 0.6

        splash_marker.lifetime.sec = 0
        splash_marker.lifetime.nanosec = 500000000  # 0.5 seconds

        marker_array.markers.append(splash_marker)

        self.extinguish_marker_pub.publish(marker_array)

    def publish_status(self):
        """Publish status text marker"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'firefighter_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position above robot
        if self.robot_position:
            marker.pose.position.x = self.robot_position.x
            marker.pose.position.y = self.robot_position.y
            marker.pose.position.z = 2.0
        else:
            marker.pose.position.z = 2.0

        marker.pose.orientation.w = 1.0

        # Status text
        status_text = f'State: {self.state.name}\n'
        status_text += f'Fires: {len(self.detected_fires)} detected, '
        status_text += f'{len(self.extinguished_fires)} extinguished'

        marker.text = status_text

        marker.scale.z = 0.3  # Text height

        # Color based on state
        if self.state == FirefighterState.EXTINGUISHING:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif self.state == FirefighterState.NAVIGATING:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif self.state == FirefighterState.COMPLETED:
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.g = 1.0

        marker.color.a = 1.0

        marker.lifetime.sec = 3

        self.status_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    try:
        firefighter = AutonomousFirefighter()
        rclpy.spin(firefighter)
    except KeyboardInterrupt:
        pass
    finally:
        if 'firefighter' in locals():
            firefighter.get_logger().info('=== Firefighting Operations Complete ===')
            firefighter.get_logger().info(
                f'Total fires extinguished: {len(firefighter.extinguished_fires)}'
            )
            firefighter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()