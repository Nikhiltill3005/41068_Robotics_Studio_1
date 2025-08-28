#!/usr/bin/env python3

"""
Drone Teleop Control Script for ROS2

This script provides keyboard control for the firefighting drone using ROS2 topics
that are bridged to Ignition Gazebo.

Controls:
- WASD: Move forward/back and strafe left/right
- QE: Rotate left/right (yaw)
- Space/Z: Move up/down (altitude)
- R: Reset (hover)
- ESC/Ctrl+C: Exit

The script publishes geometry_msgs/Twist commands to /drone/cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import time

class DroneTelopROS(Node):
    def __init__(self):
        super().__init__('drone_teleop_ros')
        
        # Publisher for drone velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Current command states
        self.linear_x = 0.0  # Forward/backward
        self.linear_y = 0.0  # Left/right (strafe)
        self.linear_z = 0.0  # Up/down
        self.angular_z = 0.0  # Yaw rotation
        
        # Control parameters
        self.max_linear_speed = 2.0   # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.max_vertical_speed = 1.0 # m/s
        
        # Running flag
        self.running = True
        
        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("Drone ROS2 Teleop Control Started!")
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("FIREFIGHTING DRONE ROS2 TELEOP CONTROL")
        print("="*50)
        print("Movement Controls:")
        print("  W/S - Forward/Backward")
        print("  A/D - Strafe Left/Right")
        print("  Q/E - Rotate Left/Right (Yaw)")
        print("  Space/Z - Up/Down")
        print("  R - Reset (Stop)")
        print("  ESC/Ctrl+C - Exit")
        print("="*50)
        print("Current Status: Ready for input")
        print("Publishing to: /drone/cmd_vel")
        print("Max speeds: Linear={:.1f}m/s, Angular={:.1f}rad/s, Vertical={:.1f}m/s".format(
            self.max_linear_speed, self.max_angular_speed, self.max_vertical_speed))
        print("\nPress keys to control the drone...")
        
    def get_key(self):
        """Get a single keypress from terminal"""
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def control_loop(self):
        """Main control loop that publishes Twist messages"""
        if not self.running:
            return
            
        # Create and populate Twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_x
        twist_msg.linear.y = self.linear_y
        twist_msg.linear.z = self.linear_z
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.angular_z
        
        # Publish the command
        self.cmd_vel_pub.publish(twist_msg)
        
    def process_key(self, key):
        """Process keyboard input"""
        # Movement controls
        if key.lower() == 'w':
            self.linear_x = self.max_linear_speed
            self.get_logger().info("Moving Forward")
        elif key.lower() == 's':
            self.linear_x = -self.max_linear_speed
            self.get_logger().info("Moving Backward")
        elif key.lower() == 'a':
            self.linear_y = self.max_linear_speed
            self.get_logger().info("Moving Left")
        elif key.lower() == 'd':
            self.linear_y = -self.max_linear_speed
            self.get_logger().info("Moving Right")
        elif key.lower() == 'q':
            self.angular_z = self.max_angular_speed
            self.get_logger().info("Rotating Left")
        elif key.lower() == 'e':
            self.angular_z = -self.max_angular_speed
            self.get_logger().info("Rotating Right")
        elif key == ' ':  # Space
            self.linear_z = self.max_vertical_speed
            self.get_logger().info("Moving Up")
        elif key.lower() == 'z':
            self.linear_z = -self.max_vertical_speed
            self.get_logger().info("Moving Down")
        elif key.lower() == 'r':
            # Reset to stop
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.linear_z = 0.0
            self.angular_z = 0.0
            self.get_logger().info("Reset - Stopped")
        elif key == '\x1b':  # ESC
            self.get_logger().info("Exiting...")
            self.running = False
            return False
        else:
            # Stop movement for any other key
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.linear_z = 0.0
            self.angular_z = 0.0
            
        return True
        
    def keyboard_thread(self):
        """Thread function for keyboard input"""
        while self.running:
            try:
                key = self.get_key()
                if not self.process_key(key):
                    break
            except KeyboardInterrupt:
                self.get_logger().info("Ctrl+C pressed. Exiting...")
                self.running = False
                break
            except:
                pass
                
    def run(self):
        """Main run function"""
        # Start keyboard input thread
        keyboard_thread = threading.Thread(target=self.keyboard_thread)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down...")
        finally:
            self.running = False
            
            # Send zero velocity
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.linear_z = 0.0
            self.angular_z = 0.0
            
            # Send stop command several times
            for _ in range(5):
                twist_msg = Twist()  # All zeros by default
                self.cmd_vel_pub.publish(twist_msg)
                time.sleep(0.1)
                
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        drone_teleop = DroneTelopROS()
        drone_teleop.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            drone_teleop.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
