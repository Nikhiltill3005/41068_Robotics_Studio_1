#!/usr/bin/env python3

"""
Drone Teleop Control Script

This script provides keyboard control for the firefighting drone in Ignition Gazebo.
Controls:
- WASD: Move forward/back and strafe left/right
- QE: Rotate left/right (yaw)
- Space/Shift: Move up/down (altitude)
- R: Reset (hover)
- ESC/Ctrl+C: Exit

The script publishes thrust commands to individual rotors to achieve desired movements.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import threading
import time

class DroneTelopControl(Node):
    def __init__(self):
        super().__init__('drone_teleop_control')
        
        # Publishers for each thruster
        self.thrust_0_pub = self.create_publisher(Float64, '/firefighting_drone/thrust_0', 10)
        self.thrust_1_pub = self.create_publisher(Float64, '/firefighting_drone/thrust_1', 10)
        self.thrust_2_pub = self.create_publisher(Float64, '/firefighting_drone/thrust_2', 10)
        self.thrust_3_pub = self.create_publisher(Float64, '/firefighting_drone/thrust_3', 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Current command states
        self.thrust_commands = [0.0, 0.0, 0.0, 0.0]  # [front_right, front_left, back_left, back_right]
        self.base_thrust = 6.0  # Base thrust to hover (approximately 1.5kg * 9.81 / 4)
        self.max_thrust_delta = 3.0  # Maximum thrust variation for control
        
        # Control inputs
        self.forward_cmd = 0.0
        self.right_cmd = 0.0
        self.up_cmd = 0.0
        self.yaw_cmd = 0.0
        
        # Running flag
        self.running = True
        
        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("Drone Teleop Control Started!")
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("FIREFIGHTING DRONE TELEOP CONTROL")
        print("="*50)
        print("Movement Controls:")
        print("  W/S - Forward/Backward")
        print("  A/D - Strafe Left/Right")
        print("  Q/E - Rotate Left/Right (Yaw)")
        print("  Space/Shift - Up/Down")
        print("  R - Reset (Hover)")
        print("  ESC/Ctrl+C - Exit")
        print("="*50)
        print("Current Status: Ready for input")
        print("Base thrust: {:.1f}N (hover)".format(self.base_thrust))
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
        """Main control loop that converts commands to thruster values"""
        if not self.running:
            return
            
        # Calculate individual thruster commands
        # Drone layout:
        #   0(FR)  1(FL)
        #      \ /
        #       X
        #      / \
        #   3(BR)  2(BL)
        
        # Base thrust for all rotors (hover)
        thrust_0 = self.base_thrust + self.up_cmd  # Front Right
        thrust_1 = self.base_thrust + self.up_cmd  # Front Left  
        thrust_2 = self.base_thrust + self.up_cmd  # Back Left
        thrust_3 = self.base_thrust + self.up_cmd  # Back Right
        
        # Forward/Backward: adjust front and back rotors
        thrust_0 -= self.forward_cmd  # Front right
        thrust_1 -= self.forward_cmd  # Front left
        thrust_2 += self.forward_cmd  # Back left
        thrust_3 += self.forward_cmd  # Back right
        
        # Left/Right: adjust left and right rotors
        thrust_0 += self.right_cmd   # Front right
        thrust_1 -= self.right_cmd   # Front left
        thrust_2 -= self.right_cmd   # Back left
        thrust_3 += self.right_cmd   # Back right
        
        # Yaw: create torque by adjusting diagonal rotors
        thrust_0 += self.yaw_cmd     # Front right
        thrust_1 -= self.yaw_cmd     # Front left
        thrust_2 += self.yaw_cmd     # Back left
        thrust_3 -= self.yaw_cmd     # Back right
        
        # Clamp thrust values
        thrust_0 = max(0, min(20.0, thrust_0))
        thrust_1 = max(0, min(20.0, thrust_1))
        thrust_2 = max(0, min(20.0, thrust_2))
        thrust_3 = max(0, min(20.0, thrust_3))
        
        # Publish thrust commands
        msg_0 = Float64()
        msg_0.data = float(thrust_0)
        self.thrust_0_pub.publish(msg_0)
        
        msg_1 = Float64()
        msg_1.data = float(thrust_1)
        self.thrust_1_pub.publish(msg_1)
        
        msg_2 = Float64()
        msg_2.data = float(thrust_2)
        self.thrust_2_pub.publish(msg_2)
        
        msg_3 = Float64()
        msg_3.data = float(thrust_3)
        self.thrust_3_pub.publish(msg_3)
        
    def process_key(self, key):
        """Process keyboard input"""
        # Movement controls
        if key.lower() == 'w':
            self.forward_cmd = self.max_thrust_delta
            print("Moving Forward")
        elif key.lower() == 's':
            self.forward_cmd = -self.max_thrust_delta
            print("Moving Backward")
        elif key.lower() == 'a':
            self.right_cmd = -self.max_thrust_delta
            print("Moving Left")
        elif key.lower() == 'd':
            self.right_cmd = self.max_thrust_delta
            print("Moving Right")
        elif key.lower() == 'q':
            self.yaw_cmd = -self.max_thrust_delta
            print("Rotating Left")
        elif key.lower() == 'e':
            self.yaw_cmd = self.max_thrust_delta
            print("Rotating Right")
        elif key == ' ':  # Space
            self.up_cmd = self.max_thrust_delta
            print("Moving Up")
        elif key.lower() == 'z':  # Using 'z' instead of shift for down
            self.up_cmd = -self.max_thrust_delta
            print("Moving Down")
        elif key.lower() == 'r':
            # Reset to hover
            self.forward_cmd = 0.0
            self.right_cmd = 0.0
            self.up_cmd = 0.0
            self.yaw_cmd = 0.0
            print("Reset - Hovering")
        elif key == '\x1b':  # ESC
            print("Exiting...")
            self.running = False
            return False
        else:
            # Stop movement for any other key
            self.forward_cmd = 0.0
            self.right_cmd = 0.0
            self.up_cmd = 0.0
            self.yaw_cmd = 0.0
            
        return True
        
    def keyboard_thread(self):
        """Thread function for keyboard input"""
        while self.running:
            try:
                key = self.get_key()
                if not self.process_key(key):
                    break
            except KeyboardInterrupt:
                print("\nCtrl+C pressed. Exiting...")
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
            print("\nShutting down...")
        finally:
            self.running = False
            # Reset thrust to zero
            self.forward_cmd = 0.0
            self.right_cmd = 0.0
            self.up_cmd = 0.0
            self.yaw_cmd = 0.0
            
            # Send zero thrust
            zero_msg = Float64()
            zero_msg.data = float(0.0)
            for _ in range(5):  # Send several times to ensure delivery
                self.thrust_0_pub.publish(zero_msg)
                self.thrust_1_pub.publish(zero_msg)
                self.thrust_2_pub.publish(zero_msg)
                self.thrust_3_pub.publish(zero_msg)
                time.sleep(0.1)
                
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        drone_teleop = DroneTelopControl()
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
