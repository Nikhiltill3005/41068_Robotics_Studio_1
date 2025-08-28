#!/usr/bin/env python3

"""
Drone Teleop Control Script for Ignition Gazebo

This script provides keyboard control for the firefighting drone using direct
Ignition Gazebo wrench commands (force/torque application).

Controls:
- WASD: Move forward/back and strafe left/right
- QE: Rotate left/right (yaw)
- Space/Z: Move up/down (altitude)
- R: Reset (hover)
- ESC/Ctrl+C: Exit

The script publishes wrench commands directly to Ignition Gazebo.
"""

import subprocess
import sys
import termios
import tty
import threading
import time
import json

class DroneTelopIgnition:
    def __init__(self):
        # Current command states
        self.forward_force = 0.0
        self.right_force = 0.0
        self.up_force = 0.0
        self.yaw_torque = 0.0
        
        # Control parameters
        self.base_lift_force = 15.0  # Force needed to hover (approximately 1.5kg * 9.81)
        self.max_control_force = 8.0  # Maximum additional force for control
        self.max_torque = 5.0  # Maximum torque for rotation
        
        # Running flag
        self.running = True
        
        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        print("Drone Teleop Control Started (Ignition Direct)!")
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("FIREFIGHTING DRONE TELEOP CONTROL")
        print("="*50)
        print("Movement Controls:")
        print("  W/S - Forward/Backward")
        print("  A/D - Strafe Left/Right")
        print("  Q/E - Rotate Left/Right (Yaw)")
        print("  Space/Z - Up/Down")
        print("  R - Reset (Hover)")
        print("  ESC/Ctrl+C - Exit")
        print("="*50)
        print("Current Status: Ready for input")
        print("Base lift force: {:.1f}N (hover)".format(self.base_lift_force))
        print("\nPress keys to control the drone...")
        
    def get_key(self):
        """Get a single keypress from terminal"""
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def send_wrench_command(self, fx=0.0, fy=0.0, fz=0.0, tx=0.0, ty=0.0, tz=0.0):
        """Send wrench command to Ignition Gazebo"""
        try:
            # Construct the message
            wrench_msg = f'entity: {{name: "firefighting_drone"}}, wrench: {{force: {{x: {fx}, y: {fy}, z: {fz}}}, torque: {{x: {tx}, y: {ty}, z: {tz}}}}}'
            
            # Send command using ign topic
            cmd = [
                'ign', 'topic', 
                '-t', '/world/drone_test_world/wrench',
                '-m', 'ignition.msgs.EntityWrench',
                '-p', wrench_msg
            ]
            
            subprocess.run(cmd, capture_output=True, timeout=0.1)
        except subprocess.TimeoutExpired:
            pass  # Ignore timeout - command was sent
        except Exception as e:
            print(f"Error sending command: {e}")
            
    def control_loop(self):
        """Main control loop that sends wrench commands"""
        while self.running:
            try:
                # Calculate total forces
                total_fx = self.right_force    # Right is positive X
                total_fy = self.forward_force  # Forward is positive Y  
                total_fz = self.base_lift_force + self.up_force  # Up is positive Z
                
                # Calculate torques (only yaw for now)
                total_tx = 0.0
                total_ty = 0.0
                total_tz = self.yaw_torque
                
                # Send wrench command
                self.send_wrench_command(total_fx, total_fy, total_fz, total_tx, total_ty, total_tz)
                
                # Sleep to avoid overwhelming the system
                time.sleep(0.1)
                
            except Exception as e:
                print(f"Error in control loop: {e}")
                break
                
    def process_key(self, key):
        """Process keyboard input"""
        # Movement controls
        if key.lower() == 'w':
            self.forward_force = self.max_control_force
            print("Moving Forward")
        elif key.lower() == 's':
            self.forward_force = -self.max_control_force
            print("Moving Backward")
        elif key.lower() == 'a':
            self.right_force = -self.max_control_force
            print("Moving Left")
        elif key.lower() == 'd':
            self.right_force = self.max_control_force
            print("Moving Right")
        elif key.lower() == 'q':
            self.yaw_torque = -self.max_torque
            print("Rotating Left")
        elif key.lower() == 'e':
            self.yaw_torque = self.max_torque
            print("Rotating Right")
        elif key == ' ':  # Space
            self.up_force = self.max_control_force
            print("Moving Up")
        elif key.lower() == 'z':
            self.up_force = -self.max_control_force
            print("Moving Down")
        elif key.lower() == 'r':
            # Reset to hover
            self.forward_force = 0.0
            self.right_force = 0.0
            self.up_force = 0.0
            self.yaw_torque = 0.0
            print("Reset - Hovering")
        elif key == '\x1b':  # ESC
            print("Exiting...")
            self.running = False
            return False
        else:
            # Stop movement for any other key
            self.forward_force = 0.0
            self.right_force = 0.0
            self.up_force = 0.0
            self.yaw_torque = 0.0
            
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
        # Start control loop thread
        control_thread = threading.Thread(target=self.control_loop)
        control_thread.daemon = True
        control_thread.start()
        
        # Start keyboard input thread
        keyboard_thread = threading.Thread(target=self.keyboard_thread)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        try:
            # Keep main thread alive
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.running = False
            
            # Send zero force to stop the drone
            self.send_wrench_command(0, 0, 0, 0, 0, 0)
            
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    try:
        drone_teleop = DroneTelopIgnition()
        drone_teleop.run()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
