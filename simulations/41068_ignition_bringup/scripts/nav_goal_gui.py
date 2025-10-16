#!/usr/bin/env python3
"""
Navigation Goal GUI for Husky Robot

A simple GUI application that allows setting navigation goals for the Husky robot
using Nav2. The GUI provides input fields for X, Y coordinates and orientation,
and sends navigation goals to the Nav2 stack.

Usage:
    python3 nav_goal_gui.py

Requirements:
    - ROS2 Humble
    - Nav2 stack running
    - Husky robot with proper namespace (/husky/)
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tkinter as tk
from tkinter import ttk, messagebox
import math


class NavGoalGUI(Node):
    def __init__(self):
        super().__init__('nav_goal_gui')
        
        # Create action client for navigation
        self.nav_action_client = ActionClient(self, NavigateToPose, '/husky/navigate_to_pose')
        
        # Wait for action server to be available
        self.get_logger().info('Waiting for navigation action server...')
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available!')
            messagebox.showerror("Error", "Navigation action server not available!\nMake sure Nav2 is running.")
            sys.exit(1)
        
        self.get_logger().info('Navigation action server connected!')
        
        # Create GUI
        self.setup_gui()
        
        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def setup_gui(self):
        """Setup the GUI components"""
        self.root = tk.Tk()
        self.root.title("Husky Navigation Goal Controller")
        self.root.geometry("420x320")
        self.root.resizable(False, False)
        # Forest green theme
        BG = '#0f1f14'
        PANEL = '#152a1d'
        SURFACE = '#1b3a28'
        TEXT = '#e6f4ea'
        ACCENT = '#2e7d32'
        ACCENT_ACTIVE = '#1b5e20'
        self.root.configure(bg=BG)
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass
        style.configure('TFrame', background=BG)
        style.configure('TLabelframe', background=PANEL, foreground=TEXT, padding=8)
        style.configure('TLabelframe.Label', background=PANEL, foreground=TEXT, font=('Arial', 11, 'bold'))
        style.configure('TLabel', background=BG, foreground=TEXT)
        style.configure('Accent.TButton', background=ACCENT, foreground=TEXT)
        style.map('Accent.TButton', background=[('active', ACCENT_ACTIVE)])
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="Husky Navigation Goals", 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # X coordinate
        ttk.Label(main_frame, text="X Coordinate:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.x_var = tk.StringVar(value="0.0")
        x_entry = ttk.Entry(main_frame, textvariable=self.x_var, width=15)
        x_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        
        # Y coordinate
        ttk.Label(main_frame, text="Y Coordinate:").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.y_var = tk.StringVar(value="0.0")
        y_entry = ttk.Entry(main_frame, textvariable=self.y_var, width=15)
        y_entry.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        
        # Orientation (yaw in degrees)
        ttk.Label(main_frame, text="Orientation (degrees):").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.yaw_var = tk.StringVar(value="0.0")
        yaw_entry = ttk.Entry(main_frame, textvariable=self.yaw_var, width=15)
        yaw_entry.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=5, padx=(10, 0))
        
        # Quick preset buttons
        preset_frame = ttk.LabelFrame(main_frame, text="Quick Presets", padding="5")
        preset_frame.grid(row=4, column=0, columnspan=2, pady=20, sticky=(tk.W, tk.E))
        
        # Preset buttons
        presets = [
            ("Origin (0,0)", 0.0, 0.0, 0.0),
            ("Forward (5,0)", 5.0, 0.0, 0.0),
            ("Right (0,5)", 0.0, 5.0, 90.0),
            ("Back (-5,0)", -5.0, 0.0, 180.0),
            ("Left (0,-5)", 0.0, -5.0, -90.0)
        ]
        
        for i, (name, x, y, yaw) in enumerate(presets):
            btn = ttk.Button(preset_frame, text=name, 
                           command=lambda x=x, y=y, yaw=yaw: self.set_preset(x, y, yaw),
                           style='Accent.TButton')
            btn.grid(row=i//3, column=i%3, padx=2, pady=2, sticky=(tk.W, tk.E))
        
        # Control buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=5, column=0, columnspan=2, pady=20)
        
        # Navigate button
        self.navigate_btn = ttk.Button(button_frame, text="Navigate to Goal", 
                                     command=self.send_navigation_goal, 
                                     style='Accent.TButton')
        self.navigate_btn.pack(side=tk.LEFT, padx=5)
        
        # Cancel button
        self.cancel_btn = ttk.Button(button_frame, text="Cancel Navigation", 
                                   command=self.cancel_navigation)
        self.cancel_btn.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(main_frame, textvariable=self.status_var)
        status_label.grid(row=6, column=0, columnspan=2, pady=10)
        
        # Configure grid weights
        main_frame.columnconfigure(1, weight=1)
        preset_frame.columnconfigure(0, weight=1)
        preset_frame.columnconfigure(1, weight=1)
        preset_frame.columnconfigure(2, weight=1)
    
    def set_preset(self, x, y, yaw):
        """Set preset values"""
        self.x_var.set(str(x))
        self.y_var.set(str(y))
        self.yaw_var.set(str(yaw))
    
    def send_navigation_goal(self):
        """Send navigation goal to Nav2"""
        try:
            # Parse input values
            x = float(self.x_var.get())
            y = float(self.y_var.get())
            yaw_deg = float(self.yaw_var.get())
            yaw_rad = math.radians(yaw_deg)
            
            # Create goal message
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            # Set position
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.pose.position.z = 0.0
            
            # Set orientation (convert yaw to quaternion)
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
            
            # Send goal
            self.get_logger().info(f'Sending navigation goal: x={x}, y={y}, yaw={yaw_deg}°')
            self.status_var.set("Sending goal...")
            self.navigate_btn.config(state='disabled')
            
            # Send goal asynchronously
            self.send_goal_future = self.nav_action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
            
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input values: {e}")
            self.status_var.set("Error: Invalid input")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send goal: {e}")
            self.status_var.set("Error sending goal")
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            self.status_var.set("Goal rejected")
            self.navigate_btn.config(state='normal')
            return
        
        self.get_logger().info('Goal accepted by server')
        self.status_var.set("Navigating...")
        
        # Get result
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.status_var.set(f"Navigating... Distance remaining: {distance:.2f}m")
    
    def result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.status_var.set("Navigation completed")
        self.navigate_btn.config(state='normal')
    
    def cancel_navigation(self):
        """Cancel current navigation"""
        if hasattr(self, 'send_goal_future') and self.send_goal_future.done():
            goal_handle = self.send_goal_future.result()
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
                self.get_logger().info('Navigation cancelled')
                self.status_var.set("Navigation cancelled")
                self.navigate_btn.config(state='normal')
    
    def timer_callback(self):
        """Timer callback for ROS2 spinning"""
        rclpy.spin_once(self, timeout_sec=0.01)
    
    def run(self):
        """Run the GUI"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down...')
        finally:
            self.destroy_node()


def main():
    """Main function"""
    rclpy.init()
    
    try:
        gui = NavGoalGUI()
        gui.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
