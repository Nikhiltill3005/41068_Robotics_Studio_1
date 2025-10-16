#!/usr/bin/env python3
"""
Navigation GUI for Husky Robot

This GUI allows users to:
- Set navigation goals by clicking on a map or entering X,Y coordinates
- Send goals to Nav2 navigation stack
- Monitor robot position and navigation status
- View the current map from SLAM

Usage:
    ros2 run 41068_ignition_bringup navigation_gui.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import math
import numpy as np
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.patches as patches
from matplotlib.ticker import NullLocator


class NavigationGUI(Node):
    def __init__(self):
        super().__init__('navigation_gui')
        
        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/husky/odometry',
            self.odom_callback,
            10
        )
        
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.map_data = None
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        
        # Goal tracking
        self.current_goal = None
        self.goal_future = None
        
        self.get_logger().info('Navigation GUI initialized')
        
        # Start GUI in separate thread
        self.gui_thread = threading.Thread(target=self.create_gui, daemon=True)
        self.gui_thread.start()
    
    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Update GUI if it exists
        if hasattr(self, 'update_robot_display'):
            self.update_robot_display()
    
    def map_callback(self, msg):
        """Update map data"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        
        # Update map display if GUI exists
        if hasattr(self, 'update_map_display'):
            self.update_map_display()
    
    def send_goal(self, x, y, yaw=0.0):
        """Send navigation goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return False
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header = Header()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        
        # Send goal
        self.goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.goal_feedback_callback
        )
        self.goal_future.add_done_callback(self.goal_response_callback)
        
        self.current_goal = (x, y, yaw)
        return True
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_feedback_callback(self, feedback_msg):
        """Handle goal feedback"""
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining
        # Update GUI with feedback if needed
    
    def goal_result_callback(self, future):
        """Handle goal completion"""
        result = future.result().result
        if result:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().warn('Goal failed or was cancelled')
        
        self.current_goal = None
    
    def cancel_goal(self):
        """Cancel current navigation goal"""
        if self.goal_future and not self.goal_future.done():
            # Cancel the goal
            self.get_logger().info('Cancelling current goal...')
            self.current_goal = None
    
    def create_gui(self):
        """Create the main GUI window"""
        self.root = tk.Tk()
        self.root.title("Husky Navigation Control")
        self.root.geometry("800x600")
        self.root.configure(bg="#1B4332")
        
        # Configure styles
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Forest.TFrame", background="#2D5A3D")
        style.configure("Forest.TLabel", background="#2D5A3D", foreground="#F8F9FA", font=("Segoe UI", 10))
        style.configure("Forest.TLabelframe", background="#2D5A3D", foreground="#95D5B2")
        style.configure("Forest.TLabelframe.Label", background="#2D5A3D", foreground="#95D5B2", font=("Segoe UI", 10, "bold"))
        style.configure("Forest.TButton", background="#40916C", foreground="white", font=("Segoe UI", 9))
        style.map("Forest.TButton", background=[("active", "#52B788")])
        style.configure("Forest.TEntry", fieldbackground="#081C15", foreground="#F8F9FA", bordercolor="#40916C")
        
        # Create main frames
        control_frame = ttk.Frame(self.root, style="Forest.TFrame")
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        
        map_frame = ttk.Frame(self.root, style="Forest.TFrame")
        map_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Control panel
        ttk.Label(control_frame, text="Navigation Control", font=('Arial', 14, 'bold'), style="Forest.TLabel").pack(pady=10)
        
        # Robot status
        status_frame = ttk.LabelFrame(control_frame, text="Robot Status", style="Forest.TLabelframe")
        status_frame.pack(fill=tk.X, pady=5)
        
        self.robot_x_label = ttk.Label(status_frame, text="X: 0.00 m", style="Forest.TLabel")
        self.robot_x_label.pack(anchor=tk.W)
        
        self.robot_y_label = ttk.Label(status_frame, text="Y: 0.00 m", style="Forest.TLabel")
        self.robot_y_label.pack(anchor=tk.W)
        
        self.robot_yaw_label = ttk.Label(status_frame, text="Yaw: 0.00 rad", style="Forest.TLabel")
        self.robot_yaw_label.pack(anchor=tk.W)
        
        # Goal input
        goal_frame = ttk.LabelFrame(control_frame, text="Set Navigation Goal", style="Forest.TLabelframe")
        goal_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(goal_frame, text="X coordinate (m):", style="Forest.TLabel").pack(anchor=tk.W)
        self.x_entry = ttk.Entry(goal_frame, width=15, style="Forest.TEntry")
        self.x_entry.pack(fill=tk.X, pady=2)
        
        ttk.Label(goal_frame, text="Y coordinate (m):", style="Forest.TLabel").pack(anchor=tk.W)
        self.y_entry = ttk.Entry(goal_frame, width=15, style="Forest.TEntry")
        self.y_entry.pack(fill=tk.X, pady=2)
        
        ttk.Label(goal_frame, text="Yaw (rad, optional):", style="Forest.TLabel").pack(anchor=tk.W)
        self.yaw_entry = ttk.Entry(goal_frame, width=15, style="Forest.TEntry")
        self.yaw_entry.pack(fill=tk.X, pady=2)
        self.yaw_entry.insert(0, "0.0")
        
        # Buttons
        button_frame = ttk.Frame(goal_frame)
        button_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(button_frame, text="Send Goal", command=self.on_send_goal, style="Forest.TButton").pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="Cancel Goal", command=self.on_cancel_goal, style="Forest.TButton").pack(side=tk.LEFT, padx=2)
        
        # Quick goals
        quick_frame = ttk.LabelFrame(control_frame, text="Quick Goals", style="Forest.TLabelframe")
        quick_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(quick_frame, text="Origin (0,0)", command=lambda: self.set_quick_goal(0, 0), style="Forest.TButton").pack(fill=tk.X, pady=1)
        ttk.Button(quick_frame, text="Forward 5m", command=self.go_forward_5m, style="Forest.TButton").pack(fill=tk.X, pady=1)
        ttk.Button(quick_frame, text="Backward 5m", command=self.go_backward_5m, style="Forest.TButton").pack(fill=tk.X, pady=1)
        
        # Map display
        ttk.Label(map_frame, text="SLAM Map (Click to set goal)", font=('Arial', 12, 'bold'), style="Forest.TLabel").pack()
        
        # Create matplotlib figure for map
        self.fig = Figure(figsize=(6, 6), dpi=100, facecolor='#2D5A3D')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#1B4332')
        self.ax.set_aspect('equal')
        self.ax.set_title('Robot Map', color='#F8F9FA', fontweight='bold')
        self.ax.set_xlabel('X (m)', color='#F8F9FA')
        self.ax.set_ylabel('Y (m)', color='#F8F9FA')
        self.ax.tick_params(colors='#95D5B2')
        
        self.canvas = FigureCanvasTkAgg(self.fig, map_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Bind click event
        self.canvas.mpl_connect('button_press_event', self.on_map_click)
        
        # Initial map setup
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.grid(True, color='#40916C', alpha=0.5)
        
        # Robot marker
        self.robot_marker = self.ax.plot(0, 0, 's', color='#28A745', markersize=10, markeredgecolor='#F8F9FA', markeredgewidth=2, label='Robot')[0]
        self.goal_marker = None
        
        self.ax.legend(facecolor='#2D5A3D', edgecolor='#40916C', labelcolor='#F8F9FA')
        self.canvas.draw()
        
        # Start GUI update timer
        self.root.after(100, self.update_gui)
        
        # Start the GUI
        self.root.mainloop()
    
    def on_send_goal(self):
        """Handle send goal button click"""
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            yaw = float(self.yaw_entry.get()) if self.yaw_entry.get() else 0.0
            
            if self.send_goal(x, y, yaw):
                messagebox.showinfo("Goal Sent", f"Navigation goal sent to ({x:.2f}, {y:.2f})")
            else:
                messagebox.showerror("Error", "Failed to send goal. Is Nav2 running?")
                
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric coordinates")
    
    def on_cancel_goal(self):
        """Handle cancel goal button click"""
        self.cancel_goal()
        messagebox.showinfo("Goal Cancelled", "Current navigation goal cancelled")
    
    def set_quick_goal(self, x, y):
        """Set a quick goal"""
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, str(x))
        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, str(y))
        self.send_goal(x, y)
    
    def go_forward_5m(self):
        """Go forward 5 meters from current position"""
        x = self.robot_x + 5.0 * math.cos(self.robot_yaw)
        y = self.robot_y + 5.0 * math.sin(self.robot_yaw)
        self.set_quick_goal(x, y)
    
    def go_backward_5m(self):
        """Go backward 5 meters from current position"""
        x = self.robot_x - 5.0 * math.cos(self.robot_yaw)
        y = self.robot_y - 5.0 * math.sin(self.robot_yaw)
        self.set_quick_goal(x, y)
    
    def on_map_click(self, event):
        """Handle map click to set goal"""
        if event.inaxes != self.ax:
            return
        
        x, y = event.xdata, event.ydata
        if x is not None and y is not None:
            self.x_entry.delete(0, tk.END)
            self.x_entry.insert(0, f"{x:.2f}")
            self.y_entry.delete(0, tk.END)
            self.y_entry.insert(0, f"{y:.2f}")
            
            # Send goal immediately on click
            self.send_goal(x, y)
    
    def update_robot_display(self):
        """Update robot position display"""
        if hasattr(self, 'robot_x_label'):
            self.robot_x_label.config(text=f"X: {self.robot_x:.2f} m")
            self.robot_y_label.config(text=f"Y: {self.robot_y:.2f} m")
            self.robot_yaw_label.config(text=f"Yaw: {self.robot_yaw:.2f} rad")
    
    def update_map_display(self):
        """Update map display"""
        try:
            if self.map_data is not None and hasattr(self, 'ax'):
                # Clear previous map
                self.ax.clear()
                
                # Display occupancy grid
                # Convert occupancy grid to image (0=free, 100=occupied, -1=unknown)
                map_img = np.zeros_like(self.map_data, dtype=np.uint8)
                map_img[self.map_data == -1] = 128  # Unknown = gray
                map_img[self.map_data == 0] = 255   # Free = white
                map_img[self.map_data > 50] = 0     # Occupied = black
                
                # Calculate map bounds in world coordinates
                x_min = self.map_origin_x
                x_max = self.map_origin_x + self.map_width * self.map_resolution
                y_min = self.map_origin_y
                y_max = self.map_origin_y + self.map_height * self.map_resolution
                
                # Display map
                self.ax.imshow(map_img, extent=[x_min, x_max, y_min, y_max], 
                              origin='lower', cmap='gray', alpha=0.8)
                
                # Update plot limits with some padding
                padding = max(1.0, (x_max - x_min) * 0.1)  # 10% padding or 1m minimum
                self.ax.set_xlim(x_min - padding, x_max + padding)
                self.ax.set_ylim(y_min - padding, y_max + padding)
                
                self.ax.set_title('SLAM Map', color='#F8F9FA', fontweight='bold')
                self.ax.set_xlabel('X (m)', color='#F8F9FA')
                self.ax.set_ylabel('Y (m)', color='#F8F9FA')
                self.ax.tick_params(colors='#95D5B2')
                self.ax.grid(True, color='#40916C', alpha=0.5)
                
                # Ensure we have proper tick configuration to avoid IndexError
                self.ax.xaxis.set_minor_locator(NullLocator())
                self.ax.yaxis.set_minor_locator(NullLocator())
        except Exception as e:
            # Log error but don't crash the GUI
            self.get_logger().warn(f'Error updating map display: {e}')
        
        # Update robot position
        if hasattr(self, 'robot_marker'):
            self.robot_marker.set_data([self.robot_x], [self.robot_y])
            
            # Add robot orientation arrow
            arrow_length = 1.0
            dx = arrow_length * math.cos(self.robot_yaw)
            dy = arrow_length * math.sin(self.robot_yaw)
            
            # Remove old arrow if exists
            for patch in self.ax.patches[:]:
                if isinstance(patch, patches.FancyArrowPatch):
                    patch.remove()
            
            arrow = patches.FancyArrowPatch(
                (self.robot_x, self.robot_y),
                (self.robot_x + dx, self.robot_y + dy),
                arrowstyle='->', mutation_scale=20, color='#28A745', linewidth=2
            )
            self.ax.add_patch(arrow)
        
        # Update goal marker
        if self.current_goal and hasattr(self, 'ax'):
            if self.goal_marker:
                self.goal_marker.remove()
            self.goal_marker = self.ax.plot(self.current_goal[0], self.current_goal[1], 
                                          '*', color='#FFD60A', markersize=15, markeredgecolor='#FF6B35', markeredgewidth=2, label='Goal')[0]
        
        if hasattr(self, 'canvas'):
            try:
                self.canvas.draw()
            except Exception as e:
                self.get_logger().warn(f'Error drawing canvas: {e}')
    
    def update_gui(self):
        """Periodic GUI update"""
        if hasattr(self, 'root'):
            self.update_robot_display()
            self.update_map_display()
            self.root.after(100, self.update_gui)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        gui_node = NavigationGUI()
        
        # Spin in background thread
        spin_thread = threading.Thread(target=lambda: rclpy.spin(gui_node), daemon=True)
        spin_thread.start()
        
        # Keep main thread alive
        spin_thread.join()
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'gui_node' in locals():
            gui_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

