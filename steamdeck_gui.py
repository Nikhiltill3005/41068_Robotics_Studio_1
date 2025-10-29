#!/usr/bin/env python3

"""
Steam Deck ROS2 GUI

Optimized layout for 1280x800 (Steam Deck) with three panes:
  - SLAM Map (Husky)
  - Switchable RGB Camera (Husky/Drone - toggle with LB/RB)
  - 2D Terrain Map (robot positions, fires, paths)

Subscribes to teleop status from the combined joystick teleop node.
All topics are ROS2 parameters for easy override via launch file.

Controls are provided by the existing combined_joy_teleop node (launched separately).
"""

import os
import math
import threading
import queue
import tkinter as tk
from tkinter import ttk
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image as ROSImage, Joy, CompressedImage
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String as ROSString
from cv_bridge import CvBridge
import cv2
from PIL import Image, ImageTk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class SteamDeckGui(Node):
    def __init__(self, root: tk.Tk):
        super().__init__('steamdeck_gui')
        self.root = root

        # Parameters (defaults can be overridden via launch)
        self.declare_parameter('husky_rgb_topic', '/husky/camera/image')
        self.declare_parameter('drone_rgb_topic', '/drone/camera/image')
        self.declare_parameter('husky_map_topic', '/husky/map')
        self.declare_parameter('husky_odom_topic', '/husky/odometry')
        self.declare_parameter('drone_odom_topic', '/drone/odometry')
        self.declare_parameter('fire_topic', '/drone/fire_scan/fire_positions')
        self.declare_parameter('teleop_status_topic', '/teleop_status')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('world_size', 50.0)
        self.declare_parameter('terrain_image', 'bushland_terrain.png')

        self.topics = {
            'husky_rgb': self.get_parameter('husky_rgb_topic').get_parameter_value().string_value,
            'drone_rgb': self.get_parameter('drone_rgb_topic').get_parameter_value().string_value,
            'husky_map': self.get_parameter('husky_map_topic').get_parameter_value().string_value,
            'husky_odom': self.get_parameter('husky_odom_topic').get_parameter_value().string_value,
            'drone_odom': self.get_parameter('drone_odom_topic').get_parameter_value().string_value,
            'fires': self.get_parameter('fire_topic').get_parameter_value().string_value,
            'teleop_status': self.get_parameter('teleop_status_topic').get_parameter_value().string_value,
            'joy': self.get_parameter('joy_topic').get_parameter_value().string_value,
        }
        self.world_size = self.get_parameter('world_size').get_parameter_value().double_value
        self.terrain_image_path = self.get_parameter('terrain_image').get_parameter_value().string_value

        # CV conversion
        self.bridge = CvBridge()
        try:
            os.environ.setdefault('OMP_NUM_THREADS', '1')
            os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
            os.environ.setdefault('MKL_NUM_THREADS', '1')
            cv2.setNumThreads(1)
        except Exception:
            pass

        # Queues for incoming frames
        self.queues = {k: queue.Queue(maxsize=1) for k in ['husky_rgb', 'drone_rgb', 'slam_map']}
        self._throttle_n = 5  # Skip more frames to reduce lag
        self._counters = {k: 0 for k in ['husky_rgb', 'drone_rgb', 'slam_map']}

        # Camera switching state (RB=Husky, LB=Drone)
        self.active_rgb_camera = 'husky'  # 'husky' or 'drone'
        self.prev_lb_pressed = False
        self.prev_rb_pressed = False

        # Position and path data
        self.positions = {'husky': None, 'drone': None}
        self.paths = {'husky': [], 'drone': []}
        self.fire_positions = []
        
        # Load terrain image
        self.terrain_img = None
        if self.terrain_image_path:
            try:
                p = os.path.expanduser(self.terrain_image_path)
                if not os.path.isabs(p):
                    # Try relative to workspace
                    p = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', self.terrain_image_path)
                img = Image.open(p).convert('RGBA')
                self.terrain_img = np.asarray(img)
                self.get_logger().info(f"Loaded terrain image: {p}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load terrain image: {e}")

        # Build UI
        self._build_ui()

        # Optimized QoS for low-latency image transport
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # Only keep latest message
        )

        # Subscribe to COMPRESSED image topics for much better network performance
        self.create_subscription(CompressedImage, self.topics['husky_rgb'] + '/compressed', 
                                lambda m: self._on_compressed_image(m, 'husky_rgb'), image_qos)
        self.create_subscription(CompressedImage, self.topics['drone_rgb'] + '/compressed', 
                                lambda m: self._on_compressed_image(m, 'drone_rgb'), image_qos)
        
        # Joystick subscription for camera switching
        self.create_subscription(Joy, self.topics['joy'], self._on_joy, 10)
        
        self.get_logger().info("Subscribed to COMPRESSED image topics for low-latency streaming")
        
        # SLAM map subscription with appropriate QoS
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(OccupancyGrid, self.topics['husky_map'], 
                                self._on_slam_map, map_qos)
        
        # Odometry subscriptions
        self.create_subscription(Odometry, self.topics['husky_odom'], self._on_husky_odom, 10)
        self.create_subscription(Odometry, self.topics['drone_odom'], self._on_drone_odom, 10)
        
        # Fire positions
        self.create_subscription(PoseArray, self.topics['fires'], self._on_fires, 10)
        
        # Teleop status
        self.create_subscription(ROSString, self.topics['teleop_status'], self._on_status, 10)

        # Periodic refresh
        self._schedule_refresh()
        self._schedule_map_refresh()

        self.get_logger().info(f"SteamDeck GUI started with topics: {self.topics}")

    def _build_ui(self) -> None:
        # Window setup for Steam Deck
        self.root.title('Robotics Control - Steam Deck')
        try:
            self.root.geometry('1280x800')
        except Exception:
            pass

        BG = '#0f172a'      # dark slate
        PANEL = '#1f2937'   # gray-800
        TEXT = '#e5e7eb'    # gray-200

        self.root.configure(bg=BG)
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass
        style.configure('Dark.TFrame', background=BG)
        style.configure('Panel.TFrame', background=PANEL)
        style.configure('Dark.TLabel', background=BG, foreground=TEXT)
        style.configure('Panel.TLabel', background=PANEL, foreground=TEXT)

        # Top bar with teleop status button
        top = ttk.Frame(self.root, padding=8, style='Dark.TFrame')
        top.pack(fill=tk.X)
        
        # Teleop status button (changes color when active)
        self.teleop_active = False
        self.teleop_button = tk.Button(
            top, 
            text='Teleop Inactive', 
            bg='#555555',  # Gray when inactive
            fg='white',
            font=('Segoe UI', 12, 'bold'),
            relief='flat',
            padx=15,
            pady=5,
            state='disabled'  # Not clickable, just visual indicator
        )
        self.teleop_button.pack(side=tk.LEFT)

        # Grid layout: 2 columns on top, 1 wide map on bottom
        grid = ttk.Frame(self.root, padding=8, style='Dark.TFrame')
        grid.pack(fill=tk.BOTH, expand=True)
        for c in range(2):
            grid.columnconfigure(c, weight=1)
        grid.rowconfigure(0, weight=1)
        grid.rowconfigure(1, weight=1)

        # Helper function to create panels
        def make_panel(parent, title: str, use_canvas=False):
            panel = ttk.Frame(parent, padding=8, style='Panel.TFrame')
            header = ttk.Label(panel, text=title, style='Panel.TLabel', 
                              font=('Segoe UI', 11, 'bold'))
            header.pack(anchor='w')
            
            if use_canvas:
                return panel, None
            else:
                label = tk.Label(panel, bg='black', fg='white', text='Waiting...')
                label.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
                return panel, label

        # Top-left: SLAM Map
        p00, _ = make_panel(grid, 'SLAM Map (Husky)', use_canvas=True)
        self.lbl_slam_map = tk.Label(p00, bg='black', fg='white', text='Waiting for map...')
        self.lbl_slam_map.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        p00.grid(row=0, column=0, sticky='nsew', padx=(0, 6), pady=(0, 6))

        # Top-right: Switchable RGB Camera (Husky or Drone)
        p01 = ttk.Frame(grid, padding=8, style='Panel.TFrame')
        self.rgb_camera_header = ttk.Label(p01, text='RGB: Husky (RB=Husky | LB=Drone)', 
                                           style='Panel.TLabel', font=('Segoe UI', 11, 'bold'))
        self.rgb_camera_header.pack(anchor='w')
        self.lbl_switchable_rgb = tk.Label(p01, bg='black', fg='white', text='Waiting...')
        self.lbl_switchable_rgb.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        p01.grid(row=0, column=1, sticky='nsew', padx=(6, 0), pady=(0, 6))

        # Bottom: 2D Terrain Map (spans full width)
        p11, _ = make_panel(grid, '2D Map (Terrain)', use_canvas=True)
        self.fig, self.ax = plt.subplots(figsize=(5, 4))
        self.fig.patch.set_facecolor(PANEL)
        self.ax.set_facecolor('#2a2a2a')
        self.ax.set_xlim(-self.world_size/2, self.world_size/2)
        self.ax.set_ylim(-self.world_size/2, self.world_size/2)
        self.ax.set_aspect('equal', 'box')
        self.ax.grid(True, color='#555555', alpha=0.3)
        self.ax.tick_params(colors=TEXT, labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_color(TEXT)
        self.canvas = FigureCanvasTkAgg(self.fig, master=p11)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        p11.grid(row=1, column=0, columnspan=2, sticky='nsew', pady=(6, 0))

    def _on_status(self, msg: ROSString) -> None:
        try:
            text = msg.data if isinstance(msg.data, str) else str(msg.data)
            # Check if teleop is active (contains "Hold" indicating user should hold a button)
            is_active = "Hold" in text
            
            if is_active and not self.teleop_active:
                # Switched to active
                self.teleop_button.configure(
                    text='Teleop Active',
                    bg='#00cc00'  # Bright green when active
                )
                self.teleop_active = True
            elif not is_active and self.teleop_active:
                # Switched to inactive
                self.teleop_button.configure(
                    text='Teleop Inactive',
                    bg='#555555'  # Gray when inactive
                )
                self.teleop_active = False
        except Exception:
            pass

    def _on_joy(self, msg: Joy) -> None:
        """Handle joystick input for camera switching.
        Button 4 (LB) = Drone RGB
        Button 5 (RB) = Husky RGB
        """
        try:
            if len(msg.buttons) > 5:
                lb_pressed = msg.buttons[4] == 1  # Left bumper
                rb_pressed = msg.buttons[5] == 1  # Right bumper
                
                # Detect rising edge (button just pressed)
                if lb_pressed and not self.prev_lb_pressed:
                    self.active_rgb_camera = 'drone'
                    self.rgb_camera_header.configure(text='RGB: Drone (RB=Husky | LB=Drone)')
                    self.get_logger().info('Switched RGB camera to DRONE')
                
                elif rb_pressed and not self.prev_rb_pressed:
                    self.active_rgb_camera = 'husky'
                    self.rgb_camera_header.configure(text='RGB: Husky (RB=Husky | LB=Drone)')
                    self.get_logger().info('Switched RGB camera to HUSKY')
                
                self.prev_lb_pressed = lb_pressed
                self.prev_rb_pressed = rb_pressed
        except Exception as e:
            self.get_logger().warn(f'Joy callback error: {e}')

    def _on_compressed_image(self, msg: CompressedImage, key: str) -> None:
        """Handle compressed image - much faster over network!"""
        try:
            self._counters[key] += 1
            if (self._counters[key] % self._throttle_n) != 0:
                return
            
            # Decode compressed JPEG directly from bytes
            np_arr = np.frombuffer(msg.data, np.uint8)
            disp = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if disp is None:
                return
            
            try:
                self.queues[key].get_nowait()
            except queue.Empty:
                pass
            self.queues[key].put_nowait(disp)
        except Exception as exc:
            self.get_logger().warn(f'Compressed image decode failed for {key}: {exc}')

    def _on_image(self, msg: ROSImage, key: str) -> None:
        """Fallback for raw images (not used if compressed available)"""
        try:
            self._counters[key] += 1
            if (self._counters[key] % self._throttle_n) != 0:
                return
            
            disp = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            try:
                self.queues[key].get_nowait()
            except queue.Empty:
                pass
            self.queues[key].put_nowait(disp)
        except Exception as exc:
            self.get_logger().warn(f'Image conversion failed for {key}: {exc}')

    def _on_slam_map(self, msg: OccupancyGrid) -> None:
        try:
            self._counters['slam_map'] += 1
            if (self._counters['slam_map'] % self._throttle_n) != 0:
                return
            
            # Convert occupancy grid to image
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            
            # Convert to grayscale image (0=free, 100=occupied, -1=unknown)
            img = np.zeros((height, width, 3), dtype=np.uint8)
            img[data == -1] = [50, 50, 50]      # unknown = gray
            img[data == 0] = [255, 255, 255]    # free = white
            img[data > 50] = [0, 0, 0]          # occupied = black
            
            # Flip vertically for display
            img = cv2.flip(img, 0)
            
            try:
                self.queues['slam_map'].get_nowait()
            except queue.Empty:
                pass
            self.queues['slam_map'].put_nowait(img)
        except Exception as exc:
            self.get_logger().warn(f'SLAM map conversion failed: {exc}')

    def _on_husky_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions['husky'] = (x, y)
        self.paths['husky'].append((x, y))
        if len(self.paths['husky']) > 2000:
            self.paths['husky'].pop(0)

    def _on_drone_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions['drone'] = (x, y)
        self.paths['drone'].append((x, y))
        if len(self.paths['drone']) > 2000:
            self.paths['drone'].pop(0)

    def _on_fires(self, msg: PoseArray) -> None:
        fires = []
        for p in msg.poses:
            fires.append((p.position.x, p.position.y))
        self.fire_positions = fires

    def _schedule_refresh(self) -> None:
        self.root.after(100, self._refresh)

    def _refresh(self) -> None:
        try:
            # Update switchable RGB camera based on active selection
            rgb_key = f'{self.active_rgb_camera}_rgb'
            frame = None
            try:
                frame = self.queues[rgb_key].get_nowait()
            except queue.Empty:
                pass
            if frame is not None:
                self._update_label_with_frame(self.lbl_switchable_rgb, frame)
            
            # Update SLAM map
            mapping = [
                ('slam_map', self.lbl_slam_map),
            ]
            for key, label in mapping:
                frame = None
                try:
                    frame = self.queues[key].get_nowait()
                except queue.Empty:
                    pass
                if frame is None:
                    continue
                self._update_label_with_frame(label, frame)
        finally:
            self._schedule_refresh()

    def _schedule_map_refresh(self) -> None:
        self.root.after(500, self._refresh_2d_map)

    def _refresh_2d_map(self) -> None:
        try:
            self.ax.clear()
            self.ax.set_xlim(-self.world_size/2, self.world_size/2)
            self.ax.set_ylim(-self.world_size/2, self.world_size/2)
            self.ax.set_aspect('equal', 'box')
            self.ax.grid(True, color='#555555', alpha=0.3)
            
            # Draw terrain image if available
            if self.terrain_img is not None:
                extent = (-self.world_size/2, self.world_size/2, 
                         -self.world_size/2, self.world_size/2)
                self.ax.imshow(self.terrain_img, extent=extent, origin='lower', 
                              zorder=0, interpolation='bilinear', alpha=0.6)
            
            # Draw paths
            if self.paths['husky']:
                xs, ys = zip(*self.paths['husky'])
                self.ax.plot(xs, ys, '-', color='cyan', linewidth=2, alpha=0.7, label='Husky')
            if self.paths['drone']:
                xs, ys = zip(*self.paths['drone'])
                self.ax.plot(xs, ys, '-', color='lime', linewidth=2, alpha=0.7, label='Drone')
            
            # Draw robot positions
            if self.positions['husky']:
                x, y = self.positions['husky']
                self.ax.plot(x, y, 'cs', markersize=12, markeredgecolor='white', markeredgewidth=2)
            if self.positions['drone']:
                x, y = self.positions['drone']
                self.ax.plot(x, y, 'g^', markersize=12, markeredgecolor='white', markeredgewidth=2)
            
            # Draw fires
            if self.fire_positions:
                fx, fy = zip(*self.fire_positions)
                self.ax.scatter(fx, fy, c='red', s=100, marker='*', 
                               edgecolors='yellow', linewidths=2, zorder=10)
            
            # Style
            TEXT = '#e5e7eb'
            self.ax.tick_params(colors=TEXT, labelsize=8)
            for spine in self.ax.spines.values():
                spine.set_color(TEXT)
            if len(self.paths['husky']) > 0 or len(self.paths['drone']) > 0:
                self.ax.legend(loc='upper right', fontsize=8, 
                              facecolor='#1f2937', edgecolor=TEXT, labelcolor=TEXT)
            
            self.canvas.draw_idle()
        except Exception as e:
            self.get_logger().warn(f'2D map refresh error: {e}')
        finally:
            self._schedule_map_refresh()

    def _update_label_with_frame(self, label: tk.Label, frame_bgr) -> None:
        # Resize with aspect ratio to fill area - use fixed small size for Steam Deck
        label.update_idletasks()
        target_w = min(480, max(200, label.winfo_width()))  # Cap at 480px for performance
        target_h = min(360, max(150, label.winfo_height()))  # Cap at 360px for performance
        h, w = frame_bgr.shape[:2]
        scale = min(target_w / w, target_h / h)
        new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
        
        # Use INTER_NEAREST for fastest resizing (acceptable for Steam Deck display)
        resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_NEAREST)

        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        photo = ImageTk.PhotoImage(image=image)

        def apply():
            label.configure(image=photo, text='')
            label.image = photo
        self.root.after(0, apply)


def start_spin(node: Node) -> None:
    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()


def main() -> None:
    rclpy.init()
    root = tk.Tk()
    app = SteamDeckGui(root)
    start_spin(app)
    try:
        root.mainloop()
    finally:
        app.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
