#!/usr/bin/env python3
"""
<<<<<<< HEAD
steamdeck_gui.py
Merged GUI: Steam Deck layout + mission log + status panel + date/time/mission timer
Camera feeds: drone_rgb, drone_ir, husky_rgb (using compressed if available, raw fallback)
SLAM map: subscribes to slam_map_topic
Other topics: odometry, fire positions, battery, obstacles, control mode
=======
Steam Deck ROS2 GUI

Optimized layout for 1280x800 (Steam Deck) with three panes:
  - Switchable RGB Camera (Husky/Drone - toggle via touchscreen button) - LEFT 50% (MAIN FOCUS)
  - SLAM Map (Husky) - top right
  - 2D Terrain Map (robot positions, fires, paths) - bottom right

Features a prominent fire detection status panel in the top bar.
All topics are ROS2 parameters for easy override via launch file.
>>>>>>> cfb7fe79ba3f400536ae46eb8dbd223350044247
"""

import os
import time
import threading
import queue
import math
from datetime import datetime

import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox

import numpy as np
from PIL import Image, ImageTk
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image as ROSImage, CompressedImage, Joy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from std_msgs.msg import String as ROSString, Float32, Bool
from cv_bridge import CvBridge

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class SteamDeckGui(Node):
    def __init__(self, root: tk.Tk):
        super().__init__('steamdeck_gui')
        self.root = root

        # ---------- PARAMETERS (declared as ROS params so user can override) ----------
        self.declare_parameter('drone_rgb_topic', '/drone/fire_scan/debug_image')
        self.declare_parameter('drone_ir_topic', '/drone/ir_camera/image_raw')
        self.declare_parameter('husky_rgb_topic', '/husky/camera/image')
        self.declare_parameter('slam_map_topic', '/map_updates')
        self.declare_parameter('husky_odom_topic', '/husky/odometry')
        self.declare_parameter('drone_odom_topic', '/drone/odometry')
        self.declare_parameter('fire_topic', '/drone/fire_scan/fire_positions')
        self.declare_parameter('control_mode_topic', '/control_mode')
        self.declare_parameter('husky_batt_topic', '/husky/battery')
        self.declare_parameter('drone_batt_topic', '/drone/battery')
        self.declare_parameter('world_size', 50.0)
        self.declare_parameter('rviz_path_topic', '/visualization/path')
        self.declare_parameter('husky_cmdvel_topic', '/husky/cmd_vel')
        self.declare_parameter('drone_cmdvel_topic', '/drone/cmd_vel')
        self.declare_parameter('terrain_image', 'bushland_terrain.png')

        # Read parameter values
        self.topics = {
            'drone_rgb': self.get_parameter('drone_rgb_topic').get_parameter_value().string_value,
            'drone_ir': self.get_parameter('drone_ir_topic').get_parameter_value().string_value,
            'husky_rgb': self.get_parameter('husky_rgb_topic').get_parameter_value().string_value,
            'slam_map': self.get_parameter('slam_map_topic').get_parameter_value().string_value,
            'husky_odom': self.get_parameter('husky_odom_topic').get_parameter_value().string_value,
            'drone_odom': self.get_parameter('drone_odom_topic').get_parameter_value().string_value,
            'fires': self.get_parameter('fire_topic').get_parameter_value().string_value,
            'control_mode': self.get_parameter('control_mode_topic').get_parameter_value().string_value,
            'husky_batt': self.get_parameter('husky_batt_topic').get_parameter_value().string_value,
            'drone_batt': self.get_parameter('drone_batt_topic').get_parameter_value().string_value,
            'rviz_path': self.get_parameter('rviz_path_topic').get_parameter_value().string_value,
        }
        self.world_size = self.get_parameter('world_size').get_parameter_value().double_value
        self.terrain_image_path = self.get_parameter('terrain_image').get_parameter_value().string_value

        # ---------- Application state ----------
        self.bridge = CvBridge()
        try:
            os.environ.setdefault('OMP_NUM_THREADS', '1')
            os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
            os.environ.setdefault('MKL_NUM_THREADS', '1')
            cv2.setNumThreads(1)
        except Exception:
            pass

        # camera queues for latest frame only (drone_rgb, drone_ir, husky_rgb, slam_map)
        self.camera_queues = {k: queue.Queue(maxsize=1) for k in ['drone_rgb', 'drone_ir', 'husky_rgb', 'slam_map']}
        self.camera_enabled = {k: True for k in self.camera_queues.keys()}
        self._throttle_n = 2
        self._counters = {k: 0 for k in self.camera_queues.keys()}

        # positions and path info
        self.positions = {'husky': None, 'drone': None}
        self.paths = {'husky': [], 'drone': []}
        self.fire_positions = []
<<<<<<< HEAD

        # batteries, obstacles, sensor health
        self.batteries = {'husky': None, 'drone': None}
        self.obstacles = {'husky': False, 'drone': False}
        self.sensor_health = {'husky_camera': True, 'drone_camera': True}

        # control mode publisher
        self.control_pub = self.create_publisher(ROSString, self.topics['control_mode'], 10)
        self.current_mode = "autonomous"

        # RViz path publish toggle and publisher
        self.publish_rviz_path = False
        self.path_pub = self.create_publisher(Path, self.topics['rviz_path'], 10)

        # teleop publishers (not used directly here but kept for completeness)
        self.teleop_pub_husky = self.create_publisher(Twist, self.get_parameter('husky_cmdvel_topic').get_parameter_value().string_value, 10)
        self.teleop_pub_drone = self.create_publisher(Twist, self.get_parameter('drone_cmdvel_topic').get_parameter_value().string_value, 10)

        # teleop/joy state
        self.teleop_active = False

        # camera selection state (for switchable large view)
        self.active_rgb_camera = 'husky'  # 'husky' or 'drone'

        # mission timing
        self.start_time = time.time()
        self.mission_running = True

        # load terrain image
=======
        self.total_fires_detected = 0  # Track total unique fires detected
        self.detected_fire_ids = set()  # Track unique fire locations to avoid double counting
        
        # Load terrain image
>>>>>>> cfb7fe79ba3f400536ae46eb8dbd223350044247
        self.terrain_img = None
        if self.terrain_image_path:
            try:
                p = os.path.expanduser(self.terrain_image_path)
                if not os.path.isabs(p):
                    p = os.path.join(os.path.dirname(__file__), self.terrain_image_path)
                img = Image.open(p).convert('RGBA')
                self.terrain_img = np.asarray(img)
                self.get_logger().info(f"Loaded terrain image: {p}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load terrain image: {e}")
                self.terrain_img = None

        # ---------- Build GUI ----------
        self._build_ui()

        # ---------- ROS Subscriptions ----------
        # Image topics - try compressed first, fallback to raw image subscription
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to compressed (if publishers provide) and raw as fallback
        try:
            # compressed endpoints conventionally are topic + '/compressed'
            self.create_subscription(CompressedImage, self.topics['drone_rgb'] + '/compressed',
                                     lambda m: self._on_compressed_image(m, 'drone_rgb'), image_qos)
        except Exception:
            pass
        self.create_subscription(ROSImage, self.topics['drone_rgb'], lambda m: self._on_image(m, 'drone_rgb'), 10)

        try:
            self.create_subscription(CompressedImage, self.topics['husky_rgb'] + '/compressed',
                                     lambda m: self._on_compressed_image(m, 'husky_rgb'), image_qos)
        except Exception:
            pass
        self.create_subscription(ROSImage, self.topics['husky_rgb'], lambda m: self._on_image(m, 'husky_rgb'), 10)

        # drone IR usually raw
        self.create_subscription(ROSImage, self.topics['drone_ir'], lambda m: self._on_image(m, 'drone_ir'), 10)

        # SLAM map (occupancy grid)
        map_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, self.topics['slam_map'], self._on_slam_map, map_qos)

        # odometry
        self.create_subscription(Odometry, self.topics['husky_odom'], self._on_husky_odom, 10)
        self.create_subscription(Odometry, self.topics['drone_odom'], self._on_drone_odom, 10)

        # fire positions
        self.create_subscription(PoseArray, self.topics['fires'], self._on_fires, 10)

        # battery topics
        self.create_subscription(Float32, self.topics['husky_batt'], self._on_husky_batt, 10)
        self.create_subscription(Float32, self.topics['drone_batt'], self._on_drone_batt, 10)

        # obstacle flags (optional)
        self.create_subscription(Bool, '/husky/obstacle', lambda m: self._on_obstacle(m, 'husky'), 10)
        self.create_subscription(Bool, '/drone/obstacle', lambda m: self._on_obstacle(m, 'drone'), 10)

<<<<<<< HEAD
        # teleop / joy input (for teleop_active visual)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        # control mode status topic
        self.create_subscription(ROSString, self.topics['control_mode'], self._on_control_mode, 10)

        # schedule GUI loops
        self.root.after(100, self._gui_video_loop)
        self.root.after(500, self._gui_map_loop)
        self.root.after(1000, self._update_time_display)

        self.get_logger().info("SteamDeck GUI initialized with topics: %s" % self.topics)

    def _build_ui(self):
        # window
        self.root.title('Robotics Control - Steam Deck')
=======
    def _build_ui(self) -> None:
        # Window setup for Steam Deck
        self.root.title('Forest Fire Detection - Steam Deck')
>>>>>>> cfb7fe79ba3f400536ae46eb8dbd223350044247
        try:
            self.root.geometry('1280x800')
        except Exception:
            pass

<<<<<<< HEAD
        BG = '#0f172a'
        PANEL = '#1f2937'
        TEXT = '#e5e7eb'
=======
        BG = '#111827'      # gray-900 (darker)
        PANEL = '#2d3748'   # smoke-gray
        TEXT = '#e5e7eb'    # gray-200
        FIRE_RED = '#ff4444'
        FIRE_ORANGE = '#ff8800'
>>>>>>> cfb7fe79ba3f400536ae46eb8dbd223350044247

        self.root.configure(bg=BG)
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass
        style.configure('Dark.TFrame', background=BG)
        style.configure('Panel.TFrame', background=PANEL)
        style.configure('Panel.TLabel', background=PANEL, foreground=TEXT)

<<<<<<< HEAD
        # Top bar
        top = ttk.Frame(self.root, padding=6, style='Dark.TFrame')
        top.pack(fill=tk.X)

        # Mode toggle button
        self.mode_btn = tk.Button(top, text="Switch to MANUAL", bg="#0078d7", fg="white",
                                  font=("Segoe UI", 10, "bold"), relief="flat", padx=8, pady=4,
                                  command=self.toggle_mode)
        self.mode_btn.pack(side=tk.LEFT, padx=4)
=======
        # Top bar with teleop status and fire detection panel
        top = ttk.Frame(self.root, padding=8, style='Dark.TFrame')
        top.pack(fill=tk.X)
        
        # Teleop status button (left side)
        self.teleop_active = False
        self.teleop_button = tk.Button(
            top, 
            text='Teleop Inactive', 
            bg='#555555',
            fg='white',
            font=('Segoe UI', 11, 'bold'),
            relief='flat',
            padx=12,
            pady=4,
            state='disabled'
        )
        self.teleop_button.pack(side=tk.LEFT)

        # FIRE DETECTION STATUS PANEL - Prominent dedicated section (right side)
        fire_panel = tk.Frame(top, bg=PANEL, relief='solid', borderwidth=3, highlightbackground='#666', highlightthickness=1)
        fire_panel.pack(side=tk.RIGHT, padx=(10, 0))
        
        # Fire icon and status
        fire_header = tk.Frame(fire_panel, bg=PANEL)
        fire_header.pack(fill=tk.X, padx=10, pady=(8, 2))
        
        fire_icon = tk.Label(
            fire_header,
            text='🔥',
            bg=PANEL,
            font=('Segoe UI', 24),
        )
        fire_icon.pack(side=tk.LEFT, padx=(0, 8))
        
        fire_text_container = tk.Frame(fire_header, bg=PANEL)
        fire_text_container.pack(side=tk.LEFT)
        
        tk.Label(
            fire_text_container,
            text='FIRE DETECTION STATUS',
            bg=PANEL,
            fg=TEXT,
            font=('Segoe UI', 9, 'bold')
        ).pack(anchor='w')
        
        # Large status text that changes based on detection
        self.fire_status_label = tk.Label(
            fire_text_container,
            text='NO FIRES DETECTED',
            bg=PANEL,
            fg='#10b981',  # Green when no fire
            font=('Segoe UI', 14, 'bold')
        )
        self.fire_status_label.pack(anchor='w')
        
        # Fire counter section
        fire_count_frame = tk.Frame(fire_panel, bg=PANEL)
        fire_count_frame.pack(fill=tk.X, padx=10, pady=(2, 8))
        
        tk.Label(
            fire_count_frame,
            text='Total Detected:',
            bg=PANEL,
            fg='#9ca3af',
            font=('Segoe UI', 9)
        ).pack(side=tk.LEFT, padx=(0, 6))
        
        self.fire_count_label = tk.Label(
            fire_count_frame,
            text='0',
            bg=PANEL,
            fg=FIRE_ORANGE,
            font=('Segoe UI', 16, 'bold')
        )
        self.fire_count_label.pack(side=tk.LEFT)

        # Grid layout: Camera on left (50%), SLAM+2D Map stacked on right (50%)
        grid = ttk.Frame(self.root, padding=8, style='Dark.TFrame')
        grid.pack(fill=tk.BOTH, expand=True)
        grid.columnconfigure(0, weight=1)  # Left column (camera)
        grid.columnconfigure(1, weight=1)  # Right column (maps)
        grid.rowconfigure(0, weight=1)  # Top row (camera / SLAM)
        grid.rowconfigure(1, weight=1)  # Bottom row (2D Map)
>>>>>>> cfb7fe79ba3f400536ae46eb8dbd223350044247

        # Emergency stop
        stop_btn = tk.Button(top, text="EMERGENCY STOP", bg="#e81123", fg="white",
                             font=("Segoe UI", 10, "bold"), relief="flat", padx=8, pady=4,
                             command=lambda: self.log("EMERGENCY STOP triggered!"))
        stop_btn.pack(side=tk.LEFT, padx=4)

        # teleop indicator
        self.teleop_button = tk.Button(top, text='Teleop Inactive', bg='#555555', fg='white',
                                       font=('Segoe UI', 11, 'bold'), relief='flat', padx=12, pady=4, state='disabled')
        self.teleop_button.pack(side=tk.LEFT, padx=6)

        # Save snapshot + export log
        ttk.Button(top, text="Save Snapshot", command=self.save_snapshot).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Export Log", command=self._export_log).pack(side=tk.LEFT, padx=4)

        # Timer & date/time labels on right
        self.timer_label = ttk.Label(top, text="Mission Time: 00:00:00")
        self.timer_label.pack(side=tk.RIGHT, padx=8)
        self.system_time_label = ttk.Label(top, text=time.strftime("%Y-%m-%d %H:%M:%S"))
        self.system_time_label.pack(side=tk.RIGHT, padx=8)

        # Main layout: left camera panel, right maps, bottom status/log
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)
        main.rowconfigure(1, weight=1)

        # Left: Multiple small camera feeds (drone_rgb, drone_ir, husky_rgb)
        cam_frame = ttk.LabelFrame(main, text="Camera Feeds", padding=8)
        cam_frame.grid(row=0, column=0, rowspan=2, sticky='nsew', padx=(0, 6), pady=(0,0))

        # Large switchable RGB (husky/drone) as main view at top of cam_frame
        header_frame = tk.Frame(cam_frame, bg=PANEL)
        header_frame.pack(fill=tk.X)
        self.rgb_camera_header = ttk.Label(header_frame, text='RGB Camera', style='Panel.TLabel', font=('Segoe UI', 11, 'bold'))
        self.rgb_camera_header.pack(side=tk.LEFT)
        self.camera_toggle_btn = tk.Button(header_frame, text='Viewing: HUSKY', bg='#2563eb', fg='white',
                                           font=('Segoe UI', 10, 'bold'), relief='flat', padx=12, pady=4,
                                           command=self._toggle_camera)
        self.camera_toggle_btn.pack(side=tk.RIGHT, padx=(10, 0))

        self.lbl_switchable_rgb = tk.Label(cam_frame, bg='black', fg='white', text='Waiting for RGB...')
        self.lbl_switchable_rgb.pack(fill=tk.BOTH, expand=True, pady=(6, 6))

        # Below that, two smaller feeds: drone_ir and hv (optionally)
        small_row = tk.Frame(cam_frame)
        small_row.pack(fill=tk.X)
        self.lbl_drone_ir = tk.Label(small_row, bg='black', fg='white', text='Drone IR')
        self.lbl_drone_ir.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=3, pady=3)
        self.lbl_husky_cam_small = tk.Label(small_row, bg='black', fg='white', text='Husky RGB (small)')
        self.lbl_husky_cam_small.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=3, pady=3)

        # Right column: SLAM (top), 2D terrain (bottom)
        # SLAM
        p_slam = ttk.LabelFrame(main, text="SLAM Map (Husky)", padding=6)
        p_slam.grid(row=0, column=1, sticky='nsew', padx=(6, 0), pady=(0, 6))
        self.lbl_slam_map = tk.Label(p_slam, bg='black', fg='white', text='Waiting for SLAM map...')
        self.lbl_slam_map.pack(fill=tk.BOTH, expand=True)

        # 2D Terrain Map
        p_terrain = ttk.LabelFrame(main, text="2D Map (Terrain)", padding=6)
        p_terrain.grid(row=1, column=1, sticky='nsew', padx=(6, 0), pady=(6, 0))
        self.fig, self.ax = plt.subplots(figsize=(5, 4))
        self.fig.patch.set_facecolor("#ffffff")
        self.ax.set_facecolor("#ffffff")
        self.ax.set_xlim(-self.world_size/2, self.world_size/2)
        self.ax.set_ylim(-self.world_size/2, self.world_size/2)
        self.ax.set_aspect('equal', 'box')
        self.ax.grid(True, color="#cccccc", alpha=0.5)
        self.canvas = FigureCanvasTkAgg(self.fig, master=p_terrain)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Bottom: Status + Mission Log
        bottom = ttk.Frame(self.root)
        bottom.pack(fill=tk.BOTH, expand=False, padx=6, pady=6)

        status_frame = ttk.LabelFrame(bottom, text="Status", padding=6)
        status_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))

        ttk.Label(status_frame, text="Husky Battery").pack(anchor="w")
        self.husky_batt_pb = ttk.Progressbar(status_frame, length=150, mode="determinate", maximum=100)
        self.husky_batt_pb.pack(anchor="w", pady=2)
        ttk.Label(status_frame, text="Drone Battery").pack(anchor="w", pady=(6, 0))
        self.drone_batt_pb = ttk.Progressbar(status_frame, length=150, mode="determinate", maximum=100)
        self.drone_batt_pb.pack(anchor="w", pady=2)

        self.fire_count_var = tk.StringVar(value="Fires: 0")
        ttk.Label(status_frame, textvariable=self.fire_count_var).pack(anchor="w", pady=(8, 0))
        self.husky_nearest_var = tk.StringVar(value="H dist to nearest fire: N/A")
        self.drone_nearest_var = tk.StringVar(value="D dist to nearest fire: N/A")
        ttk.Label(status_frame, textvariable=self.husky_nearest_var).pack(anchor="w")
        ttk.Label(status_frame, textvariable=self.drone_nearest_var).pack(anchor="w")

        self.husky_obs_var = tk.StringVar(value="H obstacle: N/A")
        self.drone_obs_var = tk.StringVar(value="D obstacle: N/A")
        ttk.Label(status_frame, textvariable=self.husky_obs_var).pack(anchor="w", pady=(6, 0))
        ttk.Label(status_frame, textvariable=self.drone_obs_var).pack(anchor="w")

        self.sensor_health_var = tk.StringVar(value="Sensors: OK")
        ttk.Label(status_frame, textvariable=self.sensor_health_var).pack(anchor="w", pady=(8, 0))

        # Mission log
        log_frame = ttk.LabelFrame(bottom, text="Mission Log", padding=6)
        log_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.log_box = scrolledtext.ScrolledText(log_frame, height=8, bg="#f7f9fb", fg="#1e2a38", font=("Consolas", 9))
        self.log_box.pack(fill=tk.BOTH, expand=True)
        self.log_box.config(state="disabled")

    def _toggle_camera(self):
        # Example: toggle all cameras on/off
        for key in self.camera_enabled:
            self.camera_enabled[key] = not self.camera_enabled[key]
        self.log("Toggled camera feed on/off")

    # ---------------- TELEOP / CONTROL ----------------
    def toggle_mode(self):
        self.current_mode = "manual" if self.current_mode == "autonomous" else "autonomous"
        m = ROSString()
        m.data = self.current_mode
        self.control_pub.publish(m)
        btn_text = "Switch to AUTO" if self.current_mode == "manual" else "Switch to MANUAL"
        self.mode_btn.configure(text=btn_text)
        self.log(f"Control mode changed to {self.current_mode.upper()}")

    # ---------------- ROS callbacks (images & sensors) ----------------
    def _on_compressed_image(self, msg: CompressedImage, key: str):
        # decode compressed image bytes (JPEG/PNG)
        try:
            self._counters[key] += 1
            if (self._counters[key] % self._throttle_n) != 0:
                return
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
            try:
                self.camera_queues[key].get_nowait()
            except queue.Empty:
                pass
            self.camera_queues[key].put_nowait(frame)
        except Exception as e:
            self.get_logger().warn(f"Compressed decode failed for {key}: {e}")

    def _on_image(self, msg: ROSImage, key: str):
        try:
            self._counters[key] += 1
            if (self._counters[key] % self._throttle_n) != 0:
                return
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            try:
                self.camera_queues[key].get_nowait()
            except queue.Empty:
                pass
            self.camera_queues[key].put_nowait(frame)
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed for {key}: {e}")
            # mark sensor health as degraded
            if 'drone' in key:
                self.sensor_health['drone_camera'] = False
            elif 'husky' in key:
                self.sensor_health['husky_camera'] = False

    def _on_slam_map(self, msg: OccupancyGrid):
        try:
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            img = np.zeros((height, width, 3), dtype=np.uint8)
            img[data == -1] = [50, 50, 50]
            img[data == 0] = [255, 255, 255]
            img[data > 50] = [0, 0, 0]
            img = cv2.flip(img, 0)
            try:
                self.camera_queues['slam_map'].get_nowait()
            except queue.Empty:
                pass
            self.camera_queues['slam_map'].put_nowait(img)
        except Exception as e:
            self.get_logger().warn(f"SLAM map conversion failed: {e}")

    def _on_husky_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions['husky'] = (x, y)
        self.paths['husky'].append((x, y))
        if len(self.paths['husky']) > 2000:
            self.paths['husky'].pop(0)
        self.update_status_panel()

    def _on_drone_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions['drone'] = (x, y)
        self.paths['drone'].append((x, y))
        if len(self.paths['drone']) > 2000:
            self.paths['drone'].pop(0)
        self.update_status_panel()

<<<<<<< HEAD
    def _on_fires(self, msg: PoseArray):
=======
    def _on_fires(self, msg: PoseArray) -> None:
        """Update fire positions and count unique fires detected."""
>>>>>>> cfb7fe79ba3f400536ae46eb8dbd223350044247
        fires = []
        for p in msg.poses:
            fire_pos = (p.position.x, p.position.y)
            fires.append(fire_pos)
            
            # Create a unique ID for this fire location (rounded to 0.5m grid to avoid duplicates)
            fire_id = (round(p.position.x * 2) / 2, round(p.position.y * 2) / 2)
            
            # If this is a new fire location, increment counter
            if fire_id not in self.detected_fire_ids:
                self.detected_fire_ids.add(fire_id)
                self.total_fires_detected = len(self.detected_fire_ids)
                self._update_fire_status()
                self.get_logger().info(f'New fire detected at ({fire_pos[0]:.1f}, {fire_pos[1]:.1f}). Total: {self.total_fires_detected}')
        
        self.fire_positions = fires
        
        # Update status even if no new fires (to handle when fires go out of view)
        if len(fires) == 0 and self.total_fires_detected == 0:
            self._update_fire_status()

    def _update_fire_status(self) -> None:
        """Update the fire detection status panel in the GUI."""
        def update():
            if self.total_fires_detected > 0:
                # FIRE DETECTED - Show alert status
                self.fire_status_label.configure(
                    text='⚠️ FIRE DETECTED ⚠️',
                    fg='#ff4444'  # Bright red
                )
                self.fire_count_label.configure(text=str(self.total_fires_detected))
            else:
                # NO FIRE - Show safe status
                self.fire_status_label.configure(
                    text='NO FIRES DETECTED',
                    fg='#10b981'  # Green
                )
                self.fire_count_label.configure(text='0')
        
        self.root.after(0, update)

    def _on_husky_batt(self, msg: Float32):
        try:
            self.batteries['husky'] = float(msg.data)
            self.log(f"Husky battery: {self.batteries['husky']:.1f}%")
            self.update_status_panel()
        except Exception:
            pass

    def _on_drone_batt(self, msg: Float32):
        try:
            self.batteries['drone'] = float(msg.data)
            self.log(f"Drone battery: {self.batteries['drone']:.1f}%")
            self.update_status_panel()
        except Exception:
            pass

    def _on_obstacle(self, msg: Bool, which: str):
        try:
            self.obstacles[which] = bool(msg.data)
            if msg.data:
                self.log(f"Obstacle detected by {which}")
            self.update_status_panel()
        except Exception:
            pass

    def _on_control_mode(self, msg: ROSString):
        # reflect control mode change (if other nodes publish)
        try:
            self.current_mode = msg.data
            self.mode_btn.configure(text=f"Mode: {self.current_mode.upper()}")
            self.log(f"Control mode reported: {self.current_mode}")
        except Exception:
            pass

    def _on_joy(self, msg: Joy):
        try:
            lb_pressed = msg.buttons[4] == 1 if len(msg.buttons) > 4 else False
            rb_pressed = msg.buttons[5] == 1 if len(msg.buttons) > 5 else False
            is_active = lb_pressed or rb_pressed
            if is_active and not self.teleop_active:
                self.teleop_button.configure(text='Teleop Active', bg='#00cc00')
                self.teleop_active = True
                self.log("Teleop activated (bumper pressed)")
            elif not is_active and self.teleop_active:
                self.teleop_button.configure(text='Teleop Inactive', bg='#555555')
                self.teleop_active = False
                self.log("Teleop deactivated (bumper released)")
        except Exception as e:
            self.get_logger().warn(f"Joy callback error: {e}")

    # ---------------- GUI loops ----------------
    def _gui_video_loop(self):
        # update camera frames every 100 ms
        try:
            # main large RGB display (husky/drone)
            rgb_key = 'husky_rgb' if self.active_rgb_camera == 'husky' else 'drone_rgb'
            try:
                frame = self.camera_queues[rgb_key].get_nowait()
            except queue.Empty:
                frame = None
            if frame is not None and self.camera_enabled.get(rgb_key, True):
                self._update_label_with_frame(self.lbl_switchable_rgb, frame)

            # small drone_ir
            try:
                frame_ir = self.camera_queues['drone_ir'].get_nowait()
            except queue.Empty:
                frame_ir = None
            if frame_ir is not None and self.camera_enabled.get('drone_ir', True):
                self._update_label_with_frame(self.lbl_drone_ir, frame_ir)

            # small husky small view
            try:
                frame_hsmall = self.camera_queues['husky_rgb'].get_nowait()
            except queue.Empty:
                frame_hsmall = None
            if frame_hsmall is not None and self.camera_enabled.get('husky_rgb', True):
                self._update_label_with_frame(self.lbl_husky_cam_small, frame_hsmall)

            # SLAM map small label area (if available)
            try:
                map_frame = self.camera_queues['slam_map'].get_nowait()
            except queue.Empty:
                map_frame = None
            if map_frame is not None:
                self._update_label_with_frame(self.lbl_slam_map, map_frame)

        finally:
            self.root.after(100, self._gui_video_loop)

    def _gui_map_loop(self):
        # Update 2D map with terrain, paths, positions, fires
        try:
            self.ax.clear()
            self.ax.set_xlim(-self.world_size/2, self.world_size/2)
            self.ax.set_ylim(-self.world_size/2, self.world_size/2)
            self.ax.set_aspect('equal', 'box')
            self.ax.grid(True, alpha=0.3)

            # terrain
            if self.terrain_img is not None:
                extent = (-self.world_size/2, self.world_size/2, -self.world_size/2, self.world_size/2)
                self.ax.imshow(self.terrain_img, extent=extent, origin='lower', zorder=0, interpolation='bilinear', alpha=0.6)

            # paths
            if self.paths['husky']:
                xs, ys = zip(*self.paths['husky'])
                self.ax.plot(xs, ys, '-', color='blue', linewidth=1, alpha=0.7, label='Husky Path')
            if self.paths['drone']:
                xs, ys = zip(*self.paths['drone'])
                self.ax.plot(xs, ys, '-', color='green', linewidth=1, alpha=0.7, label='Drone Path')

            # robots
            if self.positions['husky']:
                x, y = self.positions['husky']
                self.ax.plot(x, y, 'bs', markersize=10)
                self.ax.text(x, y - 0.7, f"H({x:.1f},{y:.1f})", fontsize=8, ha='center')
            if self.positions['drone']:
                x, y = self.positions['drone']
                self.ax.plot(x, y, 'g^', markersize=10)
                self.ax.text(x, y + 0.7, f"D({x:.1f},{y:.1f})", fontsize=8, ha='center')

            # fires
            if self.fire_positions:
                fx, fy = zip(*self.fire_positions)
                self.ax.scatter(fx, fy, c='r', s=80)
                for i, (px, py) in enumerate(self.fire_positions):
                    self.ax.text(px + 0.4, py + 0.4, f"F{i+1}", color='red', weight='bold', fontsize=8)

            self.ax.legend(loc='upper right')
            self.canvas.draw_idle()
        except Exception as e:
            self.get_logger().warn(f'2D map refresh error: {e}')
        finally:
            self.root.after(500, self._gui_map_loop)

    def _update_label_with_frame(self, label: tk.Label, frame_bgr) -> None:
        # Resize frame preserving aspect ratio to fit label
        try:
            label.update_idletasks()
            target_w = min(640, max(160, label.winfo_width()))
            target_h = min(480, max(120, label.winfo_height()))
            h, w = frame_bgr.shape[:2]
            scale = min(target_w / w, target_h / h)
            new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
            resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            photo = ImageTk.PhotoImage(img)
            def apply():
                label.configure(image=photo, text='')
                label.image = photoion_bringup/scripts/steamdeck_gui.py 


            self.root.after(0, apply)
        except Exception as e:
            self.get_logger().warn(f'Failed to update label with frame: {e}')

    # ---------------- Status & Logging ----------------
    def log(self, text: str):
        ts = datetime.now().strftime("%H:%M:%S")
        try:
            self.log_box.config(state='normal')
            self.log_box.insert(tk.END, f"[{ts}] {text}\n")
            self.log_box.see(tk.END)
            self.log_box.config(state='disabled')
        except Exception:
            pass
        try:
            self.get_logger().info(text)
        except Exception:
            pass

    def update_status_panel(self):
        # fire count
        try:
            self.fire_count_var.set(f"Fires: {len(self.fire_positions)}")
        except Exception:
            self.fire_count_var.set("Fires: N/A")

        # batteries
        try:
            if self.batteries.get('husky') is not None:
                self.husky_batt_pb['value'] = max(0, min(100, float(self.batteries['husky'])))
        except Exception:
            pass
        try:
            if self.batteries.get('drone') is not None:
                self.drone_batt_pb['value'] = max(0, min(100, float(self.batteries['drone'])))
        except Exception:
            pass

        # obstacles & sensors
        self.husky_obs_var.set(f"H obstacle: {self.obstacles.get('husky', 'N/A')}")
        self.drone_obs_var.set(f"D obstacle: {self.obstacles.get('drone', 'N/A')}")
        health_str = "OK" if all(self.sensor_health.values()) else "DEGRADED"
        self.sensor_health_var.set(f"Sensors: {health_str}")

        # nearest-fire distances
        def _nearest(which):
            pos = self.positions.get(which)
            if not pos or not self.fire_positions:
                return None
            distances = [math.hypot(pos[0] - fx, pos[1] - fy) for fx, fy in self.fire_positions]
            return min(distances) if distances else None

        hn = _nearest('husky')
        dn = _nearest('drone')
        if hn is not None:
            self.husky_nearest_var.set(f"H dist to nearest fire: {hn:.2f} m")
        else:
            self.husky_nearest_var.set("H dist to nearest fire: N/A")
        if dn is not None:
            self.drone_nearest_var.set(f"D dist to nearest fire: {dn:.2f} m")
        else:
            self.drone_nearest_var.set("D dist to nearest fire: N/A")

    def _status_tick(self):
        try:
            self.update_status_panel()
        except Exception:
            pass
        self.root.after(1000, self._status_tick)

    # ---------------- Timer / Clock ----------------
    def _update_time_display(self):
        elapsed = int(time.time() - self.start_time) if self.mission_running else 0
        hrs = elapsed // 3600
        mins = (elapsed % 3600) // 60
        secs = elapsed % 60
        self.timer_label.config(text=f"Mission Time: {hrs:02d}:{mins:02d}:{secs:02d}")
        self.system_time_label.config(text=time.strftime("%Y-%m-%d %H:%M:%S"))
        self.root.after(1000, self._update_time_display)

    # ---------------- Snapshot / Log export ----------------
    def save_snapshot(self):
        try:
            ts = datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
            folder = filedialog.askdirectory(title="Choose folder to save snapshot") or os.getcwd()
            outdir = os.path.join(folder, f"snapshot_{ts}")
            os.makedirs(outdir, exist_ok=True)
            # save map
            map_path = os.path.join(outdir, f"map_{ts}.png")
            self.fig.savefig(map_path)
            # save camera frames (if available)
            for key, q in self.camera_queues.items():
                try:
                    frame = q.get_nowait()
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(rgb)
                    img.save(os.path.join(outdir, f"{key}_{ts}.png"))
                except queue.Empty:
                    continue
            self.log(f"Snapshot saved to {outdir}")
            messagebox.showinfo("Snapshot", f"Saved to {outdir}")
        except Exception as e:
            messagebox.showerror("Snapshot error", f"Failed to save snapshot: {e}")

    def _export_log(self):
        try:
            fname = filedialog.asksaveasfilename(title="Save mission log", defaultextension=".txt",
                                                 filetypes=[("Text files","*.txt"),("All files","*.*")])
            if not fname:
                return
            with open(fname, 'w') as f:
                f.write(self.log_box.get('1.0', tk.END))
            messagebox.showinfo("Saved", f"Mission log saved to {fname}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save log: {e}")

    # ---------------- Run & teardown ----------------
def start_spin(node: Node):
    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()

def main():
    rclpy.init()
    root = tk.Tk()
    gui_node = SteamDeckGui(root)
    start_spin(gui_node)
    # start periodic status tick
    gui_node._status_tick()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()