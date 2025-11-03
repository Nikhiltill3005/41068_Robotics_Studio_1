#!/usr/bin/env python3
"""
steamdeck_gui_modern.py
Modern tabbed GUI for Steam Deck with enhanced aesthetics
Tab 1: Camera feeds in 2x2 grid (Drone IR, UGV RGB, SLAM, Terrain map)
Tab 2: Fire detection list with positions
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

# Disable matplotlib threading
plt.ioff()


class ModernSteamDeckGui(Node):
    def __init__(self, root: tk.Tk):
        super().__init__('steamdeck_gui_modern')
        self.root = root

        # ---------- PARAMETERS ----------
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
        # self.declare_parameter('terrain_image', 'bushland_terrain.png')

        # Read parameters
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
        # self.terrain_image_path = self.get_parameter('terrain_image').get_parameter_value().string_value

        # ---------- Application state ----------
        self.bridge = CvBridge()
        try:
            os.environ.setdefault('OMP_NUM_THREADS', '1')
            os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
            os.environ.setdefault('MKL_NUM_THREADS', '1')
            cv2.setNumThreads(1)
        except Exception:
            pass

        # Camera queues
        self.camera_queues = {k: queue.Queue(maxsize=1) for k in ['drone_rgb', 'drone_ir', 'husky_rgb', 'slam_map']}
        self.camera_enabled = {k: True for k in self.camera_queues.keys()}
        self._throttle_n = 2
        self._counters = {k: 0 for k in self.camera_queues.keys()}

        # Positions and paths
        self.positions = {'husky': None, 'drone': None}
        self.paths = {'husky': [], 'drone': []}
        self.fire_positions = []

        # Batteries, obstacles, sensors
        self.batteries = {'husky': None, 'drone': None}
        self.obstacles = {'husky': False, 'drone': False}
        self.sensor_health = {'husky_camera': True, 'drone_camera': True}

        # Control mode
        self.control_pub = self.create_publisher(ROSString, self.topics['control_mode'], 10)
        self.current_mode = "autonomous"

        # Path publishing
        self.publish_rviz_path = False
        self.path_pub = self.create_publisher(Path, self.topics['rviz_path'], 10)

        # Teleop publishers
        self.teleop_pub_husky = self.create_publisher(Twist, self.get_parameter('husky_cmdvel_topic').get_parameter_value().string_value, 10)
        self.teleop_pub_drone = self.create_publisher(Twist, self.get_parameter('drone_cmdvel_topic').get_parameter_value().string_value, 10)
        self.teleop_active = False

        # Drone IR view mode (fire detection overlay vs. raw)
        self.ir_show_fire = False

        # Mission timing
        self.start_time = time.time()
        self.mission_running = True
        
        # Thread safety flags
        self._updating_video = False
        self._updating_map = False

        # Load terrain image
        # self.terrain_img = None
        # if self.terrain_image_path:
        #     try:
        #         p = os.path.expanduser(self.terrain_image_path)
        #         if not os.path.isabs(p):
        #             p = os.path.join(os.path.dirname(__file__), self.terrain_image_path)
        #         img = Image.open(p).convert('RGBA')
        #         self.terrain_img = np.asarray(img)
        #         self.get_logger().info(f"Loaded terrain image: {p}")
        #     except Exception as e:
        #         self.get_logger().warn(f"Failed to load terrain image: {e}")
        #         self.terrain_img = None

        # ---------- Build UI ----------
        self._build_ui()

        # ---------- ROS Subscriptions ----------
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Compressed subscriptions with fallback
        try:
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

        # Drone IR
        self.create_subscription(ROSImage, self.topics['drone_ir'], lambda m: self._on_image(m, 'drone_ir'), 10)

        # SLAM map
        map_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, self.topics['slam_map'], self._on_slam_map, map_qos)

        # Odometry
        self.create_subscription(Odometry, self.topics['husky_odom'], self._on_husky_odom, 10)
        self.create_subscription(Odometry, self.topics['drone_odom'], self._on_drone_odom, 10)

        # Fire positions
        self.create_subscription(PoseArray, self.topics['fires'], self._on_fires, 10)

        # Battery
        self.create_subscription(Float32, self.topics['husky_batt'], self._on_husky_batt, 10)
        self.create_subscription(Float32, self.topics['drone_batt'], self._on_drone_batt, 10)

        # Obstacles
        self.create_subscription(Bool, '/husky/obstacle', lambda m: self._on_obstacle(m, 'husky'), 10)
        self.create_subscription(Bool, '/drone/obstacle', lambda m: self._on_obstacle(m, 'drone'), 10)

        # Joystick
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        # Control mode
        self.create_subscription(ROSString, self.topics['control_mode'], self._on_control_mode, 10)

        # GUI update loops
        self.root.after(100, self._gui_video_loop)
        self.root.after(500, self._gui_map_loop)
        self.root.after(1000, self._update_time_display)
        self.root.after(500, self._update_fire_list)

        self.get_logger().info("Modern SteamDeck GUI initialized")

    def _build_ui(self):
        """Build modern tabbed UI"""
        self.root.title('Robotics Control - Modern Interface')
        self.root.geometry('1280x800')

        # Color scheme - modern dark theme
        BG = '#0a0e27'
        PANEL = '#1a1f3a'
        ACCENT = '#3b82f6'
        TEXT = '#e5e7eb'
        CARD = '#252b47'
        SUCCESS = '#10b981'
        WARNING = '#f59e0b'
        DANGER = '#ef4444'

        self.root.configure(bg=BG)
        
        # Configure styles
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass
        
        style.configure('Modern.TFrame', background=BG)
        style.configure('Card.TFrame', background=CARD, relief='flat')
        style.configure('Modern.TLabel', background=CARD, foreground=TEXT, font=('Segoe UI', 10))
        style.configure('Title.TLabel', background=CARD, foreground=TEXT, font=('Segoe UI', 12, 'bold'))
        style.configure('Modern.TNotebook', background=BG, borderwidth=0)
        style.configure('Modern.TNotebook.Tab', background=PANEL, foreground=TEXT, 
                       padding=[20, 10], font=('Segoe UI', 11, 'bold'))
        style.map('Modern.TNotebook.Tab', 
                 background=[('selected', ACCENT)],
                 foreground=[('selected', 'white')])

        # ---------- TOP BAR ----------
        top_bar = tk.Frame(self.root, bg=PANEL, height=60)
        top_bar.pack(fill=tk.X, pady=(0, 2))
        top_bar.pack_propagate(False)

        # Left side - Control buttons
        left_controls = tk.Frame(top_bar, bg=PANEL)
        left_controls.pack(side=tk.LEFT, padx=15, pady=10)

        self.mode_btn = tk.Button(left_controls, text="● AUTONOMOUS", bg=SUCCESS, fg='white',
                                font=("Segoe UI", 10, "bold"), relief="flat", padx=15, pady=8,
                                cursor='hand2', command=self.toggle_mode)
        self.mode_btn.pack(side=tk.LEFT, padx=5)

        stop_btn = tk.Button(left_controls, text="⏹ EMERGENCY STOP", bg=DANGER, fg='white',
                            font=("Segoe UI", 10, "bold"), relief="flat", padx=15, pady=8,
                            cursor='hand2', command=self._emergency_stop)
        stop_btn.pack(side=tk.LEFT, padx=5)

        self.teleop_indicator = tk.Label(left_controls, text='◉ Teleop', bg='#555555', fg='white',
                                    font=('Segoe UI', 10, 'bold'), padx=12, pady=8)
        self.teleop_indicator.pack(side=tk.LEFT, padx=5)

        # Right side - Time and info
        right_info = tk.Frame(top_bar, bg=PANEL)
        right_info.pack(side=tk.RIGHT, padx=15, pady=10)

        self.fire_count_header = tk.Label(right_info, text='🔥 Fires: 0', bg=PANEL, fg=WARNING,
                                         font=('Segoe UI', 12, 'bold'))
        self.fire_count_header.pack(side=tk.LEFT, padx=15)

        self.timer_label = tk.Label(right_info, text="⏱ 00:00:00", bg=PANEL, fg=TEXT,
                                    font=('Segoe UI', 11, 'bold'))
        self.timer_label.pack(side=tk.LEFT, padx=10)

        self.system_time_label = tk.Label(right_info, text=time.strftime("%H:%M:%S"), 
                                          bg=PANEL, fg='#9ca3af',
                                          font=('Segoe UI', 10))
        self.system_time_label.pack(side=tk.LEFT, padx=5)

        # ---------- NOTEBOOK (TABS) ----------
        self.notebook = ttk.Notebook(self.root, style='Modern.TNotebook')
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create tabs
        self.tab1 = tk.Frame(self.notebook, bg=BG)
        self.tab2 = tk.Frame(self.notebook, bg=BG)

        self.notebook.add(self.tab1, text='📹  CAMERA FEEDS')
        self.notebook.add(self.tab2, text='🔥  FIRE DETECTION')

        # Build tab contents
        self._build_tab1()
        self._build_tab2()

        # ---------- BOTTOM BAR ----------
        bottom_bar = tk.Frame(self.root, bg=PANEL, height=40)
        bottom_bar.pack(fill=tk.X, pady=(2, 0))
        bottom_bar.pack_propagate(False)

        # Status indicators
        status_frame = tk.Frame(bottom_bar, bg=PANEL)
        status_frame.pack(side=tk.LEFT, padx=15)

        tk.Label(status_frame, text='UGV:', bg=PANEL, fg='#9ca3af', 
                font=('Segoe UI', 9)).pack(side=tk.LEFT, padx=5)
        self.husky_batt_label = tk.Label(status_frame, text='100%', bg=PANEL, fg=SUCCESS,
                                         font=('Segoe UI', 9, 'bold'))
        self.husky_batt_label.pack(side=tk.LEFT)

        tk.Label(status_frame, text='  |  Drone:', bg=PANEL, fg='#9ca3af',
                font=('Segoe UI', 9)).pack(side=tk.LEFT, padx=5)
        self.drone_batt_label = tk.Label(status_frame, text='100%', bg=PANEL, fg=SUCCESS,
                                         font=('Segoe UI', 9, 'bold'))
        self.drone_batt_label.pack(side=tk.LEFT)

        # Action buttons
        action_frame = tk.Frame(bottom_bar, bg=PANEL)
        action_frame.pack(side=tk.RIGHT, padx=15)

        tk.Button(action_frame, text='📸 Snapshot', bg=CARD, fg=TEXT,
                 font=('Segoe UI', 9), relief='flat', padx=10, pady=5,
                 cursor='hand2', command=self.save_snapshot).pack(side=tk.LEFT, padx=3)
        
        tk.Button(action_frame, text='💾 Export Log', bg=CARD, fg=TEXT,
                 font=('Segoe UI', 9), relief='flat', padx=10, pady=5,
                 cursor='hand2', command=self._export_log).pack(side=tk.LEFT, padx=3)

    def _build_tab1(self):
        """Tab 1: Camera Feeds in 2x2 grid"""
        BG = '#0a0e27'
        CARD = '#252b47'
        TEXT = '#e5e7eb'
        ACCENT = '#3b82f6'

        # Main container
        container = tk.Frame(self.tab1, bg=BG)
        container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Configure grid weights for 2x2
        container.grid_rowconfigure(0, weight=1)
        container.grid_rowconfigure(1, weight=1)
        container.grid_columnconfigure(0, weight=1)
        container.grid_columnconfigure(1, weight=1)

        # ---------- TOP LEFT: Drone IR Camera ----------
        tl_frame = tk.Frame(container, bg=CARD, highlightbackground='#3f3f46', 
                           highlightthickness=2)
        tl_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)
        
        tl_header = tk.Frame(tl_frame, bg=CARD, height=45)
        tl_header.pack(fill=tk.X)
        tl_header.pack_propagate(False)
        
        tk.Label(tl_header, text='🌡️  DRONE IR CAMERA', bg=CARD, fg=TEXT,
                font=('Segoe UI', 11, 'bold')).pack(side=tk.LEFT, padx=10, pady=8)
        
        self.ir_mode_btn = tk.Button(tl_header, text='Raw View', bg='#4b5563', fg='white',
                                     font=('Segoe UI', 9), relief='flat', padx=12, pady=4,
                                     cursor='hand2', command=self._toggle_ir_mode)
        self.ir_mode_btn.pack(side=tk.RIGHT, padx=10, pady=8)
        
        self.lbl_drone_ir = tk.Label(tl_frame, bg='#000000', fg='#666666',
                                     text='Waiting for IR feed...',
                                     font=('Consolas', 10))
        self.lbl_drone_ir.pack(fill=tk.BOTH, expand=True, padx=3, pady=(0, 3))

        # ---------- BOTTOM LEFT: UGV RGBD Camera ----------
        bl_frame = tk.Frame(container, bg=CARD, highlightbackground='#3f3f46',
                           highlightthickness=2)
        bl_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        
        bl_header = tk.Frame(bl_frame, bg=CARD, height=45)
        bl_header.pack(fill=tk.X)
        bl_header.pack_propagate(False)
        
        tk.Label(bl_header, text='🚗  UGV RGBD CAMERA', bg=CARD, fg=TEXT,
                font=('Segoe UI', 11, 'bold')).pack(side=tk.LEFT, padx=10, pady=8)
        
        self.lbl_husky_rgb = tk.Label(bl_frame, bg='#000000', fg='#666666',
                                      text='Waiting for UGV feed...',
                                      font=('Consolas', 10))
        self.lbl_husky_rgb.pack(fill=tk.BOTH, expand=True, padx=3, pady=(0, 3))

        # ---------- TOP RIGHT: SLAM Map ----------
        tr_frame = tk.Frame(container, bg=CARD, highlightbackground='#3f3f46',
                           highlightthickness=2)
        tr_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)
        
        tr_header = tk.Frame(tr_frame, bg=CARD, height=45)
        tr_header.pack(fill=tk.X)
        tr_header.pack_propagate(False)
        
        tk.Label(tr_header, text='🗺️  SLAM MAP', bg=CARD, fg=TEXT,
                font=('Segoe UI', 11, 'bold')).pack(side=tk.LEFT, padx=10, pady=8)
        
        self.lbl_slam_map = tk.Label(tr_frame, bg='#000000', fg='#666666',
                                     text='Waiting for SLAM data...',
                                     font=('Consolas', 10))
        self.lbl_slam_map.pack(fill=tk.BOTH, expand=True, padx=3, pady=(0, 3))

        # ---------- BOTTOM RIGHT: 2D Terrain Map ----------
        br_frame = tk.Frame(container, bg=CARD, highlightbackground='#3f3f46',
                           highlightthickness=2)
        br_frame.grid(row=1, column=1, sticky='nsew', padx=5, pady=5)
        
        br_header = tk.Frame(br_frame, bg=CARD, height=45)
        br_header.pack(fill=tk.X)
        br_header.pack_propagate(False)
        
        tk.Label(br_header, text='🌍  2D TERRAIN MAP', bg=CARD, fg=TEXT,
                font=('Segoe UI', 11, 'bold')).pack(side=tk.LEFT, padx=10, pady=8)
        
        # Matplotlib figure for terrain map
        map_container = tk.Frame(br_frame, bg='#000000')
        map_container.pack(fill=tk.BOTH, expand=True, padx=3, pady=(0, 3))
        
        # Create figure with explicit backend
        self.fig = plt.figure(figsize=(6, 5), facecolor='#1a1a1a')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#1a1a1a")
        self.ax.set_xlim(-self.world_size/2, self.world_size/2)
        self.ax.set_ylim(-self.world_size/2, self.world_size/2)
        self.ax.grid(True, alpha=0.2, color='#4b5563')
        self.ax.tick_params(colors='#9ca3af', labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_color('#4b5563')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=map_container)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Image reference storage to prevent garbage collection
        self.image_refs = {}

    def _build_tab2(self):
        """Tab 2: Fire Detection List"""
        BG = '#0a0e27'
        CARD = '#252b47'
        TEXT = '#e5e7eb'
        WARNING = '#f59e0b'

        # Main container
        container = tk.Frame(self.tab2, bg=BG)
        container.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)

        # Header section
        header_frame = tk.Frame(container, bg=CARD, height=120)
        header_frame.pack(fill=tk.X, pady=(0, 20))
        header_frame.pack_propagate(False)

        # Fire count display
        count_frame = tk.Frame(header_frame, bg=CARD)
        count_frame.pack(expand=True)

        tk.Label(count_frame, text='🔥', bg=CARD, fg=WARNING,
                font=('Segoe UI', 48)).pack(side=tk.LEFT, padx=10)
        
        count_container = tk.Frame(count_frame, bg=CARD)
        count_container.pack(side=tk.LEFT, padx=10)
        
        tk.Label(count_container, text='FIRES DETECTED', bg=CARD, fg='#9ca3af',
                font=('Segoe UI', 12)).pack(anchor='w')
        
        self.fire_count_big = tk.Label(count_container, text='0', bg=CARD, fg=WARNING,
                                       font=('Segoe UI', 36, 'bold'))
        self.fire_count_big.pack(anchor='w')

        # Statistics frame
        stats_frame = tk.Frame(container, bg=CARD)
        stats_frame.pack(fill=tk.X, pady=(0, 20))
        
        stats_inner = tk.Frame(stats_frame, bg=CARD)
        stats_inner.pack(padx=20, pady=15)
        
        # Distance to nearest fire (Husky)
        husky_stat = tk.Frame(stats_inner, bg=CARD)
        husky_stat.pack(side=tk.LEFT, padx=20)
        tk.Label(husky_stat, text='UGV → Nearest Fire', bg=CARD, fg='#9ca3af',
                font=('Segoe UI', 10)).pack()
        self.husky_fire_dist = tk.Label(husky_stat, text='N/A', bg=CARD, fg=TEXT,
                                        font=('Segoe UI', 16, 'bold'))
        self.husky_fire_dist.pack()
        
        # Distance to nearest fire (Drone)
        drone_stat = tk.Frame(stats_inner, bg=CARD)
        drone_stat.pack(side=tk.LEFT, padx=20)
        tk.Label(drone_stat, text='Drone → Nearest Fire', bg=CARD, fg='#9ca3af',
                font=('Segoe UI', 10)).pack()
        self.drone_fire_dist = tk.Label(drone_stat, text='N/A', bg=CARD, fg=TEXT,
                                        font=('Segoe UI', 16, 'bold'))
        self.drone_fire_dist.pack()

        # Fire list section
        list_frame = tk.Frame(container, bg=CARD)
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        list_header = tk.Frame(list_frame, bg='#1f2937', height=40)
        list_header.pack(fill=tk.X)
        list_header.pack_propagate(False)
        
        tk.Label(list_header, text='FIRE LOCATIONS', bg='#1f2937', fg=TEXT,
                font=('Segoe UI', 11, 'bold')).pack(side=tk.LEFT, padx=15, pady=10)
        
        # Scrollable list
        list_container = tk.Frame(list_frame, bg=CARD)
        list_container.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Canvas for scrolling
        self.fire_canvas = tk.Canvas(list_container, bg=CARD, highlightthickness=0)
        scrollbar = ttk.Scrollbar(list_container, orient='vertical', 
                                 command=self.fire_canvas.yview)
        self.fire_list_frame = tk.Frame(self.fire_canvas, bg=CARD)
        
        self.fire_list_frame.bind(
            '<Configure>',
            lambda e: self.fire_canvas.configure(scrollregion=self.fire_canvas.bbox('all'))
        )
        
        self.fire_canvas.create_window((0, 0), window=self.fire_list_frame, anchor='nw')
        self.fire_canvas.configure(yscrollcommand=scrollbar.set)
        
        self.fire_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    def _toggle_ir_mode(self):
        """Toggle between raw IR view and fire detection view"""
        self.ir_show_fire = not self.ir_show_fire
        mode_text = 'Fire Detection' if self.ir_show_fire else 'Raw View'
        self.ir_mode_btn.config(text=mode_text, 
                               bg='#dc2626' if self.ir_show_fire else '#4b5563')
        self.get_logger().info(f"IR mode: {mode_text}")

    # ---------------- ROS CALLBACKS ----------------
    def _on_compressed_image(self, msg: CompressedImage, key: str):
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

    def _on_drone_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions['drone'] = (x, y)
        self.paths['drone'].append((x, y))
        if len(self.paths['drone']) > 2000:
            self.paths['drone'].pop(0)

    def _on_fires(self, msg: PoseArray):
        fires = []
        for p in msg.poses:
            fires.append((p.position.x, p.position.y))
        self.fire_positions = fires
        self.get_logger().info(f"Fire updates: {len(fires)} detected")

    def _on_husky_batt(self, msg: Float32):
        try:
            self.batteries['husky'] = float(msg.data)
        except Exception:
            pass

    def _on_drone_batt(self, msg: Float32):
        try:
            self.batteries['drone'] = float(msg.data)
        except Exception:
            pass

    def _on_obstacle(self, msg: Bool, which: str):
        try:
            self.obstacles[which] = bool(msg.data)
        except Exception:
            pass

    def _on_control_mode(self, msg: ROSString):
        try:
            self.current_mode = msg.data
        except Exception:
            pass

    def _on_joy(self, msg: Joy):
        try:
            lb_pressed = msg.buttons[4] == 1 if len(msg.buttons) > 4 else False
            rb_pressed = msg.buttons[5] == 1 if len(msg.buttons) > 5 else False
            is_active = lb_pressed or rb_pressed
            if is_active != self.teleop_active:
                self.teleop_active = is_active
                if is_active:
                    self.teleop_indicator.configure(text='◉ Teleop ACTIVE', bg='#10b981')
                else:
                    self.teleop_indicator.configure(text='◉ Teleop', bg='#555555')
        except Exception as e:
            self.get_logger().warn(f"Joy callback error: {e}")

    # ---------------- GUI UPDATE LOOPS ----------------
    def _gui_video_loop(self):
        """Update all camera feeds"""
        if self._updating_video:
            self.root.after(100, self._gui_video_loop)
            return
            
        self._updating_video = True
        try:
            # Drone IR
            try:
                frame = self.camera_queues['drone_ir'].get_nowait()
                if frame is not None and frame.size > 0:
                    self._update_label_with_frame(self.lbl_drone_ir, frame)
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().warn(f"Drone IR update error: {e}")

            # Husky RGB
            try:
                frame = self.camera_queues['husky_rgb'].get_nowait()
                if frame is not None and frame.size > 0:
                    self._update_label_with_frame(self.lbl_husky_rgb, frame)
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().warn(f"Husky RGB update error: {e}")

            # SLAM map
            try:
                frame = self.camera_queues['slam_map'].get_nowait()
                if frame is not None and frame.size > 0:
                    self._update_label_with_frame(self.lbl_slam_map, frame)
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().warn(f"SLAM map update error: {e}")

        except Exception as e:
            self.get_logger().error(f"Video loop error: {e}")
        finally:
            self._updating_video = False
            self.root.after(100, self._gui_video_loop)

    def _gui_map_loop(self):
        """Update 2D terrain map"""
        if self._updating_map:
            self.root.after(500, self._gui_map_loop)
            return
            
        self._updating_map = True
        try:
            self._update_terrain_map()
        except Exception as e:
            self.get_logger().warn(f'2D map refresh error: {e}')
        finally:
            self._updating_map = False
            self.root.after(500, self._gui_map_loop)
    
    def _update_terrain_map(self):
        """Update the terrain map visualization"""
        try:
            self.ax.clear()
            self.ax.set_facecolor("#1a1a1a")
            self.ax.set_xlim(-self.world_size/2, self.world_size/2)
            self.ax.set_ylim(-self.world_size/2, self.world_size/2)
            self.ax.set_aspect('equal', 'box')
            self.ax.grid(True, alpha=0.2, color='#4b5563')
            self.ax.tick_params(colors='#9ca3af', labelsize=8)
            for spine in self.ax.spines.values():
                spine.set_color('#4b5563')

            # Terrain background
            if self.terrain_img is not None:
                extent = (-self.world_size/2, self.world_size/2, 
                         -self.world_size/2, self.world_size/2)
                self.ax.imshow(self.terrain_img, extent=extent, origin='lower', 
                              zorder=0, interpolation='bilinear', alpha=0.5)

            # Paths
            if self.paths['husky']:
                xs, ys = zip(*self.paths['husky'])
                self.ax.plot(xs, ys, '-', color='#3b82f6', linewidth=2, 
                            alpha=0.8, label='UGV')
            if self.paths['drone']:
                xs, ys = zip(*self.paths['drone'])
                self.ax.plot(xs, ys, '-', color='#10b981', linewidth=2, 
                            alpha=0.8, label='Drone')

            # Current positions
            if self.positions['husky']:
                x, y = self.positions['husky']
                self.ax.plot(x, y, 'o', color='#3b82f6', markersize=12, 
                            markeredgecolor='white', markeredgewidth=2)
            if self.positions['drone']:
                x, y = self.positions['drone']
                self.ax.plot(x, y, '^', color='#10b981', markersize=12, 
                            markeredgecolor='white', markeredgewidth=2)

            # Fire positions
            if self.fire_positions:
                fx, fy = zip(*self.fire_positions)
                self.ax.scatter(fx, fy, c='#ef4444', s=150, marker='*', 
                               edgecolors='white', linewidths=2, zorder=10)
                for i, (px, py) in enumerate(self.fire_positions):
                    self.ax.text(px + 0.8, py + 0.8, f"F{i+1}", 
                                color='#ef4444', weight='bold', fontsize=10,
                                bbox=dict(boxstyle='round,pad=0.3', 
                                         facecolor='white', alpha=0.8))

            if self.paths['husky'] or self.paths['drone']:
                self.ax.legend(loc='upper right', framealpha=0.9, 
                              facecolor='#1f2937', edgecolor='#4b5563',
                              labelcolor='#e5e7eb')
            
            self.canvas.draw()
        except Exception as e:
            self.get_logger().warn(f'Terrain map update error: {e}')

    def _update_fire_list(self):
        """Update fire list in Tab 2"""
        try:
            # Update header count
            count = len(self.fire_positions)
            self.fire_count_header.config(text=f'🔥 Fires: {count}')
            self.fire_count_big.config(text=str(count))

            # Update distance displays
            def _nearest(which):
                pos = self.positions.get(which)
                if not pos or not self.fire_positions:
                    return None
                distances = [math.hypot(pos[0] - fx, pos[1] - fy) 
                           for fx, fy in self.fire_positions]
                return min(distances) if distances else None

            husky_dist = _nearest('husky')
            drone_dist = _nearest('drone')
            
            if husky_dist is not None:
                self.husky_fire_dist.config(text=f'{husky_dist:.2f} m')
            else:
                self.husky_fire_dist.config(text='N/A')
            
            if drone_dist is not None:
                self.drone_fire_dist.config(text=f'{drone_dist:.2f} m')
            else:
                self.drone_fire_dist.config(text='N/A')

            # Update battery displays
            if self.batteries.get('husky') is not None:
                batt_val = self.batteries['husky']
                color = '#10b981' if batt_val > 30 else '#f59e0b' if batt_val > 15 else '#ef4444'
                self.husky_batt_label.config(text=f'{batt_val:.0f}%', fg=color)
            
            if self.batteries.get('drone') is not None:
                batt_val = self.batteries['drone']
                color = '#10b981' if batt_val > 30 else '#f59e0b' if batt_val > 15 else '#ef4444'
                self.drone_batt_label.config(text=f'{batt_val:.0f}%', fg=color)

            # Rebuild fire list
            for widget in self.fire_list_frame.winfo_children():
                widget.destroy()

            if not self.fire_positions:
                no_fire = tk.Label(self.fire_list_frame, 
                                  text='No fires detected yet',
                                  bg='#252b47', fg='#9ca3af',
                                  font=('Segoe UI', 12, 'italic'),
                                  pady=40)
                no_fire.pack(fill=tk.X)
            else:
                for i, (x, y) in enumerate(self.fire_positions):
                    fire_card = tk.Frame(self.fire_list_frame, bg='#1f2937', 
                                        highlightbackground='#4b5563',
                                        highlightthickness=1)
                    fire_card.pack(fill=tk.X, padx=10, pady=5)
                    
                    inner = tk.Frame(fire_card, bg='#1f2937')
                    inner.pack(fill=tk.X, padx=15, pady=12)
                    
                    # Fire icon and ID
                    left = tk.Frame(inner, bg='#1f2937')
                    left.pack(side=tk.LEFT)
                    tk.Label(left, text='🔥', bg='#1f2937', 
                            font=('Segoe UI', 24)).pack(side=tk.LEFT, padx=(0, 10))
                    tk.Label(left, text=f'FIRE #{i+1}', bg='#1f2937', 
                            fg='#e5e7eb', font=('Segoe UI', 14, 'bold')).pack(side=tk.LEFT)
                    
                    # Position
                    right = tk.Frame(inner, bg='#1f2937')
                    right.pack(side=tk.RIGHT)
                    tk.Label(right, text=f'Position: ({x:.2f}, {y:.2f})', 
                            bg='#1f2937', fg='#9ca3af',
                            font=('Segoe UI', 11)).pack()

        except Exception as e:
            self.get_logger().warn(f'Fire list update error: {e}')
        finally:
            self.root.after(500, self._update_fire_list)

    def _update_label_with_frame(self, label: tk.Label, frame_bgr) -> None:
        """Update a label widget with an image frame"""
        try:
            # Get label dimensions
            label.update_idletasks()
            target_w = max(160, label.winfo_width())
            target_h = max(120, label.winfo_height())
            
            # Resize frame
            h, w = frame_bgr.shape[:2]
            if h == 0 or w == 0:
                return
                
            scale = min(target_w / w, target_h / h)
            new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
            
            resized = cv2.resize(frame_bgr, (new_w, new_h), 
                               interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            photo = ImageTk.PhotoImage(img)
            
            # Store reference to prevent garbage collection
            label_id = str(id(label))
            self.image_refs[label_id] = photo
            
            # Update on main thread
            label.configure(image=photo, text='')
            label.image = photo  # Additional reference
            
        except Exception as e:
            self.get_logger().warn(f'Failed to update label with frame: {e}')

    def _update_time_display(self):
        """Update time displays"""
        elapsed = int(time.time() - self.start_time) if self.mission_running else 0
        hrs = elapsed // 3600
        mins = (elapsed % 3600) // 60
        secs = elapsed % 60
        self.timer_label.config(text=f"⏱ {hrs:02d}:{mins:02d}:{secs:02d}")
        self.system_time_label.config(text=time.strftime("%H:%M:%S"))
        self.root.after(1000, self._update_time_display)

    # ---------------- CONTROL ACTIONS ----------------
    def toggle_mode(self):
        """Toggle between autonomous and manual mode"""
        self.current_mode = "manual" if self.current_mode == "autonomous" else "autonomous"
        m = ROSString()
        m.data = self.current_mode
        self.control_pub.publish(m)
        
        if self.current_mode == "autonomous":
            self.mode_btn.config(text="● AUTONOMOUS", bg='#10b981')
        else:
            self.mode_btn.config(text="● MANUAL", bg='#f59e0b')
        
        self.get_logger().info(f"Control mode: {self.current_mode.upper()}")

    def _emergency_stop(self):
        """Emergency stop action"""
        self.get_logger().warn("EMERGENCY STOP ACTIVATED!")
        messagebox.showwarning("Emergency Stop", 
                              "Emergency stop has been triggered!")

    def save_snapshot(self):
        """Save current state snapshot"""
        try:
            ts = datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
            folder = filedialog.askdirectory(title="Save Snapshot") or os.getcwd()
            outdir = os.path.join(folder, f"snapshot_{ts}")
            os.makedirs(outdir, exist_ok=True)
            
            # Save map
            map_path = os.path.join(outdir, f"map_{ts}.png")
            self.fig.savefig(map_path, dpi=150, facecolor='#1a1a1a')
            
            # Save camera frames
            for key, q in self.camera_queues.items():
                try:
                    frame = q.get_nowait()
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(rgb)
                    img.save(os.path.join(outdir, f"{key}_{ts}.png"))
                except queue.Empty:
                    continue
            
            self.get_logger().info(f"Snapshot saved to {outdir}")
            messagebox.showinfo("Snapshot Saved", f"Saved to:\n{outdir}")
        except Exception as e:
            messagebox.showerror("Snapshot Error", f"Failed: {e}")

    def _export_log(self):
        """Export mission log (placeholder - extend as needed)"""
        try:
            fname = filedialog.asksaveasfilename(
                title="Export Mission Log",
                defaultextension=".txt",
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
            )
            if not fname:
                return
            
            with open(fname, 'w') as f:
                f.write(f"Mission Log Export - {datetime.now()}\n")
                f.write(f"{'='*50}\n\n")
                f.write(f"Mission Time: {self.timer_label.cget('text')}\n")
                f.write(f"Fires Detected: {len(self.fire_positions)}\n")
                f.write(f"Control Mode: {self.current_mode}\n\n")
                f.write("Fire Positions:\n")
                for i, (x, y) in enumerate(self.fire_positions):
                    f.write(f"  Fire {i+1}: ({x:.2f}, {y:.2f})\n")
            
            messagebox.showinfo("Log Exported", f"Log saved to:\n{fname}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed: {e}")


def start_spin(node: Node):
    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()


def main():
    rclpy.init()
    root = tk.Tk()
    gui_node = ModernSteamDeckGui(root)
    start_spin(gui_node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()