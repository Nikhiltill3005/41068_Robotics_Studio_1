#!/usr/bin/env python3
"""
steamdeck_gui.py
Merged GUI: Steam Deck layout + mission log + status panel + date/time/mission timer
Camera feeds: drone_rgb, husky_rgb (using compressed if available, raw fallback)
Fire Detection View: subscribes to /drone/fire_scan/debug_image
Other topics: odometry, fire positions, battery, obstacles, control mode
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

        # camera queues for latest frame only (drone_rgb, drone_ir, husky_rgb, fire_scan)
        self.camera_queues = {k: queue.Queue(maxsize=1) for k in ['drone_rgb', 'drone_ir', 'husky_rgb', 'fire_scan']}
        self.camera_enabled = {k: True for k in self.camera_queues.keys()}
        self._throttle_n = 2
        self._counters = {k: 0 for k in self.camera_queues.keys()}

        # positions and path info
        self.positions = {'husky': None, 'drone': None}
        self.paths = {'husky': [], 'drone': []}
        self.fire_positions = []

        # batteries, obstacles, sensor health
        self.batteries = {'husky': None, 'drone': None}
        self.obstacles = {'husky': False, 'drone': False}
        self.sensor_health = {'husky_camera': True, 'drone_camera': True}
        
        # firefighter status (from /firefighter/status_steamdeck)
        self.firefighter_state = "UNKNOWN"
        self.fires_detected = 0
        self.fires_extinguished = 0
        self.fires_since_charge = 0
        self.target_coords = "none"
        self.at_base = False
        
        # Track previous values to avoid duplicate logging
        self.prev_state = "UNKNOWN"
        self.prev_fires_detected = 0
        self.prev_fires_extinguished = 0
        self.prev_battery = None
        self.prev_at_base = False
        self.last_battery_log_time = 0
        
        # Track extinguished fires to remove from map
        self.extinguished_fire_positions = []
        self.extinguish_threshold = 2.0  # Distance threshold to consider a fire extinguished

        # control mode publisher
        self.control_pub = self.create_publisher(ROSString, self.topics['control_mode'], 10)
        self.current_mode = "autonomous"
        
        # Fire scan control publisher
        self.fire_scan_pub = self.create_publisher(Bool, '/drone/fire_scan/start_search', 10)
        self.scan_active = False

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
        
        # Tab/View management
        self.current_tab = "overview"  # overview, mission, maps

        # load terrain image
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

        # Fire scan debug image - subscribe to the fire detection debug visualization
        self.create_subscription(ROSImage, '/drone/fire_scan/debug_image', lambda m: self._on_image(m, 'fire_scan'), 10)

        # odometry
        self.create_subscription(Odometry, self.topics['husky_odom'], self._on_husky_odom, 10)
        self.create_subscription(Odometry, self.topics['drone_odom'], self._on_drone_odom, 10)

        # fire positions
        self.create_subscription(PoseArray, self.topics['fires'], self._on_fires, 10)

        # battery topics (now from status_steamdeck)
        self.create_subscription(ROSString, '/firefighter/status_steamdeck', self._on_firefighter_status, 10)

        # obstacle flags (optional)
        self.create_subscription(Bool, '/husky/obstacle', lambda m: self._on_obstacle(m, 'husky'), 10)
        self.create_subscription(Bool, '/drone/obstacle', lambda m: self._on_obstacle(m, 'drone'), 10)

        # teleop / joy input (for teleop_active visual)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        # control mode status topic
        self.create_subscription(ROSString, self.topics['control_mode'], self._on_control_mode, 10)
        
        # Fire scan status subscriber
        self.create_subscription(Bool, '/drone/fire_scan/start_search', self._on_scan_status, 10)

        # schedule GUI loops
        self.root.after(100, self._gui_video_loop)
        self.root.after(500, self._gui_map_loop)
        self.root.after(1000, self._update_time_display)

        self.get_logger().info("SteamDeck GUI initialized with topics: %s" % self.topics)
        
        # Log startup message
        self.log("Steam Deck GUI initialized - Mission started")

    def _build_ui(self):
        # window
        self.root.title('Robotics Control - Steam Deck')
        try:
            self.root.geometry('1280x800')    # Steam Deck native resolution
        except Exception:
            pass

        BG = '#0f172a'
        PANEL = '#1f2937'
        TEXT = '#e5e7eb'

        self.root.configure(bg=BG)
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass
        style.configure('Dark.TFrame', background=BG)
        style.configure('Panel.TFrame', background=PANEL)
        style.configure('Panel.TLabel', background=PANEL, foreground=TEXT)

        # ---------- TOP BAR ----------
        top = ttk.Frame(self.root, padding=6, style='Dark.TFrame')
        top.pack(fill=tk.X)

        # Tab buttons on the left
        tab_frame = tk.Frame(top, bg=BG)
        tab_frame.pack(side=tk.LEFT)
        
        self.tab_overview_btn = tk.Button(tab_frame, text="Overview", bg="#2563eb", fg="white",
                                         font=("Segoe UI", 10, "bold"), relief="flat", padx=12, pady=4,
                                         command=lambda: self.switch_tab("overview"))
        self.tab_overview_btn.pack(side=tk.LEFT, padx=(0, 2))
        
        self.tab_mission_btn = tk.Button(tab_frame, text="Mission Control", bg="#374151", fg="white",
                                        font=("Segoe UI", 10, "bold"), relief="flat", padx=12, pady=4,
                                        command=lambda: self.switch_tab("mission"))
        self.tab_mission_btn.pack(side=tk.LEFT, padx=2)
        
        self.tab_maps_btn = tk.Button(tab_frame, text="Maps", bg="#374151", fg="white",
                                      font=("Segoe UI", 10, "bold"), relief="flat", padx=12, pady=4,
                                      command=lambda: self.switch_tab("maps"))
        self.tab_maps_btn.pack(side=tk.LEFT, padx=2)

        # Control buttons in the middle
        control_frame = tk.Frame(top, bg=BG)
        control_frame.pack(side=tk.LEFT, padx=20)
        
        self.mode_btn = tk.Button(control_frame, text="Switch to MANUAL", bg="#0078d7", fg="white",
                                font=("Segoe UI", 9, "bold"), relief="flat", padx=8, pady=4,
                                command=self.toggle_mode)
        self.mode_btn.pack(side=tk.LEFT, padx=2)

        stop_btn = tk.Button(control_frame, text="E-STOP", bg="#e81123", fg="white",
                            font=("Segoe UI", 9, "bold"), relief="flat", padx=8, pady=4,
                            command=lambda: self.log("EMERGENCY STOP ACTIVATED!"))
        stop_btn.pack(side=tk.LEFT, padx=2)

        self.teleop_button = tk.Button(control_frame, text='Teleop Off', bg='#555555', fg='white',
                                    font=('Segoe UI', 9, 'bold'), relief='flat', padx=10, pady=4, state='disabled')
        self.teleop_button.pack(side=tk.LEFT, padx=2)

        # Status info on the right
        self.timer_label = ttk.Label(top, text="Mission: 00:00:00", font=("Segoe UI", 9))
        self.timer_label.pack(side=tk.RIGHT, padx=4)
        self.system_time_label = ttk.Label(top, text=time.strftime("%H:%M:%S"), font=("Segoe UI", 9))
        self.system_time_label.pack(side=tk.RIGHT, padx=4)

        # ---------- MAIN LAYOUT - CONTAINER FOR TABS ----------
        self.main_container = ttk.Frame(self.root)
        self.main_container.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        
        # Create all tab frames (they'll be shown/hidden)
        self.overview_frame = ttk.Frame(self.main_container)
        self.mission_frame = ttk.Frame(self.main_container)
        self.maps_frame = ttk.Frame(self.main_container)

        # Build each tab
        self._build_overview_tab()
        self._build_mission_tab()
        self._build_maps_tab()
        
        # Show overview tab by default
        self.switch_tab("overview")
    
    def _build_overview_tab(self):
        """Build the Overview tab: Camera + Fire Detection + Quick Status"""
        self.overview_frame.columnconfigure(0, weight=2)
        self.overview_frame.columnconfigure(1, weight=1)
        self.overview_frame.rowconfigure(0, weight=1)
        
        # LEFT: Cameras
        cam_panel = ttk.Frame(self.overview_frame)
        cam_panel.grid(row=0, column=0, sticky='nsew', padx=(0, 4))
        cam_panel.rowconfigure(0, weight=2)
        cam_panel.rowconfigure(1, weight=1)
        cam_panel.columnconfigure(0, weight=1)
        
        # RGB Camera
        cam_frame = ttk.LabelFrame(cam_panel, text="RGB Camera", padding=4)
        cam_frame.grid(row=0, column=0, sticky='nsew', pady=(0, 4))
        cam_frame.rowconfigure(1, weight=1)
        cam_frame.columnconfigure(0, weight=1)
        
        self.cam_toggle_btn = tk.Button(cam_frame, text="View: Husky RGB", bg="#2563eb", fg="white",
                                        font=("Segoe UI", 10, "bold"), relief="flat",
                                        command=self._toggle_rgb_camera)
        self.cam_toggle_btn.grid(row=0, column=0, sticky="ew", pady=4)
        
        self.lbl_switchable_rgb = tk.Label(cam_frame, bg='black', fg='white', text='Waiting for RGB...')
        self.lbl_switchable_rgb.grid(row=1, column=0, sticky='nsew', pady=4)
        
        # Fire Detection View
        fire_frame = ttk.LabelFrame(cam_panel, text="Fire Detection View", padding=4)
        fire_frame.grid(row=1, column=0, sticky='nsew')
        
        self.lbl_fire_scan = tk.Label(fire_frame, bg='black', fg='white', text='Waiting for fire scan...')
        self.lbl_fire_scan.pack(fill=tk.BOTH, expand=True)
        
        # RIGHT: Quick Status
        status_panel = ttk.LabelFrame(self.overview_frame, text="Mission Status", padding=6)
        status_panel.grid(row=0, column=1, sticky='nsew', padx=(4, 0))
        
        ttk.Label(status_panel, text="Husky Battery", font=("Segoe UI", 10, "bold")).pack(anchor="w")
        self.husky_batt_pb = ttk.Progressbar(status_panel, length=200, mode="determinate", maximum=100)
        self.husky_batt_pb.pack(anchor="w", pady=2)
        self.batt_label = ttk.Label(status_panel, text="--.--%", font=("Segoe UI", 9))
        self.batt_label.pack(anchor="w", pady=(0, 8))
        
        self.state_var = tk.StringVar(value="State: UNKNOWN")
        ttk.Label(status_panel, textvariable=self.state_var, font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(8, 4))
        
        self.fire_count_var = tk.StringVar(value="Detected: 0")
        ttk.Label(status_panel, textvariable=self.fire_count_var, font=("Segoe UI", 9)).pack(anchor="w", pady=2)
        self.extinguished_var = tk.StringVar(value="Extinguished: 0")
        ttk.Label(status_panel, textvariable=self.extinguished_var, font=("Segoe UI", 9)).pack(anchor="w", pady=2)
        self.since_charge_var = tk.StringVar(value="Since charge: 0")
        ttk.Label(status_panel, textvariable=self.since_charge_var, font=("Segoe UI", 9)).pack(anchor="w", pady=2)
        
        self.target_var = tk.StringVar(value="Target: none")
        ttk.Label(status_panel, textvariable=self.target_var, font=("Segoe UI", 9)).pack(anchor="w", pady=(8, 2))
        
        self.sensor_health_var = tk.StringVar(value="Sensors: OK")
        ttk.Label(status_panel, textvariable=self.sensor_health_var, font=("Segoe UI", 9)).pack(anchor="w", pady=(8, 0))
    
    def _build_mission_tab(self):
        """Build the Mission Control tab: Scan controls + Status + Mission Log"""
        self.mission_frame.columnconfigure(0, weight=1)
        self.mission_frame.rowconfigure(0, weight=0)
        self.mission_frame.rowconfigure(1, weight=1)
        
        # Mission Control Panel
        ctrl_panel = ttk.LabelFrame(self.mission_frame, text="Fire Scan Control", padding=10)
        ctrl_panel.grid(row=0, column=0, sticky='ew', padx=4, pady=(0, 8))
        
        # Scan buttons
        btn_frame = tk.Frame(ctrl_panel, bg='#1f2937')
        btn_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.scan_start_btn = tk.Button(btn_frame, text="Start Fire Scan", bg="#10b981", fg="white",
                                        font=("Segoe UI", 11, "bold"), relief="flat", padx=20, pady=8,
                                        command=self.start_fire_scan)
        self.scan_start_btn.pack(side=tk.LEFT, padx=(0, 8))
        
        self.scan_stop_btn = tk.Button(btn_frame, text="Pause Fire Scan", bg="#f59e0b", fg="white",
                                       font=("Segoe UI", 11, "bold"), relief="flat", padx=20, pady=8,
                                       command=self.stop_fire_scan, state='disabled')
        self.scan_stop_btn.pack(side=tk.LEFT)
        
        self.scan_status_var = tk.StringVar(value="Scan Status: Inactive")
        ttk.Label(ctrl_panel, textvariable=self.scan_status_var, font=("Segoe UI", 10)).pack(anchor="w", pady=(0, 10))
        
        # Search pattern
        ttk.Label(ctrl_panel, text="Search Pattern Spacing:", font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(10, 4))
        self.pattern_var = tk.StringVar(value="10m")
        pattern_frame = tk.Frame(ctrl_panel, bg='#1f2937')
        pattern_frame.pack(fill=tk.X)
        
        tk.Radiobutton(pattern_frame, text="5m grid (Dense)", variable=self.pattern_var, value="5m",
                      bg='#1f2937', fg='#e5e7eb', selectcolor="#1f2937", font=("Segoe UI", 10),
                      command=self.update_search_pattern).pack(side=tk.LEFT, padx=(0, 10))
        tk.Radiobutton(pattern_frame, text="10m grid (Normal)", variable=self.pattern_var, value="10m",
                      bg='#1f2937', fg='#e5e7eb', selectcolor="#1f2937", font=("Segoe UI", 10),
                      command=self.update_search_pattern).pack(side=tk.LEFT, padx=10)
        tk.Radiobutton(pattern_frame, text="15m grid (Fast)", variable=self.pattern_var, value="15m",
                      bg='#1f2937', fg='#e5e7eb', selectcolor="#1f2937", font=("Segoe UI", 10),
                      command=self.update_search_pattern).pack(side=tk.LEFT, padx=10)
        
        # Mission Log
        log_frame = ttk.LabelFrame(self.mission_frame, text="Mission Log", padding=6)
        log_frame.grid(row=1, column=0, sticky='nsew', padx=4)
        self.log_box = scrolledtext.ScrolledText(log_frame, bg="#f7f9fb", fg="#1e2a38", font=("Consolas", 10), wrap=tk.WORD)
        self.log_box.pack(fill=tk.BOTH, expand=True)
        self.log_box.config(state="disabled")
    
    def _build_maps_tab(self):
        """Build the Maps tab: Full-screen 2D terrain map"""
        self.maps_frame.columnconfigure(0, weight=1)
        self.maps_frame.rowconfigure(0, weight=1)
        
        terrain_frame = ttk.LabelFrame(self.maps_frame, text="2D Terrain Map - Robot Positions & Fire Locations", padding=6)
        terrain_frame.grid(row=0, column=0, sticky='nsew', padx=4, pady=4)
        
        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        self.ax.set_facecolor("#ffffff")
        self.ax.set_xlim(-self.world_size/2, self.world_size/2)
        self.ax.set_ylim(-self.world_size/2, self.world_size/2)
        self.ax.grid(True, alpha=0.3)
        self.canvas = FigureCanvasTkAgg(self.fig, master=terrain_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def switch_tab(self, tab_name):
        """Switch between different tabs"""
        # Hide all tabs
        self.overview_frame.pack_forget()
        self.mission_frame.pack_forget()
        self.maps_frame.pack_forget()
        
        # Reset button colors
        self.tab_overview_btn.config(bg="#374151")
        self.tab_mission_btn.config(bg="#374151")
        self.tab_maps_btn.config(bg="#374151")
        
        # Show selected tab and highlight button
        if tab_name == "overview":
            self.overview_frame.pack(fill=tk.BOTH, expand=True)
            self.tab_overview_btn.config(bg="#2563eb")
            self.current_tab = "overview"
        elif tab_name == "mission":
            self.mission_frame.pack(fill=tk.BOTH, expand=True)
            self.tab_mission_btn.config(bg="#2563eb")
            self.current_tab = "mission"
        elif tab_name == "maps":
            self.maps_frame.pack(fill=tk.BOTH, expand=True)
            self.tab_maps_btn.config(bg="#2563eb")
            self.current_tab = "maps"

    def _toggle_camera(self):
        # Example: toggle all cameras on/off
        for key in self.camera_enabled:
            self.camera_enabled[key] = not self.camera_enabled[key]
        self.log("Toggled camera feed on/off")

    def _toggle_rgb_camera(self):
        self.active_rgb_camera = 'drone' if self.active_rgb_camera == 'husky' else 'husky'
        label = f"View: {self.active_rgb_camera.capitalize()} RGB"
        self.cam_toggle_btn.config(text=label)
        self.log(f"Camera: {self.active_rgb_camera.capitalize()} RGB")


    # ---------------- TELEOP / CONTROL ----------------
    def toggle_mode(self):
        self.current_mode = "manual" if self.current_mode == "autonomous" else "autonomous"
        m = ROSString()
        m.data = self.current_mode
        self.control_pub.publish(m)
        btn_text = "Switch to AUTO" if self.current_mode == "manual" else "Switch to MANUAL"
        self.mode_btn.configure(text=btn_text)
        self.log(f"Mode: {self.current_mode.upper()}")
    
    def start_fire_scan(self):
        """Start the fire scanning pattern"""
        msg = Bool()
        msg.data = True
        self.fire_scan_pub.publish(msg)
        self.scan_active = True
        self.scan_start_btn.config(state='disabled')
        self.scan_stop_btn.config(state='normal')
        self.scan_status_var.set("Scan: Active")
        self.log("Fire scan started")
    
    def stop_fire_scan(self):
        """Stop the fire scanning pattern"""
        msg = Bool()
        msg.data = False
        self.fire_scan_pub.publish(msg)
        self.scan_active = False
        self.scan_start_btn.config(state='normal')
        self.scan_stop_btn.config(state='disabled')
        self.scan_status_var.set("Scan: Paused")
        self.log("Fire scan paused")
    
    def update_search_pattern(self):
        """Update search pattern spacing"""
        pattern = self.pattern_var.get()
        spacing = int(pattern.replace('m', ''))
        self.log(f"Search pattern updated: {spacing}m grid spacing")
        # Note: Pattern will take effect on next scan start
        # You could add a parameter service call here to update the node in real-time

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

    def _on_fires(self, msg: PoseArray):
        fires = []
        for p in msg.poses:
            fires.append((p.position.x, p.position.y))
        self.fire_positions = fires
        # Only log if fire count changed (don't spam the log)
        # This message is now redundant since we get fire info from status topic
        self.update_status_panel()

    def _on_firefighter_status(self, msg: ROSString):
        """
        Parse firefighter status message format:
        STATE|BATTERY|DETECTED|EXTINGUISHED|SINCE_CHARGE|TARGET_X,TARGET_Y|AT_BASE
        """
        try:
            parts = msg.data.split('|')
            if len(parts) >= 7:
                self.firefighter_state = parts[0]
                self.batteries['husky'] = float(parts[1])
                self.fires_detected = int(parts[2])
                self.fires_extinguished = int(parts[3])
                self.fires_since_charge = int(parts[4])
                self.target_coords = parts[5]
                self.at_base = parts[6] == "true"
                
                # Log only significant changes
                current_time = time.time()
                battery = self.batteries['husky']
                
                # Log state changes
                if self.firefighter_state != self.prev_state:
                    self.log(f"State changed: {self.firefighter_state}")
                    self.prev_state = self.firefighter_state
                
                # Log when new fires detected
                if self.fires_detected > self.prev_fires_detected:
                    new_count = self.fires_detected - self.prev_fires_detected
                    self.log(f"{new_count} new fire(s) detected! Total: {self.fires_detected}")
                    self.prev_fires_detected = self.fires_detected
                elif self.fires_detected < self.prev_fires_detected:
                    self.log(f"Fires detected updated: {self.fires_detected}")
                    self.prev_fires_detected = self.fires_detected
                
                # Log when fires extinguished and mark fire position as extinguished
                if self.fires_extinguished > self.prev_fires_extinguished:
                    self.log(f"Fire extinguished! Total put out: {self.fires_extinguished}")
                    # Find the nearest fire to husky position and mark it as extinguished
                    if self.positions.get('husky') and self.fire_positions:
                        husky_pos = self.positions['husky']
                        nearest_fire = None
                        nearest_dist = float('inf')
                        for fire in self.fire_positions:
                            dist = math.hypot(fire[0] - husky_pos[0], fire[1] - husky_pos[1])
                            if dist < nearest_dist and dist < self.extinguish_threshold:
                                nearest_dist = dist
                                nearest_fire = fire
                        if nearest_fire and nearest_fire not in self.extinguished_fire_positions:
                            self.extinguished_fire_positions.append(nearest_fire)
                            self.log(f"Fire at ({nearest_fire[0]:.1f}, {nearest_fire[1]:.1f}) removed from map")
                    self.prev_fires_extinguished = self.fires_extinguished
                
                # Log battery milestones (every 10%) and warnings
                if self.prev_battery is not None:
                    if battery <= 20 and self.prev_battery > 20:
                        self.log(f"LOW BATTERY: {battery:.1f}%")
                    elif battery <= 10 and self.prev_battery > 10:
                        self.log(f"CRITICAL BATTERY: {battery:.1f}%")
                    elif int(battery / 10) != int(self.prev_battery / 10):
                        # Log every 10% change
                        if (current_time - self.last_battery_log_time) > 30:  # Don't spam
                            self.log(f"Battery: {battery:.1f}%")
                            self.last_battery_log_time = current_time
                self.prev_battery = battery
                
                # Log charging events
                if self.at_base and not self.prev_at_base:
                    self.log(f"Arrived at charging station (Battery: {battery:.1f}%)")
                elif not self.at_base and self.prev_at_base:
                    self.log(f"Departed charging station (Battery: {battery:.1f}%)")
                self.prev_at_base = self.at_base
                
                self.update_status_panel()
        except Exception as e:
            self.get_logger().warn(f"Failed to parse firefighter status: {e}")

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
    
    def _on_scan_status(self, msg: Bool):
        """Update scan status from fire scan node"""
        try:
            was_active = self.scan_active
            self.scan_active = msg.data
            
            if self.scan_active:
                self.scan_start_btn.config(state='disabled')
                self.scan_stop_btn.config(state='normal')
                self.scan_status_var.set("Scan: Active")
                if not was_active:
                    self.log("Fire scan resumed")
            else:
                self.scan_start_btn.config(state='normal')
                self.scan_stop_btn.config(state='disabled')
                self.scan_status_var.set("Scan: Paused")
                if was_active:
                    self.log("Fire scan auto-paused")
        except Exception as e:
            self.get_logger().warn(f"Scan status callback error: {e}")

    def _on_joy(self, msg: Joy):
        try:
            lb_pressed = msg.buttons[4] == 1 if len(msg.buttons) > 4 else False
            rb_pressed = msg.buttons[5] == 1 if len(msg.buttons) > 5 else False
            is_active = lb_pressed or rb_pressed
            if is_active and not self.teleop_active:
                self.teleop_button.configure(text='Teleop On', bg='#00cc00')
                self.teleop_active = True
                self.log("Manual control activated")
            elif not is_active and self.teleop_active:
                self.teleop_button.configure(text='Teleop Off', bg='#555555')
                self.teleop_active = False
                self.log("Manual control deactivated")
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

            # Fire scan debug image display
            try:
                fire_frame = self.camera_queues['fire_scan'].get_nowait()
            except queue.Empty:
                fire_frame = None
            if fire_frame is not None:
                self._update_label_with_frame(self.lbl_fire_scan, fire_frame)

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

            # fires (filter out extinguished ones)
            if self.fire_positions:
                # Filter out extinguished fires
                active_fires = []
                for fire in self.fire_positions:
                    is_extinguished = False
                    for ext_fire in self.extinguished_fire_positions:
                        # Check if this fire matches an extinguished one (within threshold)
                        dist = math.hypot(fire[0] - ext_fire[0], fire[1] - ext_fire[1])
                        if dist < 0.5:  # Small threshold for matching
                            is_extinguished = True
                            break
                    if not is_extinguished:
                        active_fires.append(fire)
                
                # Display only active (unextinguished) fires
                if active_fires:
                    fx, fy = zip(*active_fires)
                    self.ax.scatter(fx, fy, c='r', s=80, label='Active Fires')
                    for i, (px, py) in enumerate(active_fires):
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
                label.image = photo
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
        # Firefighter state
        try:
            self.state_var.set(f"State: {self.firefighter_state}")
        except Exception:
            self.state_var.set("State: UNKNOWN")
        
        # Fire statistics from firefighter status
        try:
            self.fire_count_var.set(f"Detected: {self.fires_detected}")
            self.extinguished_var.set(f"Extinguished: {self.fires_extinguished}")
            self.since_charge_var.set(f"Since charge: {self.fires_since_charge}")
            self.target_var.set(f"Target: {self.target_coords}")
        except Exception:
            pass

        # Husky battery (from firefighter status)
        try:
            if self.batteries.get('husky') is not None:
                batt_val = float(self.batteries['husky'])
                self.husky_batt_pb['value'] = max(0, min(100, batt_val))
                self.batt_label.config(text=f"{batt_val:.1f}%")
        except Exception:
            pass

        # sensor health
        health_str = "OK" if all(self.sensor_health.values()) else "DEGRADED"
        self.sensor_health_var.set(f"Sensors: {health_str}")

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
        self.timer_label.config(text=f"Mission: {hrs:02d}:{mins:02d}:{secs:02d}")
        self.system_time_label.config(text=time.strftime("%H:%M:%S"))
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
            self.log(f"Failed to save snapshot: {e}")
            messagebox.showerror("Snapshot error", f"Failed to save snapshot: {e}")

    def _export_log(self):
        try:
            fname = filedialog.asksaveasfilename(title="Save mission log", defaultextension=".txt",
                                                 filetypes=[("Text files","*.txt"),("All files","*.*")])
            if not fname:
                return
            with open(fname, 'w') as f:
                f.write(self.log_box.get('1.0', tk.END))
            self.log(f"Mission log exported to {fname}")
            messagebox.showinfo("Saved", f"Mission log saved to {fname}")
        except Exception as e:
            self.log(f"Failed to export log: {e}")
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
