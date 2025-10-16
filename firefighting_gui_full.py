#!/usr/bin/env python3

"""
Firefighting GUI - Extended

Command to run GUI: python3 firefighting_gui_full.py 
Camera feeds are locked at 320 x 400 px  (can adjust)

Press Tab key to switch between Drone and Husky teleop (not the display label in top menu)
Using SHIFT + W/A/S/D results in faster movement 

Features:
- Teleop mode toggle (manual/autonomous) (publishes to /control_mode)
- Live 4-camera display (drone/husky RGB + IR) with enable/disable toggles
- 2D map with robot positions, fire positions, path visualization
  - also publishes nav_msgs/Path for RViz when enabled
- Battery indicators for both robots (subscribes to std_msgs/Float32 topics)
- Fire count & nearest-fire distance metrics
- Save snapshot (map + currently visible camera frames) to disk
- Mission timer + system time display
- Sensor status panel with simple health indicators
- Buttons: Clear paths, Toggle RViz publishing, Emergency Stop, Return Home
- Topics are parameters (change via ROS2 param system)
"""

import os
import time
import threading
import queue
import math
from datetime import datetime

import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
from PIL import Image, ImageTk
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image as ROSImage
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from std_msgs.msg import String, Float32, Bool
from cv_bridge import CvBridge

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class FirefightingGUI(Node):
    #def __init__(self, root: tk.Tk):
    def __init__(self, root):
        super().__init__('firefighting_gui_full')
        self.root = root # Save reference to Tkinter root window

        # ---------- PARAMETERS (changeable) ----------
        # Topics (declared as ROS params so user can override)
        self.declare_parameter('drone_rgb_topic', '/drone/fire_scan/debug_image')
        self.declare_parameter('drone_ir_topic', '/drone/ir_camera/image_raw')
        self.declare_parameter('husky_rgb_topic', '/husky/camera/image')
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

        # Get parameter values
        self.topics = {
            'drone_rgb': self.get_parameter('drone_rgb_topic').get_parameter_value().string_value,
            'drone_ir': self.get_parameter('drone_ir_topic').get_parameter_value().string_value,
            'husky_rgb': self.get_parameter('husky_rgb_topic').get_parameter_value().string_value,
            'husky_odom': self.get_parameter('husky_odom_topic').get_parameter_value().string_value,
            'drone_odom': self.get_parameter('drone_odom_topic').get_parameter_value().string_value,
            'fires': self.get_parameter('fire_topic').get_parameter_value().string_value,
            'control_mode': self.get_parameter('control_mode_topic').get_parameter_value().string_value,
            'husky_batt': self.get_parameter('husky_batt_topic').get_parameter_value().string_value,
            'drone_batt': self.get_parameter('drone_batt_topic').get_parameter_value().string_value,
            'rviz_path': self.get_parameter('rviz_path_topic').get_parameter_value().string_value,
            'husky_cmdvel': self.get_parameter('husky_cmdvel_topic').get_parameter_value().string_value,
            'drone_cmdvel': self.get_parameter('drone_cmdvel_topic').get_parameter_value().string_value,
        }
        self.world_size = self.get_parameter('world_size').get_parameter_value().double_value

        # ---------- Application state ----------
        self.bridge = CvBridge()
        self.camera_queues = {k: queue.Queue(maxsize=1) for k in ['drone_rgb', 'drone_ir', 'husky_rgb']}
        self.camera_enabled = {k: True for k in self.camera_queues.keys()}

        # Positions and paths
        self.positions = {'husky': None, 'drone': None}
        self.paths = {'husky': [], 'drone': []}  # lists of (x,y) for path visualization
        self.fire_positions = []  # list of (x,y)
        self.batteries = {'husky': None, 'drone': None}
        self.obstacles = {'husky': False, 'drone': False}
        self.sensor_health = {'husky_camera': True, 'drone_camera': True}  # simple flags
        self.start_time = time.time()
        self.mission_running = True

        # Control mode publisher
        self.control_pub = self.create_publisher(String, self.topics['control_mode'], 10)
        self.current_mode = "autonomous"
        self.path_pub = self.create_publisher(Path, '/visualization/path', 10)

        # ---------------- TELEOP ADDITION START ----------------
        # Publishers for manual teleoperation
        self.teleop_pub_husky = self.create_publisher(Twist, self.topics['husky_cmdvel'], 10)
        self.teleop_pub_drone = self.create_publisher(Twist, self.topics['drone_cmdvel'], 10)

        # Teleop parameters
        self.active_robot = 'drone'
        self.key_state = set()
        self.speed_normal = 0.5
        self.speed_turbo = 1.2
        self.angular_speed = 1.0
        self.altitude_speed = 0.4

        # bind key events
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.teleop_timer = self.create_timer(0.1, self.update_teleop)
        # ---------------- TELEOP ADDITION END ----------------

        self.setup_gui()
        self.root.after(100, self.gui_video_loop)
        self.root.after(500, self.gui_map_loop)
        self.root.after(1000, self.update_timer_display)

        self.get_logger().info('Firefighting GUI initialised.')
        # ---------------- TELEOP ADDITION END ----------------

        # Path publisher for RViz (optional)
        self.publish_rviz_path = False
        self.path_pub = self.create_publisher(Path, self.topics['rviz_path'], 10)

        # ---------- ROS2 subscriptions ----------
        # Camera image subscriptions
        # use lambda to forward topic key
        self.create_subscription(ROSImage, self.topics['drone_rgb'], lambda msg: self.image_cb(msg, 'drone_rgb'), 10)
        self.create_subscription(ROSImage, self.topics['drone_ir'], lambda msg: self.image_cb(msg, 'drone_ir'), 10)
        self.create_subscription(ROSImage, self.topics['husky_rgb'], lambda msg: self.image_cb(msg, 'husky_rgb'), 10)

        # Odometry -> positions and path history
        self.create_subscription(Odometry, self.topics['husky_odom'], self.husky_odom_cb, 10)
        self.create_subscription(Odometry, self.topics['drone_odom'], self.drone_odom_cb, 10)

        # Fires
        self.create_subscription(PoseArray, self.topics['fires'], self.fire_cb, 10)

        # Battery topics (assume std_msgs/Float32 battery percent)
        self.create_subscription(Float32, self.topics['husky_batt'], self.husky_batt_cb, 10)
        self.create_subscription(Float32, self.topics['drone_batt'], self.drone_batt_cb, 10)

        # Optional obstacle topics (Bool) - not required; if absent, 'N/A'
        self.create_subscription(Bool, '/husky/obstacle', lambda m: self.obstacle_cb(m, 'husky'), 10)
        self.create_subscription(Bool, '/drone/obstacle', lambda m: self.obstacle_cb(m, 'drone'), 10)

        # ---------- GUI layout ----------
        self.get_logger().info('Firefighting GUI initialized.')
        self.get_logger().info(f"Subscribed topics: {self.topics}")

    # -------------------- ROS CALLBACKS --------------------
    def image_cb(self, msg: ROSImage, key: str):
        # Convert ROS image to OpenCV BGR; store only if camera enabled
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.camera_enabled.get(key, True):
                # keep latest only
                try:
                    self.camera_queues[key].get_nowait()
                except queue.Empty:
                    pass
                self.camera_queues[key].put_nowait(frame)
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed for {key}: {e}")
            # mark sensor health false
            self.sensor_health[f"{'drone' if 'drone' in key else 'husky'}_camera"] = False

    def husky_odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions['husky'] = (x, y)
        self.paths['husky'].append((x, y))
        # optionally limit path length to avoid huge memory
        if len(self.paths['husky']) > 2000:
            self.paths['husky'].pop(0)
        # immediate map update
        self.schedule_map_update()

    def drone_odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions['drone'] = (x, y)
        self.paths['drone'].append((x, y))
        if len(self.paths['drone']) > 2000:
            self.paths['drone'].pop(0)
        self.schedule_map_update()

    def fire_cb(self, msg: PoseArray):
        fires = []
        for p in msg.poses:
            fires.append((p.position.x, p.position.y))
        self.fire_positions = fires
        # log
        self.log(f"Fire updates: {len(fires)} detected")
        self.schedule_map_update()

    def husky_batt_cb(self, msg: Float32):
        self.batteries['husky'] = float(msg.data)

    def drone_batt_cb(self, msg: Float32):
        self.batteries['drone'] = float(msg.data)

    def obstacle_cb(self, msg: Bool, which: str):
        self.obstacles[which] = bool(msg.data)
        if msg.data:
            self.log(f"Obstacle detected by {which}")

    # -------------------- GUI / Widgets --------------------
    def setup_gui(self):
        self.root.title("Firefighting Robot Control Center")
        self.root.geometry("1600x1000")
        self.root.configure(bg="#f0f8f0")

        # ---------- Forest Green Style ----------
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TButton", font=("Segoe UI", 10), padding=6, background="#2d5a2d", foreground="white")
        style.map("TButton", background=[("active", "#1a3d1a")])
        style.configure("TLabel", background="#f0f8f0", foreground="#1a3d1a", font=("Segoe UI", 10))
        style.configure("TLabelframe", background="#e8f5e8", foreground="#1a3d1a", padding=10)
        style.configure("TLabelframe.Label", font=("Segoe UI", 11, "bold"))
        style.configure("TCheckbutton", background="#e8f5e8", foreground="#1a3d1a")

        # ---------- Top Bar ----------
        topbar = ttk.Frame(self.root, padding=6)
        topbar.pack(fill=tk.X, pady=4)

        self.mode_btn = tk.Button(
            topbar, text="Switch to MANUAL", bg="#2d5a2d", fg="white",
            font=("Segoe UI", 10, "bold"), relief="flat", padx=8, pady=4,
            command=self.toggle_mode
        )
        self.mode_btn.pack(side=tk.LEFT, padx=4)

        stop_btn = tk.Button(
            topbar, text="EMERGENCY STOP", bg="#e81123", fg="white",
            font=("Segoe UI", 10, "bold"), relief="flat", padx=8, pady=4,
            command=self.emergency_stop
        )
        stop_btn.pack(side=tk.LEFT, padx=4)

        self.teleop_label = tk.Label(topbar, text=f"Control: {self.active_robot.upper()}",
                                    bg="#f0f8f0", fg="#1a3d1a", font=("Segoe UI", 10))
        self.teleop_label.pack(side=tk.LEFT, padx=10)

        ttk.Button(topbar, text="Return Home", command=lambda: self.log("Return Home triggered")).pack(side=tk.LEFT, padx=4)
        ttk.Button(topbar, text="Clear Paths", command=self.clear_paths).pack(side=tk.LEFT, padx=4)
        self.rviz_toggle_btn = ttk.Button(topbar, text="Enable RViz Path Pub", command=self.toggle_rviz_publish)
        self.rviz_toggle_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(topbar, text="Save Snapshot", command=self.save_snapshot).pack(side=tk.LEFT, padx=4)

        self.timer_label = ttk.Label(topbar, text="Mission Time: 00:00:00")
        self.timer_label.pack(side=tk.RIGHT, padx=8)
        self.system_time_label = ttk.Label(topbar, text=time.strftime("%Y-%m-%d %H:%M:%S"))
        self.system_time_label.pack(side=tk.RIGHT, padx=8)

        # ---------- Camera + Map ----------
        middle = ttk.Frame(self.root)
        middle.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        cam_frame = ttk.LabelFrame(middle, text="Camera Feeds", padding=8)
        cam_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))

        self.cam_labels = {}
        cams = ["drone_rgb", "drone_ir", "husky_rgb"]
        for idx, key in enumerate(cams):
            lbl_frame = tk.Frame(cam_frame, width=320, height=240, bg="#d4e6d4", highlightbackground="#2d5a2d", highlightthickness=2)
            lbl_frame.grid(row=idx // 2, column=idx % 2, padx=6, pady=6)
            lbl_frame.grid_propagate(False)
            lbl = tk.Label(lbl_frame, text=f"Waiting for {key}", bg="#d4e6d4", fg="#1a3d1a", anchor="center")
            lbl.pack(fill=tk.BOTH, expand=True)
            self.cam_labels[key] = lbl

        # Checkbox row
        chk_frame = ttk.Frame(cam_frame)
        chk_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        for key in cams:
            var = tk.BooleanVar(value=True)
            chk = ttk.Checkbutton(chk_frame, text=f"{key}", variable=var, command=lambda k=key, v=var: self.toggle_camera(k, v.get()))
            chk.pack(side=tk.LEFT, padx=6)
            setattr(self, f"{key}_enabled_var", var)

        # ---------- Map ----------
        map_frame = ttk.LabelFrame(middle, text="2D Map", padding=8)
        map_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.fig.patch.set_facecolor("#f0f8f0")
        self.ax.set_facecolor("#e8f5e8")
        self.ax.grid(True, color="#2d5a2d", alpha=0.3)
        self.ax.set_xlim(-self.world_size/2, self.world_size/2)
        self.ax.set_ylim(-self.world_size/2, self.world_size/2)
        self.ax.set_title("Map (meters)", color="#1a3d1a", fontsize=11)
        self.husky_marker, = self.ax.plot([], [], "bs", markersize=10, label="Husky Path", color="#2d5a2d")
        self.drone_marker, = self.ax.plot([], [], "g^", markersize=10, label="Drone Path", color="#4a7c4a")
        self.fire_scatter = self.ax.scatter([], [], c="#ff4444", s=80, label="Fire")
        self.ax.legend(loc="upper right")
        self.canvas = FigureCanvasTkAgg(self.fig, master=map_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # ---------- Bottom: Status + Log ----------
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
        self.log_box = scrolledtext.ScrolledText(log_frame, height=10, bg="#e8f5e8", fg="#1a3d1a", font=("Consolas", 9))
        self.log_box.pack(fill=tk.BOTH, expand=True)
        self.log_box.config(state="disabled")


    # ---------------- TELEOP ADDITION START ----------------
    def on_key_press(self, event):
        key = event.keysym.lower()
        self.key_state.add(key)

        if key == 'tab':
            self.active_robot = 'husky' if self.active_robot == 'drone' else 'drone'
            self.teleop_label.config(text=f"Control: {self.active_robot.upper()}")
            self.log(f"[Teleop] Switched control to {self.active_robot.upper()}")
        elif key == 'space':
            self.stop_movement()

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in self.key_state:
            self.key_state.remove(key)

    def update_teleop(self):
        if not self.key_state:
            return
        twist = Twist()
        speed = self.speed_turbo if 'shift_l' in self.key_state or 'shift_r' in self.key_state else self.speed_normal

        if self.active_robot == 'husky':
            if 'w' in self.key_state: twist.linear.x += speed
            if 's' in self.key_state: twist.linear.x -= speed
            if 'a' in self.key_state: twist.angular.z += self.angular_speed
            if 'd' in self.key_state: twist.angular.z -= self.angular_speed
            self.teleop_pub_husky.publish(twist)

        elif self.active_robot == 'drone':
            if 'w' in self.key_state: twist.linear.x += speed
            if 's' in self.key_state: twist.linear.x -= speed
            if 'a' in self.key_state: twist.linear.y += speed
            if 'd' in self.key_state: twist.linear.y -= speed
            if 'q' in self.key_state: twist.angular.z += self.angular_speed
            if 'e' in self.key_state: twist.angular.z -= self.angular_speed
            if 'r' in self.key_state: twist.linear.z += self.altitude_speed
            if 'f' in self.key_state: twist.linear.z -= self.altitude_speed
            self.teleop_pub_drone.publish(twist)

    def stop_movement(self):
        self.teleop_pub_husky.publish(Twist())
        self.teleop_pub_drone.publish(Twist())
        self.log("[Teleop] Emergency STOP issued!")
    # ---------------- TELEOP ADDITION END ----------------


    # ---------------- UI HELPERS ----------------
    def toggle_camera(self, key: str, enabled: bool):
        self.camera_enabled[key] = enabled
        self.log(f"Camera {key} {'enabled' if enabled else 'disabled'}")

    def toggle_mode(self):
        self.current_mode = "manual" if self.current_mode == "autonomous" else "autonomous"
        m = String()
        m.data = self.current_mode
        self.control_pub.publish(m)
        btn_text = "Switch to AUTO" if self.current_mode == "manual" else "Switch to MANUAL"
        self.mode_btn.configure(text=btn_text)
        self.log(f"Control mode changed to {self.current_mode.upper()}")

    def emergency_stop(self):
        self.stop_movement()  # publishes zero Twist (acting as a stop command)
        self.log("EMERGENCY STOP triggered!")

    def clear_paths(self):
        self.paths = {'husky': [], 'drone': []}
        self.log("Cleared stored paths.")
        self.schedule_map_update()

    def toggle_rviz_publish(self):
        self.publish_rviz_path = not self.publish_rviz_path
        text = "Disable RViz Path Pub" if self.publish_rviz_path else "Enable RViz Path Pub"
        self.rviz_toggle_btn.configure(text=text)
        self.log(f"RViz path publish {'enabled' if self.publish_rviz_path else 'disabled'}")

    def save_snapshot(self):
        # Save map figure and camera frames into a timestamped folder
        ts = datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
        folder = filedialog.askdirectory(title="Choose folder to save snapshot") or os.getcwd()
        outdir = os.path.join(folder, f"snapshot_{ts}")
        os.makedirs(outdir, exist_ok=True)
        # save map
        map_path = os.path.join(outdir, f"map_{ts}.png")
        self.fig.savefig(map_path)
        # save available camera frames
        for key, q in self.camera_queues.items():
            try:
                frame = q.get_nowait()
                # convert to BGR->RGB for PIL
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb)
                img.save(os.path.join(outdir, f"{key}_{ts}.png"))
            except queue.Empty:
                continue
        self.log(f"Snapshot saved to {outdir}")
        messagebox.showinfo("Snapshot", f"Saved to {outdir}")

    def log(self, text: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_box.config(state='normal')
        self.log_box.insert(tk.END, f"[{ts}] {text}\n")
        self.log_box.see(tk.END)
        self.log_box.config(state='disabled')
        self.get_logger().info(text)

    # ---------------- GUI LOOPS ----------------
    def gui_video_loop(self):
        # update camera frames every 100 ms (only if enabled)
        for key, lbl in self.cam_labels.items():
            if not self.camera_enabled.get(key, True):
                # show placeholder
                lbl.configure(text=f"{key} (disabled)", image='', bg='gray20')
                continue
            try:
                frame = self.camera_queues[key].get_nowait()
                w, h = 320, 240  # match CAM_W and CAM_H used in setup_gui()
                h0, w0 = frame.shape[:2]
                scale = min(w / w0, h / h0)
                new_w, new_h = int(w0 * scale), int(h0 * scale)
                resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
                rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                img = ImageTk.PhotoImage(Image.fromarray(rgb))
                lbl.configure(image=img, text='')
                lbl.image = img
            except queue.Empty:
                # no new frame; leave existing or show waiting
                pass
        self.root.after(100, self.gui_video_loop)

    def schedule_map_update(self):
        # throttle map updates by scheduling after short delay (avoid flooding)
        try:
            # use after_cancel/reschedule approach? For simplicity just call update soon
            self.root.after(200, self.gui_map_loop)
        except Exception:
            pass

    def gui_map_loop(self):
        # Update static elements (positions, paths, fires)
        self.ax.clear()
        self.ax.set_xlim(-self.world_size/2, self.world_size/2)
        self.ax.set_ylim(-self.world_size/2, self.world_size/2)
        self.ax.set_title("Map (meters)")
        self.ax.grid(True, alpha=0.3)

        # plot paths
        if self.paths['husky']:
            xs, ys = zip(*self.paths['husky'])
            self.ax.plot(xs, ys, '-', color='#2d5a2d', linewidth=2, alpha=0.8, label='Husky Path')
        if self.paths['drone']:
            xs, ys = zip(*self.paths['drone'])
            self.ax.plot(xs, ys, '-', color='#4a7c4a', linewidth=2, alpha=0.8, label='Drone Path')

        # plot robots
        if self.positions['husky']:
            x, y = self.positions['husky']
            self.ax.plot(x, y, 's', markersize=12, color='#2d5a2d', markeredgecolor='#1a3d1a', markeredgewidth=2)
            self.ax.text(x, y - 0.7, f"H({x:.1f},{y:.1f})", fontsize=8, ha='center', color='#1a3d1a', weight='bold')
        if self.positions['drone']:
            x, y = self.positions['drone']
            self.ax.plot(x, y, '^', markersize=12, color='#4a7c4a', markeredgecolor='#2d5a2d', markeredgewidth=2)
            self.ax.text(x, y + 0.7, f"D({x:.1f},{y:.1f})", fontsize=8, ha='center', color='#1a3d1a', weight='bold')

        # fires
        if self.fire_positions:
            fx, fy = zip(*self.fire_positions)
            self.ax.scatter(fx, fy, c='#ff4444', s=100, edgecolors='#cc0000', linewidth=2)
            for i, (px, py) in enumerate(self.fire_positions):
                self.ax.text(px + 0.4, py + 0.4, f"F{i+1}", color='#cc0000', weight='bold', fontsize=9)

        self.ax.legend(loc='upper right')
        self.canvas.draw_idle()

        # update status panel values
        self.fire_count_var.set(f"Fires: {len(self.fire_positions)}")
        self.husky_batt_var.set(f"H: {self.batteries['husky'] if self.batteries['husky'] is not None else 'N/A'}")
        self.drone_batt_var.set(f"D: {self.batteries['drone'] if self.batteries['drone'] is not None else 'N/A'}")
        self.husky_obs_var.set(f"H obstacle: {self.obstacles['husky']}")
        self.drone_obs_var.set(f"D obstacle: {self.obstacles['drone']}")
        # sensor health
        health_str = "OK" if all(self.sensor_health.values()) else "DEGRADED"
        self.sensor_health_var.set(f"Sensors: {health_str}")

        # nearest-fire distances
        if self.fire_positions:
            if self.positions['husky']:
                self.husky_nearest_var.set(f"H dist to nearest fire: {self._nearest_fire_distance('husky'):.2f} m")
            else:
                self.husky_nearest_var.set("H dist to nearest fire: N/A")
            if self.positions['drone']:
                self.drone_nearest_var.set(f"D dist to nearest fire: {self._nearest_fire_distance('drone'):.2f} m")
            else:
                self.drone_nearest_var.set("D dist to nearest fire: N/A")
        else:
            self.husky_nearest_var.set("H dist to nearest fire: N/A")
            self.drone_nearest_var.set("D dist to nearest fire: N/A")

        # publish path for RViz if enabled
        if self.publish_rviz_path:
            self._publish_paths_rviz()

    # ---------------- UTILITIES ----------------
    def _nearest_fire_distance(self, which: str) -> float:
        pos = self.positions.get(which)
        if not pos or not self.fire_positions:
            return float('nan')
        distances = [math.hypot(pos[0] - fx, pos[1] - fy) for fx, fy in self.fire_positions]
        return min(distances) if distances else float('nan')

    def _publish_paths_rviz(self):
        # Publish both paths as a single nav_msgs/Path (multiple options available)
        # We'll publish two separate Path messages (husky/drone) with different frame_id names
        now = self.get_clock().now().to_msg()
        # Husky Path
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = 'map'
        for (x, y) in self.paths['husky']:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)
        # Drone path could be published on the same topic with different header or other topic; keep simple.

    def update_timer_display(self):
        # update mission timer and system time
        elapsed = int(time.time() - self.start_time) if self.mission_running else 0
        hrs = elapsed // 3600
        mins = (elapsed % 3600) // 60
        secs = elapsed % 60
        self.timer_label.config(text=f"Mission Time: {hrs:02d}:{mins:02d}:{secs:02d}")
        self.system_time_label.config(text=time.strftime("%Y-%m-%d %H:%M:%S"))
        self.root.after(1000, self.update_timer_display)


def start_ros_spin(node: Node):
    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()


def main():
    rclpy.init()
    root = tk.Tk()
    gui_node = FirefightingGUI(root)
    start_ros_spin(gui_node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()