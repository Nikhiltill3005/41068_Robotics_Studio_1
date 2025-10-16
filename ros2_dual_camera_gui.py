#!/usr/bin/env python3
"""
ROS2 Dual Camera GUI (Tkinter)
- Displays drone and ground vehicle camera feeds
- Subscribes to ROS2 topics and updates GUI thread-safely
- Minimal structure ready for adding controls
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
from typing import Optional
import queue
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
from PIL import Image, ImageTk


# ================== TOPIC CONFIG (EDIT HERE) ==================
# Set your ROS2 image topics here. The GUI can toggle the drone topic
# between IR and RGB by pressing a button in the Drone panel header.
DRONE_TOPIC_IR: str  = '/drone/ir_camera/image_raw'   # <- set your IR topic
DRONE_TOPIC_RGB: str = '/drone/fire_scan/debug_image'  # <- set your RGB topic
GROUND_TOPIC: str    = '/husky/camera/image'          # <- set your ground topic
# ==============================================================


class Ros2DualCameraGui(Node):
    def __init__(self, root: tk.Tk):
        super().__init__('ros2_dual_camera_gui')
        self.root = root
        self.root.title('Forest Fire Detection - Local GUI')
        self.root.geometry('1200x900')
        self.bridge = CvBridge()

        # Reduce native thread contention (helps avoid segfaults under load)
        try:
            os.environ.setdefault('OMP_NUM_THREADS', '1')
            os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
            os.environ.setdefault('MKL_NUM_THREADS', '1')
            cv2.setNumThreads(1)
        except Exception:
            pass

        # Latest frames (BGR OpenCV images)
        self._drone_frame = None
        self._ground_frame = None
        self._lock = threading.Lock()

        # Bounded queues to decouple ROS callbacks from GUI thread
        self._drone_q: queue.Queue = queue.Queue(maxsize=1)
        self._ground_q: queue.Queue = queue.Queue(maxsize=1)

        # Lightweight throttling (process every Nth frame)
        self._drone_ctr = 0
        self._ground_ctr = 0
        self._throttle_n = 2  # increase to drop more frames if needed

        # Build UI
        self._build_layout()

        # Track which drone topic is active
        self.current_drone_topic: str = DRONE_TOPIC_IR

        # ROS2 subscriptions (use sensor data QoS to avoid backpressure issues)
        self.drone_sub = self.create_subscription(ROSImage, self.current_drone_topic, self._on_drone_image, qos_profile_sensor_data)
        self.ground_sub = self.create_subscription(ROSImage, GROUND_TOPIC, self._on_ground_image, qos_profile_sensor_data)

        # Start periodic GUI refresh timer
        self._schedule_gui_refresh()

    # ---------------- UI -----------------
    def _build_layout(self) -> None:
        # Forest Green Color Scheme for Firefighting Robot
        BG = '#0f1b0f'            # dark forest green
        PANEL_BG = '#1a3d1a'      # forest green
        TEXT = '#e8f5e8'          # light forest green
        DRONE = '#4a7c4a'         # drone forest green
        GROUND = '#2d5a2d'        # ground forest green
        FIRE = '#ff4444'          # fire-red (kept for visibility)

        self.root.configure(bg=BG)
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass
        style.configure('Dark.TFrame', background=BG)
        style.configure('Panel.TFrame', background=PANEL_BG)
        style.configure('Dark.TLabel', background=BG, foreground=TEXT)
        style.configure('Panel.TLabel', background=PANEL_BG, foreground=TEXT)
        style.configure('Accent.TButton', foreground=TEXT, background=PANEL_BG)
        style.map('Accent.TButton', background=[('active', '#2d5a2d')])

        container = ttk.Frame(self.root, padding=12, style='Dark.TFrame')
        container.pack(fill=tk.BOTH, expand=True)

        # Title
        title = ttk.Label(container, text='Forest Fire Detection System', style='Dark.TLabel',
                          font=('Segoe UI', 18, 'bold'))
        title.pack(anchor=tk.W, pady=(0, 12))

        # Notebook with two tabs: Cameras (stacked vertically) and Map (empty placeholder)
        notebook = ttk.Notebook(container)
        notebook.pack(fill=tk.BOTH, expand=True)

        cameras_tab = ttk.Frame(notebook, style='Dark.TFrame')
        map_tab = ttk.Frame(notebook, style='Dark.TFrame')
        notebook.add(cameras_tab, text='Cameras')
        notebook.add(map_tab, text='Map (2D)')

        # Stack panels vertically in Cameras tab
        cameras_tab.columnconfigure(0, weight=1)
        cameras_tab.rowconfigure(0, weight=1)
        cameras_tab.rowconfigure(1, weight=1)

        # Drone panel (top)
        drone_panel = ttk.Frame(cameras_tab, padding=10, style='Panel.TFrame')
        drone_panel.grid(row=0, column=0, sticky='nsew', pady=(0, 8))
        header = ttk.Frame(drone_panel, style='Panel.TFrame')
        header.pack(fill=tk.X)
        ttk.Label(header, text='🚁 Drone Camera', style='Panel.TLabel',
                  font=('Segoe UI', 12, 'bold')).pack(side=tk.LEFT, anchor=tk.W)
        ttk.Button(header, text='Swap Topic', style='Accent.TButton',
                   command=self.toggle_drone_topic).pack(side=tk.RIGHT)
        self.drone_topic_label = ttk.Label(header, text='IR', width=6, anchor='center', style='Panel.TLabel')
        self.drone_topic_label.pack(side=tk.RIGHT, padx=(0, 8))
        self.drone_label = tk.Label(drone_panel, bg='#0f1b0f', fg='#e8f5e8',
                                    text='Waiting for drone frames...')
        self.drone_label.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        # Ground panel (bottom)
        ground_panel = ttk.Frame(cameras_tab, padding=10, style='Panel.TFrame')
        ground_panel.grid(row=1, column=0, sticky='nsew', pady=(8, 0))
        ttk.Label(ground_panel, text='🚗 Ground Vehicle Camera', style='Panel.TLabel',
                  font=('Segoe UI', 12, 'bold')).pack(anchor=tk.W)
        self.ground_label = tk.Label(ground_panel, bg='#0f1b0f', fg='#e8f5e8',
                                     text='Waiting for ground frames...')
        self.ground_label.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        # Map tab placeholder content (2D world map for fire locations)
        map_tab.columnconfigure(0, weight=1)
        map_tab.rowconfigure(0, weight=1)
        map_header = ttk.Frame(map_tab, padding=10, style='Panel.TFrame')
        map_header.grid(row=0, column=0, sticky='new')
        ttk.Label(map_header, text='🗺️ Fire Map (2D) - Placeholder', style='Panel.TLabel',
                  font=('Segoe UI', 12, 'bold')).pack(anchor=tk.W)
        # Empty canvas you can draw onto later
        self.map_canvas = tk.Canvas(map_tab, bg='#0f1b0f', highlightthickness=0)
        self.map_canvas.grid(row=1, column=0, sticky='nsew', padx=10, pady=(0, 10))
        # Example frame border inside canvas region (visual placeholder)
        self.map_canvas.create_rectangle(10, 10, 300, 200, outline=FIRE, width=3)
        self.map_canvas.create_text(20, 20, anchor='nw', fill='#e8f5e8',
                                    text='Add your 2D world rendering here...')

    # ------------- ROS callbacks ---------
    def _on_drone_image(self, msg: ROSImage) -> None:
        try:
            self._drone_ctr += 1
            if (self._drone_ctr % self._throttle_n) != 0:
                return
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # keep only the latest
            try:
                self._drone_q.get_nowait()
            except queue.Empty:
                pass
            self._drone_q.put_nowait(frame)
        except Exception as exc:
            self.get_logger().warn(f'Drone image conversion failed: {exc}')

    def _on_ground_image(self, msg: ROSImage) -> None:
        try:
            self._ground_ctr += 1
            if (self._ground_ctr % self._throttle_n) != 0:
                return
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # keep only the latest
            try:
                self._ground_q.get_nowait()
            except queue.Empty:
                pass
            self._ground_q.put_nowait(frame)
        except Exception as exc:
            self.get_logger().warn(f'Ground image conversion failed: {exc}')

    # ------------- GUI refresh -----------
    def _schedule_gui_refresh(self) -> None:
        # Refresh every ~100ms
        self.root.after(100, self._refresh_images)

    def _refresh_images(self) -> None:
        try:
            # Pull latest frames from queues (non-blocking)
            drone = None
            ground = None
            try:
                drone = self._drone_q.get_nowait()
            except queue.Empty:
                pass
            try:
                ground = self._ground_q.get_nowait()
            except queue.Empty:
                pass

            if drone is not None:
                self._update_label_with_frame(self.drone_label, drone)
            if ground is not None:
                self._update_label_with_frame(self.ground_label, ground)
        finally:
            self._schedule_gui_refresh()

    def _update_label_with_frame(self, label: tk.Label, frame_bgr) -> None:
        # Resize to fit label area while keeping aspect ratio
        label.update_idletasks()
        target_w = max(200, label.winfo_width())
        target_h = max(150, label.winfo_height())
        h, w = frame_bgr.shape[:2]
        scale = min(target_w / w, target_h / h)
        new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
        resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)

        # Convert to Tk image
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        photo = ImageTk.PhotoImage(image=image)

        # Thread-safe set on main thread
        def apply():
            label.configure(image=photo, text='')
            label.image = photo  # keep reference
        self.root.after(0, apply)

    # ------------- Topic switching ---------
    def toggle_drone_topic(self) -> None:
        """
        Toggle the drone subscription between DRONE_TOPIC_IR and DRONE_TOPIC_RGB.
        Safe to call from the Tkinter thread; uses rclpy Subscription management.
        """
        new_topic = DRONE_TOPIC_RGB if self.current_drone_topic == DRONE_TOPIC_IR else DRONE_TOPIC_IR
        try:
            # Destroy previous subscription and create a new one
            if self.drone_sub is not None:
                self.destroy_subscription(self.drone_sub)
            self.drone_sub = self.create_subscription(ROSImage, new_topic, self._on_drone_image, 10)
            self.current_drone_topic = new_topic
            # Update small label to show current mode
            self.drone_topic_label.configure(text='RGB' if new_topic == DRONE_TOPIC_RGB else 'IR')
            # Give visual feedback in the panel title area (optional print)
            self.get_logger().info(f'Switched drone topic to: {new_topic}')
        except Exception as exc:
            self.get_logger().error(f'Failed to switch drone topic: {exc}')


def start_ros_spin(node: Node) -> None:
    def spin():
        try:
            rclpy.spin(node)
        except Exception as exc:
            node.get_logger().error(f'ROS spin error: {exc}')
    t = threading.Thread(target=spin, daemon=True)
    t.start()


def main() -> None:
    rclpy.init()
    root = tk.Tk()
    app = Ros2DualCameraGui(root)
    start_ros_spin(app)

    try:
        root.mainloop()
    finally:
        app.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
