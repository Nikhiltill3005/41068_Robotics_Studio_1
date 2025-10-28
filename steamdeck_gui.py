#!/usr/bin/env python3

"""
Steam Deck ROS2 GUI

- Optimized layout for 1280x800 (Steam Deck) with four video panes:
  - Drone RGB, Drone Depth, Husky RGB, Husky Depth
- Subscribes to teleop status from the combined joystick teleop node
- All topics are ROS2 parameters for easy override via launch file

Controls are provided by the existing combined_joy_teleop node (launched separately).
"""

import os
import threading
import queue
import tkinter as tk
from tkinter import ttk
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String as ROSString
from cv_bridge import CvBridge
import cv2
from PIL import Image, ImageTk


class SteamDeckGui(Node):
    def __init__(self, root: tk.Tk):
        super().__init__('steamdeck_gui')
        self.root = root

        # Parameters (defaults can be overridden via launch)
        # Drone
        self.declare_parameter('drone_rgb_topic', '/drone/camera/rgb/image_raw')
        self.declare_parameter('drone_depth_topic', '/drone/camera/depth/image_raw')
        # Husky
        self.declare_parameter('husky_rgb_topic', '/husky/camera/rgb/image_raw')
        self.declare_parameter('husky_depth_topic', '/husky/camera/depth/image_raw')
        # Teleop status topic (from combined_joy_teleop)
        self.declare_parameter('teleop_status_topic', '/teleop_status')

        self.topics = {
            'drone_rgb': self.get_parameter('drone_rgb_topic').get_parameter_value().string_value,
            'drone_depth': self.get_parameter('drone_depth_topic').get_parameter_value().string_value,
            'husky_rgb': self.get_parameter('husky_rgb_topic').get_parameter_value().string_value,
            'husky_depth': self.get_parameter('husky_depth_topic').get_parameter_value().string_value,
            'teleop_status': self.get_parameter('teleop_status_topic').get_parameter_value().string_value,
        }

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
        self.queues = {k: queue.Queue(maxsize=1) for k in ['drone_rgb', 'drone_depth', 'husky_rgb', 'husky_depth']}
        self._throttle_n = 2
        self._counters = {k: 0 for k in self.queues.keys()}

        # Build UI
        self._build_ui()

        # Subscriptions
        self.create_subscription(ROSImage, self.topics['drone_rgb'], lambda m: self._on_image(m, 'drone_rgb'), qos_profile_sensor_data)
        self.create_subscription(ROSImage, self.topics['drone_depth'], lambda m: self._on_image(m, 'drone_depth'), qos_profile_sensor_data)
        self.create_subscription(ROSImage, self.topics['husky_rgb'], lambda m: self._on_image(m, 'husky_rgb'), qos_profile_sensor_data)
        self.create_subscription(ROSImage, self.topics['husky_depth'], lambda m: self._on_image(m, 'husky_depth'), qos_profile_sensor_data)
        self.create_subscription(ROSString, self.topics['teleop_status'], self._on_status, 10)

        # Periodic refresh
        self._schedule_refresh()

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
        ACCENT = '#2563eb'  # blue-600

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

        # Top bar
        top = ttk.Frame(self.root, padding=8, style='Dark.TFrame')
        top.pack(fill=tk.X)
        self.status_label = ttk.Label(top, text='Teleop: —', style='Dark.TLabel', font=('Segoe UI', 12, 'bold'))
        self.status_label.pack(side=tk.LEFT)

        # 2x2 grid of video panes
        grid = ttk.Frame(self.root, padding=8, style='Dark.TFrame')
        grid.pack(fill=tk.BOTH, expand=True)
        for c in range(2):
            grid.columnconfigure(c, weight=1)
        for r in range(2):
            grid.rowconfigure(r, weight=1)

        def make_panel(parent, title: str):
            panel = ttk.Frame(parent, padding=8, style='Panel.TFrame')
            header = ttk.Label(panel, text=title, style='Panel.TLabel', font=('Segoe UI', 11, 'bold'))
            header.pack(anchor='w')
            label = tk.Label(panel, bg='black', fg='white', text='Waiting for frames...')
            label.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
            return panel, label

        p00, self.lbl_drone_rgb = make_panel(grid, 'Drone RGB')
        p01, self.lbl_drone_depth = make_panel(grid, 'Drone Depth')
        p10, self.lbl_husky_rgb = make_panel(grid, 'Husky RGB')
        p11, self.lbl_husky_depth = make_panel(grid, 'Husky Depth')
        p00.grid(row=0, column=0, sticky='nsew', padx=(0, 6), pady=(0, 6))
        p01.grid(row=0, column=1, sticky='nsew', padx=(6, 0), pady=(0, 6))
        p10.grid(row=1, column=0, sticky='nsew', padx=(0, 6), pady=(6, 0))
        p11.grid(row=1, column=1, sticky='nsew', padx=(6, 0), pady=(6, 0))

    def _on_status(self, msg: ROSString) -> None:
        try:
            text = msg.data if isinstance(msg.data, str) else str(msg.data)
            self.status_label.configure(text=f'Teleop: {text}')
        except Exception:
            pass

    def _on_image(self, msg: ROSImage, key: str) -> None:
        try:
            self._counters[key] += 1
            if (self._counters[key] % self._throttle_n) != 0:
                return
            # Depth often comes as 16UC1 or 32FC1. Try sensible conversions.
            encoding = msg.encoding.lower()
            if 'depth' in key and encoding in ('16uc1', 'mono16'):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                # Normalize for display
                disp = cv2.convertScaleAbs(cv_img, alpha=0.03)
                disp = cv2.applyColorMap(disp, cv2.COLORMAP_TURBO)
            elif 'depth' in key and encoding in ('32fc1',):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                clipped = cv2.threshold(cv_img, 10.0, 10.0, cv2.THRESH_TRUNC)[1]
                norm = cv2.normalize(clipped, None, 0, 255, cv2.NORM_MINMAX)
                disp = cv2.applyColorMap(norm.astype('uint8'), cv2.COLORMAP_TURBO)
            else:
                disp = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            try:
                self.queues[key].get_nowait()
            except queue.Empty:
                pass
            self.queues[key].put_nowait(disp)
        except Exception as exc:
            self.get_logger().warn(f'Image conversion failed for {key}: {exc}')

    def _schedule_refresh(self) -> None:
        self.root.after(100, self._refresh)

    def _refresh(self) -> None:
        try:
            mapping = [
                ('drone_rgb', self.lbl_drone_rgb),
                ('drone_depth', self.lbl_drone_depth),
                ('husky_rgb', self.lbl_husky_rgb),
                ('husky_depth', self.lbl_husky_depth),
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

    def _update_label_with_frame(self, label: tk.Label, frame_bgr) -> None:
        # Resize with aspect ratio to fill area
        label.update_idletasks()
        target_w = max(200, label.winfo_width())
        target_h = max(150, label.winfo_height())
        h, w = frame_bgr.shape[:2]
        scale = min(target_w / w, target_h / h)
        new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
        resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)

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


