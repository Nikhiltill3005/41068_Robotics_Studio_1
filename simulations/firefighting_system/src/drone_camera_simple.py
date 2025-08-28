#!/usr/bin/env python3

"""
Simple Drone Camera Monitor

This script monitors the camera feeds from the firefighting drone without OpenCV
dependencies. It displays camera status and can save raw image data.

Usage:
    python3 drone_camera_simple.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time

class SimpleCameraMonitor(Node):
    def __init__(self):
        super().__init__('drone_camera_monitor')
        
        # Camera subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/drone/rgb_camera/image',
            self.rgb_callback,
            10
        )
        
        self.thermal_sub = self.create_subscription(
            Image,
            '/drone/thermal_camera/image',
            self.thermal_callback,
            10
        )
        
        # Status tracking
        self.rgb_count = 0
        self.thermal_count = 0
        self.rgb_last_time = 0
        self.thermal_last_time = 0
        
        # Timer for status updates
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info("=== Drone Camera Monitor Started ===")
        self.get_logger().info("Monitoring camera feeds...")
        self.get_logger().info("Press Ctrl+C to exit")
        
    def rgb_callback(self, msg):
        """Callback for RGB camera"""
        self.rgb_count += 1
        self.rgb_last_time = time.time()
        
        if self.rgb_count == 1:
            self.get_logger().info(f"📷 RGB Camera Connected!")
            self.get_logger().info(f"   Resolution: {msg.width}x{msg.height}")
            self.get_logger().info(f"   Encoding: {msg.encoding}")
            
    def thermal_callback(self, msg):
        """Callback for thermal camera"""
        self.thermal_count += 1
        self.thermal_last_time = time.time()
        
        if self.thermal_count == 1:
            self.get_logger().info(f"🔥 Thermal Camera Connected!")
            self.get_logger().info(f"   Resolution: {msg.width}x{msg.height}")
            self.get_logger().info(f"   Encoding: {msg.encoding}")
            
    def print_status(self):
        """Print periodic status updates"""
        current_time = time.time()
        
        # Calculate frame rates
        rgb_active = (current_time - self.rgb_last_time) < 3.0 if self.rgb_last_time > 0 else False
        thermal_active = (current_time - self.thermal_last_time) < 3.0 if self.thermal_last_time > 0 else False
        
        print("\n" + "="*50)
        print("DRONE CAMERA STATUS")
        print("="*50)
        
        # RGB Camera Status
        if self.rgb_count > 0:
            status = "🟢 ACTIVE" if rgb_active else "🔴 INACTIVE"
            print(f"📷 RGB Camera: {status}")
            print(f"   Frames received: {self.rgb_count}")
            if rgb_active:
                # Rough frame rate calculation
                if self.rgb_count > 10:
                    elapsed = current_time - (self.rgb_last_time - 2.0)
                    fps = min(10.0 / elapsed, 30.0) if elapsed > 0 else 0
                    print(f"   Estimated FPS: {fps:.1f}")
        else:
            print("📷 RGB Camera: 🔴 NO SIGNAL")
            
        # Thermal Camera Status  
        if self.thermal_count > 0:
            status = "🟢 ACTIVE" if thermal_active else "🔴 INACTIVE"
            print(f"🔥 Thermal Camera: {status}")
            print(f"   Frames received: {self.thermal_count}")
            if thermal_active:
                # Rough frame rate calculation
                if self.thermal_count > 5:
                    elapsed = current_time - (self.thermal_last_time - 2.0)
                    fps = min(5.0 / elapsed, 10.0) if elapsed > 0 else 0
                    print(f"   Estimated FPS: {fps:.1f}")
        else:
            print("🔥 Thermal Camera: 🔴 NO SIGNAL")
            
        print("="*50)
        
        # Instructions
        if self.rgb_count == 0 and self.thermal_count == 0:
            print("💡 Make sure the drone simulation is running!")
            print("   Start with: ./scripts/launch_integrated_sim.sh")
        elif self.rgb_count > 0 or self.thermal_count > 0:
            print("✅ Camera feeds are working!")
            print("📺 For visual display, fix NumPy/OpenCV compatibility")
            print("🎮 Control drone in another terminal")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = SimpleCameraMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n🛑 Camera monitor stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            monitor.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
