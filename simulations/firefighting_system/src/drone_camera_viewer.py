#!/usr/bin/env python3

"""
Drone Camera Viewer

This script displays the RGB and thermal camera feeds from the firefighting drone
in real-time. Perfect for monitoring fire detection and surveillance operations.

Usage:
    python3 drone_camera_viewer.py

Controls:
    'q' - Quit
    's' - Save current frame
    'r' - Toggle RGB camera view
    't' - Toggle thermal camera view
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class DroneCameraViewer(Node):
    def __init__(self):
        super().__init__('drone_camera_viewer')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
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
        
        # Current images
        self.rgb_image = None
        self.thermal_image = None
        
        # Display flags
        self.show_rgb = True
        self.show_thermal = True
        
        # Status tracking
        self.rgb_received = False
        self.thermal_received = False
        
        self.get_logger().info("Drone Camera Viewer Started!")
        self.get_logger().info("Waiting for camera feeds...")
        
    def rgb_callback(self, msg):
        """Callback for RGB camera"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if not self.rgb_received:
                self.rgb_received = True
                self.get_logger().info("RGB camera feed connected!")
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")
            
    def thermal_callback(self, msg):
        """Callback for thermal camera"""
        try:
            # Convert thermal image (grayscale) to color for better visualization
            thermal_gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            
            # Apply thermal effect: invert and enhance contrast for heat simulation
            thermal_enhanced = cv2.equalizeHist(255 - thermal_gray)
            
            # Apply colormap for thermal visualization (hot colors)
            self.thermal_image = cv2.applyColorMap(thermal_enhanced, cv2.COLORMAP_JET)
            
            if not self.thermal_received:
                self.thermal_received = True
                self.get_logger().info("Thermal camera feed connected!")
        except Exception as e:
            self.get_logger().error(f"Thermal conversion error: {e}")
            
    def save_frame(self):
        """Save current camera frames"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        if self.rgb_image is not None:
            filename = f"drone_rgb_{timestamp}.jpg"
            cv2.imwrite(filename, self.rgb_image)
            self.get_logger().info(f"Saved RGB frame: {filename}")
            
        if self.thermal_image is not None:
            filename = f"drone_thermal_{timestamp}.jpg"
            cv2.imwrite(filename, self.thermal_image)
            self.get_logger().info(f"Saved thermal frame: {filename}")
            
    def run(self):
        """Main display loop"""
        self.get_logger().info("Press 'q' to quit, 's' to save frames, 'r' to toggle RGB, 't' to toggle thermal")
        
        while rclpy.ok():
            # Spin once to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Prepare display images
            display_images = []
            window_names = []
            
            # RGB camera display
            if self.show_rgb and self.rgb_image is not None:
                rgb_display = self.rgb_image.copy()
                
                # Add overlay text
                cv2.putText(rgb_display, "RGB Camera - Downward View", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(rgb_display, f"Resolution: {rgb_display.shape[1]}x{rgb_display.shape[0]}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                display_images.append(rgb_display)
                window_names.append("Drone RGB Camera")
                
            # Thermal camera display
            if self.show_thermal and self.thermal_image is not None:
                thermal_display = self.thermal_image.copy()
                
                # Add overlay text
                cv2.putText(thermal_display, "Thermal Camera - Heat Detection", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(thermal_display, f"Resolution: {thermal_display.shape[1]}x{thermal_display.shape[0]}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(thermal_display, "Hot = Red/Yellow, Cool = Blue/Purple", 
                           (10, thermal_display.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                display_images.append(thermal_display)
                window_names.append("Drone Thermal Camera")
            
            # Show status if no feeds
            if not display_images:
                if not self.rgb_received and not self.thermal_received:
                    status_img = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(status_img, "Waiting for camera feeds...", 
                               (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    cv2.putText(status_img, "Make sure drone simulation is running", 
                               (120, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                    display_images.append(status_img)
                    window_names.append("Drone Camera Status")
            
            # Display all images
            for img, name in zip(display_images, window_names):
                cv2.imshow(name, img)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Quitting camera viewer...")
                break
            elif key == ord('s'):
                self.save_frame()
            elif key == ord('r'):
                self.show_rgb = not self.show_rgb
                self.get_logger().info(f"RGB display: {'ON' if self.show_rgb else 'OFF'}")
            elif key == ord('t'):
                self.show_thermal = not self.show_thermal
                self.get_logger().info(f"Thermal display: {'ON' if self.show_thermal else 'OFF'}")
        
        # Cleanup
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = DroneCameraViewer()
        viewer.run()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            viewer.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
