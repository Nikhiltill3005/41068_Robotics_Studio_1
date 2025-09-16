#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscriber to thermal camera (reduced queue size to prevent lag)
        self.image_sub = self.create_subscription(
            Image,
            '/drone/ir_camera/image_raw',
            self.image_callback,
            1  # Reduced from 10 to 1 to prevent buffering lag
        )
        
        # Publisher for detected fire locations
        self.fire_location_pub = self.create_publisher(
            PointStamped,
            '/drone/fire_detection/location',
            10
        )
        
        # Publisher for debug image (optional)
        self.debug_image_pub = self.create_publisher(
            Image,
            '/drone/fire_detection/debug_image',
            10
        )
        
        # Detection parameters - tuned for thermal camera
        self.brightness_threshold = 200  # Adjust based on your camera format
        self.min_area = 50  # Minimum fire area in pixels
        self.max_area = 10000  # Maximum area to avoid false positives
        
        # Throttling to reduce processing load
        self.last_process_time = self.get_clock().now()
        self.process_interval = 0.5  # Process every 0.5 seconds instead of every frame
        self.frame_count = 0
        
        self.get_logger().info('Fire Detection Node started - monitoring thermal camera')

    def image_callback(self, msg):
        # Throttle processing to reduce CPU load
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_process_time).nanoseconds / 1e9
        
        if time_diff < self.process_interval:
            return  # Skip this frame
            
        self.last_process_time = current_time
        self.frame_count += 1
        
        try:
            # Convert ROS image to OpenCV based on encoding
            if msg.encoding == 'mono8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
                self.brightness_threshold = 200  # For 8-bit images (0-255)
            elif msg.encoding == 'mono16':
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono16")
                self.brightness_threshold = 50000  # For 16-bit images (0-65535)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                self.brightness_threshold = 200

            # Only log debug info occasionally to reduce CPU load
            if self.frame_count % 20 == 0:  # Log every 20th processed frame (every 10 seconds at 0.5s interval)
                min_val = np.min(cv_image)
                max_val = np.max(cv_image)
                mean_val = np.mean(cv_image)
                self.get_logger().info(f'Frame {self.frame_count} - Pixel range: {min_val}-{max_val}, Mean: {mean_val:.1f}')
            
            
            # Detect fires
            fire_locations = self.detect_fires(cv_image)
            
            # Publish fire locations
            for location in fire_locations:
                self.publish_fire_location(location, msg.header)
            
            # Create and publish debug image
            debug_image = self.create_debug_image(cv_image, fire_locations)
            self.publish_debug_image(debug_image, msg.header)
            
            if len(fire_locations) > 0:
                self.get_logger().info(f'FIRE DETECTED! {len(fire_locations)} fires at: {fire_locations}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing thermal image: {str(e)}')

    def detect_fires(self, image):
        """Detect bright spots in thermal image that could be fires"""
        
        # Convert 16-bit image to 8-bit for OpenCV compatibility
        if image.dtype == np.uint16:
            # Scale 16-bit (0-65535) to 8-bit (0-255)
            image_8bit = (image / 256).astype(np.uint8)
            # Adjust threshold for 8-bit scale
            threshold = max(int(self.brightness_threshold / 256), 200)
        else:
            image_8bit = image
            threshold = self.brightness_threshold
        
        # Apply threshold to find bright regions
        _, thresh = cv2.threshold(image_8bit, threshold, 255, cv2.THRESH_BINARY)
        
        # Find contours of bright regions
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        fire_locations = []
        
        for contour in contours:
            # Calculate area of the bright region
            area = cv2.contourArea(contour)
            
            # Filter by area to remove noise and very large regions
            if self.min_area < area < self.max_area:
                # Calculate center of the bright region
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    fire_locations.append((center_x, center_y, area))
        
        return fire_locations

    def publish_fire_location(self, location, header):
        """Publish detected fire location with pixel coordinates"""
        point_msg = PointStamped()
        point_msg.header = header
        point_msg.point.x = float(location[0])  # pixel x
        point_msg.point.y = float(location[1])  # pixel y  
        point_msg.point.z = float(location[2])  # area in pixels
        
        self.fire_location_pub.publish(point_msg)

    def create_debug_image(self, original_image, fire_locations):
        """Create debug image with fire locations marked"""
        # Convert to 8-bit for display if needed
        if original_image.dtype == np.uint16:
            display_image = (original_image / 256).astype(np.uint8)
        else:
            display_image = original_image.copy()
        
        # Convert to color for better visualization
        debug_image = cv2.cvtColor(display_image, cv2.COLOR_GRAY2BGR)
        
        # Draw detection overlay
        for location in fire_locations:
            center = (location[0], location[1])
            area = location[2]
            radius = max(int(np.sqrt(area / np.pi)), 10)
            
            # Draw red circle around fire
            cv2.circle(debug_image, center, radius, (0, 0, 255), 2)
            # Draw center crosshair
            cv2.drawMarker(debug_image, center, (0, 255, 0), cv2.MARKER_CROSS, 10, 2)
            # Add text with coordinates and area
            cv2.putText(debug_image, f'FIRE ({center[0]},{center[1]}) {area}px', 
                       (center[0] + 15, center[1] - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return debug_image

    def publish_debug_image(self, debug_image, header):
        """Publish debug image with fire detection overlay"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing debug image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    fire_detection_node = FireDetectionNode()
    
    try:
        rclpy.spin(fire_detection_node)
    except KeyboardInterrupt:
        fire_detection_node.get_logger().info('Fire detection node shutting down')
    
    fire_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()