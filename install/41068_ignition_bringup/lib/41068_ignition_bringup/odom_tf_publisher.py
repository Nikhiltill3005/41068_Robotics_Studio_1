#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create publisher for /odom topic
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscribe to /husky/odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/husky/odometry',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odom TF Publisher started')
    
    def odom_callback(self, msg):
        # Republish the odometry message to /odom
        self.odom_publisher.publish(msg)
        
        # Create and publish the transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Copy pose information
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

