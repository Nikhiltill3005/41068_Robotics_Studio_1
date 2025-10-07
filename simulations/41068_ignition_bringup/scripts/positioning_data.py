#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

class RobotFireVisualizer(Node):
    def __init__(self):
        super().__init__('robot_fire_visualizer')
        
        # Declare and get parameters for topic names
        self.declare_parameter('robot1_topic', '/husky/odometry')
        self.declare_parameter('robot2_topic', '/drone/odometry')
        self.declare_parameter('fire_topic', '/drone/fire_scan/fire_positions')
        self.declare_parameter('world_size', 50.0)
        
        robot1_topic = self.get_parameter('robot1_topic').get_parameter_value().string_value
        robot2_topic = self.get_parameter('robot2_topic').get_parameter_value().string_value
        fire_topic = self.get_parameter('fire_topic').get_parameter_value().string_value
        self.world_size = self.get_parameter('world_size').get_parameter_value().double_value
        
        # Store latest positions
        self.robot1_position = None
        self.robot2_position = None
        self.fire_positions = []
        
        # Create subscriptions
        self.subscription_robot1 = self.create_subscription(
            Odometry,
            robot1_topic,
            self.robot1_callback,
            10)
        
        self.subscription_robot2 = self.create_subscription(
            Odometry,
            robot2_topic,
            self.robot2_callback,
            10)
        
        self.subscription_fire = self.create_subscription(
            PoseArray,
            fire_topic,
            self.fire_callback,
            10)
        
        self.get_logger().info(f'Subscribed to Husky: {robot1_topic}')
        self.get_logger().info(f'Subscribed to Drone: {robot2_topic}')
        self.get_logger().info(f'Subscribed to Fire Detection: {fire_topic}')
        self.get_logger().info(f'World size: {self.world_size}x{self.world_size}')
        
        # Setup matplotlib
        self.setup_plot()
        
    def setup_plot(self):
        """Initialize the matplotlib figure and axis"""
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots(figsize=(50, 50))
        self.ax.set_xlim(-self.world_size/2, self.world_size/2)
        self.ax.set_ylim(-self.world_size/2, self.world_size/2)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Robot and Fire Position Map')
        
        # Create initial plot elements
        self.husky_plot, = self.ax.plot([], [], 'bs', markersize=15, label='Husky')
        self.drone_plot, = self.ax.plot([], [], 'g^', markersize=15, label='Drone')
        self.fire_plot, = self.ax.plot([], [], 'ro', markersize=12, label='Fire')
        
        self.ax.legend(loc='upper right')
        
        # Add text annotations
        self.husky_text = self.ax.text(0, 0, '', fontsize=8, ha='center')
        self.drone_text = self.ax.text(0, 0, '', fontsize=8, ha='center')
        self.fire_texts = []
        
    def robot1_callback(self, msg):
        """Callback for Husky odometry"""
        self.robot1_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        self.update_plot()
    
    def robot2_callback(self, msg):
        """Callback for Drone odometry"""
        self.robot2_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        self.update_plot()
    
    def fire_callback(self, msg):
        """Callback for fire positions"""
        self.fire_positions = []
        for pose in msg.poses:
            self.fire_positions.append({
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            })
        self.update_plot()
    
    def update_plot(self):
        """Update the matplotlib plot with current positions"""
        # Update Husky position
        if self.robot1_position is not None:
            x, y = self.robot1_position['x'], self.robot1_position['y']
            self.husky_plot.set_data([x], [y])
            self.husky_text.set_position((x, y - 1.5))
            self.husky_text.set_text(f'H({x:.1f}, {y:.1f})')
        
        # Update Drone position
        if self.robot2_position is not None:
            x, y = self.robot2_position['x'], self.robot2_position['y']
            self.drone_plot.set_data([x], [y])
            self.drone_text.set_position((x, y + 1.5))
            self.drone_text.set_text(f'D({x:.1f}, {y:.1f})')
        
        # Clear old fire text annotations
        for text in self.fire_texts:
            text.remove()
        self.fire_texts.clear()
        
        # Update Fire positions
        if self.fire_positions:
            fire_x = [pos['x'] for pos in self.fire_positions]
            fire_y = [pos['y'] for pos in self.fire_positions]
            self.fire_plot.set_data(fire_x, fire_y)
            
            # Add fire labels
            for i, pos in enumerate(self.fire_positions):
                text = self.ax.text(
                    pos['x'] + 0.8, pos['y'] + 0.8, 
                    f'F{i+1}', 
                    fontsize=8, 
                    color='red',
                    weight='bold'
                )
                self.fire_texts.append(text)
        else:
            self.fire_plot.set_data([], [])
        
        # Redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = RobotFireVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    
    visualizer.destroy_node()
    rclpy.shutdown()
    plt.close('all')

if __name__ == '__main__':
    main()