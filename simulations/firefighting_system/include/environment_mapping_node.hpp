#ifndef ENVIRONMENT_MAPPING_NODE_HPP
#define ENVIRONMENT_MAPPING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <vector>

class EnvironmentMappingNode : public rclcpp::Node
{
public:
  EnvironmentMappingNode();

private:
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // TF components
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Map parameters
  double map_resolution_;
  int map_width_, map_height_;
  double map_origin_x_, map_origin_y_;
  
  // Current map data
  std::vector<int8_t> occupancy_grid_;
  
  // Robot pose
  double robot_x_, robot_y_, robot_theta_;
  
  // Callback functions
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timer_callback();
  
  // Helper functions
  void initialize_map();
  void update_map_from_laser(const sensor_msgs::msg::LaserScan& scan);
  void mark_line_free(int x0, int y0, int x1, int y1);
  void publish_map();
  void update_robot_pose();
};

#endif // ENVIRONMENT_MAPPING_NODE_HPP
