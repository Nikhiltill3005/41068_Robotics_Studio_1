#ifndef UGV_MOVEMENT_NODE_HPP
#define UGV_MOVEMENT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>

class UGVMovementNode : public rclcpp::Node
{
public:
  UGVMovementNode();

private:
  // Publishers
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  // Subscribers
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  
  // TF components
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Robot state
  double x_, y_, theta_;
  double linear_velocity_, angular_velocity_;
  double steering_angle_;
  
  // Robot parameters
  double wheelbase_;
  double max_steering_angle_;
  double max_speed_;
  
  // Callback functions
  void drive_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timer_callback();
  
  // Helper functions
  void update_robot_state();
  void publish_odometry();
  void publish_tf();
  void publish_drive_command();
  void apply_movement_limits();
  double normalize_angle(double angle);
};

#endif // UGV_MOVEMENT_NODE_HPP
