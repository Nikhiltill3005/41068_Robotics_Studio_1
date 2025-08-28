#ifndef TELEOP_CONTROL_NODE_HPP
#define TELEOP_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class TeleopControlNode : public rclcpp::Node
{
public:
  TeleopControlNode();

private:
  // Publishers
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  
  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Node parameters
  double max_steering_angle_;
  double max_speed_;
  double steering_sensitivity_;
  double speed_sensitivity_;
  
  // Current state
  double current_steering_angle_;
  double current_speed_;
  
  // Callback functions
  void command_callback(const std_msgs::msg::String::SharedPtr msg);
  void timer_callback();
  void publish_drive_command();
  
  // Helper functions
  void process_keyboard_input(const std::string& input);
  void stop_robot();
  void set_speed(double speed);
  void set_steering_angle(double angle);
};

#endif // TELEOP_CONTROL_NODE_HPP
