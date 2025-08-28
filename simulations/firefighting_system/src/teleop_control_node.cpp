#include "teleop_control_node.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

TeleopControlNode::TeleopControlNode()
: Node("teleop_control_node"),
  max_steering_angle_(0.5),
  max_speed_(2.0),
  steering_sensitivity_(0.1),
  speed_sensitivity_(0.5),
  current_steering_angle_(0.0),
  current_speed_(0.0)
{
  // Initialize publishers
  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
    "/cmd_ackermann", 10);
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);
  
  // Initialize subscribers
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/teleop_commands", 10,
    std::bind(&TeleopControlNode::command_callback, this, std::placeholders::_1));
  
  // Initialize timer for periodic updates
  timer_ = this->create_wall_timer(
    50ms, std::bind(&TeleopControlNode::timer_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "Teleop Control Node initialized");
  RCLCPP_INFO(this->get_logger(), "Use WASD keys for movement, Q/E for steering, Space to stop");
}

void TeleopControlNode::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  process_keyboard_input(msg->data);
}

void TeleopControlNode::timer_callback()
{
  publish_drive_command();
}

void TeleopControlNode::publish_drive_command()
{
  // Publish Ackermann drive message
  auto drive_msg = ackermann_msgs::msg::AckermannDrive();
  drive_msg.steering_angle = current_steering_angle_;
  drive_msg.speed = current_speed_;
  drive_msg.steering_angle_velocity = 0.0;
  drive_msg.acceleration = 0.0;
  drive_msg.jerk = 0.0;
  
  drive_pub_->publish(drive_msg);
  
  // Also publish Twist message for compatibility
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = current_speed_;
  twist_msg.angular.z = current_speed_ * tan(current_steering_angle_) / 0.5; // Approximate
  
  twist_pub_->publish(twist_msg);
}

void TeleopControlNode::process_keyboard_input(const std::string& input)
{
  if (input.empty()) return;
  
  char key = input[0];
  
  switch (key) {
    case 'w':
    case 'W':
      set_speed(current_speed_ + speed_sensitivity_);
      break;
    case 's':
    case 'S':
      set_speed(current_speed_ - speed_sensitivity_);
      break;
    case 'a':
    case 'A':
      set_steering_angle(current_steering_angle_ - steering_sensitivity_);
      break;
    case 'd':
    case 'D':
      set_steering_angle(current_steering_angle_ + steering_sensitivity_);
      break;
    case 'q':
    case 'Q':
      set_steering_angle(current_steering_angle_ - steering_sensitivity_);
      break;
    case 'e':
    case 'E':
      set_steering_angle(current_steering_angle_ + steering_sensitivity_);
      break;
    case ' ':
      stop_robot();
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown command: %c", key);
      break;
  }
}

void TeleopControlNode::stop_robot()
{
  current_speed_ = 0.0;
  current_steering_angle_ = 0.0;
  RCLCPP_INFO(this->get_logger(), "Robot stopped");
}

void TeleopControlNode::set_speed(double speed)
{
  current_speed_ = std::clamp(speed, -max_speed_, max_speed_);
  RCLCPP_INFO(this->get_logger(), "Speed set to: %.2f", current_speed_);
}

void TeleopControlNode::set_steering_angle(double angle)
{
  current_steering_angle_ = std::clamp(angle, -max_steering_angle_, max_steering_angle_);
  RCLCPP_INFO(this->get_logger(), "Steering angle set to: %.2f", current_steering_angle_);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
