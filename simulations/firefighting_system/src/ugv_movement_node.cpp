#include "ugv_movement_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

UGVMovementNode::UGVMovementNode()
: Node("ugv_movement_node"),
  x_(0.0), y_(0.0), theta_(0.0),
  linear_velocity_(0.0), angular_velocity_(0.0),
  steering_angle_(0.0),
  wheelbase_(0.5),
  max_steering_angle_(0.5),
  max_speed_(2.0)
{
  // Initialize publishers
  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
    "/husky_velocity_controller/cmd_ackermann", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", 10);
  
  // Initialize subscribers
  drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
    "/cmd_ackermann", 10,
    std::bind(&UGVMovementNode::drive_callback, this, std::placeholders::_1));
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&UGVMovementNode::twist_callback, this, std::placeholders::_1));
  
  // Initialize TF components
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize timer for periodic updates
  timer_ = this->create_wall_timer(
    50ms, std::bind(&UGVMovementNode::timer_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "UGV Movement Node initialized");
}

void UGVMovementNode::drive_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
{
  // Apply movement limits
  steering_angle_ = std::clamp(static_cast<double>(msg->steering_angle), -max_steering_angle_, max_steering_angle_);
  linear_velocity_ = std::clamp(static_cast<double>(msg->speed), -max_speed_, max_speed_);
  
  // Calculate angular velocity based on Ackermann steering geometry
  if (std::abs(steering_angle_) > 0.001) {
    angular_velocity_ = linear_velocity_ * tan(steering_angle_) / wheelbase_;
  } else {
    angular_velocity_ = 0.0;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Drive command: speed=%.2f, steering=%.2f", 
                linear_velocity_, steering_angle_);
}

void UGVMovementNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Convert Twist to Ackermann drive
  linear_velocity_ = std::clamp(msg->linear.x, -max_speed_, max_speed_);
  angular_velocity_ = std::clamp(msg->angular.z, -max_speed_/wheelbase_, max_speed_/wheelbase_);
  
  // Calculate steering angle from angular velocity
  if (std::abs(linear_velocity_) > 0.001) {
    steering_angle_ = std::clamp(atan(angular_velocity_ * wheelbase_ / linear_velocity_), 
                                -max_steering_angle_, max_steering_angle_);
  } else {
    steering_angle_ = 0.0;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Twist command: linear=%.2f, angular=%.2f", 
                linear_velocity_, angular_velocity_);
}

void UGVMovementNode::timer_callback()
{
  update_robot_state();
  publish_odometry();
  publish_tf();
  publish_drive_command();
}

void UGVMovementNode::update_robot_state()
{
  // Simple kinematic model update
  double dt = 0.05; // 50ms timer
  
  // Update position
  x_ += linear_velocity_ * cos(theta_) * dt;
  y_ += linear_velocity_ * sin(theta_) * dt;
  theta_ += angular_velocity_ * dt;
  
  // Normalize angle
  theta_ = normalize_angle(theta_);
}

void UGVMovementNode::publish_odometry()
{
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  
  // Set position
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;
  
  // Set orientation
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  
  // Set velocity
  odom_msg.twist.twist.linear.x = linear_velocity_;
  odom_msg.twist.twist.angular.z = angular_velocity_;
  
  odom_pub_->publish(odom_msg);
}

void UGVMovementNode::publish_tf()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  
  transform.transform.translation.x = x_;
  transform.transform.translation.y = y_;
  transform.transform.translation.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  
  tf_broadcaster_->sendTransform(transform);
}

void UGVMovementNode::publish_drive_command()
{
  // Publish the processed drive command to the robot
  auto drive_msg = ackermann_msgs::msg::AckermannDrive();
  drive_msg.steering_angle = steering_angle_;
  drive_msg.speed = linear_velocity_;
  drive_msg.steering_angle_velocity = 0.0;
  drive_msg.acceleration = 0.0;
  drive_msg.jerk = 0.0;
  
  drive_pub_->publish(drive_msg);
}

double UGVMovementNode::normalize_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UGVMovementNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
