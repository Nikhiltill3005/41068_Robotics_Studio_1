#include "autonomous_navigation_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <functional>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

AutonomousNavigationNode::AutonomousNavigationNode()
: Node("autonomous_navigation_node"),
  current_state_(NavigationState::IDLE),
  current_path_index_(0),
  robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0),
  goal_tolerance_(0.1),
  max_linear_velocity_(1.0),
  max_angular_velocity_(1.0),
  path_following_gain_(1.0)
{
  // Initialize publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10);
  
  // Initialize subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&AutonomousNavigationNode::map_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&AutonomousNavigationNode::odom_callback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&AutonomousNavigationNode::goal_callback, this, std::placeholders::_1));
  
  // Initialize TF components
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize timer for periodic updates
  timer_ = this->create_wall_timer(
    100ms, std::bind(&AutonomousNavigationNode::timer_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "Autonomous Navigation Node initialized");
}

void AutonomousNavigationNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Store map information for path planning
  // In a real implementation, you'd use this for obstacle avoidance
  RCLCPP_DEBUG(this->get_logger(), "Received map update");
}

void AutonomousNavigationNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  update_robot_pose();
}

void AutonomousNavigationNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_goal_ = *msg;
  current_state_ = NavigationState::PLANNING;
  RCLCPP_INFO(this->get_logger(), "New goal received at (%.2f, %.2f)", 
               current_goal_.pose.position.x, current_goal_.pose.position.y);
}

void AutonomousNavigationNode::timer_callback()
{
  switch (current_state_) {
    case NavigationState::IDLE:
      // Wait for goal
      break;
      
    case NavigationState::PLANNING:
      plan_path();
      break;
      
    case NavigationState::EXECUTING:
      execute_path();
      break;
      
    case NavigationState::REACHED_GOAL:
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
      current_state_ = NavigationState::IDLE;
      break;
      
    case NavigationState::FAILED:
      RCLCPP_WARN(this->get_logger(), "Navigation failed, returning to IDLE");
      current_state_ = NavigationState::IDLE;
      break;
  }
}

void AutonomousNavigationNode::update_robot_pose()
{
  // This would typically get the robot pose from TF or odometry
  // For now, we'll use the values from the odometry callback
  // In a real implementation, you'd query the TF tree
}

void AutonomousNavigationNode::plan_path()
{
  // Simple straight-line path planning
  // In a real implementation, you'd use A* or RRT for path planning
  
  planned_path_.clear();
  current_path_index_ = 0;
  
  // Create a simple path from current position to goal
  auto path_point = geometry_msgs::msg::PoseStamped();
  path_point.header.frame_id = "map";
  path_point.header.stamp = this->now();
  
  // Add current position
  path_point.pose.position.x = robot_x_;
  path_point.pose.position.y = robot_y_;
  path_point.pose.position.z = 0.0;
  planned_path_.push_back(path_point);
  
  // Add goal position
  path_point.pose.position.x = current_goal_.pose.position.x;
  path_point.pose.position.y = current_goal_.pose.position.y;
  planned_path_.push_back(path_point);
  
  RCLCPP_INFO(this->get_logger(), "Path planned with %zu waypoints", planned_path_.size());
  current_state_ = NavigationState::EXECUTING;
  
  publish_path_visualization();
}

void AutonomousNavigationNode::execute_path()
{
  if (planned_path_.empty() || current_path_index_ >= planned_path_.size()) {
    current_state_ = NavigationState::FAILED;
    return;
  }
  
  // Check if we've reached the goal
  if (is_goal_reached()) {
    current_state_ = NavigationState::REACHED_GOAL;
    stop_robot();
    return;
  }
  
  // Follow the planned path
  follow_path();
}

void AutonomousNavigationNode::follow_path()
{
  if (current_path_index_ >= planned_path_.size()) {
    return;
  }
  
  // Get current waypoint
  const auto& waypoint = planned_path_[current_path_index_];
  
  // Calculate distance and angle to waypoint
  double distance = calculate_distance_to_goal();
  double angle = calculate_angle_to_goal();
  
  // Simple proportional control for path following
  double linear_vel = std::min(distance * path_following_gain_, max_linear_velocity_);
  double angular_vel = std::clamp(angle * path_following_gain_, -max_angular_velocity_, max_angular_velocity_);
  
  // Publish velocity command
  auto cmd_vel = geometry_msgs::msg::Twist();
  cmd_vel.linear.x = linear_vel;
  cmd_vel.angular.z = angular_vel;
  cmd_vel_pub_->publish(cmd_vel);
  
  // Check if we're close enough to move to next waypoint
  if (distance < goal_tolerance_) {
    current_path_index_++;
    RCLCPP_DEBUG(this->get_logger(), "Moving to waypoint %zu", current_path_index_);
  }
}

bool AutonomousNavigationNode::is_goal_reached()
{
  double distance = calculate_distance_to_goal();
  return distance < goal_tolerance_;
}

void AutonomousNavigationNode::stop_robot()
{
  auto cmd_vel = geometry_msgs::msg::Twist();
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_vel);
}

void AutonomousNavigationNode::publish_path_visualization()
{
  // For now, just log path information
  if (current_path_index_ < planned_path_.size()) {
    RCLCPP_DEBUG(this->get_logger(), "Current waypoint: (%.2f, %.2f)", 
                  planned_path_[current_path_index_].pose.position.x,
                  planned_path_[current_path_index_].pose.position.y);
  }
}

double AutonomousNavigationNode::calculate_distance_to_goal()
{
  double dx = current_goal_.pose.position.x - robot_x_;
  double dy = current_goal_.pose.position.y - robot_y_;
  return sqrt(dx * dx + dy * dy);
}

double AutonomousNavigationNode::calculate_angle_to_goal()
{
  double dx = current_goal_.pose.position.x - robot_x_;
  double dy = current_goal_.pose.position.y - robot_y_;
  double target_angle = atan2(dy, dx);
  
  // Calculate angle difference
  double angle_diff = target_angle - robot_theta_;
  
  // Normalize angle to [-π, π]
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
  
  return angle_diff;
}

bool AutonomousNavigationNode::is_path_valid()
{
  // Simple path validation - check if path points are reasonable
  for (const auto& point : planned_path_) {
    if (std::isnan(point.pose.position.x) || std::isnan(point.pose.position.y)) {
      return false;
    }
  }
  return true;
}

void AutonomousNavigationNode::replan_if_needed()
{
  // Check if current path is still valid
  if (!is_path_valid()) {
    RCLCPP_WARN(this->get_logger(), "Path invalid, replanning...");
    current_state_ = NavigationState::PLANNING;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomousNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
