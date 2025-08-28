#ifndef AUTONOMOUS_NAVIGATION_NODE_HPP
#define AUTONOMOUS_NAVIGATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <vector>
#include <queue>

class AutonomousNavigationNode : public rclcpp::Node
{
public:
  AutonomousNavigationNode();

private:
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  
  // TF components
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Navigation state
  enum class NavigationState {
    IDLE,
    PLANNING,
    EXECUTING,
    REACHED_GOAL,
    FAILED
  };
  NavigationState current_state_;
  
  // Current goal and path
  geometry_msgs::msg::PoseStamped current_goal_;
  std::vector<geometry_msgs::msg::PoseStamped> planned_path_;
  size_t current_path_index_;
  
  // Robot pose
  double robot_x_, robot_y_, robot_theta_;
  
  // Navigation parameters
  double goal_tolerance_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double path_following_gain_;
  
  // Callback functions
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void timer_callback();
  
  // Helper functions
  void update_robot_pose();
  void plan_path();
  void execute_path();
  void follow_path();
  bool is_goal_reached();
  void stop_robot();
  void publish_path_visualization();
  double calculate_distance_to_goal();
  double calculate_angle_to_goal();
  bool is_path_valid();
  void replan_if_needed();
};

#endif // AUTONOMOUS_NAVIGATION_NODE_HPP
