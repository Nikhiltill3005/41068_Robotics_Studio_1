#include "environment_mapping_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <functional>
#include <cmath>

using namespace std::chrono_literals;

EnvironmentMappingNode::EnvironmentMappingNode()
: Node("environment_mapping_node"),
  map_resolution_(0.05),
  map_width_(1000), map_height_(1000),
  map_origin_x_(-25.0), map_origin_y_(-25.0),
  robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0)
{
  // Initialize publishers
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", 10);
  
  // Initialize subscribers
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&EnvironmentMappingNode::laser_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&EnvironmentMappingNode::odom_callback, this, std::placeholders::_1));
  
  // Initialize TF components
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize timer for periodic updates
  timer_ = this->create_wall_timer(
    100ms, std::bind(&EnvironmentMappingNode::timer_callback, this));
  
  // Initialize map
  initialize_map();
  
  RCLCPP_INFO(this->get_logger(), "Environment Mapping Node initialized");
}

void EnvironmentMappingNode::initialize_map()
{
  occupancy_grid_.resize(map_width_ * map_height_, -1); // Unknown cells
  
  // Set center area as free space initially
  int center_x = map_width_ / 2;
  int center_y = map_height_ / 2;
  int free_radius = 10; // 10 cells radius of free space around center
  
  for (int x = center_x - free_radius; x <= center_x + free_radius; ++x) {
    for (int y = center_y - free_radius; y <= center_y + free_radius; ++y) {
      if (x >= 0 && x < map_width_ && y >= 0 && y < map_height_) {
        int index = y * map_width_ + x;
        occupancy_grid_[index] = 0; // Free space
      }
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Map initialized with %dx%d cells", map_width_, map_height_);
}

void EnvironmentMappingNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  update_map_from_laser(*msg);
}

void EnvironmentMappingNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  update_robot_pose();
}

void EnvironmentMappingNode::timer_callback()
{
  publish_map();
}

void EnvironmentMappingNode::update_map_from_laser(const sensor_msgs::msg::LaserScan& scan)
{
  // Get robot pose in map frame
  double robot_map_x = robot_x_ - map_origin_x_;
  double robot_map_y = robot_y_ - map_origin_y_;
  
  // Convert to map coordinates
  int robot_map_cell_x = static_cast<int>(robot_map_x / map_resolution_);
  int robot_map_cell_y = static_cast<int>(robot_map_y / map_resolution_);
  
  // Process each laser reading
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    if (std::isnan(scan.ranges[i]) || std::isinf(scan.ranges[i])) {
      continue;
    }
    
    double range = scan.ranges[i];
    if (range < scan.range_min || range > scan.range_max) {
      continue;
    }
    
    // Calculate angle for this reading
    double angle = scan.angle_min + i * scan.angle_increment;
    double world_angle = robot_theta_ + angle;
    
    // Calculate endpoint of laser ray
    double end_x = robot_map_x + range * cos(world_angle);
    double end_y = robot_map_y + range * sin(world_angle);
    
    // Convert to map coordinates
    int end_cell_x = static_cast<int>(end_x / map_resolution_);
    int end_cell_y = static_cast<int>(end_y / map_resolution_);
    
    // Mark cells along the ray as free space
    mark_line_free(robot_map_cell_x, robot_map_cell_y, end_cell_x, end_cell_y);
    
    // Mark endpoint as occupied (if not too far)
    if (range < scan.range_max - 0.1) {
      if (end_cell_x >= 0 && end_cell_x < map_width_ && 
          end_cell_y >= 0 && end_cell_y < map_height_) {
        int index = end_cell_y * map_width_ + end_cell_x;
        occupancy_grid_[index] = 100; // Occupied
      }
    }
  }
}

void EnvironmentMappingNode::mark_line_free(int x0, int y0, int x1, int y1)
{
  // Bresenham's line algorithm to mark cells as free
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  
  int x = x0, y = y0;
  
  while (true) {
    // Mark current cell as free
    if (x >= 0 && x < map_width_ && y >= 0 && y < map_height_) {
      int index = y * map_width_ + x;
      if (occupancy_grid_[index] == -1) { // Only mark unknown cells as free
        occupancy_grid_[index] = 0;
      }
    }
    
    if (x == x1 && y == y1) break;
    
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}


void EnvironmentMappingNode::publish_map()
{
  auto map_msg = nav_msgs::msg::OccupancyGrid();
  map_msg.header.stamp = this->now();
  map_msg.header.frame_id = "map";
  
  map_msg.info.resolution = map_resolution_;
  map_msg.info.width = map_width_;
  map_msg.info.height = map_height_;
  map_msg.info.origin.position.x = map_origin_x_;
  map_msg.info.origin.position.y = map_origin_y_;
  map_msg.info.origin.position.z = 0.0;
  
  // Set orientation (identity quaternion)
  map_msg.info.origin.orientation.x = 0.0;
  map_msg.info.origin.orientation.y = 0.0;
  map_msg.info.origin.orientation.z = 0.0;
  map_msg.info.origin.orientation.w = 1.0;
  
  map_msg.data = occupancy_grid_;
  
  map_pub_->publish(map_msg);
}


void EnvironmentMappingNode::update_robot_pose()
{
  // This would typically get the robot pose from TF or odometry
  // For now, we'll use the values from the odometry callback
  // In a real implementation, you'd query the TF tree
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EnvironmentMappingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
