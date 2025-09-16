#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <vector>
#include <string>

class DroneSearchPattern : public rclcpp::Node {
public:
    DroneSearchPattern() : rclcpp::Node("drone_search_pattern") {
        // Declare parameters for search pattern
        this->declare_parameter<double>("search_altitude", 3.0);           // Height to maintain during search
        this->declare_parameter<double>("search_speed", 1.0);              // Linear speed during search (m/s)
        this->declare_parameter<double>("pattern_width", 10.0);            // Width of search area (m)
        this->declare_parameter<double>("pattern_length", 15.0);           // Length of search area (m)
        this->declare_parameter<double>("lane_spacing", 2.0);              // Distance between search lanes (m)
        this->declare_parameter<double>("waypoint_tolerance", 0.3);        // How close to get to waypoints (m)
        this->declare_parameter<double>("fire_investigation_time", 10.0);   // How long to hover over detected fire (s)
        this->declare_parameter<double>("fire_hover_altitude", 2.0);       // Altitude when investigating fire
        this->declare_parameter<bool>("auto_start", true);                 // Start search pattern automatically
        
        // Read parameters
        search_altitude_ = this->get_parameter("search_altitude").as_double();
        search_speed_ = this->get_parameter("search_speed").as_double();
        pattern_width_ = this->get_parameter("pattern_width").as_double();
        pattern_length_ = this->get_parameter("pattern_length").as_double();
        lane_spacing_ = this->get_parameter("lane_spacing").as_double();
        waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
        fire_investigation_time_ = this->get_parameter("fire_investigation_time").as_double();
        fire_hover_altitude_ = this->get_parameter("fire_hover_altitude").as_double();
        auto_start_ = this->get_parameter("auto_start").as_bool();

        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("search_status", 10);

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 10,  // Use relative topic name since node is in drone namespace
            std::bind(&DroneSearchPattern::odometry_callback, this, std::placeholders::_1));
        
        fire_detection_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "fire_detection/location", 10,  // Use relative topic name since node is in drone namespace
            std::bind(&DroneSearchPattern::fire_detection_callback, this, std::placeholders::_1));

        // Initialize state
        current_state_ = SearchState::STARTING;
        current_waypoint_index_ = 0;
        fire_detected_ = false;
        
        // Generate search pattern waypoints
        generate_search_pattern();
        
        // Main control timer (20 Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DroneSearchPattern::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Drone Search Pattern Node initialized");
        RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints for search pattern", waypoints_.size());
        
        if (auto_start_) {
            RCLCPP_INFO(this->get_logger(), "Auto-starting search pattern");
            current_state_ = SearchState::ASCENDING;
        }
    }

private:
    enum class SearchState {
        STARTING,           // Initial state - waiting for command
        ASCENDING,          // Rising to search altitude
        SEARCHING,          // Following search pattern
        FIRE_INVESTIGATING, // Hovering over detected fire
        RETURNING,          // Returning to start position
        COMPLETED           // Search pattern completed
    };

    struct Waypoint {
        double x, y, z;
        Waypoint(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    };

    // ROS2 publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr fire_detection_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Parameters
    double search_altitude_;
    double search_speed_;
    double pattern_width_;
    double pattern_length_;
    double lane_spacing_;
    double waypoint_tolerance_;
    double fire_investigation_time_;
    double fire_hover_altitude_;
    bool auto_start_;

    // State variables
    SearchState current_state_;
    nav_msgs::msg::Odometry current_odom_;
    bool odom_received_ = false;
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    bool fire_detected_;
    rclcpp::Time fire_detection_start_;
    Waypoint start_position_{0.0, 0.0, 0.0};

    void generate_search_pattern() {
        waypoints_.clear();
        
        // Start from current position (will be updated when odometry is received)
        double start_x = 0.0;  // Will be updated from odometry
        double start_y = 0.0;
        
        // Generate zigzag pattern
        double current_x = start_x - pattern_length_ / 2.0;
        double current_y = start_y - pattern_width_ / 2.0;
        
        int num_lanes = static_cast<int>(pattern_width_ / lane_spacing_) + 1;
        bool going_forward = true;
        
        for (int lane = 0; lane < num_lanes; ++lane) {
            if (going_forward) {
                // Go from left to right
                waypoints_.emplace_back(current_x + pattern_length_, current_y, search_altitude_);
            } else {
                // Go from right to left  
                waypoints_.emplace_back(current_x, current_y, search_altitude_);
            }
            
            // Move to next lane
            current_y += lane_spacing_;
            going_forward = !going_forward;
        }
        
        RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints", waypoints_.size());
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
        odom_received_ = true;
        
        // Debug: Log altitude every few messages
        static int odom_count = 0;
        odom_count++;
        if (odom_count % 20 == 0) {  // Every 20 messages (every second at 20Hz)
            RCLCPP_INFO(this->get_logger(), "Odometry: altitude = %.2fm", msg->pose.pose.position.z);
        }
        
        // Update start position on first odometry message
        if (current_state_ == SearchState::STARTING) {
            start_position_.x = msg->pose.pose.position.x;
            start_position_.y = msg->pose.pose.position.y;
            start_position_.z = msg->pose.pose.position.z;
            
            RCLCPP_INFO(this->get_logger(), "Set start position: (%.2f, %.2f, %.2f)", 
                       start_position_.x, start_position_.y, start_position_.z);
            
            // Regenerate pattern based on actual starting position
            generate_search_pattern();
        }
    }

    void fire_detection_callback(const geometry_msgs::msg::PointStamped::SharedPtr /*msg*/) {
        if (current_state_ == SearchState::SEARCHING && !fire_detected_) {
            RCLCPP_INFO(this->get_logger(), "FIRE DETECTED! Switching to investigation mode");
            fire_detected_ = true;
            fire_detection_start_ = this->now();
            current_state_ = SearchState::FIRE_INVESTIGATING;
            
            // Publish status update
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "FIRE_DETECTED - Investigating";
            status_pub_->publish(status_msg);
        }
    }

    void control_loop() {
        if (!odom_received_) {
            return;
        }

        auto cmd_vel = geometry_msgs::msg::Twist();
        auto status_msg = std_msgs::msg::String();

        switch (current_state_) {
            case SearchState::STARTING:
                // Wait for start command or auto-start
                cmd_vel = geometry_msgs::msg::Twist(); // Zero velocity
                status_msg.data = "WAITING_TO_START";
                break;

            case SearchState::ASCENDING:
                {
                    // Rise to search altitude
                    cmd_vel = ascend_to_altitude();
                    
                    double current_alt = current_odom_.pose.pose.position.z;
                    double alt_error = search_altitude_ - current_alt;
                    status_msg.data = "ASCENDING - Current: " + std::to_string(current_alt) + 
                                     "m, Target: " + std::to_string(search_altitude_) + "m";
                    
                    if (std::abs(alt_error) < waypoint_tolerance_) {
                        current_state_ = SearchState::SEARCHING;
                        RCLCPP_INFO(this->get_logger(), "Reached search altitude %.2fm, starting pattern", current_alt);
                    }
                    break;
                }

            case SearchState::SEARCHING:
                // Follow search pattern
                cmd_vel = navigate_to_waypoint();
                status_msg.data = "SEARCHING - Waypoint " + std::to_string(current_waypoint_index_ + 1) + 
                                 "/" + std::to_string(waypoints_.size());
                break;

            case SearchState::FIRE_INVESTIGATING:
                {
                    // Hover over fire for investigation
                    cmd_vel = hover_over_fire();
                    
                    auto time_investigating = (this->now() - fire_detection_start_).seconds();
                    status_msg.data = "INVESTIGATING_FIRE - " + std::to_string(time_investigating) + "s";
                    
                    if (time_investigating >= fire_investigation_time_) {
                        RCLCPP_INFO(this->get_logger(), "Fire investigation complete, resuming search");
                        fire_detected_ = false;
                        current_state_ = SearchState::SEARCHING;
                    }
                    break;
                }

            case SearchState::RETURNING:
                // Return to start position
                cmd_vel = return_to_start();
                status_msg.data = "RETURNING_TO_START";
                break;

            case SearchState::COMPLETED:
                // Pattern completed
                cmd_vel = geometry_msgs::msg::Twist(); // Zero velocity
                status_msg.data = "SEARCH_PATTERN_COMPLETED";
                break;
        }

        cmd_vel_pub_->publish(cmd_vel);
        status_pub_->publish(status_msg);
    }

    geometry_msgs::msg::Twist ascend_to_altitude() {
        auto cmd = geometry_msgs::msg::Twist();
        
        double current_alt = current_odom_.pose.pose.position.z;
        double altitude_error = search_altitude_ - current_alt;
        
        // Safety check: Emergency stop if too high
        if (current_alt > 10.0) {
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY: Drone too high at %.2fm! Stopping ascent.", current_alt);
            cmd.linear.z = -2.0;  // Force descent
            return cmd;
        }
        
        // Proportional altitude control - can go up or down
        if (std::abs(altitude_error) > waypoint_tolerance_) {
            cmd.linear.z = std::max(-1.0, std::min(1.0, altitude_error * 0.8)); // Proportional control, max ±1 m/s
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Altitude control: current=%.2fm, target=%.2fm, cmd_z=%.2f", 
                                current_alt, search_altitude_, cmd.linear.z);
        } else {
            cmd.linear.z = 0.0;
        }
        
        return cmd;
    }

    geometry_msgs::msg::Twist navigate_to_waypoint() {
        auto cmd = geometry_msgs::msg::Twist();
        
        if (current_waypoint_index_ >= waypoints_.size()) {
            // All waypoints completed
            current_state_ = SearchState::RETURNING;
            return cmd;
        }
        
        const auto& target = waypoints_[current_waypoint_index_];
        
        double dx = target.x - current_odom_.pose.pose.position.x;
        double dy = target.y - current_odom_.pose.pose.position.y;
        double dz = target.z - current_odom_.pose.pose.position.z;
        
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < waypoint_tolerance_) {
            // Reached waypoint, move to next
            current_waypoint_index_++;
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_index_);
            return cmd;
        }
        
        // Proportional navigation
        double speed_factor = std::min(1.0, distance / 2.0); // Slow down when close
        cmd.linear.x = (dx / distance) * search_speed_ * speed_factor;
        cmd.linear.y = (dy / distance) * search_speed_ * speed_factor;
        cmd.linear.z = std::max(-0.5, std::min(0.5, dz * 0.5)); // Gentle altitude correction
        
        return cmd;
    }

    geometry_msgs::msg::Twist hover_over_fire() {
        auto cmd = geometry_msgs::msg::Twist();
        
        // Maintain position and descend to fire investigation altitude
        double altitude_error = fire_hover_altitude_ - current_odom_.pose.pose.position.z;
        cmd.linear.z = std::max(-0.5, std::min(0.5, altitude_error * 0.5));
        
        // Stay in place (could add position holding here if needed)
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
        
        return cmd;
    }

    geometry_msgs::msg::Twist return_to_start() {
        auto cmd = geometry_msgs::msg::Twist();
        
        double dx = start_position_.x - current_odom_.pose.pose.position.x;
        double dy = start_position_.y - current_odom_.pose.pose.position.y;
        double dz = start_position_.z - current_odom_.pose.pose.position.z;
        
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < waypoint_tolerance_) {
            current_state_ = SearchState::COMPLETED;
            RCLCPP_INFO(this->get_logger(), "Search pattern completed!");
            return cmd;
        }
        
        // Navigate back to start
        cmd.linear.x = (dx / distance) * search_speed_;
        cmd.linear.y = (dy / distance) * search_speed_;
        cmd.linear.z = std::max(-0.5, std::min(0.5, dz * 0.5));
        
        return cmd;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneSearchPattern>());
    rclcpp::shutdown();
    return 0;
}
