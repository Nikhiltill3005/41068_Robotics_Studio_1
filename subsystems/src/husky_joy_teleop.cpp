#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <string>
#include <algorithm>

class HuskyJoyTeleop : public rclcpp::Node {
public:
    HuskyJoyTeleop() : rclcpp::Node("husky_joy_teleop") {
        // Parameters for controller mapping and scaling
        this->declare_parameter<int>("axis_linear_x", 1);       // Left stick Y (forward/backward)
        this->declare_parameter<int>("axis_angular_z", 3);      // Right stick X (turn left/right)
        this->declare_parameter<int>("enable_button", 4);       // LB button
        this->declare_parameter<int>("turbo_button", 5);        // RB button
        this->declare_parameter<double>("scale_linear", 2.0);   // m/s normal
        this->declare_parameter<double>("scale_angular", 4.0);  // rad/s normal
        this->declare_parameter<double>("scale_linear_turbo", 4.0);  // m/s turbo
        this->declare_parameter<double>("scale_angular_turbo", 8.0); // rad/s turbo

        // Read parameters
        axis_linear_x_ = this->get_parameter("axis_linear_x").as_int();
        axis_angular_z_ = this->get_parameter("axis_angular_z").as_int();
        enable_button_ = this->get_parameter("enable_button").as_int();
        turbo_button_ = this->get_parameter("turbo_button").as_int();
        scale_linear_ = this->get_parameter("scale_linear").as_double();
        scale_angular_ = this->get_parameter("scale_angular").as_double();
        scale_linear_turbo_ = this->get_parameter("scale_linear_turbo").as_double();
        scale_angular_turbo_ = this->get_parameter("scale_angular_turbo").as_double();

        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("teleop_status", 10);

        // Subscriber for joystick
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&HuskyJoyTeleop::joy_callback, this, std::placeholders::_1));

        // Timer to publish at a constant rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&HuskyJoyTeleop::publish_command, this));

        // Init twist
        twist_msg_ = geometry_msgs::msg::Twist();

        RCLCPP_INFO(this->get_logger(), "Husky Xbox Teleop Started");
        RCLCPP_INFO(this->get_logger(), "Hold LB to enable movement. RB = turbo");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {
        // Default: stop
        twist_msg_ = geometry_msgs::msg::Twist();

        if (static_cast<size_t>(enable_button_) >= joy->buttons.size()) {
            publish_status("Enable button index out of range");
            return;
        }

        const bool enabled = joy->buttons[enable_button_] != 0;
        const bool turbo = (static_cast<size_t>(turbo_button_) < joy->buttons.size()) && (joy->buttons[turbo_button_] != 0);

        if (!enabled) {
            // When not enabled, publish zero (no motion)
            twist_msg_ = geometry_msgs::msg::Twist();
            publish_status("Hold LB to enable movement");
            return;
        }

        // Read axes safely
        auto read_axis = [&](int idx) -> double {
            if (idx >= 0 && static_cast<size_t>(idx) < joy->axes.size()) {
                return joy->axes[idx];
            }
            return 0.0;
        };

        const double linear = read_axis(axis_linear_x_);  // forward/back
        const double angular = read_axis(axis_angular_z_); // turn left/right

        if (turbo) {
            twist_msg_.linear.x = linear * scale_linear_turbo_;
            twist_msg_.angular.z = angular * scale_angular_turbo_;
            publish_status("TURBO MODE - Moving");
        } else {
            twist_msg_.linear.x = linear * scale_linear_;
            twist_msg_.angular.z = angular * scale_angular_;
            publish_status("Normal Mode - Moving");
        }

        // Throttled log
        if (std::fabs(twist_msg_.linear.x) > 0.05 || std::fabs(twist_msg_.angular.z) > 0.05) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "linear: %.2f angular: %.2f %s",
                                 twist_msg_.linear.x, twist_msg_.angular.z, turbo ? "(TURBO)" : "");
        }
    }

    void publish_command() {
        cmd_vel_pub_->publish(twist_msg_);
    }

    void publish_status(const std::string &message) {
        auto msg = std_msgs::msg::String();
        msg.data = message;
        status_pub_->publish(msg);
    }

    // Params
    int axis_linear_x_;
    int axis_angular_z_;
    int enable_button_;
    int turbo_button_;

    double scale_linear_;
    double scale_angular_;
    double scale_linear_turbo_;
    double scale_angular_turbo_;

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist twist_msg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HuskyJoyTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
