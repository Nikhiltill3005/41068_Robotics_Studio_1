#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <string>
#include <algorithm>

class DroneJoyTeleop : public rclcpp::Node {
public:
    DroneJoyTeleop() : rclcpp::Node("drone_joy_teleop") {
        // Parameters for controller mapping and scaling
        this->declare_parameter<int>("axis_linear_x", 1);       // Left stick Y (forward/backward)
        this->declare_parameter<int>("axis_linear_y", 0);       // Left stick X (strafe left/right)
        this->declare_parameter<int>("axis_linear_z", -1);      // Disabled when using buttons for altitude
        this->declare_parameter<int>("axis_angular_z", 3);      // Right stick X (yaw left/right)
        // Button-based altitude control (A up, X down by default)
        this->declare_parameter<int>("button_altitude_up", 0);   // A button index
        this->declare_parameter<int>("button_altitude_down", 2); // X button index
        // Triggers for altitude control (kept for compatibility; not used if buttons are configured)
        this->declare_parameter<int>("axis_trigger_left", 2);   // LT axis index
        this->declare_parameter<int>("axis_trigger_right", 5);  // RT axis index
        this->declare_parameter<std::string>("trigger_input_type", std::string("auto")); // auto | pm1 | 0to1
        this->declare_parameter<double>("trigger_deadzone", 0.05); // Ignore small trigger noise
        this->declare_parameter<int>("enable_button", 4);       // LB button
        this->declare_parameter<int>("turbo_button", 5);        // RB button
        this->declare_parameter<double>("scale_linear_xy", 1.5);// m/s normal
        this->declare_parameter<double>("scale_linear_z", 1.0); // m/s normal
        this->declare_parameter<double>("scale_angular_z", 2.0);// rad/s normal
        this->declare_parameter<double>("scale_linear_xy_turbo", 3.0);
        this->declare_parameter<double>("scale_linear_z_turbo", 2.0);
        this->declare_parameter<double>("scale_angular_z_turbo", 4.0);
        // Hover settings (disabled by default to avoid drift when not enabled)
        this->declare_parameter<bool>("hover_when_disabled", false);
        this->declare_parameter<double>("hover_linear_z", 0.3); // tune as needed if enabling hover

        // Read parameters
        axis_linear_x_ = this->get_parameter("axis_linear_x").as_int();
        axis_linear_y_ = this->get_parameter("axis_linear_y").as_int();
        axis_linear_z_ = this->get_parameter("axis_linear_z").as_int();
        axis_angular_z_ = this->get_parameter("axis_angular_z").as_int();
        button_altitude_up_ = this->get_parameter("button_altitude_up").as_int();
        button_altitude_down_ = this->get_parameter("button_altitude_down").as_int();
        axis_trigger_left_ = this->get_parameter("axis_trigger_left").as_int();
        axis_trigger_right_ = this->get_parameter("axis_trigger_right").as_int();
        trigger_input_type_ = this->get_parameter("trigger_input_type").as_string();
        trigger_deadzone_ = this->get_parameter("trigger_deadzone").as_double();
        enable_button_ = this->get_parameter("enable_button").as_int();
        turbo_button_ = this->get_parameter("turbo_button").as_int();
        scale_linear_xy_ = this->get_parameter("scale_linear_xy").as_double();
        scale_linear_z_ = this->get_parameter("scale_linear_z").as_double();
        scale_angular_z_ = this->get_parameter("scale_angular_z").as_double();
        scale_linear_xy_turbo_ = this->get_parameter("scale_linear_xy_turbo").as_double();
        scale_linear_z_turbo_ = this->get_parameter("scale_linear_z_turbo").as_double();
        scale_angular_z_turbo_ = this->get_parameter("scale_angular_z_turbo").as_double();
        hover_when_disabled_ = this->get_parameter("hover_when_disabled").as_bool();
        hover_linear_z_ = this->get_parameter("hover_linear_z").as_double();

        // Publisher for velocity commands (publish to namespaced cmd_vel)
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("teleop_status", 10);

        // Subscriber for joystick
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&DroneJoyTeleop::joy_callback, this, std::placeholders::_1));

        // Timer to publish at a constant rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&DroneJoyTeleop::publish_command, this));

        // Init twist
        twist_msg_ = geometry_msgs::msg::Twist();

        RCLCPP_INFO(this->get_logger(), "Drone Xbox Teleop Started");
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
            // When not enabled, publish zero by default (no motion)
            twist_msg_ = geometry_msgs::msg::Twist();
            if (hover_when_disabled_) {
                twist_msg_.linear.z = hover_linear_z_;
                publish_status("Hovering (LB to control, RB turbo)");
            } else {
                publish_status("Hold LB to enable movement");
            }
            return;
        }

        // Read axes safely
        auto read_axis = [&](int idx) -> double {
            if (idx >= 0 && static_cast<size_t>(idx) < joy->axes.size()) {
                return joy->axes[idx];
            }
            return 0.0;
        };

        const double ax_x = read_axis(axis_linear_x_);  // forward/back (Y stick)
        const double ax_y = read_axis(axis_linear_y_);  // left/right (X stick)
        // Altitude via buttons: A up, X down (overrides triggers). If both pressed, cancel.
        auto read_button = [&](int idx) -> int {
            if (idx >= 0 && static_cast<size_t>(idx) < joy->buttons.size()) {
                return joy->buttons[idx] ? 1 : 0;
            }
            return 0;
        };
        const bool btn_up = read_button(button_altitude_up_);
        const bool btn_down = read_button(button_altitude_down_);
        double ax_z = 0.0;
        if (btn_up && !btn_down) {
            ax_z = 1.0;
        } else if (btn_down && !btn_up) {
            ax_z = -1.0;
        } else if (axis_linear_z_ >= 0) {
            // Optional fallback to stick if explicitly enabled by parameter
            ax_z = read_axis(axis_linear_z_);
        } else {
            ax_z = 0.0;
        }
        const double az_z = read_axis(axis_angular_z_); // yaw (X stick)

        if (turbo) {
            twist_msg_.linear.x = ax_x * scale_linear_xy_turbo_;
            twist_msg_.linear.y = ax_y * scale_linear_xy_turbo_;
            twist_msg_.linear.z = ax_z * scale_linear_z_turbo_;
            twist_msg_.angular.z = az_z * scale_angular_z_turbo_;
            publish_status("TURBO MODE - Moving");
        } else {
            twist_msg_.linear.x = ax_x * scale_linear_xy_;
            twist_msg_.linear.y = ax_y * scale_linear_xy_;
            twist_msg_.linear.z = ax_z * scale_linear_z_;
            twist_msg_.angular.z = az_z * scale_angular_z_;
            publish_status("Normal Mode - Moving");
        }

        // Throttled log
        if (std::fabs(twist_msg_.linear.x) > 0.05 || std::fabs(twist_msg_.linear.y) > 0.05 ||
            std::fabs(twist_msg_.linear.z) > 0.05 || std::fabs(twist_msg_.angular.z) > 0.05) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "vx: %.2f vy: %.2f vz: %.2f yaw: %.2f %s",
                                 twist_msg_.linear.x, twist_msg_.linear.y, twist_msg_.linear.z,
                                 twist_msg_.angular.z, turbo ? "(TURBO)" : "");
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
    int axis_linear_y_;
    int axis_linear_z_;
    int axis_angular_z_;
    int axis_trigger_left_;
    int axis_trigger_right_;
    int button_altitude_up_;
    int button_altitude_down_;
    int enable_button_;
    int turbo_button_;

    double scale_linear_xy_;
    double scale_linear_z_;
    double scale_angular_z_;
    double scale_linear_xy_turbo_;
    double scale_linear_z_turbo_;
    double scale_angular_z_turbo_;
    // Trigger handling
    std::string trigger_input_type_;
    double trigger_deadzone_;
    bool hover_when_disabled_;
    double hover_linear_z_;

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist twist_msg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneJoyTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
