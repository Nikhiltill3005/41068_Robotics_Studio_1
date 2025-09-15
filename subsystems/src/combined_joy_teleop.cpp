#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <string>

enum class ActiveVehicle {
    Husky,
    Drone
};

class CombinedJoyTeleop : public rclcpp::Node {
public:
    CombinedJoyTeleop() : rclcpp::Node("combined_joy_teleop"),
                          active_vehicle_(ActiveVehicle::Husky),
                          prev_toggle_pressed_(false) {
        // Common parameters
        this->declare_parameter<int>("enable_button", 4);       // LB
        this->declare_parameter<int>("turbo_button", 5);        // RB
        this->declare_parameter<int>("toggle_button", 3);       // Y button (Xbox)

        // Husky params
        this->declare_parameter<int>("husky_axis_linear_x", 1); // Left stick Y
        this->declare_parameter<int>("husky_axis_angular_z", 3);// Right stick X
        this->declare_parameter<double>("husky_scale_linear", 2.0);
        this->declare_parameter<double>("husky_scale_angular", 4.0);
        this->declare_parameter<double>("husky_scale_linear_turbo", 4.0);
        this->declare_parameter<double>("husky_scale_angular_turbo", 8.0);

        // Drone params
        this->declare_parameter<int>("drone_axis_linear_x", 1); // Left stick Y
        this->declare_parameter<int>("drone_axis_linear_y", 0); // Left stick X
        this->declare_parameter<int>("drone_axis_linear_z", -1);// disabled (use buttons)
        this->declare_parameter<int>("drone_axis_angular_z", 3);// Right stick X
        this->declare_parameter<int>("drone_button_altitude_up", 0);   // A
        this->declare_parameter<int>("drone_button_altitude_down", 2); // X
        this->declare_parameter<double>("drone_scale_linear_xy", 1.5);
        this->declare_parameter<double>("drone_scale_linear_z", 1.0);
        this->declare_parameter<double>("drone_scale_angular_z", 2.0);
        this->declare_parameter<double>("drone_scale_linear_xy_turbo", 3.0);
        this->declare_parameter<double>("drone_scale_linear_z_turbo", 2.0);
        this->declare_parameter<double>("drone_scale_angular_z_turbo", 4.0);
        
        // Hover settings to prevent drone from falling
        this->declare_parameter<bool>("drone_hover_when_disabled", true);
        this->declare_parameter<double>("drone_hover_linear_z", 0.1);

        // Read parameters
        enable_button_ = this->get_parameter("enable_button").as_int();
        turbo_button_ = this->get_parameter("turbo_button").as_int();
        toggle_button_ = this->get_parameter("toggle_button").as_int();

        husky_axis_linear_x_ = this->get_parameter("husky_axis_linear_x").as_int();
        husky_axis_angular_z_ = this->get_parameter("husky_axis_angular_z").as_int();
        husky_scale_linear_ = this->get_parameter("husky_scale_linear").as_double();
        husky_scale_angular_ = this->get_parameter("husky_scale_angular").as_double();
        husky_scale_linear_turbo_ = this->get_parameter("husky_scale_linear_turbo").as_double();
        husky_scale_angular_turbo_ = this->get_parameter("husky_scale_angular_turbo").as_double();

        drone_axis_linear_x_ = this->get_parameter("drone_axis_linear_x").as_int();
        drone_axis_linear_y_ = this->get_parameter("drone_axis_linear_y").as_int();
        drone_axis_linear_z_ = this->get_parameter("drone_axis_linear_z").as_int();
        drone_axis_angular_z_ = this->get_parameter("drone_axis_angular_z").as_int();
        drone_button_altitude_up_ = this->get_parameter("drone_button_altitude_up").as_int();
        drone_button_altitude_down_ = this->get_parameter("drone_button_altitude_down").as_int();
        drone_scale_linear_xy_ = this->get_parameter("drone_scale_linear_xy").as_double();
        drone_scale_linear_z_ = this->get_parameter("drone_scale_linear_z").as_double();
        drone_scale_angular_z_ = this->get_parameter("drone_scale_angular_z").as_double();
        drone_scale_linear_xy_turbo_ = this->get_parameter("drone_scale_linear_xy_turbo").as_double();
        drone_scale_linear_z_turbo_ = this->get_parameter("drone_scale_linear_z_turbo").as_double();
        drone_scale_angular_z_turbo_ = this->get_parameter("drone_scale_angular_z_turbo").as_double();
        drone_hover_when_disabled_ = this->get_parameter("drone_hover_when_disabled").as_bool();
        drone_hover_linear_z_ = this->get_parameter("drone_hover_linear_z").as_double();

        // Topic parameters
        this->declare_parameter<std::string>("husky_cmd_vel_topic", std::string("/husky/cmd_vel"));
        this->declare_parameter<std::string>("drone_cmd_vel_topic", std::string("/drone/cmd_vel"));
        husky_cmd_vel_topic_ = this->get_parameter("husky_cmd_vel_topic").as_string();
        drone_cmd_vel_topic_ = this->get_parameter("drone_cmd_vel_topic").as_string();

        // Publishers
        husky_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(husky_cmd_vel_topic_, 10);
        drone_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(drone_cmd_vel_topic_, 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/teleop_status", 10);

        // Joy subscriber
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&CombinedJoyTeleop::joy_callback, this, std::placeholders::_1));

        // Timer to continuously publish last command (20 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&CombinedJoyTeleop::publish_commands, this));
        // Periodic status publisher (1 Hz) so /teleop_status is always visible
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&CombinedJoyTeleop::publish_periodic_status, this));

        // Init
        last_cmd_husky_ = geometry_msgs::msg::Twist();
        last_cmd_drone_ = geometry_msgs::msg::Twist();

        RCLCPP_INFO(this->get_logger(), "Combined Joy Teleop Started (Select toggles vehicle)");
        report_active();
        // Immediate status on startup
        publish_periodic_status();
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {
        // Safety checks
        auto read_button = [&](int idx) -> int {
            if (idx >= 0 && static_cast<size_t>(idx) < joy->buttons.size()) {
                return joy->buttons[idx] ? 1 : 0;
            }
            return 0;
        };
        auto read_axis = [&](int idx) -> double {
            if (idx >= 0 && static_cast<size_t>(idx) < joy->axes.size()) {
                return joy->axes[idx];
            }
            return 0.0;
        };

        // Toggle on rising edge of Select/Back
        const bool toggle_pressed = read_button(toggle_button_);
        if (toggle_pressed && !prev_toggle_pressed_) {
            active_vehicle_ = (active_vehicle_ == ActiveVehicle::Husky) ? ActiveVehicle::Drone : ActiveVehicle::Husky;
            // Zero both on switch to avoid carry-over
            last_cmd_husky_ = geometry_msgs::msg::Twist();
            last_cmd_drone_ = geometry_msgs::msg::Twist();
            report_active();
        }
        prev_toggle_pressed_ = toggle_pressed;

        const bool lb_pressed = read_button(enable_button_);  // LB
        const bool rb_pressed = read_button(turbo_button_);   // RB

        // Bumper-based selection and enable: RB selects Husky, LB selects Drone.
        if (rb_pressed || lb_pressed) {
            ActiveVehicle prev = active_vehicle_;
            if (rb_pressed) {
                active_vehicle_ = ActiveVehicle::Husky;  // RB overrides if both pressed
            } else if (lb_pressed) {
                active_vehicle_ = ActiveVehicle::Drone;
            }
            if (active_vehicle_ != prev) {
                // Zero both on switch to avoid carry-over
                last_cmd_husky_ = geometry_msgs::msg::Twist();
                last_cmd_drone_ = geometry_msgs::msg::Twist();
                report_active();
            }
        } else {
            // Neither bumper held: disable motion and stop active vehicle
            if (active_vehicle_ == ActiveVehicle::Husky) {
                last_cmd_husky_ = geometry_msgs::msg::Twist();
                publish_status("Hold RB (Husky) or LB (Drone) to enable");
            } else {
                // For drone, apply hover thrust to prevent falling if enabled
                last_cmd_drone_ = geometry_msgs::msg::Twist();
                if (drone_hover_when_disabled_) {
                    last_cmd_drone_.linear.z = drone_hover_linear_z_;
                    publish_status("Drone: Hovering (Hold LB to control, RB for Husky)");
                } else {
                    publish_status("Hold RB (Husky) or LB (Drone) to enable");
                }
            }
            return;
        }

        if (active_vehicle_ == ActiveVehicle::Husky) {
            geometry_msgs::msg::Twist cmd;
            const double linear = read_axis(husky_axis_linear_x_);
            const double angular = read_axis(husky_axis_angular_z_);
            cmd.linear.x = linear * husky_scale_linear_;
            cmd.angular.z = angular * husky_scale_angular_;
            publish_status("Husky: Moving");
            last_cmd_husky_ = cmd;
        } else {
            geometry_msgs::msg::Twist cmd;
            const double ax_x = read_axis(drone_axis_linear_x_);
            const double ax_y = read_axis(drone_axis_linear_y_);
            const double az_z = read_axis(drone_axis_angular_z_);
            const bool up = read_button(drone_button_altitude_up_);
            const bool down = read_button(drone_button_altitude_down_);
            double ax_z = 0.0;
            if (up && !down) ax_z = 1.0;
            else if (down && !up) ax_z = -1.0;
            else if (drone_axis_linear_z_ >= 0) ax_z = read_axis(drone_axis_linear_z_);
            
            // No turbo for drone when using bumper-based selection (RB reserved for Husky)
            cmd.linear.x = ax_x * drone_scale_linear_xy_;
            cmd.linear.y = ax_y * drone_scale_linear_xy_;
            cmd.linear.z = ax_z * drone_scale_linear_z_;
            cmd.angular.z = az_z * drone_scale_angular_z_;
            
            // Add hover compensation if no altitude control is being used
            if (!up && !down && ax_z == 0.0 && drone_hover_when_disabled_) {
                cmd.linear.z += drone_hover_linear_z_;
                publish_status("Drone: Moving with hover compensation");
            } else {
                publish_status("Drone: Normal Mode");
            }
            
            last_cmd_drone_ = cmd;
        }
    }

    void publish_commands() {
        // Publish commands for active vehicle at 20 Hz
        if (active_vehicle_ == ActiveVehicle::Husky) {
            husky_cmd_vel_pub_->publish(last_cmd_husky_);
            // Send zero to drone to ensure it stops if we switched from drone
            geometry_msgs::msg::Twist zero_cmd;
            if (drone_hover_when_disabled_) {
                zero_cmd.linear.z = drone_hover_linear_z_;
            }
            drone_cmd_vel_pub_->publish(zero_cmd);
        } else {
            drone_cmd_vel_pub_->publish(last_cmd_drone_);
            // Send zero to husky to ensure it stops if we switched from husky
            geometry_msgs::msg::Twist zero_cmd;
            husky_cmd_vel_pub_->publish(zero_cmd);
        }
    }

    void publish_status(const std::string &message) {
        auto msg = std_msgs::msg::String();
        msg.data = message;
        status_pub_->publish(msg);
    }

    void report_active() {
        if (active_vehicle_ == ActiveVehicle::Husky) {
            RCLCPP_INFO(this->get_logger(), "Active vehicle: HUSKY (Hold RB to move)");
            publish_status("Active: Husky - Hold RB to move");
        } else {
            RCLCPP_INFO(this->get_logger(), "Active vehicle: DRONE (Hold LB to move)");
            publish_status("Active: Drone - Hold LB to move");
        }
    }

    void publish_periodic_status() {
        if (active_vehicle_ == ActiveVehicle::Husky) {
            publish_status("Active: Husky (Hold RB to move, LB for Drone, Y to toggle)");
        } else {
            publish_status("Active: Drone (Hold LB to move, RB for Husky, Y to toggle)");
        }
    }

    // Parameters
    int enable_button_;
    int turbo_button_;
    int toggle_button_;

    int husky_axis_linear_x_;
    int husky_axis_angular_z_;
    double husky_scale_linear_;
    double husky_scale_angular_;
    double husky_scale_linear_turbo_;
    double husky_scale_angular_turbo_;

    int drone_axis_linear_x_;
    int drone_axis_linear_y_;
    int drone_axis_linear_z_;
    int drone_axis_angular_z_;
    int drone_button_altitude_up_;
    int drone_button_altitude_down_;
    double drone_scale_linear_xy_;
    double drone_scale_linear_z_;
    double drone_scale_angular_z_;
    double drone_scale_linear_xy_turbo_;
    double drone_scale_linear_z_turbo_;
    double drone_scale_angular_z_turbo_;
    bool drone_hover_when_disabled_;
    double drone_hover_linear_z_;

    // State
    ActiveVehicle active_vehicle_;
    bool prev_toggle_pressed_;
    geometry_msgs::msg::Twist last_cmd_husky_;
    geometry_msgs::msg::Twist last_cmd_drone_;

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr husky_cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drone_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::string husky_cmd_vel_topic_;
    std::string drone_cmd_vel_topic_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CombinedJoyTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


