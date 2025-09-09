#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

class HuskyJoyTeleop : public rclcpp::Node
{
public:
    HuskyJoyTeleop() : Node("husky_joy_teleop")
    {
        // Declare parameters for Xbox controller mapping
        this->declare_parameter<int>("axis_linear_x", 1);    // Left stick Y (forward/backward thrust)
        this->declare_parameter<int>("axis_angular_z", 3);   // Right stick X (steering left/right)
        this->declare_parameter<int>("enable_button", 4);    // LB button (left bumper)
        this->declare_parameter<int>("turbo_button", 5);     // RB button (right bumper)
        this->declare_parameter<double>("scale_linear", 2.0);     // Normal linear speed
        this->declare_parameter<double>("scale_angular", 4.0);    // Normal angular speed
        this->declare_parameter<double>("scale_linear_turbo", 4.0); // Turbo linear speed
        this->declare_parameter<double>("scale_angular_turbo", 8.0); // Turbo angular speed
        
        // Get parameters
        axis_linear_x_ = this->get_parameter("axis_linear_x").as_int();
        axis_angular_z_ = this->get_parameter("axis_angular_z").as_int();
        enable_button_ = this->get_parameter("enable_button").as_int();
        turbo_button_ = this->get_parameter("turbo_button").as_int();
        scale_linear_ = this->get_parameter("scale_linear").as_double();
        scale_angular_ = this->get_parameter("scale_angular").as_double();
        scale_linear_turbo_ = this->get_parameter("scale_linear_turbo").as_double();
        scale_angular_turbo_ = this->get_parameter("scale_angular_turbo").as_double();

        // Create publisher for velocity commands (namespaced for husky)
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/husky/cmd_vel", 10);
        
        // Create subscriber for joy messages
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&HuskyJoyTeleop::joy_callback, this, std::placeholders::_1));
        
        // Create status publisher
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/husky/teleop_status", 10);
        
        // Initialize twist message
        twist_msg_.linear.x = 0.0;
        twist_msg_.linear.y = 0.0;
        twist_msg_.linear.z = 0.0;
        twist_msg_.angular.x = 0.0;
        twist_msg_.angular.y = 0.0;
        twist_msg_.angular.z = 0.0;
        
        // Create timer for publishing commands at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&HuskyJoyTeleop::publish_command, this));
        
        RCLCPP_INFO(this->get_logger(), "Husky Joy Teleop Controller Started!");
        RCLCPP_INFO(this->get_logger(), "Xbox Controller Mapping:");
        RCLCPP_INFO(this->get_logger(), "  Left Stick Y-axis: Forward/Backward Thrust");
        RCLCPP_INFO(this->get_logger(), "  Right Stick X-axis: Left/Right Steering");
        RCLCPP_INFO(this->get_logger(), "  LB (Left Bumper): Enable movement");
        RCLCPP_INFO(this->get_logger(), "  RB (Right Bumper): Turbo mode");
        RCLCPP_INFO(this->get_logger(), "  Hold LB to enable movement, add RB for turbo");
        
        publish_status("Husky Joy Teleop Ready - Hold LB to enable movement");
    }
    
private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        // Store the latest joy message
        last_joy_ = joy;
        
        // Reset twist message
        twist_msg_.linear.x = 0.0;
        twist_msg_.linear.y = 0.0;
        twist_msg_.linear.z = 0.0;
        twist_msg_.angular.x = 0.0;
        twist_msg_.angular.y = 0.0;
        twist_msg_.angular.z = 0.0;
        
        // Check if we have enough buttons and axes
        if (joy->buttons.size() <= std::max(enable_button_, turbo_button_) ||
            joy->axes.size() <= std::max(axis_linear_x_, axis_angular_z_))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Joy message doesn't have enough buttons/axes");
            return;
        }
        
        // Check if enable button is pressed
        if (joy->buttons[enable_button_])
        {
            // Get axis values
            double linear = joy->axes[axis_linear_x_];
            double angular = joy->axes[axis_angular_z_];
            
            // Check for turbo mode
            bool turbo = joy->buttons[turbo_button_];
            
            // Apply scaling
            if (turbo)
            {
                twist_msg_.linear.x = linear * scale_linear_turbo_;
                twist_msg_.angular.z = angular * scale_angular_turbo_;
                publish_status("TURBO MODE - Moving");
            }
            else
            {
                twist_msg_.linear.x = linear * scale_linear_;
                twist_msg_.angular.z = angular * scale_angular_;
                publish_status("Normal Mode - Moving");
            }
            
            // Log movement info (throttled to avoid spam)
            if (std::abs(linear) > 0.1 || std::abs(angular) > 0.1)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Linear: %.2f, Angular: %.2f %s", 
                    twist_msg_.linear.x, twist_msg_.angular.z,
                    turbo ? "(TURBO)" : "");
            }
        }
        else
        {
            // Enable button not pressed - stop the robot
            publish_status("Hold LB to enable movement");
        }
    }
    
    void publish_command()
    {
        // Always publish the current twist message
        cmd_vel_pub_->publish(twist_msg_);
    }
    
    void publish_status(const std::string& message)
    {
        auto status_msg = std_msgs::msg::String();
        status_msg.data = message;
        status_pub_->publish(status_msg);
    }
    
    // ROS2 components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters for controller mapping
    int axis_linear_x_;
    int axis_angular_z_;
    int enable_button_;
    int turbo_button_;
    double scale_linear_;
    double scale_angular_;
    double scale_linear_turbo_;
    double scale_angular_turbo_;
    
    // Current command and joy state
    geometry_msgs::msg::Twist twist_msg_;
    sensor_msgs::msg::Joy::SharedPtr last_joy_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<HuskyJoyTeleop>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Husky Joy Teleop node...");
    
    try {
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
