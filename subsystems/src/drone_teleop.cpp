#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class DroneTeleop : public rclcpp::Node
{
public:
    DroneTeleop() : Node("drone_teleop")
    {
        // Create publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/drone/cmd_vel", 10);
        
        // Create subscriber for IMU data (optional, for status display)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/drone/imu", 10,
            std::bind(&DroneTeleop::imu_callback, this, std::placeholders::_1));
        
        // Create status publisher
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/drone/teleop_status", 10);
        
        // Initialize terminal settings
        setup_terminal();
        
        // Create timer for publishing commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&DroneTeleop::publish_command, this));
        
        // Initialize twist message
        twist_msg_.linear.x = 0.0;
        twist_msg_.linear.y = 0.0;
        twist_msg_.linear.z = 0.0;
        twist_msg_.angular.x = 0.0;
        twist_msg_.angular.y = 0.0;
        twist_msg_.angular.z = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Drone Teleop Controller Started!");
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  w/s: forward/backward");
        RCLCPP_INFO(this->get_logger(), "  a/d: left/right");
        RCLCPP_INFO(this->get_logger(), "  q/e: up/down");
        RCLCPP_INFO(this->get_logger(), "  z/c: yaw left/right");
        RCLCPP_INFO(this->get_logger(), "  x: stop");
        RCLCPP_INFO(this->get_logger(), "  Ctrl+C: exit");
    }
    
    ~DroneTeleop()
    {
        restore_terminal();
    }
    
private:
    void setup_terminal()
    {
        // Get terminal settings
        tcgetattr(STDIN_FILENO, &old_termios_);
        new_termios_ = old_termios_;
        
        // Disable canonical mode and echo
        new_termios_.c_lflag &= ~(ICANON | ECHO);
        new_termios_.c_cc[VMIN] = 0;
        new_termios_.c_cc[VTIME] = 0;
        
        // Apply new settings
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios_);
        
        // Make stdin non-blocking
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    
    void restore_terminal()
    {
        // Restore original terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }
    
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Store latest IMU data for status display
        latest_imu_ = *msg;
    }
    
    void publish_command()
    {
        char key;
        bool key_pressed = false;
        
        // Check for key press
        if (read(STDIN_FILENO, &key, 1) > 0)
        {
            key_pressed = true;
            process_key(key);
        }
        
        // Publish current command
        cmd_vel_pub_->publish(twist_msg_);
        
        // Publish status if key was pressed
        if (key_pressed)
        {
            publish_status();
        }
    }
    
    void process_key(char key)
    {
        // Reset all velocities
        twist_msg_.linear.x = 0.0;
        twist_msg_.linear.y = 0.0;
        twist_msg_.linear.z = 0.0;
        twist_msg_.angular.x = 0.0;
        twist_msg_.angular.y = 0.0;
        twist_msg_.angular.z = 0.0;
        
        switch (key)
        {
            case 'w':
                twist_msg_.linear.x = 1.0;  // Forward
                RCLCPP_INFO(this->get_logger(), "Moving FORWARD");
                break;
            case 's':
                twist_msg_.linear.x = -1.0; // Backward
                RCLCPP_INFO(this->get_logger(), "Moving BACKWARD");
                break;
            case 'a':
                twist_msg_.linear.y = 1.0;  // Left
                RCLCPP_INFO(this->get_logger(), "Moving LEFT");
                break;
            case 'd':
                twist_msg_.linear.y = -1.0; // Right
                RCLCPP_INFO(this->get_logger(), "Moving RIGHT");
                break;
            case 'q':
                twist_msg_.linear.z = 1.0;  // Up
                RCLCPP_INFO(this->get_logger(), "Moving UP");
                break;
            case 'e':
                twist_msg_.linear.z = -1.0; // Down
                RCLCPP_INFO(this->get_logger(), "Moving DOWN");
                break;
            case 'z':
                twist_msg_.angular.z = 1.0; // Yaw left
                RCLCPP_INFO(this->get_logger(), "Yawing LEFT");
                break;
            case 'c':
                twist_msg_.angular.z = -1.0; // Yaw right
                RCLCPP_INFO(this->get_logger(), "Yawing RIGHT");
                break;
            case 'x':
                // All velocities already reset to 0
                RCLCPP_INFO(this->get_logger(), "STOPPING");
                break;
            case 27: // ESC key
                RCLCPP_INFO(this->get_logger(), "Exiting...");
                rclcpp::shutdown();
                break;
            default:
                // Unknown key, keep current velocities
                break;
        }
    }
    
    void publish_status()
    {
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "Linear: [" + 
                         std::to_string(twist_msg_.linear.x) + ", " +
                         std::to_string(twist_msg_.linear.y) + ", " +
                         std::to_string(twist_msg_.linear.z) + "] " +
                         "Angular: [" +
                         std::to_string(twist_msg_.angular.x) + ", " +
                         std::to_string(twist_msg_.angular.y) + ", " +
                         std::to_string(twist_msg_.angular.z) + "]";
        
        status_pub_->publish(status_msg);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::Twist twist_msg_;
    sensor_msgs::msg::Imu latest_imu_;
    
    struct termios old_termios_, new_termios_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DroneTeleop>();
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

