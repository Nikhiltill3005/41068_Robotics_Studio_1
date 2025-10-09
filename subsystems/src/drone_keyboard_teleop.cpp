#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <cmath>
#include <string>
#include <map>

enum class ActiveVehicle {
    Husky,
    Drone
};

class CombinedKeyboardTeleop : public rclcpp::Node
{
public:
    CombinedKeyboardTeleop() : rclcpp::Node("combined_keyboard_teleop"),
                               active_vehicle_(ActiveVehicle::Husky)
    {
        // Husky params
        this->declare_parameter<double>("husky_scale_linear", 2.0);
        this->declare_parameter<double>("husky_scale_angular", 4.0);
        this->declare_parameter<double>("husky_scale_linear_turbo", 4.0);
        this->declare_parameter<double>("husky_scale_angular_turbo", 8.0);

        // Drone params
        this->declare_parameter<double>("drone_scale_linear_xy", 1.5);
        this->declare_parameter<double>("drone_scale_linear_z", 1.0);
        this->declare_parameter<double>("drone_scale_angular_z", 2.0);
        this->declare_parameter<double>("drone_scale_linear_xy_turbo", 3.0);
        this->declare_parameter<double>("drone_scale_linear_z_turbo", 2.0);
        this->declare_parameter<double>("drone_scale_angular_z_turbo", 4.0);
        
        // Hover settings
        this->declare_parameter<bool>("drone_hover_when_disabled", true);
        this->declare_parameter<double>("drone_hover_linear_z", 0.1);

        // Key timeout (ms) - if no key update, assume released
        this->declare_parameter<double>("key_timeout", 50.0);

        // Read parameters
        husky_scale_linear_ = this->get_parameter("husky_scale_linear").as_double();
        husky_scale_angular_ = this->get_parameter("husky_scale_angular").as_double();
        husky_scale_linear_turbo_ = this->get_parameter("husky_scale_linear_turbo").as_double();
        husky_scale_angular_turbo_ = this->get_parameter("husky_scale_angular_turbo").as_double();

        drone_scale_linear_xy_ = this->get_parameter("drone_scale_linear_xy").as_double();
        drone_scale_linear_z_ = this->get_parameter("drone_scale_linear_z").as_double();
        drone_scale_angular_z_ = this->get_parameter("drone_scale_angular_z").as_double();
        drone_scale_linear_xy_turbo_ = this->get_parameter("drone_scale_linear_xy_turbo").as_double();
        drone_scale_linear_z_turbo_ = this->get_parameter("drone_scale_linear_z_turbo").as_double();
        drone_scale_angular_z_turbo_ = this->get_parameter("drone_scale_angular_z_turbo").as_double();
        drone_hover_when_disabled_ = this->get_parameter("drone_hover_when_disabled").as_bool();
        drone_hover_linear_z_ = this->get_parameter("drone_hover_linear_z").as_double();
        
        key_timeout_ms_ = this->get_parameter("key_timeout").as_double();

        // Topic parameters
        this->declare_parameter<std::string>("husky_cmd_vel_topic", std::string("/husky/cmd_vel"));
        this->declare_parameter<std::string>("drone_cmd_vel_topic", std::string("/drone/cmd_vel"));
        husky_cmd_vel_topic_ = this->get_parameter("husky_cmd_vel_topic").as_string();
        drone_cmd_vel_topic_ = this->get_parameter("drone_cmd_vel_topic").as_string();

        // Publishers
        husky_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(husky_cmd_vel_topic_, 10);
        drone_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(drone_cmd_vel_topic_, 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/teleop_status", 10);

        // Timer at 50 Hz for smooth control
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&CombinedKeyboardTeleop::update, this));

        // Setup terminal
        setup_terminal();

        // Initialize
        last_cmd_husky_ = geometry_msgs::msg::Twist();
        last_cmd_drone_ = geometry_msgs::msg::Twist();

        print_instructions();
        RCLCPP_INFO(this->get_logger(), "Combined Keyboard Teleop Started");
        report_active();
    }

    ~CombinedKeyboardTeleop()
    {
        // Stop both vehicles
        last_cmd_husky_ = geometry_msgs::msg::Twist();
        last_cmd_drone_ = geometry_msgs::msg::Twist();
        husky_cmd_vel_pub_->publish(last_cmd_husky_);
        drone_cmd_vel_pub_->publish(last_cmd_drone_);
        restore_terminal();
    }

private:
    void print_instructions()
    {
        printf("\n");
        printf("===========================================\n");
        printf("Combined Keyboard Teleop Controls:\n");
        printf("-------------------------------------------\n");
        printf("  TAB       : Toggle between Husky/Drone\n");
        printf("-------------------------------------------\n");
        printf("HUSKY MODE:\n");
        printf("  W/S       : Forward/Backward\n");
        printf("  A/D       : Turn Left/Right\n");
        printf("  SHIFT     : Turbo mode\n");
        printf("-------------------------------------------\n");
        printf("DRONE MODE:\n");
        printf("  W/S       : Forward/Backward\n");
        printf("  A/D       : Strafe Left/Right\n");
        printf("  Q/E       : Yaw Left/Right\n");
        printf("  R/F       : Altitude Up/Down\n");
        printf("  SHIFT     : Turbo mode\n");
        printf("-------------------------------------------\n");
        printf("  SPACE     : Emergency Stop\n");
        printf("  X/Ctrl+C  : Quit\n");
        printf("===========================================\n");
        printf("\nHold multiple keys for diagonal/combined movement.\n\n");
        fflush(stdout);
    }

    void report_active()
    {
        if (active_vehicle_ == ActiveVehicle::Husky)
        {
            RCLCPP_INFO(this->get_logger(), ">>> ACTIVE VEHICLE: HUSKY <<<");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), ">>> ACTIVE VEHICLE: DRONE <<<");
        }
    }

    void setup_terminal()
    {
        tcgetattr(STDIN_FILENO, &original_terminal_);
        
        struct termios raw = original_terminal_;
        raw.c_lflag &= ~(ECHO | ICANON);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    void restore_terminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
    }

    void process_keyboard_input()
    {
        char c;
        auto now = this->now();
        
        // Read all available characters
        while (read(STDIN_FILENO, &c, 1) > 0)
        {
            // Handle shift (uppercase letters)
            if (c >= 'A' && c <= 'Z')
            {
                key_states_['_'] = now; // '_' represents shift
                c = c + 32; // Convert to lowercase
            }
            
            // Update key state timestamp
            key_states_[c] = now;
        }

        // Clean up old key states (keys assumed released after timeout)
        auto it = key_states_.begin();
        while (it != key_states_.end())
        {
            auto age_ms = (now - it->second).seconds() * 1000.0;
            if (age_ms > key_timeout_ms_)
            {
                it = key_states_.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    bool is_key_pressed(char key)
    {
        return key_states_.find(key) != key_states_.end();
    }

    void update()
    {
        // Process all keyboard input
        process_keyboard_input();

        // Check for special commands
        if (is_key_pressed('\t'))
        {
            // Toggle vehicle (with debounce)
            if (!tab_pressed_)
            {
                active_vehicle_ = (active_vehicle_ == ActiveVehicle::Husky) ? 
                                  ActiveVehicle::Drone : ActiveVehicle::Husky;
                last_cmd_husky_ = geometry_msgs::msg::Twist();
                last_cmd_drone_ = geometry_msgs::msg::Twist();
                report_active();
                tab_pressed_ = true;
            }
        }
        else
        {
            tab_pressed_ = false;
        }

        if (is_key_pressed(' '))
        {
            // Emergency stop
            last_cmd_husky_ = geometry_msgs::msg::Twist();
            last_cmd_drone_ = geometry_msgs::msg::Twist();
            husky_cmd_vel_pub_->publish(last_cmd_husky_);
            drone_cmd_vel_pub_->publish(last_cmd_drone_);
            return;
        }

        if (is_key_pressed('x') || is_key_pressed('\x03'))
        {
            rclcpp::shutdown();
            return;
        }

        // Check for turbo (shift pressed)
        bool turbo = is_key_pressed('_');

        // Calculate velocities based on currently pressed keys
        double vx = 0.0, vy = 0.0, vz = 0.0, vyaw = 0.0;

        // Forward/Backward (both vehicles)
        if (is_key_pressed('w')) vx += 1.0;
        if (is_key_pressed('s')) vx -= 1.0;

        if (active_vehicle_ == ActiveVehicle::Husky)
        {
            // Husky: A/D for turning - always apply simultaneously with forward/back
            if (is_key_pressed('a')) vyaw += 1.0;
            if (is_key_pressed('d')) vyaw -= 1.0;

            // NO normalization - let both axes work at full strength
            // W+A = full forward + full left turn (rightward arc)
            // This creates natural curved motion
            
            double scale_linear = turbo ? husky_scale_linear_turbo_ : husky_scale_linear_;
            double scale_angular = turbo ? husky_scale_angular_turbo_ : husky_scale_angular_;

            last_cmd_husky_.linear.x = vx * scale_linear;
            last_cmd_husky_.angular.z = vyaw * scale_angular;

            // Zero drone with hover
            last_cmd_drone_ = geometry_msgs::msg::Twist();
            if (drone_hover_when_disabled_)
            {
                last_cmd_drone_.linear.z = drone_hover_linear_z_;
            }
        }
        else // Drone
        {
            // Drone: Full multi-axis control
            // A/D for strafing
            if (is_key_pressed('a')) vy += 1.0;
            if (is_key_pressed('d')) vy -= 1.0;
            
            // Q/E for yaw - always apply simultaneously with other movements
            if (is_key_pressed('q')) vyaw += 1.0;
            if (is_key_pressed('e')) vyaw -= 1.0;
            
            // R/F for altitude - always apply simultaneously with other movements
            if (is_key_pressed('r')) vz += 1.0;
            if (is_key_pressed('f')) vz -= 1.0;

            // Only normalize XY to prevent excessive speed on diagonals
            // Keep yaw and altitude at full strength for responsive control
            double xy_magnitude = std::sqrt(vx * vx + vy * vy);
            if (xy_magnitude > 1.0)
            {
                vx /= xy_magnitude;
                vy /= xy_magnitude;
            }

            // Don't normalize Z and yaw - allow full simultaneous control
            // This means W+E = full forward + full yaw right (forward arc)
            // W+D+R+E = diagonal forward-right + ascending + yaw right (spiral!)
            
            double scale_xy = turbo ? drone_scale_linear_xy_turbo_ : drone_scale_linear_xy_;
            double scale_z = turbo ? drone_scale_linear_z_turbo_ : drone_scale_linear_z_;
            double scale_yaw = turbo ? drone_scale_angular_z_turbo_ : drone_scale_angular_z_;

            last_cmd_drone_.linear.x = vx * scale_xy;
            last_cmd_drone_.linear.y = vy * scale_xy;
            last_cmd_drone_.linear.z = vz * scale_z;
            last_cmd_drone_.angular.z = vyaw * scale_yaw;

            // Add hover compensation if no altitude input
            if (vz == 0.0 && drone_hover_when_disabled_)
            {
                last_cmd_drone_.linear.z += drone_hover_linear_z_;
            }

            // Zero husky
            last_cmd_husky_ = geometry_msgs::msg::Twist();
        }

        // Publish commands
        husky_cmd_vel_pub_->publish(last_cmd_husky_);
        drone_cmd_vel_pub_->publish(last_cmd_drone_);
    }

    // Parameters
    double husky_scale_linear_;
    double husky_scale_angular_;
    double husky_scale_linear_turbo_;
    double husky_scale_angular_turbo_;

    double drone_scale_linear_xy_;
    double drone_scale_linear_z_;
    double drone_scale_angular_z_;
    double drone_scale_linear_xy_turbo_;
    double drone_scale_linear_z_turbo_;
    double drone_scale_angular_z_turbo_;
    bool drone_hover_when_disabled_;
    double drone_hover_linear_z_;
    double key_timeout_ms_;

    std::string husky_cmd_vel_topic_;
    std::string drone_cmd_vel_topic_;

    // State
    ActiveVehicle active_vehicle_;
    geometry_msgs::msg::Twist last_cmd_husky_;
    geometry_msgs::msg::Twist last_cmd_drone_;
    std::map<char, rclcpp::Time> key_states_; // Track key press timestamps
    bool tab_pressed_ = false; // Debounce tab

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr husky_cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drone_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Terminal state
    struct termios original_terminal_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CombinedKeyboardTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}