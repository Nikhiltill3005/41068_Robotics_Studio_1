# 🔥 Firefighting Robot System

A ROS2-based firefighting robot system using the Husky UGV platform with Ignition Gazebo simulation. This project implements a complete autonomous firefighting solution with four specialized nodes working collaboratively.

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Ignition Gazebo](https://img.shields.io/badge/Simulator-Ignition%20Gazebo%20Fortress-orange.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

## 🎯 Project Overview

This system implements a complete firefighting robot solution with four main nodes working together:

- **🔥 Teleop Control Node** - Manual control interface and command processing
- **🤖 UGV Movement Node** - Robot movement execution and kinematics
- **🗺️ Environment Mapping Node** - Environment mapping and fire detection
- **🧭 Autonomous Navigation Node** - Path planning and autonomous navigation


## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Teleop       │    │   Environment   │    │   Autonomous   │
│   Control      │    │   Mapping       │    │   Navigation   │
│   Node         │    │   Node          │    │   Node         │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    UGV Movement Node                           │
│              (Movement Execution & Kinematics)                 │
└─────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Husky Robot Platform                        │
│                    (Ignition Gazebo)                          │
└─────────────────────────────────────────────────────────────────┘
```

## 📋 Prerequisites

- **Ubuntu 22.04** (recommended)
- **ROS2 Humble** ([Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))
- **Ignition Gazebo Fortress**
- **Git**

### Installing Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade

# Install ROS2 Humble
sudo apt install ros-humble-desktop

# Install Ignition Gazebo Fortress
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress

# Install additional ROS2 packages
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros-ign ros-humble-ros-ign-interfaces
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
sudo apt install ros-humble-teleop-twist-keyboard

# Install development tools
sudo apt install ros-dev-tools
```

## 🚀 Installation

### 1. Clone the Repository

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/your-username/firefighting-robot-system.git
```



### 3. Build the Package

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the firefighting system package
colcon build --symlink-install --packages-select firefighting_system

# Source the workspace
source install/setup.bash
```

### 3. Verify Installation

```bash
# Check if package is available
ros2 pkg list | grep firefighting

# List available nodes
ros2 run firefighting_system --help
```

## 🎮 Usage

### Launch the Complete System

```bash
# Basic launch with default settings
ros2 launch firefighting_system firefighting_system.launch.py

# Launch with custom parameters
ros2 launch firefighting_system firefighting_system.launch.py \
  world:=large_demo \
  use_slam:=true \
  use_nav2:=true \
  use_rviz:=true


```

### Launch Individual Nodes

```bash
# Teleop control only
ros2 run firefighting_system teleop_control_node

# UGV movement only
ros2 run firefighting_system ugv_movement_node

# Environment mapping only
ros2 run firefighting_system environment_mapping_node

# Autonomous navigation only
ros2 run firefighting_system autonomous_navigation_node


```

### Control Interface

#### Keyboard Controls (WASD)
- **W/S** - Increase/Decrease speed
- **A/D** - Turn left/right
- **Q/E** - Fine steering control
- **Space** - Stop robot

#### External Commands
Send commands via the `/teleop_commands` topic:
```bash
ros2 topic pub /teleop_commands std_msgs/msg/String "data: 'w'"
```

## 🔧 Configuration

### Node Parameters

Each node can be configured via ROS2 parameters:

```bash
# View current parameters
ros2 param list

# Set parameters
ros2 param set /teleop_control_node max_speed 3.0
ros2 param set /ugv_movement_node wheelbase 0.6
```

### Configuration File

Edit `config/firefighting_config.yaml` to modify default parameters:

```yaml
firefighting_system:
  ros__parameters:
    teleop_control_node:
      max_steering_angle: 0.5
      max_speed: 2.0
      steering_sensitivity: 0.1
      speed_sensitivity: 0.5
    
    ugv_movement_node:
      wheelbase: 0.5
      max_steering_angle: 0.5
      max_speed: 2.0
    
    environment_mapping_node:
      map_resolution: 0.05
      map_width: 1000
      map_height: 1000
      map_origin_x: -25.0
      map_origin_y: -25.0
    
    autonomous_navigation_node:
      goal_tolerance: 0.1
      max_linear_velocity: 1.0
      max_angular_velocity: 1.0
      path_following_gain: 1.0
```

## 📡 Topics and Communication

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_ackermann` | `ackermann_msgs/AckermannDrive` | Ackermann drive commands |
| `/cmd_vel` | `geometry_msgs/Twist` | Twist velocity commands |
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid map |

| `/planned_path` | `geometry_msgs/PoseStamped` | Current navigation waypoint |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/teleop_commands` | `std_msgs/String` | Teleop control commands |
| `/scan` | `sensor_msgs/LaserScan` | Laser scan data |
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid map |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goals |

## 🧪 Testing

### Simulation Testing

1. **Launch with Ignition Gazebo**:
   ```bash
   ros2 launch firefighting_system firefighting_system.launch.py
   ```

2. **Test each node individually**:
   ```bash
   # Test teleop control
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   
   # Test movement
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
   ```

3. **Verify topic communication**:
   ```bash
   # List active topics
   ros2 topic list
   
   # Monitor specific topics
   ros2 topic echo /odom
   ros2 topic echo /map
   ```

### Unit Testing

```bash
# Build with testing enabled
colcon build --cmake-args -DBUILD_TESTING=ON

# Run tests
colcon test --packages-select firefighting_system

# View test results
colcon test-result --verbose
```

## 🛠️ Development

### Project Structure

```
firefighting_system/
├── include/                          # Header files
│   ├── teleop_control_node.hpp      # Teleop control interface
│   ├── ugv_movement_node.hpp        # UGV movement and kinematics
│   ├── environment_mapping_node.hpp  # Environment mapping
│   └── autonomous_navigation_node.hpp # Autonomous navigation and path planning
├── src/                             # Source files
│   ├── teleop_control_node.cpp      # Manual control implementation
│   ├── ugv_movement_node.cpp        # Movement execution
│   ├── environment_mapping_node.cpp  # Mapping
│   └── autonomous_navigation_node.cpp # Navigation implementation
├── launch/                          # Launch files
│   └── firefighting_system.launch.py # Complete system launch
├── config/                          # Configuration files
│   └── firefighting_config.yaml     # Node parameters
├── package.xml                      # Package dependencies
├── CMakeLists.txt                   # Build configuration
└── README.md                        # This file
```

### Adding New Features

1. **Create header file** in `include/`
2. **Implement source file** in `src/`
3. **Update CMakeLists.txt** to build new node
4. **Add to launch file** if needed
5. **Test and document**

### Code Style Guidelines

- Follow ROS2 C++ style guidelines
- Use meaningful variable names
- Add proper error handling
- Include comprehensive logging
- Document all public interfaces

## 🐛 Troubleshooting

### Common Issues

#### Build Errors
```bash
# Clean build directory
rm -rf build/ install/ log/

# Rebuild package
colcon build --symlink-install --packages-select firefighting_system
```

#### TF Errors
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Verify transforms are being published
ros2 topic echo /tf
```

#### Topic Not Found
```bash
# List active topics
ros2 topic list

# Check node connections
ros2 node info /node_name
```

#### Simulation Crashes
```bash
# Restart Gazebo
pkill gzserver
pkill gzclient

# Clear cache
rm -rf ~/.gazebo/
```

### Debug Commands

```bash
# Check node status
ros2 node list
ros2 node info /node_name

# Monitor topics
ros2 topic list
ros2 topic echo /topic_name

# Check parameters
ros2 param list
ros2 param get /node_name parameter_name

# View TF tree
ros2 run tf2_tools view_frames
```

## 🤝 Contributing

We welcome contributions! Here's how you can help:

### Development Workflow

1. **Fork the repository**
2. **Create a feature branch**:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Make your changes**
4. **Test thoroughly**
5. **Commit your changes**:
   ```bash
   git commit -m "Add feature: description of changes"
   ```
6. **Push to your fork**:
   ```bash
   git push origin feature/your-feature-name
   ```
7. **Submit a pull request**

### Team Collaboration

This project is designed for collaborative development:

- **Node 1**: Teleop Control - Handle manual control and command processing
- **Node 2**: UGV Movement - Implement robot movement and kinematics
- **Node 3**: Environment Mapping - Develop mapping
- **Node 4**: Autonomous Navigation - Create navigation and path planning

### Code Review Guidelines

- Ensure all tests pass
- Follow coding standards
- Add appropriate documentation
- Include example usage
- Update relevant documentation

## 📚 Documentation

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Ignition Gazebo Documentation](https://gazebosim.org/docs)
- [Husky Robot Documentation](https://husky.clearpathrobotics.com/)
- [Navigation2 Documentation](https://navigation.ros.org/)

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## 👥 Team

This system was developed collaboratively for the **41068 Robotics Studio I** course.

### Contributors

- **Teleop Control Team** - Manual control interface
- **UGV Movement Team** - Robot movement and kinematics
- **Environment Mapping Team** - Mapping and fire detection
- **Autonomous Navigation Team** - Navigation and path planning

## 🙏 Acknowledgments

- **Clearpath Robotics** for the Husky platform
- **Open Robotics** for Ignition Gazebo
- **ROS2 Community** for the excellent framework
- **41068 Course Staff** for guidance and support

## 📞 Support

If you encounter any issues or have questions:

1. **Check the troubleshooting section** above
2. **Search existing issues** on GitHub
3. **Create a new issue** with detailed information
4. **Contact the development team**

---

**Happy Firefighting! 🔥🤖**

*Remember: Safety first! This is a simulation system for educational purposes.*
