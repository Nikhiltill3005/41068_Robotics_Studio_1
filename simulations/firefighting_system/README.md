# 🔥🚁 Firefighting Robot System with Drone

A complete ROS2-based firefighting robot system featuring both ground (Husky UGV) and aerial (Drone) platforms with Ignition Gazebo simulation and RViz visualization. This project implements a comprehensive firefighting solution with specialized nodes for teleoperation, movement, mapping, and autonomous navigation, plus an integrated firefighting drone with dual camera systems.

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Ignition Gazebo](https://img.shields.io/badge/Simulator-Ignition%20Gazebo%20Fortress-orange.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

## 🎯 Project Overview

This system implements a complete firefighting robot solution with ground and aerial platforms:

### 🤖 **Ground Platform (Husky UGV)**
- **🔥 Teleop Control Node** - Manual control interface and command processing
- **🤖 UGV Movement Node** - Robot movement execution and kinematics
- **🗺️ Environment Mapping Node** - Environment mapping and fire detection
- **🧭 Autonomous Navigation Node** - Path planning and autonomous navigation

### 🚁 **Aerial Platform (Firefighting Drone)**
- **📷 RGB Camera** - High-resolution visual surveillance (640x480@30fps)
- **🔥 Thermal Camera** - Heat detection and fire identification (320x240@10fps)
- **🎮 Multiple Teleop Options** - ROS2 Twist, Direct Ignition, Individual thrust control
- **📡 Full ROS Integration** - Pose, odometry, IMU, camera feeds
- **📺 RViz Visualization** - Real-time camera feeds and flight path tracking

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
│                    Integrated Simulation                       │
│        Husky Robot + Firefighting Drone (Ignition Gazebo)     │
│           📷 RGB Camera  🔥 Thermal Camera  📺 RViz           │
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
sudo apt install ros-humble-rviz2 ros-humble-rqt-image-view

# Camera and image processing (for drone cameras)
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# Install development tools
sudo apt install ros-dev-tools
```

## 🚀 Installation

### 1. Clone the Repository

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/your-username/41068_Robotics_Studio_1.git
```

### 2. Build the Package

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

# List available launch files
ros2 launch firefighting_system --help
```

## 🎮 Usage - Quick Start

### 🚁📺 **Complete System with RViz (Recommended)**

**🎯 Method 1: Using wrapper script (recommended for model path issues)**
```bash
# Navigate to firefighting_system directory
cd ~/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system

# Basic launch with RViz
./launch_drone_rviz.sh

# Launch with automatic teleop controls
./launch_drone_rviz.sh use_teleop:=true

# Launch with specific world
./launch_drone_rviz.sh world:=large_demo

# Combined options
./launch_drone_rviz.sh world:=large_demo use_teleop:=true
```

**🎯 Method 2: Direct ROS2 launch (if environment is pre-configured)**
```bash
# First set environment variables manually
export IGN_GAZEBO_RESOURCE_PATH="~/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system/models:~/ros2_ws/src/41068_Robotics_Studio_1/simulations/41068_ignition_bringup/models:$IGN_GAZEBO_RESOURCE_PATH"
export GZ_SIM_RESOURCE_PATH="~/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system/models:~/ros2_ws/src/41068_Robotics_Studio_1/simulations/41068_ignition_bringup/models:$GZ_SIM_RESOURCE_PATH"

# Then launch
ros2 launch firefighting_system drone_with_rviz.launch.py
ros2 launch firefighting_system drone_with_rviz.launch.py use_teleop:=true
ros2 launch firefighting_system drone_with_rviz.launch.py world:=large_demo
```

### 🎮 **Manual Control Setup**
```bash
# Start simulation first
ros2 launch firefighting_system drone_with_rviz.launch.py

# Then in separate terminals:
# Husky control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/husky_velocity_controller/cmd_vel_unstamped

# Drone control  
ros2 run firefighting_system drone_teleop_ros.py
```

### 📺 **RViz Only (for running simulation)**
```bash
# If simulation is already running
ros2 launch firefighting_system rviz_only.launch.py
```

### 🎮 **Teleop Controls Only**
```bash
# Launch both control terminals
ros2 launch firefighting_system teleop_controls.launch.py

# Launch only drone control
ros2 launch firefighting_system teleop_controls.launch.py drone_only:=true

# Launch only husky control
ros2 launch firefighting_system teleop_controls.launch.py husky_only:=true
```

## 📋 Launch File Options

### `drone_with_rviz.launch.py`
**Complete simulation launcher with RViz**

**Arguments:**
- `world` (default: `simple_trees`): World to load (`simple_trees` or `large_demo`)
- `use_rviz` (default: `true`): Whether to launch RViz
- `use_teleop` (default: `false`): Whether to auto-launch teleop terminals

**Examples:**
```bash
# Default: simple_trees world with RViz
ros2 launch firefighting_system drone_with_rviz.launch.py

# Large demo world
ros2 launch firefighting_system drone_with_rviz.launch.py world:=large_demo

# Without RViz (simulation only)
ros2 launch firefighting_system drone_with_rviz.launch.py use_rviz:=false

# With automatic teleop terminals
ros2 launch firefighting_system drone_with_rviz.launch.py use_teleop:=true

# Combined options
ros2 launch firefighting_system drone_with_rviz.launch.py world:=large_demo use_teleop:=true
```

### `rviz_only.launch.py`
**RViz visualization only**

**Arguments:**
- `rviz_config` (default: `drone_rviz.rviz`): RViz configuration file

### `teleop_controls.launch.py`
**Teleop control terminals**

**Arguments:**
- `drone_only` (default: `false`): Launch only drone teleop
- `husky_only` (default: `false`): Launch only husky teleop

## 🛠️ Individual ROS2 Run Commands

### **Camera Monitoring**
```bash
# Simple camera status monitor (no OpenCV needed)
ros2 run firefighting_system drone_camera_simple.py

# Visual camera viewer (if NumPy issues fixed)
ros2 run firefighting_system drone_camera_viewer.py
```

### **Robot Control**
```bash
# Husky teleop control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/husky_velocity_controller/cmd_vel_unstamped

# Drone teleop control (ROS2 topics)
ros2 run firefighting_system drone_teleop_ros.py

# Drone teleop control (direct Ignition)
ros2 run firefighting_system drone_teleop_ignition.py

# Alternative drone control (individual thrusters)
ros2 run firefighting_system drone_teleop_control.py
```

### **Visualization Tools**
```bash
# RViz with drone configuration
rviz2 -d ~/ros2_ws/install/firefighting_system/share/firefighting_system/config/drone_rviz.rviz

# Image viewer for RGB camera
ros2 run rqt_image_view rqt_image_view /drone/rgb_camera/image

# Image viewer for thermal camera
ros2 run rqt_image_view rqt_image_view /drone/thermal_camera/image
```

## 🔍 Diagnostic Commands

### **Topic Information**
```bash
# List all available topics
ros2 topic list

# Check camera topics
ros2 topic list | grep camera

# Check drone topics
ros2 topic list | grep drone

# Check Husky topics
ros2 topic list | grep husky
```

### **Topic Monitoring**
```bash
# Monitor RGB camera feed
ros2 topic echo /drone/rgb_camera/image --max-count 1

# Monitor thermal camera feed
ros2 topic echo /drone/thermal_camera/image --max-count 1

# Check camera frame rates
ros2 topic hz /drone/rgb_camera/image
ros2 topic hz /drone/thermal_camera/image

# Monitor drone pose
ros2 topic echo /drone/pose

# Monitor drone commands
ros2 topic echo /drone/cmd_vel
```

### **Node Information**
```bash
# List running nodes
ros2 node list

# Check if bridge is running
ros2 node list | grep bridge

# Check if Gazebo is running
ros2 node list | grep gazebo

# Node details
ros2 node info /rviz2
```

## 📺 RViz Interface Guide

### 🎯 Camera View Setup

When RViz launches, you'll see:

1. **📷 RGB Camera Panel**: 
   - Topic: `/drone/rgb_camera/image`
   - Format: RGB8 (color)
   - Resolution: 640x480 @ ~30fps
   - Use: Navigation, visual inspection

2. **🔥 Thermal Camera Panel**:
   - Topic: `/drone/thermal_camera/image` 
   - Format: MONO8 (grayscale processed as thermal)
   - Resolution: 320x240 @ ~10fps
   - Use: Heat detection, fire identification

### 🎮 3D View Controls

- **🔄 Rotate**: Left mouse drag
- **📏 Zoom**: Mouse wheel or right mouse drag
- **🔄 Pan**: Middle mouse drag or Shift + left mouse drag
- **🎯 Focus**: Click on object then use 'F' key

### 📊 Display Panel Controls

- **✅ Enable/Disable**: Check/uncheck display items
- **🎨 Color Settings**: Click color boxes to change visualization colors
- **📐 Scale**: Adjust size of arrows, markers, etc.
- **🔧 Topic**: Change which topics to visualize

### RViz Display Features

- **📷 RGB Camera Panel**: Live drone camera feed
- **🔥 Thermal Camera Panel**: Heat detection visualization
- **🎯 Drone Pose**: Real-time position/orientation
- **📈 Drone Path**: Flight trail visualization
- **🤖 Husky Model**: 3D robot visualization
- **🔍 Laser Scan**: LiDAR point cloud
- **📡 TF Frames**: Coordinate systems

## 🎯 Typical Workflow

### **1. Start Complete System**
```bash
# Terminal 1: Launch everything
./launch_drone_rviz.sh use_teleop:=true
```

### **2. Manual Control Setup**
```bash
# Terminal 1: Main simulation
./launch_drone_rviz.sh

# Terminal 2: Husky control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/husky_velocity_controller/cmd_vel_unstamped

# Terminal 3: Drone control
ros2 run firefighting_system drone_teleop_ros.py

# Terminal 4: Camera monitoring (optional)
ros2 run firefighting_system drone_camera_simple.py
```

### **3. Add RViz to Running Simulation**
```bash
# If simulation is already running without RViz
ros2 launch firefighting_system rviz_only.launch.py
```

## 🚁 Drone System Details

### 📷 **Camera Specifications**

#### RGB Camera
- **Field of View**: 60° (1.047 radians)
- **Format**: RGB8 (24-bit color)
- **Noise**: Gaussian (mean=0, stddev=0.01)
- **Orientation**: Downward-facing for ground surveillance

#### Thermal Camera  
- **Field of View**: 60° (1.047 radians)
- **Format**: MONO8 (8-bit grayscale, thermal-processed)
- **Noise**: Gaussian (mean=0, stddev=0.005)
- **Visualization**: Heat-map coloring in viewer
- **Orientation**: Downward-facing for fire detection

### 🎮 **Control Options**

#### ROS2 Twist Control (Recommended)
```bash
ros2 run firefighting_system drone_teleop_ros.py
```
- Uses standard `geometry_msgs/Twist` messages
- Compatible with ROS2 ecosystem
- Integrates with navigation stack

#### Direct Ignition Control
```bash
ros2 run firefighting_system drone_teleop_ignition.py
```
- Direct communication with Ignition Gazebo
- Lower latency control
- Bypass ROS2 bridge

#### Individual Thrust Control
```bash
ros2 run firefighting_system drone_teleop_control.py
```
- Direct rotor thrust control
- Fine-grained flight control
- Advanced flight maneuvers

### 📡 **Drone Topics**

#### Published Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/drone/odometry` | `nav_msgs/Odometry` | Drone position and velocity |
| `/drone/pose` | `geometry_msgs/PoseStamped` | Drone pose |
| `/drone/imu` | `sensor_msgs/Imu` | IMU data |
| `/drone/rgb_camera/image` | `sensor_msgs/Image` | RGB camera feed |
| `/drone/thermal_camera/image` | `sensor_msgs/Image` | Thermal camera feed |

#### Subscribed Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/drone/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |

## 🔧 Configuration

### 📄 **Configuration Files**

#### Drone RViz Configuration
- **File**: `config/drone_rviz.rviz`
- **Purpose**: Pre-configured RViz layout for drone visualization
- **Features**: Camera feeds, pose display, flight path tracking

#### Camera Configuration
- **RGB Camera**: 640x480 resolution, 30fps, RGB8 format
- **Thermal Camera**: 320x240 resolution, 10fps, MONO8 format
- **Both cameras**: Downward-facing, 60° FOV

#### Bridge Configuration
- **File**: `config/gazebo_bridge_with_drone.yaml` (in 41068_ignition_bringup)
- **Purpose**: ROS-Ignition topic bridging for both Husky and Drone

### 🎛️ **Node Parameters**

Each node can be configured via ROS2 parameters:

```bash
# View drone-related parameters
ros2 param list | grep drone

# Set drone parameters
ros2 param set /drone_teleop_node max_velocity 5.0
```

## 🛠️ NumPy Compatibility Fix

If you encounter NumPy issues with the camera viewer:

### **Quick Solution: Use Simple Monitor**
```bash
ros2 run firefighting_system drone_camera_simple.py
```

### **Fix NumPy Issue**
```bash
# Method 1: Downgrade NumPy
pip install "numpy<2.0"

# Method 2: Use system packages
sudo apt remove python3-numpy
sudo apt install python3-numpy

# Method 3: Virtual environment
python3 -m venv ~/camera_env
source ~/camera_env/bin/activate
pip install "numpy<2.0" opencv-python
```

### **Alternative: Use ROS2 Image Tools**
```bash
# View RGB camera
ros2 run rqt_image_view rqt_image_view /drone/rgb_camera/image

# View thermal camera
ros2 run rqt_image_view rqt_image_view /drone/thermal_camera/image
```

## 🧪 Testing

### **Camera System Test**
```bash
# Check available camera topics
ros2 topic list | grep camera

# Test RGB camera data
ros2 topic echo /drone/rgb_camera/image --max-count 1

# Test thermal camera data  
ros2 topic echo /drone/thermal_camera/image --max-count 1

# Check frame rates
ros2 topic hz /drone/rgb_camera/image
ros2 topic hz /drone/thermal_camera/image
```

### **Flight Test Patterns**

#### Basic Camera Test
1. Start simulation with RViz
2. Take off drone (use teleop controls)
3. Hover at ~5m altitude
4. Observe camera feeds in RViz
5. Move drone over different terrain
6. Check thermal vs RGB differences

#### Path Following Test
1. Enable "Drone Path" display in RViz
2. Fly drone in patterns (circles, figure-8)
3. Observe path trail visualization
4. Use path data for flight analysis

### **Simulation Testing**

1. **Launch with RViz**:
   ```bash
   ./launch_drone_rviz.sh use_teleop:=true
   ```

2. **Test each control method**:
   ```bash
   # Test ROS2 twist control
   ros2 run firefighting_system drone_teleop_ros.py
   
   # Test direct Ignition control
   ros2 run firefighting_system drone_teleop_ignition.py
   ```

3. **Verify camera feeds**:
   ```bash
   # Monitor camera topics
   ros2 topic echo /drone/rgb_camera/image --max-count 1
   ros2 topic echo /drone/thermal_camera/image --max-count 1
   ```

## 🐛 Troubleshooting

### **Common Issues**

#### Model Not Found Error
```
[Err] Unable to find uri[model://firefighting_drone]
```
**Solution**: Use the wrapper script which sets environment variables:
```bash
./launch_drone_rviz.sh
```

#### Camera Feeds Not Showing in RViz
```bash
# Check if simulation is running
ros2 node list | grep -E "(gazebo|bridge)"

# Verify camera topics exist
ros2 topic list | grep camera

# Check bridge is working
ros2 node list | grep bridge
```

#### Drone Not Moving
```bash
# Check if drone teleop is publishing
ros2 topic echo /drone/cmd_vel

# Verify bridge is translating commands
ros2 topic list | grep cmd_vel

# Check Ignition topics
ign topic -l | grep firefighting_drone
```

#### NumPy/OpenCV Issues
```bash
# Use simple camera monitor instead
ros2 run firefighting_system drone_camera_simple.py

# Or fix NumPy version
pip install "numpy<2.0"
```

### **Debug Commands**

```bash
# Check all nodes
ros2 node list

# Monitor drone topics
ros2 topic list | grep drone

# Check topic data
ros2 topic echo /drone/pose
ros2 topic echo /drone/imu

# Monitor camera performance  
ros2 topic hz /drone/rgb_camera/image
ros2 topic hz /drone/thermal_camera/image

# View TF tree
ros2 run tf2_tools view_frames
```

## 📊 Expected Performance

### **System Performance**
- **Simulation**: Runs smoothly on modern hardware
- **Camera Feeds**: RGB @30fps, Thermal @10fps
- **Control Latency**: <100ms for ROS2 topics
- **RViz Update**: Real-time camera and pose visualization

### **Camera Performance**
- **RGB Camera**: High resolution for navigation
- **Thermal Camera**: Lower resolution optimized for heat detection
- **Network Load**: Manageable with compression options available

## 🎯 Mission Scenarios

### **🔥 Firefighting Mission Example**
1. **Ground Survey**: Use Husky to patrol and map the area
2. **Aerial Reconnaissance**: Deploy drone for aerial surveillance
3. **Heat Detection**: Use thermal camera to identify fire hotspots
4. **Coordination**: Both platforms work together for comprehensive coverage
5. **Real-time Monitoring**: RViz provides mission control interface

### **🏗️ Integration Features**
- **Dual Platform Control**: Simultaneous ground and air operations
- **Shared World**: Both robots operate in the same simulation environment
- **Unified Visualization**: Single RViz interface for both platforms
- **Coordinated Mission Planning**: Complementary capabilities

## 🤝 Contributing

We welcome contributions! Here's how you can help:

### **Development Areas**
- **Ground Robot**: Husky navigation and mapping improvements
- **Drone System**: Flight control and camera processing
- **Integration**: Coordination between platforms
- **Visualization**: RViz enhancements and mission interfaces

### **Development Workflow**
1. **Fork the repository**
2. **Create a feature branch**
3. **Test thoroughly with both platforms**
4. **Submit a pull request**

## 📚 Documentation Files

This README consolidates information from several specialized documentation files:

- **`ROS2_COMMANDS.md`**: Complete ROS2 command reference
- **`RVIZ_USAGE.md`**: Detailed RViz setup and usage guide
- **`NUMPY_FIX.md`**: NumPy compatibility solutions
- **`DRONE_INSTALLATION.md`**: Drone-specific setup instructions

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## 👥 Team

This integrated system was developed collaboratively for the **41068 Robotics Studio I** course, featuring both ground and aerial robotics platforms.

### Contributors
- **Ground Platform Team**: Husky UGV system development
- **Aerial Platform Team**: Firefighting drone system development
- **Integration Team**: Combined system coordination
- **Visualization Team**: RViz and monitoring systems

## 🙏 Acknowledgments

- **Clearpath Robotics** for the Husky platform
- **Open Robotics** for Ignition Gazebo and ROS2
- **ROS2 Community** for the excellent framework
- **41068 Course Staff** for guidance and support

## 📞 Support

If you encounter any issues or have questions:

1. **Check the troubleshooting section** above
2. **Review the diagnostic commands**
3. **Search existing issues** on GitHub
4. **Create a new issue** with detailed information
5. **Contact the development team**

---

## 🚀 **Quick Start Summary**

### **🎯 Recommended Launch Sequence:**

```bash
# 1. Navigate to project directory
cd ~/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system

# 2. Launch complete system with auto-teleop
./launch_drone_rviz.sh use_teleop:=true

# 3. Wait for Gazebo, RViz, and teleop windows to open
# 4. Use WASD controls in teleop terminals
# 5. Monitor camera feeds in RViz
# 6. Enjoy coordinated firefighting missions! 🔥🤖🚁
```

**Happy Firefighting! 🔥🤖🚁**

*Remember: Safety first! This is a simulation system for educational purposes.*