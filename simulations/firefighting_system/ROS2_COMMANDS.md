# 🚀 ROS2 Commands for Firefighting Drone System

Clean ROS2 launch and run commands for the firefighting drone simulation with RViz visualization.

## 🎯 Quick Start Commands

### 📺 **Complete Simulation with RViz**

**🎯 Method 1: Using wrapper script (recommended for model path issues)**
```bash
# Navigate to firefighting_system directory
cd /home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system

# Basic launch with RViz
./launch_drone_rviz.sh

# Launch with specific world
./launch_drone_rviz.sh world:=large_demo

# Launch with automatic teleop controls
./launch_drone_rviz.sh use_teleop:=true
```

**🎯 Method 2: Direct ROS2 launch (if environment is pre-configured)**
```bash
# First set environment variables manually
export IGN_GAZEBO_RESOURCE_PATH="/home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system/models:/home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/41068_ignition_bringup/models:$IGN_GAZEBO_RESOURCE_PATH"
export GZ_SIM_RESOURCE_PATH="/home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system/models:/home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/41068_ignition_bringup/models:$GZ_SIM_RESOURCE_PATH"

# Then launch
ros2 launch firefighting_system drone_with_rviz.launch.py
ros2 launch firefighting_system drone_with_rviz.launch.py world:=large_demo
ros2 launch firefighting_system drone_with_rviz.launch.py use_teleop:=true
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

**Examples:**
```bash
# Default drone configuration
ros2 launch firefighting_system rviz_only.launch.py

# Custom configuration (if you create one)
ros2 launch firefighting_system rviz_only.launch.py rviz_config:=my_config.rviz
```

### `teleop_controls.launch.py`
**Teleop control terminals**

**Arguments:**
- `drone_only` (default: `false`): Launch only drone teleop
- `husky_only` (default: `false`): Launch only husky teleop

**Examples:**
```bash
# Both Husky and Drone controls
ros2 launch firefighting_system teleop_controls.launch.py

# Only drone control
ros2 launch firefighting_system teleop_controls.launch.py drone_only:=true

# Only husky control  
ros2 launch firefighting_system teleop_controls.launch.py husky_only:=true
```

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
rviz2 -d /home/odyssey/ros2_ws/install/firefighting_system/share/firefighting_system/config/drone_rviz.rviz

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

### **Service Calls**
```bash
# List available services
ros2 service list

# Call a service (example: save camera image)
ros2 service call /drone/rgb_camera/save_image sensor_msgs/srv/SetString "{data: 'capture.jpg'}"
```

## 🎯 Typical Workflow

### **1. Start Complete System**
```bash
# Terminal 1: Launch everything
ros2 launch firefighting_system drone_with_rviz.launch.py use_teleop:=true
```

### **2. Manual Control Setup**
```bash
# Terminal 1: Main simulation
ros2 launch firefighting_system drone_with_rviz.launch.py

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

## 🎨 RViz Features Available

When RViz launches, you'll see:
- **📷 RGB Camera Panel**: Live drone camera feed
- **🔥 Thermal Camera Panel**: Heat detection visualization
- **🎯 Drone Pose**: Real-time position/orientation
- **📈 Drone Path**: Flight trail visualization
- **🤖 Husky Model**: 3D robot visualization
- **🔍 Laser Scan**: LiDAR point cloud
- **📡 TF Frames**: Coordinate systems

## 🚁 Ready to Fly!

**Quick test sequence:**
1. `ros2 launch firefighting_system drone_with_rviz.launch.py use_teleop:=true`
2. Wait for Gazebo, RViz, and teleop terminals to open
3. Use teleop controls to move both robots
4. Watch camera feeds in RViz
5. Monitor flight path visualization

**Perfect for firefighting mission simulation!** 🔥👨‍🚒🚁
