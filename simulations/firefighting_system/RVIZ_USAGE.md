# 📺 RViz Visualization for Firefighting Drone System

This guide explains how to use RViz for visualizing the firefighting drone simulation, including camera feeds, robot poses, and sensor data.

## 🚀 Quick Start

### Option 1: Launch Everything with RViz (Recommended)
```bash
cd /home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system
./scripts/launch_with_rviz.sh
```

### Option 2: Launch with Teleop Controls Auto-opened
```bash
cd /home/odyssey/ros2_ws
source install/setup.bash
ros2 launch firefighting_system drone_with_rviz.launch.py use_teleop:=true
```

### Option 3: Connect RViz to Running Simulation
```bash
# First, start simulation in one terminal:
./scripts/launch_integrated_sim.sh

# Then, in another terminal, start RViz:
./scripts/run_rviz_only.sh
```

## 📋 RViz Display Panels

The custom RViz configuration includes the following displays:

### 🎥 Camera Feeds
- **📷 RGB Camera**: Live feed from drone's RGB camera (`/drone/rgb_camera/image`)
- **🔥 Thermal Camera**: Thermal imaging feed (`/drone/thermal_camera/image`)

### 🤖 Robot Visualization
- **🚁 Drone Pose**: Real-time drone position and orientation (`/drone/pose`)
- **📈 Drone Path**: Trail showing drone's flight path (`/drone/path`)
- **🤖 Husky Robot**: 3D model of the ground robot
- **📡 TF Frames**: Coordinate system visualization

### 📊 Sensor Data
- **🔍 Laser Scan**: Husky's LiDAR data (`/scan`)
- **🧭 IMU Data**: Integrated in TF frames

### 🌍 Environment
- **🏗️ Grid**: Reference grid for spatial orientation
- **🗺️ Map Frame**: Fixed coordinate reference

## 🎮 Manual Control Setup

### Husky Control
```bash
# In a new terminal:
cd /home/odyssey/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/husky_velocity_controller/cmd_vel_unstamped
```

### Drone Control
```bash
# In a new terminal:
cd /home/odyssey/ros2_ws
source install/setup.bash
ros2 run firefighting_system drone_teleop_ros.py
```

### Camera Monitoring (if NumPy issues resolved)
```bash
# Alternative: Simple camera monitor
python3 src/41068_Robotics_Studio_1/simulations/firefighting_system/src/drone_camera_simple.py

# Or visual camera viewer (if NumPy fixed)
python3 src/41068_Robotics_Studio_1/simulations/firefighting_system/src/drone_camera_viewer.py
```

## 📺 RViz Interface Guide

### 🎯 Camera View Setup

1. **RGB Camera Panel**: 
   - Topic: `/drone/rgb_camera/image`
   - Format: RGB8 (color)
   - Resolution: 640x480 @ ~30fps
   - Use: Navigation, visual inspection

2. **Thermal Camera Panel**:
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

## 🛠️ Customizing RViz Configuration

### Adding New Displays

1. Click **"Add"** button in Displays panel
2. Choose display type (Image, Pose, Path, etc.)
3. Set topic name
4. Configure display properties

### Common Display Types for Drone System

- **Image**: For camera feeds
- **Pose**: For robot positions
- **Path**: For trajectory visualization
- **TF**: For coordinate frames
- **Marker**: For custom 3D objects
- **PointCloud**: For 3D sensor data

### Saving Custom Configuration

1. Arrange displays as desired
2. File → Save Config As...
3. Save to `config/my_drone_config.rviz`
4. Update launch file to use new config

## 🔧 Troubleshooting

### Camera Feeds Not Showing

```bash
# Check if simulation is running
ros2 node list | grep -E "(gazebo|bridge)"

# Verify camera topics exist
ros2 topic list | grep camera

# Check image data
ros2 topic echo /drone/rgb_camera/image --max-count 1
ros2 topic echo /drone/thermal_camera/image --max-count 1

# Test frame rates
ros2 topic hz /drone/rgb_camera/image
ros2 topic hz /drone/thermal_camera/image
```

### Robot Model Not Loading

```bash
# Check robot description topic
ros2 topic echo /robot_description --max-count 1

# Verify TF frames
ros2 run tf2_tools view_frames.py
```

### Poor Performance

1. **Reduce Image Quality**:
   - Right-click camera display → Properties
   - Increase "Transport Hint" to "compressed"

2. **Lower Update Rates**:
   - Set camera "Unreliable" to true
   - Reduce "Queue Size" to 5

3. **Disable Unused Displays**:
   - Uncheck displays you're not actively using

### Display Panels Missing

1. **Window → Displays**: Show Displays panel
2. **Window → Tool Properties**: Show Tool Properties
3. **View → Toolbars**: Show/hide toolbars

## 📚 Advanced Features

### 🎬 Recording and Playback

```bash
# Record ROS bag with camera data
ros2 bag record /drone/rgb_camera/image /drone/thermal_camera/image /drone/pose

# Playback for analysis
ros2 bag play <bag_file>
```

### 📸 Image Capture

```bash
# Save current camera frame
ros2 service call /drone/rgb_camera/save_image sensor_msgs/srv/SetString "{data: 'capture.jpg'}"
```

### 🎯 Interactive Markers

RViz supports interactive markers for:
- Setting drone waypoints
- Adjusting camera parameters
- Manual positioning commands

## 🎯 Camera Configuration Details

### RGB Camera Specifications
- **Field of View**: 60° (1.047 radians)
- **Format**: RGB8 (24-bit color)
- **Noise**: Gaussian (mean=0, stddev=0.01)
- **Orientation**: Downward-facing for ground surveillance

### Thermal Camera Specifications  
- **Field of View**: 60° (1.047 radians)
- **Format**: MONO8 (8-bit grayscale, thermal-processed)
- **Noise**: Gaussian (mean=0, stddev=0.005)
- **Visualization**: Heat-map coloring in viewer
- **Orientation**: Downward-facing for fire detection

## 🚁 Flight Patterns for Testing

### Basic Camera Test
1. Start simulation with RViz
2. Take off drone (use teleop controls)
3. Hover at ~5m altitude
4. Observe camera feeds in RViz
5. Move drone over different terrain
6. Check thermal vs RGB differences

### Path Following Test
1. Enable "Drone Path" display in RViz
2. Fly drone in patterns (circles, figure-8)
3. Observe path trail visualization
4. Use path data for flight analysis

## 🎮 Control Integration

The RViz setup works seamlessly with:
- **Husky ground robot** control via teleop_twist_keyboard
- **Drone control** via custom teleop scripts
- **Camera switching** between RGB and thermal views
- **Real-time pose** tracking and visualization

Perfect for firefighting mission simulation where you need both aerial surveillance and ground navigation! 🚁🔥👨‍🚒
