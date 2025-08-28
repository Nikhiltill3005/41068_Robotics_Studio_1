# 🚁 Firefighting Drone Installation Guide

This guide will help you install and use the firefighting drone model in your ROS2 workspace and Ignition Gazebo simulation.

## 📋 Prerequisites

- **ROS2 Humble** installed and working
- **Ignition Gazebo Fortress** installed
- **Git** for cloning the repository
- **Basic knowledge** of ROS2 and Gazebo

## 🚀 Quick Installation

### **1. Clone the Repository**
```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the firefighting system repository
git clone https://github.com/your-username/firefighting-robot-system.git

# Navigate to the firefighting system package
cd firefighting-robot-system
```

### **2. Build the Package**
```bash
# Return to workspace root
cd ~/ros2_ws

# Build the firefighting system package
colcon build --symlink-install --packages-select firefighting_system

# Source the workspace
source install/setup.bash
```

### **3. Install the Drone Model**
```bash
# Copy the drone model to Gazebo models directory
cp -r simulations/firefighting_system/models/firefighting_drone ~/.gazebo/models/

# Or install system-wide (requires sudo)
sudo cp -r simulations/firefighting_system/models/firefighting_drone /usr/share/gazebo-11/models/
```

## 🔧 Manual Installation

### **Option 1: User Directory (Recommended)**
```bash
# Create Gazebo models directory if it doesn't exist
mkdir -p ~/.gazebo/models

# Copy the drone model
cp -r simulations/firefighting_system/models/firefighting_drone ~/.gazebo/models/

# Verify installation
ls ~/.gazebo/models/firefighting_drone/
```

### **Option 2: System Directory**
```bash
# Copy to system-wide models directory
sudo cp -r simulations/firefighting_system/models/firefighting_drone /usr/share/gazebo-11/models/

# Verify installation
ls /usr/share/gazebo-11/models/firefighting_drone/
```

### **Option 3: Package Installation**
```bash
# The drone model is automatically installed with the package
# No additional steps needed if you built the package
```

## 🧪 Testing the Installation

### **Test 1: Launch Gazebo and Insert Drone**
```bash
# Launch Gazebo
gazebo --verbose worlds/empty.world

# In Gazebo:
# 1. Click "Insert" tab
# 2. Search for "firefighting_drone"
# 3. Click to place in world
# 4. The drone should appear with all sensors
```

### **Test 2: Use the Test World**
```bash
# Launch the complete test world with drone
ros2 launch firefighting_system drone_test.launch.py

# This will launch:
# - Gazebo with test world
# - Firefighting drone
# - Husky robot
# - Test fire sources
# - Drone surveillance node
```

### **Test 3: Test Individual Components**
```bash
# Test drone node only
ros2 run firefighting_system drone_surveillance_node

# Test with custom world
ros2 launch firefighting_system drone_test.launch.py world_file:=your_custom_world.sdf
```

## 📡 Available Topics

After launching the drone, you should see these topics:

```bash
# List all drone-related topics
ros2 topic list | grep -E "(drone|infrared|thermal)"

# Monitor specific topics
ros2 topic echo /infrared_camera/image_raw
ros2 topic echo /thermal_sensor
ros2 topic echo /drone/pose
ros2 topic echo /gps
ros2 topic echo /imu
```

## 🎮 Control the Drone

### **Basic Commands**
```bash
# Take off
ros2 topic pub /drone/mission_commands std_msgs/msg/String "data: 'takeoff'"

# Land
ros2 topic pub /drone/mission_commands std_msgs/msg/String "data: 'land'"

# Start survey mission
ros2 topic pub /drone/mission_commands std_msgs/msg/String "data: 'survey'"

# Return to base
ros2 topic pub /drone/mission_commands std_msgs/msg/String "data: 'return_home'"
```

### **Movement Control**
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# Move up
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {z: 0.0}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}"
```

## 🔥 Fire Detection Testing

### **Simulate Fire Detection**
```bash
# Send a fire location to trigger drone mission
ros2 topic pub /fire_locations geometry_msgs/msg/PoseStamped "{pose: {position: {x: 10.0, y: 15.0, z: 0.0}}}"

# The drone should automatically:
# 1. Take off
# 2. Fly to fire location
# 3. Perform aerial survey
# 4. Detect fire with infrared camera
```

### **Monitor Fire Detection**
```bash
# Watch drone behavior
ros2 topic echo /drone/pose

# Monitor fire detection
ros2 topic echo /fire_locations

# Check thermal sensor data
ros2 topic echo /thermal_sensor
```

## 🌍 Using in Your Own World

### **Add to World File**
```xml
<!-- Add this to your .sdf world file -->
<include>
  <uri>model://firefighting_drone</uri>
  <name>firefighting_drone_1</name>
  <pose>0 0 5 0 0 0</pose>
</include>
```

### **Add to Launch File**
```python
# In your ROS2 launch file
from launch.actions import SpawnEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Spawn the drone
spawn_drone = SpawnEntity(
    entity=SdfEntity(
        source=PathJoinSubstitution([
            FindPackageShare('firefighting_system'),
            'models', 'firefighting_drone', 'model.sdf'
        ])
    ),
    arguments=['-entity', 'firefighting_drone', '-x', '0', '-y', '0', '-z', '5']
)
```

## 🐛 Troubleshooting

### **Common Issues**

#### **Drone Model Not Found**
```bash
# Check if model is installed correctly
ls ~/.gazebo/models/firefighting_drone/

# Verify model.config and model.sdf exist
ls ~/.gazebo/models/firefighting_drone/model.*

# Restart Gazebo
pkill gzserver
pkill gzclient
```

#### **Sensors Not Working**
```bash
# Check if ROS2 topics are published
ros2 topic list | grep drone

# Verify sensor plugins are loaded
ros2 topic echo /infrared_camera/image_raw

# Check Gazebo console for plugin errors
```

#### **Performance Issues**
```bash
# Reduce camera resolution in model.sdf
# Change from 640x480 to 320x240

# Reduce update rates
# Change from 30Hz to 15Hz for camera
# Change from 100Hz to 50Hz for IMU
```

### **Debug Commands**
```bash
# Check drone node status
ros2 node list | grep drone
ros2 node info /drone_surveillance_node

# Monitor drone parameters
ros2 param list /drone_surveillance_node
ros2 param get /drone_surveillance_node max_altitude

# Check TF tree
ros2 run tf2_tools view_frames
```

## 🔧 Customization

### **Modify Drone Parameters**
```yaml
# Edit config/firefighting_config.yaml
drone_surveillance_node:
  ros__parameters:
    max_altitude: 100.0        # Increase max altitude
    survey_radius: 200.0       # Increase survey radius
    fire_detection_range: 50.0 # Increase detection range
```

### **Modify Sensor Properties**
```xml
<!-- Edit models/firefighting_drone/model.sdf -->
<camera>
  <horizontal_fov>0.785</horizontal_fov>  <!-- 45 degrees -->
  <image>
    <width>1280</width>  <!-- 4K resolution -->
    <height>720</height>
  </image>
</camera>
```

### **Add New Sensors**
```xml
<!-- Add new sensor link and joint in model.sdf -->
<link name="new_sensor_link">
  <!-- Sensor definition -->
</link>

<joint name="new_sensor_joint" type="fixed">
  <parent>base_link</parent>
  <child>new_sensor_link</child>
</joint>
```

## 📚 Next Steps

### **1. Test Basic Functionality**
- Verify drone spawns correctly
- Test takeoff and landing
- Verify all sensors are working

### **2. Test Fire Detection**
- Add fire sources to your world
- Test infrared camera detection
- Verify thermal sensor readings

### **3. Integrate with Your System**
- Connect drone to your firefighting nodes
- Test coordination between drone and ground robots
- Implement custom fire detection algorithms

### **4. Customize for Your Needs**
- Modify sensor properties
- Add new equipment
- Adjust flight parameters

## 🤝 Getting Help

If you encounter issues:

1. **Check the troubleshooting section** above
2. **Verify all prerequisites** are installed
3. **Check Gazebo console** for error messages
4. **Verify ROS2 topics** are being published
5. **Create an issue** on the GitHub repository

## 🎉 Success!

Once you see the drone in Gazebo with all sensors working, you're ready to use it for firefighting missions!

---

**Happy Flying! 🚁🔥🤖**
