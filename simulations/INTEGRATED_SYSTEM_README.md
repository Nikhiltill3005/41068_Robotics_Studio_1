# Integrated Husky + Drone Simulation System

This integrated system allows you to control both the Husky UGV and the Firefighting Drone simultaneously in the same simulation environment.

## 🚀 Quick Start

### Method 1: One-Command Launch (Recommended)

**Terminal 1 - Start Integrated Simulation:**
```bash
cd /home/odyssey/ros2_ws
source install/setup.bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py
```

**Terminal 2 - Open Both Teleop Controls:**
```bash
cd /home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system
./scripts/run_dual_teleop.sh
```

This will automatically open two new terminal windows - one for Husky control and one for Drone control.

### Method 2: Manual Control Setup

**Terminal 1 - Start Simulation:**
```bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py
```

**Terminal 2 - Husky Control:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

**Terminal 3 - Drone Control:**
```bash
cd /home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system
python3 src/drone_teleop_ros.py
```

## 🎮 Controls

### 🚙 Husky UGV Controls (Terminal 2)
- **W/S**: Forward/Backward
- **A/D**: Turn Left/Right
- **Space**: Stop
- **Q**: Quit

### 🚁 Drone Controls (Terminal 3)
- **W/S**: Move Forward/Backward
- **A/D**: Strafe Left/Right
- **Q/E**: Rotate Left/Right (Yaw)
- **Space/Z**: Move Up/Down
- **R**: Reset to hover
- **ESC**: Exit

## 🌍 Available Worlds

Both robots will spawn in your chosen world:

```bash
# Simple trees world (default)
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=simple_trees

# Large demo world
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=large_demo
```

### Robot Spawn Locations

**Simple Trees World:**
- **Husky**: Origin (0, 0, 0.4)
- **Drone**: (3.0, -3.0, 0.5) - 3m east, 3m south of origin, 0.5m altitude

**Large Demo World:**
- **Husky**: Origin (0, 0, 0.4)  
- **Drone**: (5.0, -5.0, 0.5) - 5m east, 5m south of origin, 0.5m altitude

## 📡 ROS2 Topics

### Husky Topics
- `/cmd_vel` - Husky movement commands (geometry_msgs/Twist)
- `/odometry` - Husky position and velocity (nav_msgs/Odometry)
- `/imu` - Husky IMU data (sensor_msgs/Imu)
- `/scan` - Lidar data (sensor_msgs/LaserScan)
- `/camera/image` - Camera feed (sensor_msgs/Image)

### Drone Topics  
- `/drone/cmd_vel` - Drone movement commands (geometry_msgs/Twist)
- `/drone/odometry` - Drone position and velocity (nav_msgs/Odometry)
- `/drone/imu` - Drone IMU data (sensor_msgs/Imu)
- `/drone/pose` - Drone pose (geometry_msgs/PoseStamped)

### Manual ROS2 Commands

You can also control the robots directly via ROS2 commands:

**Husky Examples:**
```bash
# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}"

# Turn left
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "angular: {z: 0.5}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

**Drone Examples:**
```bash
# Take off
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "linear: {z: 1.0}"

# Move forward
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "linear: {x: 1.0}"

# Strafe right
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "linear: {y: -1.0}"

# Rotate right
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "angular: {z: -0.5}"

# Stop/hover
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist "{}"
```

## 🛠️ System Architecture

### Components
1. **Ignition Gazebo** - Physics simulation environment
2. **ROS-Ignition Bridge** - Connects ROS2 and Ignition topics
3. **Husky Robot** - Ground vehicle with sensors
4. **Firefighting Drone** - Quadcopter with multirotor physics
5. **Teleop Controllers** - Keyboard input interfaces

### Bridge Configuration
The system uses an enhanced bridge configuration (`gazebo_bridge_with_drone.yaml`) that includes mappings for both robots, allowing seamless communication between ROS2 and the simulation.

## 🔧 Launch Options

```bash
# Basic launch (both robots, simple_trees world)
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py

# With different world
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=large_demo

# With RViz for visualization
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py rviz:=true

# Without Nav2 (navigation stack)
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py nav2:=false

# Disable simulation time
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py use_sim_time:=false
```

## 📊 Monitoring and Debugging

### Check Available Topics
```bash
ros2 topic list
```

### Monitor Robot Status
```bash
# Husky odometry
ros2 topic echo /odometry

# Drone odometry  
ros2 topic echo /drone/odometry

# IMU data
ros2 topic echo /imu
ros2 topic echo /drone/imu
```

### View in RViz
```bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py rviz:=true
```

## 🎯 Use Cases

### Coordinated Missions
- **Ground Reconnaissance**: Use Husky to navigate terrain while drone provides aerial overview
- **Search and Rescue**: Coordinate between ground-based detailed search and aerial pattern coverage
- **Firefighting**: Ground robot suppresses fires while drone provides real-time intelligence
- **Mapping**: Combine lidar from ground vehicle with aerial perspective for comprehensive mapping

### Development and Testing
- **Algorithm Development**: Test coordination algorithms between ground and aerial robots
- **Sensor Fusion**: Combine data from multiple robot platforms
- **Mission Planning**: Develop multi-robot autonomous missions
- **Human-Robot Interaction**: Practice coordinated teleoperation

## 🚨 Troubleshooting

### Simulation Won't Start
1. Source workspace: `source install/setup.bash`
2. Check Ignition installation: `ign gazebo --version`
3. Verify packages built: `colcon build --packages-select 41068_ignition_bringup firefighting_system`

### Robots Don't Respond
1. Check topics exist: `ros2 topic list | grep cmd_vel`
2. Verify bridge is running: `ros2 node list | grep parameter_bridge`
3. Test manual commands: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`

### Drone Falls/Doesn't Hover
1. The drone requires constant control input or uses automatic hover mode
2. Use `R` key in teleop to reset to hover
3. Verify physics settings are reasonable

### Missing Models
1. Ensure firefighting_system package is built
2. Check model path in environment: `echo $IGN_GAZEBO_RESOURCE_PATH`
3. Verify models directory exists: `ls ~/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system/models/`

## 📝 Files Modified/Created

### Modified Files
- `41068_ignition_bringup/worlds/simple_trees.sdf` - Added drone spawn
- `41068_ignition_bringup/worlds/large_demo.sdf` - Added drone spawn  
- `41068_ignition_bringup/launch/41068_ignition.launch.py` - Enhanced for dual robot support

### New Files
- `41068_ignition_bringup/config/gazebo_bridge_with_drone.yaml` - Combined bridge config
- `firefighting_system/src/drone_teleop_ros.py` - ROS2 drone teleop
- `firefighting_system/scripts/run_dual_teleop.sh` - Dual teleop launcher
- `firefighting_system/models/firefighting_drone/` - Complete drone model

## 🎓 Next Steps

1. **Autonomous Coordination**: Implement autonomous coordination between robots
2. **Mission Planning**: Create waypoint-based missions for both robots
3. **Sensor Integration**: Add more sensors and sensor fusion algorithms
4. **Emergency Response**: Develop specific firefighting and rescue scenarios
5. **Multi-Robot SLAM**: Implement collaborative mapping
6. **Communication Protocols**: Add robot-to-robot communication simulation
