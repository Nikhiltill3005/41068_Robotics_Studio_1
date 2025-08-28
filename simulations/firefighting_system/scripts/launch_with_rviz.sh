#!/bin/bash

echo "========================================================"
echo "🚁📺 FIREFIGHTING DRONE SIMULATION WITH RVIZ"
echo "========================================================"

# Set up environment
cd "$(dirname "$0")/.."
FIREFIGHTING_DIR=$(pwd)
IGNITION_DIR="$FIREFIGHTING_DIR/../41068_ignition_bringup"

# Set model paths for Ignition Gazebo
export IGN_GAZEBO_RESOURCE_PATH="$FIREFIGHTING_DIR/models:$IGNITION_DIR/models:$IGN_GAZEBO_RESOURCE_PATH"
export GZ_SIM_RESOURCE_PATH="$FIREFIGHTING_DIR/models:$IGNITION_DIR/models:$GZ_SIM_RESOURCE_PATH"

echo "🔧 Environment configured:"
echo "   Firefighting models: $FIREFIGHTING_DIR/models"
echo "   Ignition models: $IGNITION_DIR/models"
echo ""

# Build the package
echo "🔨 Building firefighting_system package..."
cd /home/odyssey/ros2_ws
colcon build --packages-select firefighting_system
source install/setup.bash

echo ""
echo "🚀 Starting simulation with RViz..."
echo ""
echo "🎮 Controls available in RViz:"
echo "   📷 RGB Camera - View from drone's RGB camera"
echo "   🔥 Thermal Camera - View from drone's thermal camera" 
echo "   🎯 Drone Pose - Live drone position and orientation"
echo "   📈 Drone Path - Trail showing drone movement"
echo "   🤖 Husky Robot - 3D model of the ground robot"
echo "   📡 TF Frames - Coordinate system visualization"
echo "   🔍 Laser Scan - Husky's LiDAR data"
echo ""
echo "🎯 Manual Controls:"
echo "   Open new terminal for Husky: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/husky_velocity_controller/cmd_vel_unstamped"
echo "   Open new terminal for Drone: ros2 run firefighting_system drone_teleop_ros.py"
echo ""
echo "⚡ Auto-launch teleop windows: ros2 launch firefighting_system drone_with_rviz.launch.py use_teleop:=true"
echo ""
echo "========================================================"

# Launch with RViz
ros2 launch firefighting_system drone_with_rviz.launch.py use_rviz:=true

echo ""
echo "🛑 Simulation stopped"
