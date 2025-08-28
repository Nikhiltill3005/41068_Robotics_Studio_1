#!/bin/bash

# 🚁📺 Firefighting Drone with RViz Launcher
# This script sets up environment variables and launches the simulation

echo "🚁📺 FIREFIGHTING DRONE SIMULATION WITH RVIZ"
echo "============================================="

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
FIREFIGHTING_DIR="$SCRIPT_DIR"
IGNITION_DIR="$FIREFIGHTING_DIR/../41068_ignition_bringup"

# Set model paths for Ignition Gazebo
export IGN_GAZEBO_RESOURCE_PATH="$FIREFIGHTING_DIR/models:$IGNITION_DIR/models:$IGN_GAZEBO_RESOURCE_PATH"
export GZ_SIM_RESOURCE_PATH="$FIREFIGHTING_DIR/models:$IGNITION_DIR/models:$GZ_SIM_RESOURCE_PATH"

echo "🔧 Model paths configured:"
echo "   Firefighting models: $FIREFIGHTING_DIR/models"
echo "   Ignition models: $IGNITION_DIR/models"
echo ""

# Build and source
echo "🔨 Building workspace..."
cd /home/odyssey/ros2_ws
colcon build --packages-select firefighting_system
source install/setup.bash

echo ""
echo "🚀 Launching simulation with RViz..."
echo ""

# Parse command line arguments for ROS2 launch
ARGS=""
for arg in "$@"; do
    ARGS="$ARGS $arg"
done

# Launch with ROS2
exec ros2 launch firefighting_system drone_with_rviz.launch.py $ARGS
