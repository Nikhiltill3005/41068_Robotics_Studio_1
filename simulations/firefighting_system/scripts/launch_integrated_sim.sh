#!/bin/bash

# Integrated Husky + Drone Simulation Launcher
# This script properly sets up the environment and launches the integrated simulation

echo "========================================================"
echo "INTEGRATED HUSKY + DRONE SIMULATION"
echo "========================================================"

# Source ROS 2 workspace
cd /home/odyssey/ros2_ws
source install/setup.bash

echo "Setting up environment for integrated simulation..."

# Set up model paths for Ignition Gazebo (include both firefighting and base models)
export IGN_GAZEBO_RESOURCE_PATH="/home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system/models:/home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/41068_ignition_bringup/models:$IGN_GAZEBO_RESOURCE_PATH"

# Also set the GZ_SIM equivalent (for newer versions)
export GZ_SIM_RESOURCE_PATH="/home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system/models:/home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/41068_ignition_bringup/models:$GZ_SIM_RESOURCE_PATH"

echo "Model paths configured:"
echo "  IGN_GAZEBO_RESOURCE_PATH: $IGN_GAZEBO_RESOURCE_PATH"
echo ""

# Check if required packages are built
if [ ! -d "install/firefighting_system" ]; then
    echo "⚠️  firefighting_system package not found in install directory"
    echo "Building required packages..."
    colcon build --packages-select firefighting_system 41068_ignition_bringup
    source install/setup.bash
fi

echo "Starting integrated simulation..."
echo ""
echo "🚙 Husky will spawn at origin (0, 0)"
echo "🚁 Drone will spawn at (3, -3, 0.5)"
echo ""
echo "After simulation starts, run the teleop script:"
echo "  ./src/41068_Robotics_Studio_1/simulations/firefighting_system/scripts/run_dual_teleop.sh"
echo ""
echo "========================================================"

# Launch the integrated simulation
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=simple_trees
