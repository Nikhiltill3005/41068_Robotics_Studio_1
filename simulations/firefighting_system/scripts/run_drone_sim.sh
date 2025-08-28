#!/bin/bash

# Firefighting Drone Simulation Runner
# This script sets up the environment and launches the drone simulation

echo "========================================================"
echo "FIREFIGHTING DRONE SIMULATION"
echo "========================================================"

# Source ROS 2 workspace
source /opt/ros/humble/setup.bash
cd /home/odyssey/ros2_ws
source install/setup.bash

echo "Starting Ignition Gazebo with drone simulation..."
echo ""
echo "Controls (run in separate terminal):"
echo "  python3 src/drone_teleop_ros.py"
echo ""
echo "Or using ROS2 commands:"
echo "  ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist '{linear: {z: 1.0}}'"
echo ""

# Set environment variables for Ignition
export IGN_GAZEBO_RESOURCE_PATH="/home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system/models:$IGN_GAZEBO_RESOURCE_PATH"

# Launch the simulation with ROS2 bridge
ros2 launch firefighting_system drone_simulation.launch.py
