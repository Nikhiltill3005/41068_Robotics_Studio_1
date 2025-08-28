#!/bin/bash

# Dual Robot Teleop Control Script
# This script opens two terminal windows for controlling both Husky and Drone

echo "========================================================"
echo "DUAL ROBOT TELEOP CONTROL"
echo "========================================================"
echo "This script will open two terminal windows:"
echo "1. Husky Teleop Control"
echo "2. Drone Teleop Control"
echo ""
echo "Make sure the simulation is already running!"
echo "========================================================"

# Source ROS 2 workspace
cd /home/odyssey/ros2_ws
source install/setup.bash

# Check if simulation is running
if ! ros2 topic list | grep -q "/cmd_vel\|/drone/cmd_vel"; then
    echo "⚠️  Warning: Simulation doesn't seem to be running!"
    echo "Please start the simulation first with:"
    echo "  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py"
    echo ""
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "🚙 Opening Husky teleop in new terminal..."
# Open Husky teleop in new terminal
gnome-terminal --title="Husky Teleop Control" --working-directory="/home/odyssey/ros2_ws" -- bash -c "
echo '========================================================'
echo 'HUSKY TELEOP CONTROL'
echo '========================================================'
echo 'Controls:'
echo '  W/S - Forward/Backward'
echo '  A/D - Turn Left/Right'
echo '  Space - Stop'
echo '  Q - Quit'
echo '========================================================'
echo ''
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
"

sleep 1

echo "🚁 Opening Drone teleop in new terminal..."
# Open Drone teleop in new terminal
gnome-terminal --title="Drone Teleop Control" --working-directory="/home/odyssey/ros2_ws" -- bash -c "
echo '========================================================'
echo 'DRONE TELEOP CONTROL'  
echo '========================================================'
echo 'Controls:'
echo '  W/S - Forward/Backward'
echo '  A/D - Strafe Left/Right'
echo '  Q/E - Rotate Left/Right'
echo '  Space/Z - Up/Down'
echo '  R - Reset (Stop)'
echo '  ESC - Exit'
echo '========================================================'
echo ''
source install/setup.bash
python3 src/41068_Robotics_Studio_1/simulations/firefighting_system/src/drone_teleop_ros.py
"

echo ""
echo "✅ Both teleop terminals opened!"
echo ""
echo "📋 Quick Reference:"
echo "🚙 Husky (Terminal 1):"
echo "   W/S: Forward/Back, A/D: Turn, Space: Stop"
echo ""
echo "🚁 Drone (Terminal 2):" 
echo "   W/S: Forward/Back, A/D: Strafe, Space/Z: Up/Down"
echo "   Q/E: Rotate, R: Reset"
echo ""
echo "Press Ctrl+C in this terminal to exit this script"
echo "(Note: This won't affect the teleop terminals)"

# Keep script running until user cancels
while true; do
    sleep 1
done
