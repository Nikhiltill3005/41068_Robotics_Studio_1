#!/bin/bash

echo "📺 Starting RViz for Drone Visualization..."
echo ""
echo "🔗 Connecting to running simulation..."
echo "   Make sure simulation is already running!"
echo ""

# Navigate to workspace and source
cd /home/odyssey/ros2_ws
source install/setup.bash

# Get the RViz config path
RVIZ_CONFIG="/home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system/config/drone_rviz.rviz"

echo "📋 RViz will display:"
echo "   📷 RGB Camera Feed"
echo "   🔥 Thermal Camera Feed" 
echo "   🎯 Drone Pose & Path"
echo "   🤖 Husky Robot Model"
echo "   🔍 Laser Scan Data"
echo "   📡 TF Coordinate Frames"
echo ""

# Launch RViz with drone configuration
rviz2 -d "$RVIZ_CONFIG"

echo "📺 RViz closed"
