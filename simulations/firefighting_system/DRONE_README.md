# Firefighting Drone Simulation

This package now includes a fully functional drone that can be controlled in the Ignition Gazebo simulator.

## Features

- **Functional Quadcopter Model**: 4-rotor drone with realistic physics
- **Keyboard Teleop Control**: Easy-to-use keyboard interface for manual control
- **Thrust-based Control**: Individual rotor thrust control for precise movement
- **Test Environment**: Dedicated world with obstacles and fire sources for testing

## Quick Start

### Method 1: Using the convenience script
```bash
# Terminal 1: Start simulation
cd /home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system
./scripts/run_drone_sim.sh

# Terminal 2: Control the drone
python3 src/drone_teleop_control.py
```

### Method 2: Manual launch
```bash
# Terminal 1: Source workspace and launch simulation
cd /home/odyssey/ros2_ws
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$(pwd)/src/41068_Robotics_Studio_1/simulations/firefighting_system/models:$IGN_GAZEBO_RESOURCE_PATH"
ros2 launch firefighting_system drone_simulation.launch.py

# Terminal 2: Control the drone (option 1)
cd /home/odyssey/ros2_ws
source install/setup.bash
python3 src/41068_Robotics_Studio_1/simulations/firefighting_system/src/drone_teleop_control.py

# Terminal 2: Control the drone (option 2 - using installed script)
ros2 run firefighting_system drone_teleop_control.py
```

## Drone Controls

The teleop control script provides intuitive keyboard controls:

### Movement Controls
- **W/S**: Move Forward/Backward
- **A/D**: Strafe Left/Right  
- **Q/E**: Rotate Left/Right (Yaw)
- **Space**: Move Up
- **Z**: Move Down
- **R**: Reset to hover mode
- **ESC/Ctrl+C**: Exit

### Control Tips
1. Start with gentle movements - the drone responds quickly
2. Use **R** to return to hover if you lose control
3. The drone will hover automatically when no keys are pressed
4. **Space** for up, **Z** for down (easier than Shift)

## Technical Details

### Drone Model
- **Mass**: 1.5 kg main body + 0.01 kg per rotor
- **Rotor Configuration**: Quadcopter (X-configuration)
- **Thrust Range**: 0-20N per rotor
- **Hover Thrust**: ~6N per rotor (total 24N for 1.5kg drone)

### Physics
- **Physics Engine**: ODE with 250Hz update rate
- **Gravity**: Standard Earth gravity (9.81 m/s²)
- **Realistic Inertia**: Proper mass distribution and rotational inertia

### Control System
- **Individual Rotor Control**: Each rotor controlled independently
- **Mixing Logic**: Forward/back, left/right, yaw, and altitude commands mixed to individual rotor thrusts
- **Safety**: Thrust clamping to prevent over-thrust

## Files Structure

```
firefighting_system/
├── models/firefighting_drone/
│   ├── model.sdf          # Drone physics and visual model
│   └── model.config       # Model metadata
├── worlds/
│   └── drone_test_world.sdf  # Test world with obstacles and fires
├── src/
│   └── drone_teleop_control.py  # Keyboard control script
├── launch/
│   └── drone_simulation.launch.py  # Launch file for simulation
└── scripts/
    └── run_drone_sim.sh   # Convenience script to start everything
```

## ROS 2 Topics

The drone publishes and subscribes to the following topics:

### Command Topics (Subscribed)
- `/firefighting_drone/thrust_0` - Front right rotor thrust
- `/firefighting_drone/thrust_1` - Front left rotor thrust  
- `/firefighting_drone/thrust_2` - Back left rotor thrust
- `/firefighting_drone/thrust_3` - Back right rotor thrust

### Sensor Topics (Published)
- `/imu` - IMU data from the drone
- `/model/firefighting_drone/pose` - Drone pose information

## Troubleshooting

### Simulation won't start
1. Make sure ROS 2 workspace is sourced: `source install/setup.bash`
2. Check that Ignition Gazebo is installed: `ign gazebo --version`
3. Verify model path is set correctly

### Drone doesn't respond to controls
1. Check that teleop script is publishing: `ros2 topic list | grep thrust`
2. Verify topics are being received: `ros2 topic echo /firefighting_drone/thrust_0`
3. Make sure simulation is running and not paused

### Drone falls or behaves erratically
1. Check physics settings in the world file
2. Verify thrust values are reasonable (0-20N range)
3. Try resetting with 'R' key in teleop

## Next Steps

1. **Autonomous Flight**: Implement autonomous navigation algorithms
2. **Sensor Integration**: Add cameras, lidar, or other sensors
3. **Mission Planning**: Create waypoint-based mission execution
4. **Fire Detection**: Integrate fire detection and suppression capabilities

## Dependencies

- ROS 2 Humble
- Ignition Gazebo (Garden or newer)
- Python 3
- Standard ROS 2 packages (geometry_msgs, std_msgs, etc.)
