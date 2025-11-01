# Husky Navigation with GUI

This package provides a complete navigation solution for the Husky robot with SLAM mapping and a Python GUI for setting navigation goals.

## Features

- **SLAM Mapping**: Real-time mapping using slam_toolbox
- **Nav2 Navigation**: Full autonomous navigation stack
- **Navigation GUI**: Interactive GUI for setting X,Y coordinate goals
- **Proper Namespacing**: All topics properly namespaced to `/husky/`
- **Click-to-Navigate**: Click on the map to set navigation goals
- **Manual Coordinate Entry**: Enter precise X,Y coordinates
- **Quick Goals**: Predefined quick navigation goals

## Usage

### 1. Launch the Simulation
First, start the combined simulation (Husky + Drone):
```bash
ros2 launch 41068_ignition_bringup 41068_ignition_combined.launch.py
```

### 2. Launch Navigation with GUI
In a new terminal, launch the navigation stack with the GUI:
```bash
ros2 launch 41068_ignition_bringup 41068_navigation.launch.py
```

Or launch without GUI:
```bash
ros2 launch 41068_ignition_bringup 41068_navigation.launch.py use_gui:=false
```

### 3. Using the Navigation GUI

The GUI provides several ways to set navigation goals:

#### Manual Coordinate Entry
1. Enter X and Y coordinates in meters in the input fields
2. Optionally set a yaw angle (orientation) in radians
3. Click "Send Goal" to navigate to the position

#### Click-to-Navigate
1. Click anywhere on the SLAM map display
2. The robot will automatically navigate to the clicked position

#### Quick Goals
- **Origin (0,0)**: Navigate back to the starting position
- **Forward 5m**: Move 5 meters forward from current position
- **Backward 5m**: Move 5 meters backward from current position

#### Goal Management
- **Cancel Goal**: Stop the current navigation goal
- **Robot Status**: View current robot position and orientation

## Topic Structure

The navigation system uses the following namespaced topics:

### Input Topics (from simulation)
- `/husky/odometry`: Robot odometry data
- `/husky/scan`: Laser scan data for obstacle detection
- `/husky/cmd_vel`: Command velocity (Nav2 output)

### Output Topics (to Nav2)
- `/map`: SLAM-generated map
- `/navigate_to_pose`: Navigation goal action server

### Topic Remapping
The system automatically remaps Nav2's `/cmd_vel` output to `/husky/cmd_vel` to match the robot's expected topic namespace.

## Configuration Files

- `config/nav2_params_husky.yaml`: Nav2 parameters with proper Husky namespacing
- `config/slam_params.yaml`: SLAM configuration for mapping
- `scripts/navigation_gui.py`: Interactive navigation GUI

## Dependencies

The navigation GUI requires the following Python packages:
- `rclpy`: ROS 2 Python client library
- `tkinter`: GUI framework (usually pre-installed)
- `matplotlib`: Map visualization
- `numpy`: Numerical operations
- `PIL` (Pillow): Image processing

Install missing dependencies:
```bash
pip3 install matplotlib pillow numpy
```

## Troubleshooting

### GUI Not Starting
- Ensure you have a display available (X11 forwarding if using SSH)
- Check that tkinter is installed: `python3 -c "import tkinter"`

### Navigation Not Working
- Verify the simulation is running first
- Check that topics are being published: `ros2 topic list | grep husky`
- Ensure the map is being generated: `ros2 topic echo /map --once`

### Robot Not Moving
- Check cmd_vel remapping: `ros2 topic echo /husky/cmd_vel`
- Verify the robot is receiving commands in simulation

### Map Not Displaying
- Wait for SLAM to generate initial map data
- Move the robot around to build the map
- Check map topic: `ros2 topic hz /map`

## Advanced Usage

### Custom Navigation Parameters
Edit `config/nav2_params_husky.yaml` to adjust:
- Robot footprint size
- Navigation speeds and accelerations
- Obstacle avoidance behavior
- Goal tolerance

### Integration with Other Systems
The navigation system can be integrated with:
- Fire detection system (existing)
- Autonomous patrol routes
- Multi-robot coordination
- Custom mission planning

## Example Workflow

1. **Start Simulation**: Launch the combined Gazebo simulation
2. **Start Navigation**: Launch navigation with GUI
3. **Build Map**: Drive the robot around manually to build initial map
4. **Set Goals**: Use GUI to navigate to specific coordinates
5. **Monitor Progress**: Watch robot navigate autonomously while avoiding obstacles

The system provides a complete autonomous navigation solution for the Husky robot in forest environments with real-time SLAM mapping and intuitive goal setting through the GUI interface.

