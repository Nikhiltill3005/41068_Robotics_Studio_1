# Drone Camera System

The firefighting drone is now equipped with dual downward-facing cameras for enhanced surveillance and fire detection capabilities.

## 📹 Camera Configuration

### RGB Camera (Blue housing)
- **Position**: Front-center of drone, facing downward
- **Resolution**: 640x480 pixels
- **Frame Rate**: 30 FPS
- **Field of View**: 60° (1.047 radians)
- **Purpose**: Visual inspection, navigation, general surveillance

### Thermal/IR Camera (Red housing)
- **Position**: Back-center of drone, facing downward  
- **Resolution**: 320x240 pixels
- **Frame Rate**: 10 FPS
- **Field of View**: 60° (1.047 radians)
- **Purpose**: Heat detection, fire identification, hot spot monitoring

## 🚁 Physical Layout

```
     [Rotor]         [Rotor]
        |               |
        +---------------+
        |  [RGB]   [IR] |  <- Drone Body
        |   📷     🔥   |
        +---------------+
        |               |
     [Rotor]         [Rotor]
        
        ⬇️              ⬇️
    Visual Feed    Heat Detection
```

## 📡 ROS2 Topics

### Camera Image Streams
- **RGB Camera**: `/drone/rgb_camera/image` (sensor_msgs/Image)
- **Thermal Camera**: `/drone/thermal_camera/image` (sensor_msgs/Image)

### Usage Examples

**View RGB camera feed:**
```bash
ros2 topic echo /drone/rgb_camera/image
```

**View thermal camera feed:**
```bash
ros2 topic echo /drone/thermal_camera/image
```

**Save camera image:**
```bash
ros2 run image_tools showimage --ros-args --remap image:=/drone/rgb_camera/image
```

## 🖥️ Camera Viewer Application

### Launch Camera Viewer
```bash
# Terminal 1: Start simulation
cd /home/odyssey/ros2_ws/src/41068_Robotics_Studio_1/simulations/firefighting_system
./scripts/launch_integrated_sim.sh

# Terminal 2: Start camera viewer
cd /home/odyssey/ros2_ws
source install/setup.bash
python3 src/41068_Robotics_Studio_1/simulations/firefighting_system/src/drone_camera_viewer.py
```

### Camera Viewer Controls
- **'q'**: Quit viewer
- **'s'**: Save current frames to files
- **'r'**: Toggle RGB camera display on/off
- **'t'**: Toggle thermal camera display on/off

### Saved Files
Images are saved with timestamps:
- `drone_rgb_YYYYMMDD_HHMMSS.jpg` - RGB camera frame
- `drone_thermal_YYYYMMDD_HHMMSS.jpg` - Thermal camera frame

## 🔥 Firefighting Applications

### Heat Detection
The thermal camera uses a color-coded heat map:
- **🔴 Red/Yellow**: Hot areas (potential fires)
- **🟡 Orange**: Warm areas (heat sources)
- **🔵 Blue/Purple**: Cool areas (safe zones)

### Surveillance Modes
1. **Search Mode**: Use RGB camera for visual navigation and area scanning
2. **Fire Detection**: Use thermal camera to identify heat signatures
3. **Dual Mode**: Monitor both feeds simultaneously for comprehensive situational awareness

## 🛠️ Technical Specifications

### Camera Models
- Both cameras are physically modeled as small boxes attached to the drone
- RGB camera: Blue housing (0.05×0.05×0.03m, 50g)
- Thermal camera: Red housing (0.05×0.05×0.03m, 50g)

### Sensor Characteristics
- **RGB**: Full color (R8G8B8 format) with gaussian noise (σ=0.007)
- **Thermal**: Grayscale (L8 format) with gaussian noise (σ=0.005)
- **Range**: 0.1m to 100m for both cameras

### Performance Impact
- Total camera mass: 100g (minimal impact on flight performance)
- Processing: Optimized for real-time streaming
- Network: Moderate bandwidth usage for image transmission

## 📊 Integration with Other Systems

### With Teleop Control
While controlling the drone, camera feeds provide:
- Ground reference for altitude estimation
- Obstacle detection for safe navigation
- Target identification for mission planning

### With Autonomous Navigation
Camera data can be used for:
- Visual odometry and SLAM
- Obstacle avoidance
- Fire detection and tracking
- Landing site identification

### With Data Logging
Camera feeds can be:
- Recorded for mission analysis
- Processed for fire mapping
- Used for emergency response documentation

## 🎯 Mission Scenarios

### Fire Surveillance Mission
1. **Launch drone** with integrated simulation
2. **Navigate to area** using teleop controls
3. **Monitor thermal feed** for heat signatures
4. **Investigate hotspots** using RGB camera for detail
5. **Save evidence** using camera viewer

### Search and Rescue
1. **Scan area** with RGB camera for visual contacts
2. **Check heat signatures** with thermal camera for survivors
3. **Document findings** with saved image frames
4. **Guide ground units** using real-time feeds

### Damage Assessment
1. **Survey affected area** from safe altitude
2. **Identify structural damage** with RGB camera
3. **Locate remaining heat sources** with thermal camera
4. **Create assessment report** from saved images

## 🔧 Troubleshooting

### No Camera Feed
1. Check simulation is running: `ros2 topic list | grep camera`
2. Verify bridge connection: `ros2 topic echo /drone/rgb_camera/image --max-count 1`
3. Restart camera viewer if needed

### Poor Image Quality
1. Check drone altitude (optimal: 2-10m for ground inspection)
2. Verify lighting conditions for RGB camera
3. Ensure thermal camera has temperature differentials to detect

### Performance Issues
1. Close unnecessary applications
2. Reduce camera frame rates if needed
3. Use single camera mode instead of dual mode

## 📈 Future Enhancements

### Potential Upgrades
- **Gimbal mount**: Stabilized camera positioning
- **Zoom capability**: Variable focal length
- **Multi-spectral**: Additional wavelength detection
- **AI processing**: Automated fire/object detection
- **Streaming**: Direct video streaming to ground station

### Advanced Features
- **Automatic fire detection**: Computer vision algorithms
- **Target tracking**: Follow specific objects or heat sources
- **Mapping integration**: Overlay camera data on SLAM maps
- **Alert system**: Automatic notifications for detected fires
