# NumPy Compatibility Fix for Camera Viewer

The visual camera viewer has a NumPy 2.x compatibility issue with OpenCV and cv_bridge. Here are several solutions:

## 🔧 Quick Solution: Use Simple Monitor

Use the simple camera monitor that doesn't require OpenCV:

```bash
cd /home/odyssey/ros2_ws
source install/setup.bash
python3 src/41068_Robotics_Studio_1/simulations/firefighting_system/src/drone_camera_simple.py
```

This will show camera status, frame counts, and FPS without visual display.

## 🛠️ Fix NumPy Issue (Choose One Method)

### Method 1: Downgrade NumPy (Recommended)

```bash
pip install "numpy<2.0"
```

### Method 2: Use Conda Environment

```bash
# Create new conda environment
conda create -n ros2_vision python=3.10 numpy=1.24
conda activate ros2_vision

# Install required packages
pip install opencv-python

# Then run the camera viewer
python3 src/41068_Robotics_Studio_1/simulations/firefighting_system/src/drone_camera_viewer.py
```

### Method 3: Virtual Environment with Specific NumPy

```bash
# Create virtual environment
python3 -m venv ~/camera_env
source ~/camera_env/bin/activate

# Install compatible versions
pip install "numpy<2.0" opencv-python

# Run camera viewer
cd /home/odyssey/ros2_ws
source install/setup.bash
python3 src/41068_Robotics_Studio_1/simulations/firefighting_system/src/drone_camera_viewer.py
```

### Method 4: System Package Manager

```bash
# Remove pip-installed numpy and opencv
pip uninstall numpy opencv-python

# Install system packages
sudo apt update
sudo apt install python3-opencv python3-numpy

# Try camera viewer again
python3 src/41068_Robotics_Studio_1/simulations/firefighting_system/src/drone_camera_viewer.py
```

## 📺 Alternative: Use ROS2 Image Tools

You can also view camera feeds using built-in ROS2 tools:

### View RGB Camera:
```bash
ros2 run rqt_image_view rqt_image_view /drone/rgb_camera/image
```

### View Thermal Camera:
```bash
ros2 run rqt_image_view rqt_image_view /drone/thermal_camera/image
```

### Save Images:
```bash
# Save single frame
ros2 service call /drone/rgb_camera/save_image sensor_msgs/srv/SetString "{data: 'rgb_frame.jpg'}"

# Or use command line tool
ros2 run image_tools showimage --ros-args --remap image:=/drone/rgb_camera/image
```

## 🎯 Test Camera System

After simulation starts, test camera connectivity:

```bash
# Check available camera topics
ros2 topic list | grep camera

# Test RGB camera data
ros2 topic echo /drone/rgb_camera/image --max-count 1

# Test thermal camera data  
ros2 topic echo /drone/thermal_camera/image --max-count 1

# Check frame rates
ros2 topic hz /drone/rgb_camera/image
ros2 topic hz /drone/thermal_camera/image
```

## 🚁 Camera Usage Instructions

1. **Start Simulation:**
   ```bash
   ./scripts/launch_integrated_sim.sh
   ```

2. **Monitor Cameras (Simple):**
   ```bash
   python3 src/drone_camera_simple.py
   ```

3. **Control Drone:**
   ```bash
   python3 src/drone_teleop_ros.py
   ```

4. **View Images (if NumPy fixed):**
   ```bash
   python3 src/drone_camera_viewer.py
   ```

## 🔍 Troubleshooting

### Camera Not Found
- Ensure simulation is running
- Check bridge is working: `ros2 node list | grep bridge`
- Verify topics exist: `ros2 topic list | grep camera`

### Low Frame Rate
- Normal for thermal camera (10fps max)
- RGB should be ~30fps
- System performance may affect rates

### No Image Data
- Check drone is spawned correctly in simulation
- Verify camera sensors are enabled
- Look for errors in simulation console

## 📊 Expected Camera Performance

### RGB Camera
- **Resolution**: 640x480
- **Frame Rate**: ~30fps
- **Format**: RGB8 (color)
- **Use Case**: Navigation, visual inspection

### Thermal Camera  
- **Resolution**: 320x240
- **Frame Rate**: ~10fps  
- **Format**: MONO8 (grayscale, thermal-processed)
- **Use Case**: Heat detection, fire identification

Both cameras face downward for ground surveillance and fire detection.
