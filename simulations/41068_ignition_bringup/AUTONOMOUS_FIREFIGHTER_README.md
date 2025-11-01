# Autonomous Firefighter System

## Overview

The Autonomous Firefighter system enables a Husky robot to autonomously detect, navigate to, and extinguish fires using Nav2 navigation and SLAM. The system uses a state machine architecture to efficiently manage fire extinguishing operations.

## System Architecture

### Components

1. **Fire Detection (Drone)**
   - `fire_centering_node.py`: Overhead drone scans for fires using thermal camera
   - Publishes fire locations to `/drone/fire_scan/fire_positions` (PoseArray)
   - Continuously updates fire list as new fires are discovered

2. **Autonomous Navigation (Husky)**
   - `autonomous_firefighter_node.py`: Main firefighting controller
   - Subscribes to fire locations and robot odometry
   - Uses Nav2 for path planning and obstacle avoidance
   - Implements state machine for fire prioritization

3. **Visualization**
   - Real-time particle effects during extinguishing
   - Status markers showing robot state
   - RViz markers for fire locations and extinguishing effects

## State Machine

The firefighter operates in the following states:

```
IDLE → PLANNING → NAVIGATING → APPROACHING → EXTINGUISHING → COMPLETED
  ↑                                                               ↓
  └───────────────────────────────────────────────────────────────┘
```

### State Descriptions

- **IDLE**: Waiting for fires to be detected by the drone
- **PLANNING**: Sorting fires by distance and creating visitation order
- **NAVIGATING**: Moving towards the fire using Nav2 (to ~1.5m distance)
- **APPROACHING**: Fine positioning at exactly target distance from fire
- **EXTINGUISHING**: Applying extinguishing action with particle effects
- **COMPLETED**: All known fires handled; monitoring for new fires

## Features

### Smart Fire Management
- **Dynamic Fire List**: Handles fires that appear over time as drone discovers them
- **Priority Sorting**: Visits closest fires first for efficiency
- **Duplicate Prevention**: Avoids re-extinguishing the same fire
- **Obstacle Avoidance**: Uses Nav2 for safe navigation

### Autonomous Operation
- **No Manual Input Required**: Fully autonomous from fire detection to extinguishing
- **Continuous Operation**: Returns to work when new fires are detected
- **Robust Error Handling**: Skips unreachable fires and continues

### Visualization
- **Particle Effects**: Blue water spray animation during extinguishing
- **Status Display**: Real-time text showing current state and fire count
- **Fire Suppression Marker**: Visual feedback at fire location

## Usage

### Launch the Autonomous Firefighter

```bash
# Basic launch (default parameters)
ros2 launch 41068_ignition_bringup autonomous_firefighter.launch.py

# Custom parameters
ros2 launch 41068_ignition_bringup autonomous_firefighter.launch.py \
    target_distance:=2.0 \
    extinguish_duration:=5.0 \
    auto_start:=true
```

### Run Standalone Node

```bash
ros2 run 41068_ignition_bringup autonomous_firefighter_node.py
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_distance` | 1.5 | Target distance from fire (meters) |
| `distance_tolerance` | 0.3 | Positioning tolerance (meters) |
| `extinguish_duration` | 3.0 | Duration to extinguish each fire (seconds) |
| `auto_start` | true | Auto-start when fires detected |
| `min_wait_time` | 5.0 | Wait time in IDLE before checking again |

### Example Configurations

**Cautious Approach (Further Distance)**
```bash
ros2 launch 41068_ignition_bringup autonomous_firefighter.launch.py \
    target_distance:=2.5 \
    distance_tolerance:=0.5 \
    extinguish_duration:=5.0
```

**Quick Response (Closer, Faster)**
```bash
ros2 launch 41068_ignition_bringup autonomous_firefighter.launch.py \
    target_distance:=1.0 \
    distance_tolerance:=0.2 \
    extinguish_duration:=2.0
```

## Topics

### Subscribed Topics
- `/drone/fire_scan/fire_positions` (geometry_msgs/PoseArray)
  - Fire locations detected by overhead drone
- `/husky/odometry` (nav_msgs/Odometry)
  - Robot position and orientation

### Published Topics
- `/firefighter/extinguish_effect` (visualization_msgs/MarkerArray)
  - Particle effects for extinguishing visualization
- `/firefighter/status` (visualization_msgs/Marker)
  - Text marker showing current state and statistics

### Action Clients
- `/navigate_to_pose` (nav2_msgs/action/NavigateToPose)
  - Nav2 navigation for autonomous movement

## Integration with Existing System

The autonomous firefighter integrates seamlessly with your existing setup:

1. **Works with SLAM**: Uses the map created by your SLAM system
2. **Uses Nav2**: Leverages your existing Nav2 configuration
3. **Compatible with GUI**: Can run alongside `navigation_gui.py` for monitoring
4. **Independent Node**: Separate from drone control - clean architecture

## Workflow Example

1. **Drone Scanning Phase**
   ```
   Drone flies overhead → Detects fires → Publishes to /fire_positions
   ```

2. **Husky Response**
   ```
   Firefighter receives fires → Plans route → Navigates to closest fire
   ```

3. **Fire Extinguishing**
   ```
   Approaches to 1.5m → Extinguishes (3s) → Moves to next fire
   ```

4. **Continuous Operation**
   ```
   Completes all fires → Waits for new detections → Resumes when found
   ```

## Visualization in RViz

To visualize the firefighter in action:

1. Add MarkerArray display for `/firefighter/extinguish_effect`
2. Add Marker display for `/firefighter/status`
3. Add PoseArray display for `/drone/fire_scan/fire_positions`

**Recommended RViz Config:**
```yaml
- Class: rviz/MarkerArray
  Name: Extinguish Effect
  Topic: /firefighter/extinguish_effect

- Class: rviz/Marker
  Name: Firefighter Status
  Topic: /firefighter/status

- Class: rviz/PoseArray
  Name: Fire Locations
  Topic: /drone/fire_scan/fire_positions
  Color: 255; 0; 0
```

## Testing

### Test Without Drone

You can test the firefighter by manually publishing fire locations:

```bash
# Publish a test fire at (5.0, 5.0)
ros2 topic pub /drone/fire_scan/fire_positions geometry_msgs/msg/PoseArray "
header:
  frame_id: 'map'
poses:
- position:
    x: 5.0
    y: 5.0
    z: 0.0
  orientation:
    w: 1.0
"
```

### Monitor State Transitions

```bash
# Watch the logs
ros2 node logger set autonomous_firefighter INFO

# View all topics
ros2 topic list | grep firefighter
```

## Troubleshooting

### Robot Not Moving
- Check Nav2 is running: `ros2 action list | grep navigate_to_pose`
- Verify robot odometry: `ros2 topic echo /husky/odometry`
- Check if fires are being received: `ros2 topic echo /drone/fire_scan/fire_positions`

### Navigation Failures
- Check SLAM map quality
- Verify costmap configuration
- Ensure target distance is reachable (not inside obstacles)

### Fires Not Detected
- Verify drone is running: `ros2 node list | grep fire`
- Check thermal camera: `ros2 topic hz /drone/ir_camera/image_raw`
- Verify fire centering node is publishing: `ros2 topic hz /drone/fire_scan/fire_positions`

## Advanced Usage

### Programmatic Control

```python
from std_msgs.msg import Bool

# Disable auto-start and manually trigger
# Set auto_start:=false in launch
# Then publish to control:
ros2 topic pub /firefighter/start std_msgs/Bool "data: true"
```

### Custom State Machine Behavior

Modify [autonomous_firefighter_node.py](scripts/autonomous_firefighter_node.py) to customize:
- Fire prioritization algorithm (currently: closest first)
- Approach strategy (currently: straight line)
- Extinguishing behavior
- State transition logic

## Performance Considerations

- **CPU Usage**: Minimal - only state machine logic, no image processing
- **Network**: Lightweight - receives PoseArray, sends goals
- **Latency**: ~100ms state machine update rate
- **Scalability**: Handles dozens of fires efficiently

## Future Enhancements

Possible improvements:
- [ ] Multi-robot coordination
- [ ] Dynamic re-planning during navigation
- [ ] Fire intensity-based prioritization
- [ ] Water/extinguisher capacity management
- [ ] Return to base for resupply
- [ ] Path optimization (TSP solver)

## Files Created

- `scripts/autonomous_firefighter_node.py` - Main firefighter node
- `launch/autonomous_firefighter.launch.py` - Launch file with parameters
- `AUTONOMOUS_FIREFIGHTER_README.md` - This documentation

## Dependencies

- ROS 2 (tested on Humble)
- Nav2
- geometry_msgs
- nav_msgs
- visualization_msgs
- nav2_msgs

## Author

Created for the 41068 Robotics Studio 1 project.

## License

[Your license here]
