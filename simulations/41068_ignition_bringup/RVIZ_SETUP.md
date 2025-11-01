# RViz Setup for Autonomous Firefighter Visualization

## Quick Setup

To see the extinguishing particle effects and firefighter status, add these displays to RViz:

### 1. Fire Extinguishing Effects

**Add MarkerArray Display:**
```
Click "Add" button in RViz
→ By display type
→ rviz/MarkerArray
→ Set Topic: /firefighter/extinguish_effect
```

This shows:
- 50 animated blue water particles spraying toward fire
- Pulsing white cylinder at fire location
- Blue splash effect on ground

### 2. Firefighter Status Text

**Add Marker Display:**
```
Click "Add" button
→ By display type
→ rviz/Marker
→ Set Topic: /firefighter/status
```

Shows current state and fire counts above robot.

### 3. Fire Locations (from Drone)

**Add PoseArray Display:**
```
Click "Add" button
→ By display type
→ rviz/PoseArray
→ Set Topic: /drone/fire_scan/fire_positions
→ Color: Red (255, 0, 0)
→ Arrow Length: 1.0
```

Shows all detected fire positions.

## Complete RViz Config

Save this to a `.rviz` file:

```yaml
Panels:
  - Class: rviz/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Alpha: 0.7
      Class: rviz/Map
      Name: Map
      Topic: /map

    - Class: rviz/TF
      Name: TF

    - Alpha: 1
      Autocompute Value Bounds: true
      Class: rviz/LaserScan
      Name: LaserScan
      Topic: /husky/scan

    - Class: rviz/MarkerArray
      Name: Extinguish Effect
      Topic: /firefighter/extinguish_effect

    - Class: rviz/Marker
      Name: Firefighter Status
      Topic: /firefighter/status

    - Alpha: 1
      Axes Length: 1
      Class: rviz/PoseArray
      Color: 255; 0; 0
      Name: Fire Locations
      Topic: /drone/fire_scan/fire_positions

  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
```

## Troubleshooting Visualization

### Particles Not Showing?

1. **Check topic is publishing:**
   ```bash
   ros2 topic hz /firefighter/extinguish_effect
   ```
   Should show ~10 Hz when extinguishing

2. **Check RViz fixed frame:**
   - Global Options → Fixed Frame should be `map`

3. **Check markers are being received:**
   ```bash
   ros2 topic echo /firefighter/extinguish_effect --no-arr
   ```

### Animation Looks Static?

- The particles update every 0.1s (state machine rate)
- Pulsing effect is time-based and should animate
- Try zooming closer to see the spray arc

### Status Not Showing?

```bash
# Check status topic
ros2 topic echo /firefighter/status

# Make sure robot odometry is working
ros2 topic echo /husky/odometry
```

## Expected Visual Effects

### During EXTINGUISHING State:

1. **Water Spray:** 50 blue spheres arcing from robot to fire
2. **Fire Suppression:** Pulsing white cylinder at fire (2m tall)
3. **Ground Splash:** Blue circle on ground around fire
4. **Status Text:** Green text above robot saying "State: EXTINGUISHING"

### Animation Details:

- Particles: Bright blue (r=0.1, g=0.6, b=1.0), 0.15m diameter
- Spray arc: Follows sine curve from robot to fire
- Update rate: 10 Hz (smooth animation)
- Lifetime: 0.5 seconds per particle
- Pulsing frequency: 5 Hz

## Performance Notes

- 50 particles × 10 Hz = 500 markers/second during extinguishing
- Status updates every 2 seconds
- All markers auto-expire (no manual cleanup needed)
- No performance impact when not extinguishing

## Topics Reference

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/firefighter/extinguish_effect` | MarkerArray | 10 Hz* | Water spray particles |
| `/firefighter/status` | Marker | 0.5 Hz | Status text |
| `/drone/fire_scan/fire_positions` | PoseArray | 1 Hz | Detected fires |

*Only publishes during EXTINGUISHING state

## Test Without Fires

To test visualization without running the drone:

```bash
# Terminal 1: Launch firefighter
ros2 launch 41068_ignition_bringup autonomous_firefighter.launch.py auto_start:=false

# Terminal 2: Publish fake fire
ros2 topic pub /drone/fire_scan/fire_positions geometry_msgs/msg/PoseArray "
header:
  frame_id: 'map'
poses:
- position: {x: 3.0, y: 0.0, z: 0.0}
  orientation: {w: 1.0}
"

# Terminal 3: Trigger firefighting
ros2 param set /autonomous_firefighter auto_start true
```

You should see the robot navigate to (3, 0) and start extinguishing with full particle effects!
