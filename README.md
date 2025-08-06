# AURA Simulation Package

A ROS2 Foxy package for simulating a differential drive mobile robot with lidar and camera sensors, designed as an autonomous library assistant robot.

## Overview

The AURA (Autonomous Ubiquitous Robotic Assistant) robot is a differential drive mobile robot equipped with:
- **Base**: 0.5m x 0.5m x 0.2m rectangular chassis
- **Drive system**: Two drive wheels + one caster wheel
- **Sensors**: RPLiDAR A1-style lidar sensor and camera
- **Environment**: Library simulation world with bookshelves, tables, and obstacles

## Package Structure

```
aura_simulation/
├── aura_simulation/           # Python package directory
│   └── __init__.py
├── config/                    # Configuration files
│   └── aura_view.rviz        # RViz configuration
├── launch/                    # Launch files
│   ├── bringup.launch.py     # Gazebo simulation launch
│   └── view_model.launch.py  # RViz visualization launch
├── resource/                  # Package resources
│   └── aura_simulation       # Package marker file
├── urdf/                      # Robot description files
│   ├── aura_robot.urdf.xacro # Main robot URDF
│   └── materials.xacro       # Material definitions
├── worlds/                    # Gazebo world files
│   └── aura_world.world      # Library environment
├── package.xml               # Package metadata
├── setup.cfg                 # Setup configuration
└── setup.py                  # Package setup script
```

## Prerequisites

- ROS2 Foxy
- Gazebo 11
- Required ROS2 packages:
  - `gazebo_ros_pkgs`
  - `robot_state_publisher`
  - `joint_state_publisher`
  - `joint_state_publisher_gui`
  - `xacro`
  - `rviz2`

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url> aura_simulation
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select aura_simulation
source install/setup.bash
```

## Usage

### 1. Launch Gazebo Simulation

To start the full simulation with Gazebo:
```bash
ros2 launch aura_simulation bringup.launch.py
```

Optional parameters:
- `x_pose:=<float>` - Initial X position (default: 0.0)
- `y_pose:=<float>` - Initial Y position (default: 0.0)
- `z_pose:=<float>` - Initial Z position (default: 0.0)
- `world:=<path>` - Custom world file path

Example with custom position:
```bash
ros2 launch aura_simulation bringup.launch.py x_pose:=1.0 y_pose:=2.0
```

### 2. View Robot Model in RViz

To visualize the robot model without simulation:
```bash
ros2 launch aura_simulation view_model.launch.py
```

Optional parameters:
- `gui:=true/false` - Enable joint state publisher GUI (default: true)
- `use_sim_time:=true/false` - Use simulation time (default: false)

### 3. Control the Robot

Once the simulation is running, you can control the robot using:

```bash
# Teleop keyboard control (if available)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/aura/cmd_vel

# Or publish directly to cmd_vel topic
ros2 topic pub /aura/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.5}}'
```

### 4. Monitor Sensor Data

View the sensor topics:
```bash
# List all topics
ros2 topic list

# View lidar data
ros2 topic echo /aura/scan

# View camera data
ros2 topic hz /aura/camera/image_raw

# View odometry
ros2 topic echo /aura/odom
```

## Robot Specifications

### Physical Properties
- **Base dimensions**: 0.5m × 0.5m × 0.2m
- **Wheel radius**: 0.1m
- **Wheel separation**: 0.4m
- **Total mass**: ~7kg (base: 5kg, wheels: 1kg each, sensors: 0.3kg)

### Sensors
- **Lidar**: 360° range sensor
  - Range: 0.1m - 10.0m
  - Resolution: 0.01m
  - Update rate: 20Hz
  - Topic: `/aura/scan`

- **Camera**: RGB camera
  - Resolution: 800×600
  - FOV: 80° horizontal
  - Update rate: 30Hz
  - Topics: `/aura/camera/image_raw`, `/aura/camera/camera_info`

### Control Interface
- **Command topic**: `/aura/cmd_vel` (geometry_msgs/Twist)
- **Odometry topic**: `/aura/odom` (nav_msgs/Odometry)
- **Joint states**: `/joint_states` (sensor_msgs/JointState)

## Environment

The library world includes:
- Two large bookshelves
- A reading table
- Boundary walls
- Scattered books for navigation testing
- Proper lighting and physics simulation

## Troubleshooting

### Common Issues

1. **"No module named 'xacro'"**
   ```bash
   sudo apt install ros-foxy-xacro
   ```

2. **Gazebo fails to start**
   - Check if Gazebo is properly installed
   - Verify GPU drivers for rendering

3. **Robot not spawning**
   - Check if all URDF files are properly formatted
   - Verify file paths in launch files

4. **No sensor data**
   - Ensure simulation time is synchronized
   - Check topic names and namespaces

### Useful Commands

```bash
# Check if package is properly installed
ros2 pkg list | grep aura_simulation

# Validate URDF
check_urdf /path/to/aura_robot.urdf

# View TF tree
ros2 run rqt_tf_tree rqt_tf_tree

# Monitor system
ros2 run rqt_graph rqt_graph
```

## Contributing

When modifying the robot description:
1. Edit the `.xacro` files in the `urdf/` directory
2. Test changes with `view_model.launch.py` before running full simulation
3. Ensure all joints have proper limits and inertial properties
4. Update this README if adding new features

## License

Apache License 2.0

## Author

Maidah Arsh (maidah@example.com)

---

**Note**: This package is designed for ROS2 Foxy. For other ROS2 distributions, dependency versions may need adjustment.
