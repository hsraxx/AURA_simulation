# AURA Simulation Package

A ROS2 Foxy package for simulating the AURA (Autonomous Library Assistant) robot, a differential drive mobile robot equipped with a lidar sensor and camera.

## Overview

The AURA robot is designed as an autonomous library assistant with the following specifications:
- **Base**: 0.5m x 0.5m x 0.2m box-shaped body
- **Drive System**: Differential drive with 2 drive wheels and 1 caster wheel
- **Sensors**: 
  - RPLiDAR A1-style lidar sensor (360° scanning)
  - Front-facing camera
- **Control**: ROS2 Control with differential drive controller

## Package Structure

```
aura_simulation/
├── config/
│   ├── aura_controllers.yaml    # ROS2 Control configuration
│   └── aura_rviz.rviz          # RViz2 configuration
├── launch/
│   ├── bringup.launch.py        # Launch Gazebo with robot
│   └── view_model.launch.py     # Launch RViz2 for model viewing
├── urdf/
│   ├── aura_robot.urdf.xacro    # Main robot description
│   └── materials.xacro          # Material definitions
├── worlds/
│   └── aura_world.world         # Simple test world
├── package.xml                  # Package dependencies
└── setup.py                     # Package setup
```

## Dependencies

### ROS2 Packages
- `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`
- `tf2_ros`, `robot_state_publisher`, `joint_state_publisher`
- `rviz2`, `xacro`, `urdf`, `urdfdom`
- `gazebo_ros`, `gazebo_ros2_control`
- `ros2_control`, `ros2_controllers`, `diff_drive_controller`

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url> aura_simulation
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select aura_simulation
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Launch Gazebo Simulation

To start the complete simulation with Gazebo:

```bash
ros2 launch aura_simulation bringup.launch.py
```

This will:
- Launch Gazebo with the AURA world
- Spawn the AURA robot
- Start the differential drive controller
- Optionally launch RViz2 (use `rviz:=true` argument)

### View Robot Model in RViz2

To view the robot model without simulation:

```bash
ros2 launch aura_simulation view_model.launch.py
```

This will:
- Launch RViz2 with robot visualization
- Start robot_state_publisher and joint_state_publisher
- Show TF tree and robot model

### Control the Robot

Once the simulation is running, you can control the robot using:

```bash
# Publish velocity commands
ros2 topic pub /aura/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# View sensor data
ros2 topic echo /aura/scan          # Lidar data
ros2 topic echo /aura/camera/image_raw  # Camera images
ros2 topic echo /aura/odom          # Odometry data
```

## Robot Description

### Physical Specifications
- **Base Dimensions**: 0.5m × 0.5m × 0.2m
- **Wheel Separation**: 0.4m
- **Wheel Radius**: 0.06m
- **Wheel Width**: 0.04m
- **Caster Wheel**: 0.02m radius sphere

### Sensor Specifications
- **Lidar**: 360° scanning, 12m range, 20Hz update rate
- **Camera**: 640×480 resolution, 30Hz update rate

### Control Parameters
- **Max Linear Velocity**: 1.0 m/s
- **Max Angular Velocity**: 2.0 rad/s
- **Controller Update Rate**: 50Hz

## Topics

### Subscribed Topics
- `/aura/cmd_vel` (geometry_msgs/Twist): Velocity commands

### Published Topics
- `/aura/scan` (sensor_msgs/LaserScan): Lidar data
- `/aura/camera/image_raw` (sensor_msgs/Image): Camera images
- `/aura/camera/camera_info` (sensor_msgs/CameraInfo): Camera parameters
- `/aura/odom` (nav_msgs/Odometry): Robot odometry
- `/joint_states` (sensor_msgs/JointState): Joint states

### TF Frames
- `odom` → `base_link`
- `base_link` → `left_wheel`
- `base_link` → `right_wheel`
- `base_link` → `caster_wheel`
- `base_link` → `lidar_link`
- `base_link` → `camera_link`

## Configuration

### Controller Configuration
The differential drive controller is configured in `config/aura_controllers.yaml` with:
- PID gains for wheel velocity control
- Velocity and acceleration limits
- Odometry publishing settings

### RViz Configuration
The RViz2 configuration in `config/aura_rviz.rviz` includes:
- Robot model visualization
- TF frame display
- Lidar scan visualization
- Camera image display

## Troubleshooting

### Common Issues

1. **Gazebo not launching**: Ensure Gazebo is installed and the world file path is correct
2. **Robot not spawning**: Check that the URDF file is valid and all dependencies are installed
3. **Controller errors**: Verify that `ros2_control` and `ros2_controllers` are properly installed
4. **RViz not showing robot**: Ensure `robot_state_publisher` is running and TF frames are published

### Debug Commands

```bash
# Check if topics are published
ros2 topic list
ros2 topic info /aura/cmd_vel

# Check TF frames
ros2 run tf2_tools view_frames

# Validate URDF
check_urdf aura_robot.urdf.xacro

# Check controller status
ros2 control list_controllers
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This package is licensed under the Apache License 2.0.

## Authors

- Maidah Arsh (maidah@example.com)

## Acknowledgments

This package is designed for educational and research purposes in autonomous robotics and library automation systems.
