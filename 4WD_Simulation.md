# 4WD Obstacle-Stopping Robot Simulation (ROS 2)

## Overview
This ROS 2 package simulates a 4-wheeled differential-drive robot in Gazebo Classic. The robot is equipped with:
- A front-mounted camera
- A 360° LIDAR sensor
- An obstacle-stop node that prevents forward motion when an obstacle is within 0.5 meters

## Features
- Differential drive motion using front wheels
- Real-time 360° LIDAR and camera data
- Automatically stops when approaching obstacles
- Fully simulated in Gazebo with a custom world and wall

## Requirements
- ROS 2 Humble
- Gazebo Classic (with `gazebo_ros` plugins)
- `teleop_twist_keyboard` package (for manual control)

## Directory Structure
```
4wd_robot_sim/
├── urdf/
│   └── 4wd_robot.xacro
├── worlds/
│   └── custom_world.sdf
├── launch/
│   └── bringup.launch.py
├── src/
│   └── obstacle_stop.py
└── README.md
```

## How to Run

### 1. Build the workspace
```bash
cd ~/ros2_ws
colcon build --packages-select 4wd_robot_sim
source install/setup.bash
```

### 2. Launch the simulation
```bash
ros2 launch 4wd_robot_sim bringup.launch.py
```

### 3. Run teleop keyboard to move the robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Use `i`, `j`, `l`, `k`, etc. to move.
- The robot will stop automatically ~0.5 meters before a wall or obstacle.

## Notes
- The obstacle_stop node subscribes to `/lidar_scan` and filters forward LIDAR data.
- It publishes to `/cmd_vel` only if the path is clear.
- The world contains a fixed wall for testing stop behavior.

## Author
Sanat — Internship Project at MaRS Club, IIITDM Kancheepuram
