# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

UiAbot (University of Agder Bot) is a ROS2-based autonomous mobile robot platform for research and education. It's a differential-drive robot with capabilities for teleoperation, SLAM, and autonomous navigation.

## Build Commands

```bash
# Build the package
cd /home/danielh/uiabot_ws
colcon build --packages-select uiabot

# Clean build
rm -rf build/uiabot install/uiabot
colcon build --packages-select uiabot

# Build with compile commands for IDE
colcon build --packages-select uiabot --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Running the System

Always source the workspace first:
```bash
source /home/danielh/uiabot_ws/install/setup.bash
```

Launch configurations:
- `ros2 launch uiabot teleop_motion_control.launch.py` - Basic motion control
- `ros2 launch uiabot teleop_perception.launch.py` - Motion + all sensors
- `ros2 launch uiabot slam_navigation.launch.py` - Full autonomous system with SLAM
- `ros2 launch uiabot localization_navigation.launch.py` - Navigation with known map

## Testing Commands

```bash
# Run tests (currently only linting)
colcon test --packages-select uiabot
colcon test-result --all
```

## Architecture

The system consists of three main nodes:

1. **control** (src/control.cpp): Converts cmd_vel to motor commands
   - Interfaces with ODrive controllers
   - Differential drive kinematics
   - Parameters: base_width=0.185m, wheel_radius=0.05m, gear_ratio=20:1

2. **mechanical_odometry** (src/mechanical_odometry.cpp): Computes wheel odometry
   - Publishes odometry and joint states
   - Updates at 100Hz
   - Base frame: "base_footprint", odom frame: "odom"

3. **imu_tf_viz** (src/imu_tf_viz.cpp): IMU visualization helper

## Key Dependencies

Hardware interfaces:
- `odrive_ros2` - Motor control via ODrive
- `bno055_i2c_ros2` - BNO055 IMU driver
- `rplidar_ros` - RPLidar driver

Navigation stack:
- `robot_localization` - EKF sensor fusion (params/ekf_params.yaml)
- `slam_toolbox` - SLAM mapping
- `nav2` - Autonomous navigation (params/nav2_params.yaml)

## Important Configuration

- **EKF** (params/ekf_params.yaml): Fuses wheel odometry velocities + IMU yaw/angular velocity
- **Nav2** (params/nav2_params.yaml): AMCL localization, DWB controller
- **URDF** (urdf/): Robot model for visualization and transforms
- **Maps** (map/): Pre-built maps for localization mode

## Development Notes

- C++17 standard required
- Compiler warnings enabled (-Wall -Wextra -Wpedantic)
- No unit tests currently implemented
- External documentation: https://drdanielh.github.io/UiAbot