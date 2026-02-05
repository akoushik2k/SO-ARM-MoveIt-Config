# SO-ARM 101 MoveIt Configuration

ROS 2 MoveIt configuration for the SO-ARM 101 robotic arm with LeRobot naming convention.

## Overview

This repository contains the MoveIt configuration and URDF description for the SO-ARM 101 6-DOF robotic arm, compatible with LeRobot and Isaac Sim.

## Packages

### `so_arm_description`
URDF robot description with LeRobot-compatible joint naming:
- **Rotation** (motor1) - Base rotation
- **Pitch** (motor2) - Shoulder pitch  
- **Elbow** (motor3) - Elbow flexion
- **Wrist_Pitch** (motor4) - Wrist pitch
- **Wrist_Roll** (motor5) - Wrist roll
- **Gripper** (motor6) - Gripper open/close

### `so_arm_moveit_config`
MoveIt 2 configuration package including:
- Motion planning configuration
- ROS 2 Control integration
- RViz visualization setup
- Joint limits and controller configuration

## Requirements

- ROS 2 Humble
- MoveIt 2
- Isaac Sim (optional, for simulation)

## Installation

```bash
# Clone the repository
git clone git@github.com:akoushik2k/SO-ARM-MoveIt-Config.git
cd SO_ARM_Project_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### Launch MoveIt with RViz

```bash
ros2 launch so_arm_moveit_config demo.launch.py
```

### Launch with Isaac Sim

1. Start Isaac Sim
2. Load your SO-ARM robot model
3. Run the ROS 2 bridge and controllers

## Joint Names (LeRobot Convention)

This configuration uses the LeRobot naming convention for compatibility:

| Joint Name | Description | Motor |
|------------|-------------|-------|
| Rotation | Base rotation | motor1 |
| Pitch | Shoulder pitch | motor2 |
| Elbow | Elbow flexion | motor3 |
| Wrist_Pitch | Wrist pitch | motor4 |
| Wrist_Roll | Wrist roll | motor5 |
| Gripper | Gripper control | motor6 |

## Configuration Files

- `config/moveit_controllers.yaml` - Controller configuration
- `config/joint_limits.yaml` - Joint limits and dynamics
- `config/so101_new_calib.ros2_control.xacro` - ROS 2 Control hardware interface

## References

- [LeRobot](https://github.com/huggingface/lerobot)
- [SO-ARM 101 Reference](https://github.com/MuammerBay/SO-ARM101_MoveIt_IsaacSim)



## Contributing

Contributions are welcome! Please open an issue or submit a pull request.
