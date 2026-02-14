# SO-ARM Behavior Tree Package

ROS 2 behavior tree system for autonomous manipulation with vision-based object detection and grasping.

## Overview

This package implements a **constrained BT DSL (Domain-Specific Language)** for robotic manipulation tasks. It integrates:

- **MoveIt 2** for motion planning
- **Vision processing** for object detection and grasp pose estimation
- **BehaviorTree.CPP** for task orchestration
- **Isaac Sim** compatible camera integration

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    BT Executor Node                          │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  BT DSL Actions:                                     │   │
│  │  • DetectObjects    • MoveToPose                     │   │
│  │  • ComputeGraspPose • AttachObject/DetachObject      │   │
│  │  • GoHome           • ClearOctomap                   │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│              Vision Processor Node                           │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  • Object Detection (YOLO)                           │   │
│  │  • Grasp Pose Estimation                             │   │
│  │  • Services: detect_objects, compute_grasp_pose      │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            ↓
                    Camera (Isaac Sim)
```

## Packages

### `so_arm_bt`
Main behavior tree package with:
- BT executor node
- Vision processor node
- Custom BT action and condition nodes
- Example behavior trees

### `so_arm_bt_interfaces`
Custom ROS 2 service definitions:
- `DetectObjects.srv` - Object detection service
- `ComputeGraspPose.srv` - Grasp pose computation service

## BT DSL Reference

### Action Nodes

| Node | Parameters | Description |
|------|------------|-------------|
| `DetectObjects` | `object_class`, `min_confidence`, `output_key` | Detect objects via camera |
| `ComputeGraspPose` | `object_id`, `object_pose`, `grasp_output_key` | Compute grasp pose |
| `MoveToPose` | `target_pose` | Move to named or Cartesian pose |
| `OpenGripper` | - | Open gripper |
| `CloseGripper` | - | Close gripper |
| `AttachObject` | `object_id`, `link_name` | Attach object to planning scene |
| `DetachObject` | `object_id` | Detach object from planning scene |
| `GoHome` | `home_position` | Return to home position |
| `ClearOctomap` | - | Clear octomap collision data |

### Control Flow (BehaviorTree.CPP)

- `Sequence` - Execute children in order
- `Fallback` - Try children until success
- `RetryUntilSuccessful` - Retry with attempts limit
- `Parallel` - Execute concurrently

## Installation

### Dependencies

```bash
# Install ROS 2 vision packages
sudo apt install ros-humble-vision-msgs \
                 ros-humble-cv-bridge \
                 ros-humble-image-transport

# Install OpenCV and Eigen
sudo apt install libopencv-dev libeigen3-dev

# Optional: YOLO ROS 2 package (or use custom ONNX model)
# git clone https://github.com/mgonzs13/yolov8_ros.git
```

### Build

```bash
cd ~/SO_ARM_Project_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select so_arm_bt_interfaces so_arm_bt

# Source
source install/setup.bash
```

## Usage

### 1. Launch Vision Processor Node

```bash
ros2 run so_arm_bt vision_processor_node \
  --ros-args --params-file src/so_arm_bt/config/vision_config.yaml
```

### 2. Run Behavior Tree

```bash
ros2 run so_arm_bt bt_executor_node \
  --ros-args -p bt_xml:=src/so_arm_bt/behavior_trees/autonomous_pick_place.xml
```

### 3. With Isaac Sim

1. Start Isaac Sim with SO-ARM robot
2. Enable camera sensor (RGB-D recommended)
3. Launch ROS 2 bridge
4. Run vision and BT nodes as above

## Example Behavior Trees

### Simple Pick and Place

```xml
<Sequence>
    <DetectObjects object_class="cup" output_key="objects" />
    <ComputeGraspPose object_id="{objects[0]}" grasp_output_key="grasp" />
    <OpenGripper />
    <MoveToPose target_pose="{grasp}" />
    <CloseGripper />
    <AttachObject object_id="{objects[0]}" />
    <MoveToPose target_pose="PlacePosition" />
    <OpenGripper />
    <DetachObject object_id="{objects[0]}" />
    <GoHome />
</Sequence>
```

### With Recovery Logic

```xml
<Fallback>
    <Sequence name="normal_pick">
        <DetectObjects object_class="cube" output_key="objs" />
        <ComputeGraspPose object_id="{objs[0]}" grasp_output_key="grasp" />
        <MoveToPose target_pose="{grasp}" />
        <CloseGripper />
    </Sequence>
    <Sequence name="recovery">
        <ClearOctomap />
        <GoHome />
    </Sequence>
</Fallback>
```

## Configuration

### Vision Config (`config/vision_config.yaml`)

```yaml
vision_processor:
  ros__parameters:
    camera_topic: "/camera/color/image_raw"
    depth_topic: "/camera/depth/image_raw"
    detection_model: "yolov8n.onnx"
    confidence_threshold: 0.5
    grasp_method: "geometric"
```

## Development Status

### Phase 1 ✅ (Complete)
- BT executor with MoveIt integration
- Basic action nodes (MoveToPose, Gripper, CheckPlanningScene)
- XML-based behavior tree loading

### Phase 2 🚧 (In Progress)
- ✅ Vision processor infrastructure
- ✅ Object detection integration (YOLO)
- ✅ Grasp pose estimation
- ✅ BT DSL action nodes
- ✅ Example behavior trees
- ⏳ Object detector implementation (YOLO inference)
- ⏳ Grasp estimator implementation
- ⏳ End-to-end testing

### Phase 3 (Future)
- LLM-based task composition
- Natural language to BT XML generation
- Advanced grasp planning (GraspNet)
- Multi-object manipulation

## Troubleshooting

### Vision service not available
- Ensure `vision_processor_node` is running
- Check camera topics: `ros2 topic list | grep camera`
- Verify camera is publishing: `ros2 topic echo /camera/color/image_raw --once`

### Detection fails
- Check ONNX model path in `vision_config.yaml`
- Verify object is in camera view
- Lower `confidence_threshold` for testing

### Planning fails
- Check MoveIt planning scene in RViz
- Verify named targets exist in SRDF
- Use `ClearOctomap` if stale collision data

## References

- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [MoveIt 2](https://moveit.ros.org/)
- [YOLOv8](https://github.com/ultralytics/ultralytics)
- [Isaac Sim](https://developer.nvidia.com/isaac-sim)

## License

Apache-2.0
