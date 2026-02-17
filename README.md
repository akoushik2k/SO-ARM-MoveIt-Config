# SO-101 Vision-Guided Autonomous Picking with NVIDIA Nemotron

This repository integrates an **Autonomous Picking Pipeline** for the SO-101 robotic arm, leveraging **NVIDIA Nemotron** to bridge the gap between human intent and robotic action via Behavior Trees.

## 🚀 The Architecture: A Decoupled Pipeline

The project follows a modern "Perception-Cognition-Action" architecture, decoupling high-level planning from low-level execution for maximum flexibility and reliability.

### Stage 1: Perception (Vision Processor)
- **Node**: `vision_processor_node.py`
- **Role**: Detects objects (e.g., "red cube") in a 2D camera stream and de-projects them into 3D space.
- **Key Solve**: Implements **unit scaling (feet to meters)** ensuring simulation coordinates match the robot's physical reach (~0.45m).
- **Industry Standard**: Uses ROS 2 `tf2` transforms with **Simulation Time Synchronization** to ensure sub-millimeter temporal alignment between the robot and its environment.

### Stage 2: Cognition (NVIDIA Nemotron Compiler)
- **Node**: `bt_compiler_node.py`
- **Role**: Translates natural language intent (e.g., *"Pick up the red cube and move it to the home position"*) into a structured Behavior Tree XML.
- **The Nemotron Edge**: Unlike static scripts, Nemotron models complex logic (Retries, Fallbacks, Gripper timing) on-the-fly, allowing the robot to reason about tasks without hardcoded state machines.

### Stage 3: Action (Behavior Tree Executor)
- **Node**: `bt_executor_node` (C++)
- **Role**: Parses the generated XML and executes motion via **MoveIt 2**.
- **Execution Efficiency**: Uses specialized nodes like `PrecisePick` for controlled hover-approaches, ensuring safety and precision during the final grasp.

---

## 🛠 Why NVIDIA Nemotron?

### 🧠 Intent-to-Action Mapping
In the industry, hardcoding every possible pick-and-place permutation is impossible. **Nemotron** acts as the cognitive engine, converting ambiguous human requests into deterministic logical structures (Behavior Trees). This enables "Zero-Code" tasking where an operator describes a goal, and the robot compiles its own execution plan.

### 🔄 Resilience & Reactivity
Nemotron understands the importance of failure recovery. By automatically wrapping detection steps in `<RetryUntilSuccessful>` decorators, it ensures the robot doesn't give up if an object is briefly occluded or if a planning attempt fails—a critical requirement for high-uptime industrial deployments.

---

## 🏭 Industry Significance

This project demonstrates a production-grade approach to **Software-Defined Robotics**:
1. **Portability**: The system uses dynamic package lookups (`get_package_share_directory`), making it deployable across different factory workstations without path re-configuration.
2. **Safety-First Planning**: By separating the "thinking" (LLM) from the "doing" (MoveIt/C++), we ensure that even if the LLM suggests an complex task, the underlying C++ safety constraints prevent physical collisions.
3. **Natural Human Interaction**: Redefines the role of the human operator from "Programmer" to "Instructors," significantly lowering the barrier to entry for robotic automation.

---

## 🚦 How to Run

### 1. Environment Setup
Avoid environment conflicts by using the system Python 3.10:
```bash
export PATH=/usr/bin:/usr/sbin:/bin:/sbin && source /opt/ros/humble/setup.bash && source install/setup.bash
```

### 2. Execution Sequence
**A. Start the Full Stack (Perception & Planner)**
```bash
ros2 launch so_arm_bt bt_executor.launch.py
```

**B. Define the Intent**
Open a new terminal and provide the task:
```bash
ros2 topic pub --once /task_intent std_msgs/msg/String "{data: 'Pick up the red cube and move it home'}"
```

**C. Execute the Compiled Task**
Once Terminal A confirms "Generated tree saved," run the executor:
```bash
ros2 launch so_arm_bt run_bt.launch.py
```

---
*Created for the SO-101 Vision Integration Project.*
