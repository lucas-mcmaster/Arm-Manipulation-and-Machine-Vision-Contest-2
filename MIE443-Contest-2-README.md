# Autonomous Mobile Manipulation: Search, Pick & Place in ROS 2

![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Iron-22314E?logo=ros)
![C++](https://img.shields.io/badge/Language-C%2B%2B17-00599C?logo=c%2B%2B)
![Python](https://img.shields.io/badge/Language-Python-3776AB?logo=python)
![Nav2](https://img.shields.io/badge/Navigation-Nav2%20(MPPI)-blue)
![MoveIt2](https://img.shields.io/badge/Manipulation-MoveIt%202-red)
![YOLOv8](https://img.shields.io/badge/Perception-YOLOv8-00FFFF)


> **MIE443 Mechatronics Systems: Design & Integration | University of Toronto**  
> *Team Members: Lucas McMaster, Nicolas Rebollo Canedo-Arguelles, Ahmed Fahmi, Bido Mohamed*  
> *Contest 2: "I Can Help You Clean — Pick and Place of Household Objects"*

---

##  Overview

This project implements a fully autonomous mobile manipulation pipeline deployed on a **Clearpath TurtleBot 4 Lite** equipped with an **SO-ARM101 6-DOF robotic arm**. Operating within an unknown $4.87 \times 4.87\text{ m}^2$ maze under a strict 300-second (5-minute) deadline, the robot must:

1. Grasp a manipulable target object from its onboard payload plate and classify it using computer vision.
2. Determine an optimal travel path to survey 5 randomly placed scene object stations.
3. Detect, classify, and match scene objects against the grasped target.
4. Dynamically localize the matching station's receptacle bin using visual fiducials (AprilTags) and execute a collision-free arm placement trajectory.
5. Survey all remaining stations and return autonomously to the starting position.

---

##  Hardware & Sensor Suite

- **Mobile Base**: iRobot Create 3 / Clearpath TurtleBot 4 Lite.
- **Sensors**:  
  - **RPLIDAR A1M8 2D LiDAR**: 360° planar scan, AMCL localization, and real-time obstacle costmaps.  
  - **Luxonis OAK-D Lite Stereo Camera**: RGB scene object classification and AprilTag bin localization.  
  - **Fused Odometry**: Wheel encoders, IMU, and optical ground tracking sensor.
- **Manipulator**: SO-ARM101 6-DOF Follower Arm with parallel gripper, controlled via serial USB bridge.
- **Compute Architecture**: Distributed setup across onboard Raspberry Pi 4 (sensor streaming, AprilTag detection server) and host laptop (Nav2 stack, MoveIt 2, YOLOv8 inference, master FSM).

---

##  Software Architecture & Control Flow

```text
+-----------------------------------------------------------------------------------+
|                            Master FSM (contest2.cpp)                              |
+-----------------------------------------------------------------------------------+
|                    |                     |                     |
v                    v                     v                     v
+--------------+     +--------------+      +--------------+      +------------------+
|  Navigation  |     | Manipulation |      | Vision / ML  |      | Visual Fiducial  |
| (Nav2 / MPPI)|     |  (MoveIt 2)  |      |   (YOLOv8)   |      |   (AprilTags)    |
+--------------+     +--------------+      +--------------+      +------------------+
|                    |                     |                     |
AMCL / NavFn         Cartesian Pose       OAK-D Service         tag36h11 (0-4)
Dijkstra Heuristic    KDL / OMPL Plan       Class Filter          tf2 Transform
```

### 9-State Finite State Machine (`contest2.cpp`)
1. `INIT`: Waits for converged AMCL initial pose; dynamically parses `coords.xml`; computes optimal visit order.
2. `PICKUP_OBJECT`: MoveIt2 Cartesian sequence lifts the object in front of the OAK-D Lite camera; YOLOv8 classifies the object (10s timeout fallback).
3. `NAVIGATE_SCENE`: Nav2 plans and moves to the next station approach pose ($0.5\text{ m}$ standoff along bin face normal $\phi_b$, heading rotated 180°).
4. `DETECT_SCENE_OBJECT`: Executes staged in-place rotation sweeps ($\pm 30^\circ$ in $10^\circ$ increments) and queries the YOLO service.
5. `CHECK_MATCH`: Evaluates if the detected scene object matches the target payload.
6. `PLACE_IN_BIN`: Navigates to $0.2\text{ m}$ offset; localizes the bin via AprilTag (`tag36h11`); performs frame transformation (`oakd_camera_frame` $\to$ `arm_mount`); executes MoveIt2 drop trajectory.
7. `RETURN_HOME`: Navigates back to the initial start coordinates.
8. `WRITE_OUTPUTS`: Generates `contest2_output.txt` containing detection names, confidence scores, and coordinates.
9. `DONE`: Idles safely upon completing mission objectives.

---

##  Algorithmic Highlights

### 1. Navigable Path-Length Traveling Salesperson Problem (TSP)
Instead of relying on naive Euclidean distance heuristics (which fail in mazes with walls), `buildVisitOrder()` queries Nav2’s `compute_path_to_pose` action server. It computes actual costmap arc lengths by:
$$L = \sum_{i=1}^n \sqrt{(x_i - x_{i-1})^2 + (y_i - y_{i-1})^2}$$

Then, a greedy nearest-neighbour sequence is computed in $<10\text{ seconds}$ across all 5 waypoints, optimizing travel time within the 300s limit.

### 2. Two-Stage YOLOv8 Perception Pipeline
- Raw detections are filtered through a strict 5-class allowlist: `[cup, bottle, clock, motorcycle, potted plant]`.
- Implements an empirical remap for known lighting edge-cases (`refrigerator` $\to$ `cup`).
- Enforces confidence thresholding ($\ge 0.20$) and multi-frame retry loops with in-place rotation sweeping.

### 3. Coordinate Frame Spatial Transformation for Bin Placement
Transforms localized AprilTag poses from the camera frame to the arm manipulator base:
$$T_{\text{armMount}}^{\text{tag}} = \left( T_{\text{oakd}}^{\text{armMount}} \right)^{-1} \cdot T_{\text{oakd}}^{\text{tag}}$$
Applies safety clearance offsets to guarantee collision-free gripper positioning over the bin drop zone.

---

##  Repository File Structure

```text
MIE443-Contest-2/
├── include/mie443_contest2/
│   ├── apriltag_detector.h     # AprilTag fiducial subscriber and pose utilities
│   ├── arm_controller.h        # MoveIt 2 Cartesian planning and gripper interface
│   ├── boxes.h                 # XML parser for station coordinates (coords.xml)
│   ├── navigation.h            # Nav2 action client interface
│   ├── robot_pose.h            # AMCL pose callback handler
│   └── yoloInterface.h         # ROS 2 service client for YOLO detection
├── src/
│   ├── contest2.cpp            # Primary high-level executive Finite State Machine
│   ├── apriltag_detector.cpp
│   ├── arm_controller.cpp
│   ├── boxes.cpp
│   ├── navigation.cpp
│   └── yoloInterface.cpp
├── scripts/
│   ├── yolo_detector.py        # Python node running Ultralytics YOLOv8 inference
│   ├── image_capture_server.py # Raspberry Pi camera capture service
│   └── launch_all.sh           # Multi-terminal initialization script
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

##  Build & Execution Guide

### Prerequisites
- ROS 2 (Humble or Iron)
- Nav2, MoveIt 2, `apriltag_ros`, `ultralytics` (YOLOv8)

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select mie443_contest2
source install/setup.bash
```

### Run
Execute the multi-process launch script to bring up navigation, vision, manipulation, and the central state machine:
```bash
./src/mie443_contest2/scripts/launch_all.sh
```
