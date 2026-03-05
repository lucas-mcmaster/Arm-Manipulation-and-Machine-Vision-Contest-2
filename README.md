# MIE443 - Contest2

## Useful Commands

1. Colcon build required packages
```cpp
colcon build --packages-select mie443_contest2 \
lerobot_description lerobot_controller lerobot_moveit
```

2. Launch Gazebo simulation
```cpp
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=lite
```

3. Launch AMCL Localization software
```cpp
ros2 launch turtlebot4_navigation localization.launch.py map:=/opt/ros/jazzy/share/turtlebot4_navigation/maps/warehouse.yaml use_sim_time:=true
```

4. Open RViz2
```cpp
ros2 launch turtlebot4_viz view_navigation.launch.py  use_sim_time:=true
```

5. Print AMCL pose estimate
```cpp
ros2 topic echo /amcl_pose
```

6. Start Nav2
```cpp
ros2 launch turtlebot4_navigation nav2.launch.py  use_sim_time:=true
```

7. Start MoveIt2 & RViz2 (start each command in separate terminals)
```cpp
ros2 launch lerobot_description so101_gazebo.launch.py
ros2 launch lerobot_controller so101_controller.launch.py
ros2 launch lerobot_moveit so101_moveit.launch.py
```

8. Print arm pose
```cpp
ros2 run tf2_ros tf2_echo world gripper
```

9. SSH into robot (check if IP is correct)
```bash
ssh ubuntu@192.168.0.125
```

10. MoveIt2 for Real-life SO-ARM 101 (run each command in separate terminals, except source)
```cpp
source contest2/bin/activate
ros2 launch lerobot_moveit so101_turtlebot.launch.py
ros2 launch lerobot_moveit so101_laptop.launch.py
```

11. Start image capture server for YOLO
```cpp
ros2 run mie443_contest2 image_capture_server
```

12. Start YOLO detector (must source contest2 venv first)
```cpp
source ~/contest2/bin/activate
ros2 run mie443_contest2 yolo_detector.py
```

13. Run contest2 code
```cpp
ros2 run mie443_contest2 contest2
```

14. Camera preview
```cpp
ros2 run rqt_image_view rqt_image_view
```

15. Start camera and arm (for YOLO)
```cpp
ros2 launch lerobot_moveit so101_turtlebot.launch.py
```
