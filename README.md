# MIE443 - Contest2

## Run Bash Script
```cpp
./contest2.sh
```

## Debugging camera
```cpp
ros2 service call /oakd/stop_camera std_srvs/srv/Trigger {}
```

```cpp
ros2 service call /oakd/start_camera std_srvs/srv/Trigger {}
```

## Switch camera ports for arm
```cpp
nano ros2_ws/src/mie443_contest2/mie443_contest2/lerobot_moveit/launch/so101_turtlebot.launch.py
```

## Proper Command Order for Contest 2 (8 terminals required)

1. Launch AMCL (on laptop terminal)
```cpp
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/nicolas-rebollo/ros2_ws/src/mie443_contest2/maps/Contest2MapPractice.yaml
```

2. Initialize NAV2 (on laptop terminal)
```cpp
ros2 launch turtlebot4_navigation nav2.launch.py
```

3. Start up RVIZ for nav2 (on laptop terminal)
```cpp
ros2 launch turtlebot4_viz view_navigation.launch.py
```

4. Start MoveIt2 on Turtlebot (SSH)
```cpp
ssh ubuntu@100.69.127.75
```
```cpp
source contest2/bin/activate
```
```cpp
ros2 launch lerobot_moveit so101_turtlebot.launch.py
```

5. Start MoveIt2 on Laptop (on laptop terminal)
```cpp
ros2 launch lerobot_moveit so101_laptop.launch.py
```

6. Start Image Capture Server (SSH)
```cpp
ssh ubuntu@100.69.127.75
```
```cpp
ros2 run mie443_contest2 image_capture_server
```

7. Start AprilTag Server
```cpp
ssh ubuntu@100.69.127.75
```
```cpp
ros2 launch apriltag_ros camera_36h11.launch.yml
```

7. Launch YOLO Detector (on laptop terminal)
```cpp
source ~/contest2/bin/activate
```
```cpp
ros2 run mie443_contest2 yolo_detector.py
```

8. Launch Contest 2 Code (on laptop terminal)
```cpp
ros2 run mie443_contest2 contest2
```


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
