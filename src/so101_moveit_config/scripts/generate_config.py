#!/usr/bin/env python3

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

moveit_config = (
    MoveItConfigsBuilder("so101")
    .robot_description(file_path="/home/nicolas-rebollo/ros2_ws/src/lerobot_description/urdf/so101.urdf")
    .to_moveit_configs()
)

generate_demo_launch(moveit_config)
