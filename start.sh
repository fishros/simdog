#!/bin/bash

# 等待时间（秒）
WAIT_TIME=8

# 启动Gazebo仿真环境
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true; exec bash"

sleep $WAIT_TIME

gnome-terminal -- bash -c "source install/setup.bash && ros2 launch lio_sam lidar.launch.py; exec bash"

gnome-terminal -- bash -c "source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"
