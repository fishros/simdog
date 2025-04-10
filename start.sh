#!/bin/bash

# 等待时间（秒）
WAIT_TIME=8

# 启动Gazebo仿真环境 3d_to_2d slamtoolbox
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch go2_config gazebo_velodyne.launch.py rviz:=false; exec bash"
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py; exec bash"
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch go2_config slam.launch.py; exec bash"   #启动slam_toolbox建图模式，如果需要定位，可以在配置文件里改，然后将导航的amcl注释掉
# gnome-terminal -- bash -c "source install/setup.bash && ros2 launch go2_config navigate.launch.py; exec bash"  #启动导航 暂时未配好 等待github 更新

sleep $WAIT_TIME

gnome-terminal -- bash -c "source install/setup.bash && ros2 launch lio_sam lidar.launch.py; exec bash"

gnome-terminal -- bash -c "source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"
