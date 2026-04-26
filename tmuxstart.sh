#!/bin/bash

# 获取脚本所在目录的绝对路径
WORKSPACE_ROOT=$(cd "$(dirname "$0")"; pwd)
INSTALL_SETUP="$WORKSPACE_ROOT/install/setup.bash"

# 检查环境文件是否存在
if [ ! -f "$INSTALL_SETUP" ]; then
    echo "错误: 找不到 $INSTALL_SETUP"
    echo "请先在 $WORKSPACE_ROOT 运行: colcon build --symlink-install"
    exit 1
fi

SESSION="simdog"

# 清理旧会话
tmux kill-session -t $SESSION 2>/dev/null

# 启动第一个任务：Gazebo
tmux new-session -d -s $SESSION -n "gazebo" "source $INSTALL_SETUP && ros2 launch go2_config gazebo_velodyne.launch.py rviz:=false; exec bash"

echo "正在启动 Gazebo，等待 8 秒..."
sleep 8

# 启动其他模块
tmux new-window -t $SESSION -n "lio_sam" "source $INSTALL_SETUP && ros2 launch lio_sam lidar.launch.py; exec bash"
tmux new-window -t $SESSION -n "ndt"     "source $INSTALL_SETUP && ros2 launch ndt_relocalization ndt_localization.launch.py; exec bash"
tmux new-window -t $SESSION -n "teleop"  "source $INSTALL_SETUP && ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"

# 切换到键盘控制窗口并进入
tmux select-window -t $SESSION:teleop
tmux attach-session -t $SESSION
