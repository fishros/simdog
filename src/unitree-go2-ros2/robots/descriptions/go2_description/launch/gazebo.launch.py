import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('go2_description')
    
    # 定义世界文件路径
    world_path = os.path.join(pkg_share, 'worlds', 'my.world')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 设置Gazebo环境变量
    # 启动Gazebo服务器
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'use_sim_time': use_sim_time,
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )
    
    # 启动机器人描述
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch', 'description.launch.py'
        )]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 启动机器人状态发布器
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'go2', '-topic', 'robot_description'],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        robot_description,
        spawn_entity
    ]) 