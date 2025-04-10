from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', ['/velodyne_points']),  # 输入点云主题
                ('scan', ['/scan'])  # 输出激光扫描主题
            ],
            parameters=[{
                'target_frame': 'velodyne',  # 修正拼写错误，确保与雷达帧一致
                'transform_tolerance': 0.01,  # TF 变换的容忍时间
                'min_height': -0.3,  # 点云的最小高度
                'max_height': 0.3,  # 点云的最大高度
                'angle_min': -3.14159,  # 激光扫描的最小角度 (-180°)
                'angle_max': 3.14159,  # 激光扫描的最大角度 (180°)
                'angle_increment': 0.0043,  # 激光扫描的角度分辨率
                'scan_time': 0.3333,  # 扫描时间
                'range_min': 0.9,  # 激光扫描的最小范围，与雷达配置一致
                'range_max': 100.0,  # 激光扫描的最大范围，与雷达配置一致
                'use_inf': True,  # 是否使用无穷远值表示超出范围的点
                'inf_epsilon': 1.0  # 无穷远值的偏移量
            }],
            name='pointcloud_to_laserscan'
        )
    ])