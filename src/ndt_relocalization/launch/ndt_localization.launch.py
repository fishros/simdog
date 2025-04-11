import os

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 定义参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_path = LaunchConfiguration('map_path', default='/home/lx/tmp/map_indoor/GlobalMap.pcd')
    map_topic = LaunchConfiguration('map_topic', default='/global_map')
    input_cloud_topic = LaunchConfiguration('input_cloud_topic', default='/velodyne_points')
    debug_mode = LaunchConfiguration('debug_mode', default='true')
    use_multi_scale_ndt = LaunchConfiguration('use_multi_scale_ndt', default='true')
    ndt_num_threads = LaunchConfiguration('ndt_num_threads', default='4')
 
    # 启动NDT重定位节点
    ndt_relocalization_node = launch_ros.actions.Node(
        package='ndt_relocalization',
        executable='ndt_relocalization_node',
        name='ndt_relocalization_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'map_path': map_path,
                'map_topic': map_topic,
                
                # 多分辨率NDT设置
                'use_multi_scale_ndt': use_multi_scale_ndt,  # 是否使用多分辨率NDT算法
                'ndt_resolutions': [4.0, 2.0, 1.0],  # 逐步减小的分辨率，从粗到细
                'ndt_num_threads': ndt_num_threads,  # 并行线程数，OMP加速版NDT使用
                
                # 单分辨率NDT参数（多分辨率模式下仍会使用部分参数）
                'ndt_resolution': 1.0,  # 单分辨率模式下使用的分辨率
                'ndt_step_size': 0.1,   # 梯度下降步长
                'ndt_epsilon': 0.01,    # 收敛阈值
                'ndt_max_iterations': 30, # 每个分辨率级别的最大迭代次数
                
                # 点云预处理参数
                'voxel_leaf_size': 0.2,  # 点云下采样体素大小
                
                # 话题配置
                'input_cloud_topic': input_cloud_topic,
                'init_pose_topic': '/initialpose',
                
                # 坐标系配置
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'robot_frame_id': 'base_link',
                
                # 初始位姿设置
                'use_initial_pose': False,  # 是否使用launch文件中的初始位姿
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_z': 0.0,
                'initial_pose_qx': 0.0,
                'initial_pose_qy': 0.0,
                'initial_pose_qz': 0.0,
                'initial_pose_qw': 1.0,
                
                # 其他设置
                'publish_tf': True,
                'debug_mode': debug_mode,
            }
        ],
    )
    
  
    rviz_config_file = os.path.join(get_package_share_directory('ndt_relocalization'), 'rviz', 'ndt_localization_enhanced.rviz')
        
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz', default='true'))
    )
    
    return launch.LaunchDescription([
        # 基本参数
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),
        launch.actions.DeclareLaunchArgument(
            'map_path',
            default_value='/home/lx/tmp/map_indoor/GlobalMap.pcd',
            description='Full path to PCD map file'),
        launch.actions.DeclareLaunchArgument(
            'map_topic',
            default_value='/global_map',
            description='Topic to publish the global map'),
        launch.actions.DeclareLaunchArgument(
            'input_cloud_topic',
            default_value='/velodyne_points',
            description='Topic of input point cloud'),
        launch.actions.DeclareLaunchArgument(
            'debug_mode',
            default_value='true',
            description='Enable detailed debug output'),
        launch.actions.DeclareLaunchArgument(
            'use_multi_scale_ndt',
            default_value='true',
            description='Enable multi-resolution NDT algorithm for better convergence'),
        launch.actions.DeclareLaunchArgument(
            'ndt_num_threads',
            default_value='4',
            description='Number of threads to use for OpenMP accelerated NDT'),
        launch.actions.DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 if true'),
        # 启动节点
        ndt_relocalization_node,
        rviz2_node
    ]) 