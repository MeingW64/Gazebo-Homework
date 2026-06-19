"""
perception.launch.py

感知组的一键启动 launch 文件。

它会启动：
    1. EKF 融合节点（或基础 vehicle_state_publisher，通过参数切换）
    2. 锥桶 RViz 可视化节点
    3. SLAM 建图节点（slam_toolbox）

使用方式：
    ros2 launch percep_localization perception.launch.py

如果你只想跑最基础的位姿发布：
    ros2 launch percep_localization perception.launch.py use_ekf:=false

启动前请确保：
    - Gazebo 仿真已经启动（ros2 launch racecar_description view_in_gazebo.launch.py）
    - 工作空间已经编译（colcon build --symlink-install）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf = LaunchConfiguration('use_ekf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用仿真时间'
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='true: 用 EKF 融合；false: 只转发 /odom'
        ),

        # ===== 模式 1：EKF 融合（默认）=====
        Node(
            package='percep_localization',
            executable='ekf_fusion_node',
            name='ekf_fusion_node',
            output='screen',
            condition=IfCondition(use_ekf),
            parameters=[{
                'use_sim_time': use_sim_time,
                'use_mag': True,
                'use_gps': True,
                'publish_rate': 50.0,
                'gps_origin_lat': 23.16,
                'gps_origin_lon': 113.40,
                'mag_ref_x': 5.5645e-6,
                'mag_ref_y': 22.8758e-6,
                'mag_ref_z': -42.3884e-6,
            }]
        ),

        # ===== 模式 2：基础 vehicle_state_publisher =====
        Node(
            package='percep_localization',
            executable='vehicle_state_publisher',
            name='vehicle_state_publisher',
            output='screen',
            condition=UnlessCondition(use_ekf),
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': '/odom',
                'output_topic': '/percep/vehicle_state',
                'publish_tf': True,
            }]
        ),

        # ===== 锥桶 RViz 可视化 =====
        Node(
            package='percep_localization',
            executable='cone_map_node',
            name='cone_map_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'sim_cone_topic': '/perception/cones',
                'scan_topic': '',
                'pointcloud_topic': '',
                'map_frame': 'odom',
                'merge_dist': 0.5,
                'publish_markers': True,
                'publish_pose_array': False,
                'publish_cone_detections': False,
            }]
        ),

        # ===== SLAM 建图 =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('percep_localization'),
                    'launch',
                    'slam.launch.py'
                )
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])
