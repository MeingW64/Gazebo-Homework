"""
slam.launch.py

启动 2D 激光 SLAM：slam_toolbox。

默认情况下，此 launch 会被 perception.launch.py 自动包含，
不需要单独运行。如需单独调试 SLAM，可使用：

    ros2 launch percep_localization slam.launch.py

slam_toolbox 会：
    - 订阅 /scan（激光雷达数据）
    - 订阅 TF（odom -> base_link，由 EKF 节点发布）
    - 发布 /map（栅格地图）
    - 发布 TF（map -> odom）

启动前请确保：
    1. Gazebo 仿真已启动（有 /scan 数据）
    2. EKF 节点已启动（有 odom -> base_link 的 TF）
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 是否使用仿真时间（Gazebo 里必须设为 true）
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 自定义参数文件路径
    slam_params_file = os.path.join(
        get_package_share_directory('percep_localization'),
        'config',
        'slam_toolbox.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用仿真时间（Gazebo 仿真必须 true）'
        ),

        # 调用 slam_toolbox 自带的在线同步 SLAM launch
        # 传入自定义参数文件，放宽 transform_timeout
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_sync_launch.py'
                )
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
            }.items()
        ),
    ])
