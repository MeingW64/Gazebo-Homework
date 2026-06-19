"""
ekf.launch.py

单独启动 EKF 融合节点，用于调试或降级场景。

默认情况下（perception.launch.py use_ekf:=true）EKF 会随感知 launch 一起启动，
不需要单独运行此文件。只有在想单独调试 EKF、或从 launch 中排除 EKF 时，
才使用：

    ros2 launch percep_localization ekf.launch.py

此节点会融合 /odom、/imu、/magnetometer、/navsat，
估计车辆的位姿（x, y, yaw）和速度（v, omega），
发布 /odom/filtered、/percep/vehicle_state 和 TF: odom -> base_link。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mag = LaunchConfiguration('use_mag')
    use_gps = LaunchConfiguration('use_gps')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用仿真时间'
        ),
        DeclareLaunchArgument(
            'use_mag',
            default_value='true',
            description='是否融合磁力计数据'
        ),
        DeclareLaunchArgument(
            'use_gps',
            default_value='true',
            description='是否融合 GPS 数据'
        ),

        Node(
            package='percep_localization',
            executable='ekf_fusion_node',
            name='ekf_fusion_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'use_mag': use_mag,
                'use_gps': use_gps,
                'publish_rate': 50.0,
                # GPS 原点和地磁场参考值必须和 racecar_description 一致
                'gps_origin_lat': 23.16,
                'gps_origin_lon': 113.40,
                'mag_ref_x': 5.5645e-6,
                'mag_ref_y': 22.8758e-6,
                'mag_ref_z': -42.3884e-6,
            }]
        ),
    ])
