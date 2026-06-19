"""
full_system.launch.py

一键启动全链路：仿真 + 感知 + 规划 + 控制。

启动节点：
  1. Gazebo 仿真（车辆 + 赛道 + 传感器 + bridge + TF）
  2. EKF 融合定位
  3. 仿真锥桶检测
  4. 锥桶可视化
  5. 路径规划
  6. 控制（默认 Pure Pursuit + PID，可选 MPC）
  7. RViz

使用方式：
  ros2 launch racecar_description full_system.launch.py
  ros2 launch racecar_description full_system.launch.py controller:=mpc target_speed:=1.5
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('racecar_description')

    controller = LaunchConfiguration('controller', default='main_controller')
    target_speed = LaunchConfiguration('target_speed', default='2.0')

    use_mpc = PythonExpression(['"', controller, '" == "mpc"'])

    # 仿真环境
    xacro_file = os.path.join(pkg, 'urdf', 'racecar.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'racecar_world.sdf')
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                      'use_sim_time': True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r ' + world_file}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', '-name', 'racecar',
            '-x', '0.0', '-y', '-15.0', '-z', '0.5',
            '-Y', '1.5708',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',
            '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/racecar/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    # EKF 已发布 odom -> base_link 的 TF，不再重复广播
    gz_resource = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg, 'tracks', 'models') + ':'
        + os.path.join(pkg, 'tracks', 'meshes')
    )

    #感知定位
    ekf_node = Node(
        package='percep_localization',
        executable='ekf_fusion_node',
        name='ekf_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_mag': True,
            'use_gps': True,
            'publish_rate': 50.0,
        }],
    )

    #感知代码
    sim_perception_node = Node(
        package='sim_perception',
        executable='sim_node',
        name='sim_node',
        output='screen',
    )

    # 锥桶RViz可视化
    cone_map_node = Node(
        package='percep_localization',
        executable='cone_map_node',
        name='cone_map_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'sim_cone_topic': '/perception/cones',
            'scan_topic': '',
            'pointcloud_topic': '',
            'map_frame': 'odom',
            'merge_dist': 0.5,
            'publish_markers': True,
            'publish_pose_array': False,
            'publish_cone_detections': True,
        }],
    )

    #路径规划
    path_planning_node = Node(
        package='path_planning',
        executable='path_planning_node',
        name='path_planning_node',
        output='screen',
    )

    #控制
    control_remaps = [
        ('/odometry/filtered', '/percep/vehicle_state'),
        ('/track_points', '/planning/trajectory'),
    ]

    control_pp = Node(
        package='racecar_control',
        executable='main_controller_node',
        name='main_controller_node',
        output='screen',
        condition=IfCondition(PythonExpression(['not ', use_mpc])),
        parameters=[{
            'target_speed': target_speed,
            'lookahead_distance': 1.5,
            'min_lookahead': 1.2,
            'lookahead_ratio': 0.50,
            'max_steering_angle': 0.50,
            'curve_gain': 8.0,
            'min_speed': 0.4,
            'cte_gain': 0.50,
            'cte_deadband': 0.01,
            'steering_filter_alpha': 0.60,
            'steering_scale': 0.70,
        }],
        remappings=control_remaps,
    )

    control_mpc = Node(
        package='racecar_control_mpc',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        condition=IfCondition(use_mpc),
        parameters=[{
            'use_sim_time': True,
            'target_speed': target_speed,
            'horizon': 30,
            'dt': 0.025,
            'weight_tracking': 2.0,
            'weight_heading': 1.5,
            'weight_smooth': 0.2,
            'weight_progress': 0.2,
        }],
        remappings=control_remaps,
    )

    # 规划路径RViz可视化 
    rviz_config = os.path.join(
        get_package_share_directory('racecar_control'), 'config', 'tracking.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='main_controller',
            description='控制算法: main_controller 或 mpc'),
        DeclareLaunchArgument(
            'target_speed', default_value='2.0',
            description='目标速度 (m/s)'),
        gz_resource, rsp, gz_sim, spawn, bridge, static_tf,
        ekf_node, sim_perception_node, cone_map_node,
        path_planning_node, control_pp, control_mpc, rviz,
    ])
