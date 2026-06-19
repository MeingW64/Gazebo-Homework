"""仅启动 Gazebo 仿真环境的启动文件，不含感知、规划、控制。"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('racecar_description')

    xacro_file = os.path.join(pkg, 'urdf', 'racecar.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'racecar_world.sdf')
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r ' + world_file}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'racecar',
                   '-x', '0.0', '-y', '-15.0', '-z', '0.5', '-Y', '1.5708'],
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

    odom_tf = Node(
        package='percep_localization',
        executable='vehicle_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    gz_resource = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg, 'tracks', 'models') + ':' + os.path.join(pkg, 'tracks', 'meshes')
    )

    return LaunchDescription([gz_resource, rsp, gz_sim, spawn, bridge, static_tf, odom_tf])
