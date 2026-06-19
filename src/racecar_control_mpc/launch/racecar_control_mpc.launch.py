"""MPC 控制节点启动文件。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    target_speed_arg = DeclareLaunchArgument(
        'target_speed', default_value='2.0',
        description='目标速度 (m/s)')
    freq_arg = DeclareLaunchArgument(
        'control_frequency', default_value='20.0',
        description='MPC 控制频率 (Hz)')

    mpc_node = Node(
        package='racecar_control_mpc',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        parameters=[{
            'target_speed':      LaunchConfiguration('target_speed'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'horizon':           30,
            'dt':                0.025,
            'weight_tracking':   2.0,
            'weight_heading':    1.5,
            'weight_smooth':     0.2,
            'weight_progress':   0.2,
        }],
        remappings=[
            ('/track_points', '/planning/trajectory'),
            ('/odometry/filtered', '/percep/vehicle_state'),
        ],
    )

    return LaunchDescription([target_speed_arg, freq_arg, mpc_node])
