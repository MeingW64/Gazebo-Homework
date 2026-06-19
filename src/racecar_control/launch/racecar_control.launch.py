#纯跟踪 + PID控制节点启动文件
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lookahead_arg = DeclareLaunchArgument(
        'lookahead_distance', default_value='1.5',
        description='前瞻距离(m)')
    target_speed_arg = DeclareLaunchArgument(
        'target_speed', default_value='2.0',
        description='目标巡航速度(m/s)')
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase', default_value='0.6',
        description='轴距 (m)')
    max_steering_arg = DeclareLaunchArgument(
        'max_steering_angle', default_value='0.50',
        description='最大转向角(rad)')
    control_freq_arg = DeclareLaunchArgument(
        'control_frequency', default_value='50.0',
        description='控制频率(Hz)')

    control_node = Node(
        package='racecar_control',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'target_speed':      LaunchConfiguration('target_speed'),
            'wheelbase':         LaunchConfiguration('wheelbase'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'speed_kp':          1.5,
            'speed_ki':          0.05,
            'speed_kd':          0.02,
            'max_accel':         2.0,
            'min_lookahead':     1.2,
            'lookahead_ratio':   0.70,
            'curve_gain':        8.0,
            'min_speed':         0.4,
            'cte_gain':          0.50,
            'cte_deadband':      0.01,
            'steering_filter_alpha': 0.60,
            'steering_scale':    0.70,
        }],
        remappings=[
            ('/track_points', '/planning/trajectory'),
            ('/odometry/filtered', '/percep/vehicle_state'),
        ],
    )

    return LaunchDescription([
        lookahead_arg, target_speed_arg, wheelbase_arg,
        max_steering_arg, control_freq_arg, control_node,
    ])
