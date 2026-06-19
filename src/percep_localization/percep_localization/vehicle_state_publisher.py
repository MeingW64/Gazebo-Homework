#!/usr/bin/env python3
"""
vehicle_state_publisher.py

这是感知组位姿估计的“最小可用节点”。

功能：
    订阅 Gazebo 发布的 /odom（轮式里程计），
    把其中的位置、姿态、速度提取出来，
    发布成规划/控制组方便使用的 /percep/vehicle_state。

"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from percep_localization.utils import quaternion_to_yaw


class VehicleStatePublisher(Node):
    """
    最小车辆状态发布节点（EKF 的降级/备用方案）。

    功能：
        - 直接转发 /odom（或任意指定的里程计话题）到 /percep/vehicle_state。
        - 发布 TF：odom -> base_link。

    使用场景：
        - EKF 节点出现故障或不需要传感器融合时，
          可用此节点作为最小可用替代，保证规划/控制组仍能收到车辆状态。
        - 在 perception.launch.py 中通过 use_ekf:=false 启用。
    """

    def __init__(self):
        # 初始化节点，名字是 vehicle_state_publisher
        super().__init__('vehicle_state_publisher')

        # 声明参数：输入的里程计话题名
        # 默认用 /odom，后续可以改成 /odom/filtered
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_topic', '/percep/vehicle_state')
        self.declare_parameter('publish_tf', True)

        odom_topic = self.get_parameter('odom_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.get_logger().info(f'订阅里程计: {odom_topic}')
        self.get_logger().info(f'发布车辆状态: {output_topic}')

        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            50  # QoS 队列大小
        )

        # 发布车辆状态
        self.state_pub = self.create_publisher(
            Odometry,
            output_topic,
            50
        )

        # 发布 TF odom -> base_link（基础模式下需要，否则 SLAM/锥桶地图找不到 TF）
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg: Odometry):
        """
        每次收到 /odom 时调用。

        /odom 里有什么？
            - pose.pose.position: x, y, z（车在 odom 坐标系下的位置）
            - pose.pose.orientation: 四元数（车的姿态）
            - twist.twist.linear: 线速度
            - twist.twist.angular: 角速度
        """
        # 直接复用 Odometry 消息，但保持 frame_id 与 TF 一致。
        out_msg = Odometry()
        out_msg.header = msg.header
        out_msg.header.frame_id = 'odom'
        out_msg.child_frame_id = 'base_link'

        # 位置
        out_msg.pose = msg.pose

        # 速度
        out_msg.twist = msg.twist

        # 额外把 yaw 角打印出来，方便你调试
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # 每秒最多打印一次，避免刷屏
        self.get_logger().debug(
            f'x={msg.pose.pose.position.x:.2f}, '
            f'y={msg.pose.pose.position.y:.2f}, '
            f'yaw={yaw:.2f} rad, '
            f'v={msg.twist.twist.linear.x:.2f} m/s'
        )

        self.state_pub.publish(out_msg)

        # 发布 TF
        if self.publish_tf:
            self.publish_odom_tf(msg)

    def publish_odom_tf(self, msg: Odometry):
        """发布 odom -> base_link 的 TF 变换。"""
        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被手动中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
