#!/usr/bin/env python3
"""
MPC 控制节点，同时优化横纵向控制。

订阅：
  /track_points      (Path)      来自路径规划
  /odometry/filtered (Odometry)  来自 EKF 状态估计

发布：
  /model/racecar/cmd_vel  (Twist)  车辆控制指令
  /track_points_viz       (Path)   路径可视化

与 pure_pursuit_node 接口一致，可互换使用。
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist

from .mpc_solver import solve


class MPCNode(Node):
    """MPC 控制节点。"""

    def __init__(self):
        super().__init__('mpc_node')

        self._declare_params()

        self._path: np.ndarray | None = None
        self._odom: Odometry | None = None
        self._prev_v: float = 0.0
        self._prev_omega: float = 0.0

        self._path_sub = self.create_subscription(
            Path, '/track_points', self._on_path, 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._on_odom, 10)

        self._cmd_pub = self.create_publisher(
            Twist, '/model/racecar/cmd_vel', 10)

        viz_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._path_viz_pub = self.create_publisher(
            Path, '/track_points_viz', viz_qos)

        freq = self.get_parameter('control_frequency').value
        self._timer = self.create_timer(1.0 / freq, self._control_loop)
        self._last_time = time.monotonic()

        self.get_logger().info('MPC 控制节点已启动 | 采样优化 6x11 候选 (66 条轨迹)')

    def _declare_params(self):
        defaults = {
            'target_speed':         2.0,
            'horizon':              30,
            'dt':                   0.025,
            'weight_tracking':      2.0,
            'weight_heading':       1.5,
            'weight_smooth':        0.2,
            'weight_progress':      0.2,
            'control_frequency':    20.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _on_path(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) >= 2:
            self._path = np.array(pts, dtype=np.float64)
            self._path_viz_pub.publish(msg)
            self.get_logger().debug(f'MPC 收到路径: {len(pts)} 个点')

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _control_loop(self):
        if self._path is None or self._odom is None:
            return

        now = time.monotonic()
        self._last_time = now

        # 提取车辆状态
        px = self._odom.pose.pose.position.x
        py = self._odom.pose.pose.position.y
        q = self._odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        vx = self._odom.twist.twist.linear.x
        vy = self._odom.twist.twist.linear.y
        _current_speed = math.hypot(vx, vy)

        target_speed = self.get_parameter('target_speed').value
        horizon = self.get_parameter('horizon').value
        model_dt = self.get_parameter('dt').value
        w_track = self.get_parameter('weight_tracking').value
        w_head = self.get_parameter('weight_heading').value
        w_smooth = self.get_parameter('weight_smooth').value
        w_progress = self.get_parameter('weight_progress').value

        # 终点检测
        if len(self._path) >= 2:
            last_pt = self._path[-1]
            prev_pt = self._path[-2]
            path_dir = np.array([last_pt[0] - prev_pt[0], last_pt[1] - prev_pt[1]])
            path_len = np.linalg.norm(path_dir)
            if path_len > 1e-6:
                path_dir /= path_len
                to_car = np.array([px - last_pt[0], py - last_pt[1]])
                if np.dot(to_car, path_dir) > 0.0:
                    self._cmd_pub.publish(self._zero_twist())
                    return

        # MPC 求解
        v_cmd, omega_cmd = solve(
            current_state=(px, py, yaw),
            current_control=(self._prev_v, self._prev_omega),
            path_xy=self._path,
            target_speed=target_speed,
            horizon=horizon,
            dt=model_dt,
            weight_tracking=w_track,
            weight_heading=w_head,
            weight_smooth=w_smooth,
            weight_progress=w_progress,
        )

        self._prev_v = v_cmd
        self._prev_omega = omega_cmd

        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = omega_cmd
        self._cmd_pub.publish(twist)

        if not hasattr(self, '_log_cnt'):
            self._log_cnt = 0
        self._log_cnt += 1
        if self._log_cnt % 20 == 0:
            self.get_logger().info(
                f'MPC: v={v_cmd:.2f} m/s | omega={omega_cmd:.3f} rad/s | '
                f'v_cur={_current_speed:.2f} m/s')

    @staticmethod
    def _zero_twist() -> Twist:
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        return t


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
