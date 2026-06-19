#!/usr/bin/env python3
"""
总控制节点
横向速度PurePursuit控制 + 纵向速度PID控制
订阅：
  /track_points            来自路径规划的path
  /odometry/filtered        来自EKF的Odometry
发布：
  /model/racecar/cmd_vel  用于车辆控制的指令Twist
  /track_points_viz       路径可视化用的Path

"""
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist

from .lateral_controller import LateralController
from .longitudinal_controller import LongitudinalController


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self._declare_params()#引入参数
        wheelbase = self.get_parameter('wheelbase').value
        kp = self.get_parameter('speed_kp').value
        ki = self.get_parameter('speed_ki').value
        kd = self.get_parameter('speed_kd').value
        max_accel = self.get_parameter('max_accel').value

        self._lateral = LateralController(wheelbase=wheelbase)
        self._longitudinal = LongitudinalController(
            kp=kp, ki=ki, kd=kd, max_accel=max_accel)
        # 状态缓存
        self._path: np.ndarray | None = None
        self._odom: Odometry | None = None
        self._curvature: float = 0.0
        self._prev_steering: float = 0.0
        
        self._path_sub = self.create_subscription(
            Path, '/track_points', self._on_path, 10)# 订阅路径话题
        self._odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._on_odom, 10)#订阅车身位姿话题
        self._cmd_pub = self.create_publisher(
            Twist, '/model/racecar/cmd_vel', 10) #发布车的控制指令
        viz_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._path_viz_pub = self.create_publisher(
            Path, '/track_points_viz', viz_qos) #发布用于在rviz中可视化用的path

        freq = self.get_parameter('control_frequency').value
        self._timer = self.create_timer(1.0 / freq, self._control_loop)
        self._last_time = time.monotonic()

        self.get_logger().info('Pure Pursuit 控制节点已启动 | 横向=PurePursuit | 纵向=PID')

    def _declare_params(self):
        defaults = {
            'lookahead_distance':   1.5,
            'target_speed':         2.0,
            'wheelbase':            0.6,
            'max_steering_angle':   0.50,
            'max_accel':            2.0,
            'speed_kp':             1.5,
            'speed_ki':             0.05,
            'speed_kd':             0.02,
            'control_frequency':    50.0,
            'min_lookahead':        1.2,
            'lookahead_ratio':      0.70,
            'curve_gain':           8.0,
            'min_speed':            0.4,
            'cte_gain':             0.40,
            'cte_deadband':         0.03,
            'steering_filter_alpha': 0.60,
            'steering_scale':       0.70,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _on_path(self, msg: Path):
        """接收路径点，更新内部缓存。"""
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) >= 2:
            self._path = np.array(pts, dtype=np.float64)
            self._path_stamp = self.get_clock().now()
            self._path_viz_pub.publish(msg)
        else:
            self.get_logger().warn('路径点数不足 (<2)，忽略')

    def _on_odom(self, msg: Odometry):
        """接收里程计，更新车辆状态缓存。"""
        self._odom = msg

    def _control_loop(self):
        """主控制循环，50Hz 定时触发。"""
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        dt = max(0.01, min(0.05, dt))  # 钳位，防止卡顿导致跳变

        if self._path is None or self._odom is None:
            return

        # 提取车辆状态
        px = self._odom.pose.pose.position.x  #车的全局坐标
        py = self._odom.pose.pose.position.y
        q = self._odom.pose.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        vx = self._odom.twist.twist.linear.x
        vy = self._odom.twist.twist.linear.y
        current_speed = math.hypot(vx, vy)

        # 读取全部参数
        target_speed = self.get_parameter('target_speed').value
        lookahead = self.get_parameter('lookahead_distance').value
        max_steering = self.get_parameter('max_steering_angle').value
        min_lookahead = self.get_parameter('min_lookahead').value
        lookahead_ratio = self.get_parameter('lookahead_ratio').value
        curve_gain = self.get_parameter('curve_gain').value
        min_speed = self.get_parameter('min_speed').value
        steering_filter_alpha = self.get_parameter('steering_filter_alpha').value
        steering_scale = self.get_parameter('steering_scale').value

        # 最近路径点用于曲率检测
        nearest_idx = int(np.argmin(
            np.hypot(self._path[:, 0] - px, self._path[:, 1] - py)))

        # 计算转向角
        steering_angle = self._lateral.compute(
            self._path, px, py, yaw, current_speed,
            lookahead_distance=lookahead,
            max_steering_angle=max_steering,
            min_lookahead=min_lookahead,
            lookahead_ratio=lookahead_ratio,
            steering_scale=steering_scale,
        )

        # 转向低通滤波，防止转向角突变导致PID中的V_safe突变
        steering_angle = (
            steering_filter_alpha * self._prev_steering
            + (1.0 - steering_filter_alpha) * steering_angle
            #0.6原 + 0.4新
        )
        self._prev_steering = steering_angle

        # 局部曲率检测（三点法，覆盖约 1.5m）
        n_path = len(self._path)
        if n_path >= 6:
            step_k = max(1, int(1.5 / 0.12))
            i0 = nearest_idx
            i1 = min(nearest_idx + step_k // 2, n_path - 1)
            i2 = min(nearest_idx + step_k, n_path - 1)
            if i2 - i0 < 2:
                i0, i1, i2 = max(0, i2 - 6), max(0, i2 - 3), i2
            p0 = self._path[i0]
            p1 = self._path[i1]
            p2 = self._path[i2]
            area2 = abs((p1[0] - p0[0]) * (p2[1] - p0[1]) -
                        (p1[1] - p0[1]) * (p2[0] - p0[0]))
            a = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            c_ = math.hypot(p0[0] - p2[0], p0[1] - p2[1])
            denom = a * b * c_
            raw_kappa = 4.0 * area2 / denom if denom > 1e-9 else 0.0
            self._curvature = 0.85 * self._curvature + 0.15 * raw_kappa
        else:
            self._curvature = 0.0

        # 纵向控制
        speed_cmd = self._longitudinal.compute(
            target_speed, current_speed, self._curvature, dt,
            min_speed=min_speed, curve_gain=curve_gain,
        )

        # 路径过时保护：超过 2 秒未收到新路径则停车
        if hasattr(self, '_path_stamp'):
            path_age = (self.get_clock().now() - self._path_stamp).nanoseconds / 1e9
            if path_age > 2.0:
                speed_cmd = 0.0

        # 终点检测：三层距离防线
        if len(self._path) >= 2:
            last_pt = self._path[-1]
            dist_to_end = math.hypot(px - last_pt[0], py - last_pt[1])

            if dist_to_end < 3.0:
                max_speed_near_end = target_speed * (dist_to_end / 3.0)
                if speed_cmd > max_speed_near_end:
                    speed_cmd = max_speed_near_end
            if dist_to_end < 0.5:
                speed_cmd = 0.0

            # 投影越线法兜底
            prev_pt = self._path[-2]
            path_dir = np.array([last_pt[0] - prev_pt[0], last_pt[1] - prev_pt[1]])
            path_len = np.linalg.norm(path_dir)
            if path_len > 1e-6:
                path_dir /= path_len
                to_car = np.array([px - last_pt[0], py - last_pt[1]])
                if np.dot(to_car, path_dir) > 0.0:
                    speed_cmd = 0.0

        # 合成 Twist 并发布
        if abs(steering_angle) > 1e-6:
            angular_z = speed_cmd * math.tan(steering_angle) / self._lateral.wheelbase
        else:
            angular_z = 0.0

        twist = Twist()
        twist.linear.x = speed_cmd
        twist.angular.z = angular_z
        self._cmd_pub.publish(twist)

        # 低频日志
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        if self._log_counter % 50 == 0:
            self.get_logger().info(
                f'delta={steering_angle:.3f} rad | '
                f'v_cmd={speed_cmd:.2f} m/s | '
                f'v_cur={current_speed:.2f} m/s | '
                f'kappa={self._curvature:.3f}')

    @staticmethod
    def _yaw_from_quaternion(x, y, z, w) -> float:
        """四元数提取 yaw 角。"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
