#!/usr/bin/env python3
"""
ekf_fusion_node.py

这是感知组位姿估计的“核心节点”。

功能：
    用扩展卡尔曼滤波（EKF）把多种传感器数据融合起来，
    估计车辆的位姿（x, y, yaw）和速度（v, omega），
    发布 /odom/filtered 和 TF: odom → base_link。

为什么需要融合？
    - /odom（轮式里程计）：高频、短时准确，但长时间会累积误差（漂移）。
    - /imu（惯性测量单元）：高频，测角速度和加速度，但积分也会漂移。
    - /magnetometer（磁力计）：测地磁场，能给绝对航向（不漂），但有噪声。
    - /navsat（GPS）：低频，但能给绝对位置（不漂），也有噪声。

EKF 的作用：
    把这些传感器“取长补短”，得到一个比单一传感器更稳定、更准确的位姿。

本节点状态向量：
    X = [x, y, yaw, v, omega]^T
    x, y     : 车辆在 odom 坐标系下的位置（m）
    yaw      : 车头朝向（rad）
    v        : 纵向速度（m/s）
    omega    : 横摆角速度（rad/s）
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, MagneticField
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

from percep_localization.utils import (
    quaternion_to_yaw,
    yaw_to_quaternion,
    normalize_angle,
    magnetic_field_to_yaw,
)


class EKFFusionNode(Node):
    """
    扩展卡尔曼滤波（EKF）传感器融合节点。

    输入：
        - /odom（轮式里程计）：高频位姿、速度
        - /imu（IMU）：角速度
        - /magnetometer（磁力计）：绝对航向
        - /navsat（GPS）：绝对位置

    输出：
        - /odom/filtered：融合后的里程计
        - /percep/vehicle_state：与 /odom/filtered 内容相同，供规划/控制组订阅
        - TF：odom -> base_link

    状态向量：X = [x, y, yaw, v, omega]^T
        x, y   : odom 坐标系下的位置（m）
        yaw    : 车头朝向（rad）
        v      : 纵向速度（m/s）
        omega  : 横摆角速度（rad/s）
    """

    def __init__(self):
        super().__init__('ekf_fusion_node')

        # ==================== 参数配置 ====================
        # 这些参数可以在 launch 文件或命令行里改
        self.declare_parameter('use_mag', True)
        self.declare_parameter('use_gps', True)
        self.declare_parameter('publish_rate', 50.0)

        # GPS 原点：世界坐标系 ENU 的 (0,0) 对应的经纬度
        # 这个值必须和 racecar_description/worlds/racecar_world.sdf 里的一致！
        self.declare_parameter('gps_origin_lat', 23.16)
        self.declare_parameter('gps_origin_lon', 113.40)

        # 地磁场参考向量（Tesla, ENU），也必须和 world 文件一致
        self.declare_parameter('mag_ref_x', 5.5645e-6)
        self.declare_parameter('mag_ref_y', 22.8758e-6)
        self.declare_parameter('mag_ref_z', -42.3884e-6)

        self.use_mag = self.get_parameter('use_mag').value
        self.use_gps = self.get_parameter('use_gps').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.gps_origin_lat = self.get_parameter('gps_origin_lat').value
        self.gps_origin_lon = self.get_parameter('gps_origin_lon').value
        self.mag_ref = np.array([
            self.get_parameter('mag_ref_x').value,
            self.get_parameter('mag_ref_y').value,
            self.get_parameter('mag_ref_z').value,
        ])

        # ==================== EKF 状态初始化 ====================
        # X = [x, y, yaw, v, omega]
        self.X = np.zeros((5, 1), dtype=float)
        # P: 状态协方差矩阵，对角线表示我们对当前状态的不确定度
        self.P = np.diag([0.5, 0.5, 0.5, 0.1, 0.1])

        # Q: 过程噪声（预测模型有多不准）
        self.Q = np.diag([0.01, 0.01, 0.005, 0.01, 0.005])

        # R_xxx: 各传感器的观测噪声（测量值有多不准）
        self.R_odom_pos = np.diag([0.02, 0.02])          # odom 位置
        self.R_odom_yaw = np.array([[0.02]])             # odom 姿态 yaw
        self.R_odom_vel = np.diag([0.01, 0.005])         # odom 速度、角速度
        self.R_mag = np.array([[0.20]])                  # 磁力计 yaw
        self.R_gps = np.diag([0.5, 0.5])                 # GPS x, y

        # 上次预测的时间戳
        self.last_predict_time = None

        # ==================== 传感器最新数据缓存 ====================
        self.latest_imu = None
        self.latest_mag = None
        self.latest_gps = None
        self.latest_odom = None

        # ==================== 订阅传感器话题 ====================
        self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        self.create_subscription(Imu, '/imu', self.imu_callback, 100)
        self.create_subscription(MagneticField, '/magnetometer', self.mag_callback, 50)
        self.create_subscription(NavSatFix, '/navsat', self.gps_callback, 10)

        # ==================== 发布融合结果 ====================
        self.odom_pub = self.create_publisher(Odometry, '/odom/filtered', 50)
        self.state_pub = self.create_publisher(Odometry, '/percep/vehicle_state', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ==================== 定时器：固定频率发布 ====================
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info('EKF 融合节点已启动')
        self.get_logger().info(f'use_mag={self.use_mag}, use_gps={self.use_gps}')

        # 初始发布一次 TF，让其他节点启动后立刻能查到 odom -> base_link
        self.publish_results(self.get_clock().now())

    # ==================== 回调函数：接收传感器数据 ====================
    def odom_callback(self, msg: Odometry):
        """接收轮式里程计。"""
        self.latest_odom = msg

    def imu_callback(self, msg: Imu):
        """接收 IMU。"""
        self.latest_imu = msg

    def mag_callback(self, msg: MagneticField):
        """接收磁力计。"""
        self.latest_mag = msg

    def gps_callback(self, msg: NavSatFix):
        """接收 GPS。"""
        self.latest_gps = msg

    # ==================== EKF 预测步骤 ====================
    def predict(self, dt: float):
        """
        EKF 预测步骤：根据运动学模型，用上一时刻状态推算下一时刻状态。

        车辆运动学模型（自行车模型近似）：
            x_{k+1} = x_k + v_k * cos(yaw_k) * dt
            y_{k+1} = y_k + v_k * sin(yaw_k) * dt
            yaw_{k+1} = yaw_k + omega_k * dt
            v_{k+1} = v_k
            omega_{k+1} = omega_k

        速度/角速度来源优先级：
            1. /odom 的 twist（最可靠，直接使用）
            2. 如果 odom 角速度很小，用 IMU 的 angular_velocity.z 补充或加权融合
            3. 都没有收到时，保持上一时刻值
        """
        x, y, yaw, v, omega = self.X.flatten()

        # 读取 /odom 的速度
        if self.latest_odom is not None:
            v = self.latest_odom.twist.twist.linear.x
            # 如果 odom 有有效角速度，直接采用
            omega_odom = self.latest_odom.twist.twist.angular.z
            if abs(omega_odom) > 1e-6:
                omega = omega_odom

        # 读取 IMU 的角速度（z 轴），并与 odom 角速度做互补加权
        if self.latest_imu is not None:
            omega_imu = self.latest_imu.angular_velocity.z
            # 当 odom 角速度有效时，主要信任 odom（70%），IMU 辅助（30%）
            # 因为轮式里程计的 yaw rate 在仿真中通常比 IMU 更稳定
            if self.latest_odom is not None and abs(self.latest_odom.twist.twist.angular.z) > 1e-6:
                omega = 0.7 * self.latest_odom.twist.twist.angular.z + 0.3 * omega_imu
            else:
                omega = omega_imu

        # 运动学预测
        x_new = x + v * math.cos(yaw) * dt
        y_new = y + v * math.sin(yaw) * dt
        yaw_new = normalize_angle(yaw + omega * dt)
        v_new = v
        omega_new = omega

        self.X = np.array([[x_new], [y_new], [yaw_new], [v_new], [omega_new]])

        # 计算雅可比矩阵 F（状态转移矩阵的线性化）
        F = np.eye(5)
        F[0, 2] = -v * math.sin(yaw) * dt
        F[0, 3] = math.cos(yaw) * dt
        F[1, 2] = v * math.cos(yaw) * dt
        F[1, 3] = math.sin(yaw) * dt
        F[2, 4] = dt

        # 更新协方差：P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q

    # ==================== EKF 更新步骤 ====================
    def update_odom(self):
        """用 /odom 的位置、姿态 yaw 和速度更新 EKF。"""
        if self.latest_odom is None:
            return

        msg = self.latest_odom

        # --- 更新位置 [x, y] ---
        z_pos = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y],
        ])
        H_pos = np.zeros((2, 5))
        H_pos[0, 0] = 1.0
        H_pos[1, 1] = 1.0
        self._ekf_update(z_pos, H_pos, self.R_odom_pos)

        # --- 更新姿态 yaw ---
        # Gazebo /odom 的四元数是仿真车辆真实姿态。这里必须融合 yaw，
        # 否则 EKF 只靠磁力计会把 base_link 朝向拉到相反方向。
        q = msg.pose.pose.orientation
        yaw_odom = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        z_yaw = np.array([[yaw_odom]])
        H_yaw = np.zeros((1, 5))
        H_yaw[0, 2] = 1.0
        self._ekf_update(z_yaw, H_yaw, self.R_odom_yaw)

        # --- 更新速度 [v, omega] ---
        z_vel = np.array([
            [msg.twist.twist.linear.x],
            [msg.twist.twist.angular.z],
        ])
        H_vel = np.zeros((2, 5))
        H_vel[0, 3] = 1.0
        H_vel[1, 4] = 1.0
        self._ekf_update(z_vel, H_vel, self.R_odom_vel)

    def update_mag(self):
        """用磁力计更新航向 yaw。"""
        if not self.use_mag or self.latest_mag is None:
            return

        msg = self.latest_mag
        yaw_mag = magnetic_field_to_yaw(
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
            self.mag_ref[0],
            self.mag_ref[1],
            self.mag_ref[2],
        )

        z = np.array([[yaw_mag]])
        H = np.zeros((1, 5))
        H[0, 2] = 1.0
        self._ekf_update(z, H, self.R_mag)

    def update_gps(self):
        """用 GPS 更新绝对位置。"""
        if not self.use_gps or self.latest_gps is None:
            return

        # GPS 经纬度转成 ENU 平面坐标（以 gps_origin 为原点）
        x_gps, y_gps = self.gps_to_enu(
            self.latest_gps.latitude,
            self.latest_gps.longitude
        )

        z = np.array([[x_gps], [y_gps]])
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        self._ekf_update(z, H, self.R_gps)

    def _ekf_update(self, z, H, R):
        """
        通用 EKF 更新公式：
            y = z - H * X          （观测残差，也叫 innovation）
            S = H * P * H^T + R    （残差协方差）
            K = P * H^T * S^{-1}   （卡尔曼增益）
            X = X + K * y
            P = (I - K * H) * P

        参数：
            z: 观测值向量
            H: 观测矩阵（把状态映射到观测空间）
            R: 观测噪声协方差
        """
        y = z - H @ self.X
        # 角度归一化（只对 yaw 这一维）
        # 因为 yaw 是角度，直接相减可能得到 2*pi 附近的错误大值
        if y.shape[0] == 1 and H.shape[1] == 5 and H[0, 2] == 1.0:
            y[0, 0] = normalize_angle(y[0, 0])

        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn('S 矩阵不可逆，跳过本次更新')
            return

        self.X = self.X + K @ y
        self.X[2, 0] = normalize_angle(self.X[2, 0])  # yaw 归一化

        I = np.eye(self.P.shape[0])
        self.P = (I - K @ H) @ self.P

    # ==================== GPS -> ENU 转换 ====================
    def gps_to_enu(self, lat, lon):
        """
        简化版 GPS 经纬度转 ENU 平面坐标。

        思路：
            在地球表面某一点附近，可以把经纬度近似看成平面坐标。
            1 纬度 ≈ 111320 米
            1 经度 ≈ 111320 * cos(纬度) 米

        参数：
            lat, lon: 当前 GPS 读数，单位度
        返回：
            x, y: ENU 坐标，单位米
        """
        dlat = math.radians(lat - self.gps_origin_lat)
        dlon = math.radians(lon - self.gps_origin_lon)

        R_earth = 6371000.0  # 地球半径，米
        lat_rad = math.radians(self.gps_origin_lat)

        x = R_earth * math.cos(lat_rad) * dlon
        y = R_earth * dlat

        return x, y

    # ==================== 主循环 ====================
    def timer_callback(self):
        """定时器回调，固定频率执行预测和更新，并发布结果。"""
        now = self.get_clock().now()

        # 第一次进来时没有 dt，先初始化时间
        if self.last_predict_time is None:
            self.last_predict_time = now
            return

        dt = (now - self.last_predict_time).nanoseconds / 1e9
        self.last_predict_time = now

        # dt 太大说明仿真暂停过，限制一下防止预测发散
        if dt <= 0.0 or dt > 0.5:
            dt = 0.0

        # 1. 预测
        if dt > 0.0:
            self.predict(dt)

        # 2. 更新（用最新传感器数据修正预测）
        self.update_odom()
        self.update_mag()
        self.update_gps()

        # 3. 发布
        self.publish_results(now)

    def publish_results(self, now):
        """发布融合后的里程计、车辆状态和 TF。"""
        x, y, yaw, v, omega = self.X.flatten()

        # 构造四元数
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        # --- 发布 /odom/filtered ---
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        odom_msg.twist.twist.linear.x = float(v)
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = float(omega)

        # 协方差（从 EKF 的 P 矩阵来）
        odom_msg.pose.covariance[0] = float(self.P[0, 0])
        odom_msg.pose.covariance[7] = float(self.P[1, 1])
        odom_msg.pose.covariance[35] = float(self.P[2, 2])
        odom_msg.twist.covariance[0] = float(self.P[3, 3])
        odom_msg.twist.covariance[35] = float(self.P[4, 4])

        self.odom_pub.publish(odom_msg)

        # --- 发布 /percep/vehicle_state ---
        # 这里和 /odom/filtered 内容一样，只是 topic 名更适合规划组使用
        state_msg = Odometry()
        state_msg.header = odom_msg.header
        state_msg.child_frame_id = odom_msg.child_frame_id
        state_msg.pose = odom_msg.pose
        state_msg.twist = odom_msg.twist
        state_msg.pose.covariance = odom_msg.pose.covariance
        state_msg.twist.covariance = odom_msg.twist.covariance
        self.state_pub.publish(state_msg)

        # --- 发布 TF: odom -> base_link ---
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.tf_broadcaster.sendTransform(t)

        # 调试打印
        self.get_logger().debug(
            f'EKF: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}, v={v:.2f}, omega={omega:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EKFFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('EKF 节点被手动中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
