#!/usr/bin/env python3
"""
路径规划节点
订阅感知锥桶数据，进行路径规划，发布轨迹
"""

import rclpy
from rclpy.node import Node
from fsd_common_msgs.msg import ConeDetections, Cone, SkidpadGlobalCenterLine
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
import tf2_ros
import math
import numpy as np


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        # 订阅 sim_perception 的锥桶检测。
        # sim_node 发布的锥桶坐标在 base_link 下；本节点在 merge_known_cones()
        # 中转换到 odom 后再规划，保证控制器可直接和 /percep/vehicle_state 对齐。
        self.cone_sub = self.create_subscription(
            ConeDetections,
            '/percep/cone_detections',
            self.cone_callback,
            10
        )

        # 发布规划路径
        self.path_pub = self.create_publisher(
            Path,
            '/planning/trajectory',
            10
        )

        # 发布全局路径
        self.global_path_pub = self.create_publisher(
            SkidpadGlobalCenterLine,
            '/estimation/slam/planned_path',
            10
        )

        # 发布控制指令的旧逻辑已停用：控制由 racecar_control/racecar_control_mpc 负责。
        # self.cmd_pub = self.create_publisher(
        #     '/control/cmd_vel',
        #     10
        # )

        # TF 变换：只在锥桶输入是 base_link/lidar_link 等局部坐标时使用。
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 存储最新数据
        self.current_cones = []
        self.known_cones = []
        self.known_cones_time = []   # 每个已知锥桶的最后观测时间（秒，ROS 时间）
        self.cone_frame_id = ''
        self.last_update_time = self.get_clock().now()

        # 参数
        self.cone_max_age = 8.0      # 锥桶超时剔除（锁定锥桶无需频繁更新）

        self.get_logger().info('路径规划节点启动成功，订阅 /percep/cone_detections (滤波锥桶)，输出 odom 路径')

    def cone_callback(self, msg: ConeDetections):
        """处理感知锥桶数据"""
        self.current_cones = msg.cone_detections
        self.cone_frame_id = msg.header.frame_id
        self.merge_known_cones(msg.cone_detections, msg.header.frame_id)
        self.last_update_time = self.get_clock().now()
        self.get_logger().debug(f'收到 {len(msg.cone_detections)} 个锥桶')

    def timer_callback(self):
        """定时发布路径"""
        if not self.known_cones:
            return

        # 基于感知数据规划路径
        path, curvatures = self.plan_path_from_cones()

        if path and len(path.poses) > 0:
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = "odom"
            self.path_pub.publish(path)

            # 发布全局路径格式
            global_path = SkidpadGlobalCenterLine()
            global_path.global_path = path
            global_path.is_reach_mid = False
            self.global_path_pub.publish(global_path)

            # 发布带曲率的速度控制指令——已禁用，控制由 racecar_control 负责
            # cmd = Twist()
            # avg_curvature = sum(curvatures) / len(curvatures) if curvatures else 0.0
            # cmd.linear.x = self.calculate_speed(avg_curvature)
            # cmd.angular.z = self.calculate_steering(path)
            # self.cmd_pub.publish(cmd)

    def plan_path_from_cones(self) -> tuple:
        """基于感知锥桶数据规划路径，返回(路径, 曲率列表)"""
        path = Path()
        path.header.frame_id = "odom"

        try:
            tf_odom_base = self._tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            vehicle_x = tf_odom_base.transform.translation.x
            vehicle_y = tf_odom_base.transform.translation.y
            q_base = tf_odom_base.transform.rotation
            vehicle_yaw = self._yaw_from_quaternion(q_base.x, q_base.y, q_base.z, q_base.w)
        except Exception as e:
            self.get_logger().warn(f'TF 查询失败: base_link -> odom: {e}')
            return path, []

        # known_cones 在 merge_known_cones() 中已统一缓存为 odom 坐标。
        source_frame = 'odom'
        global_frames = {'odom', 'world', 'map'}
        tf_to_odom = None
        if source_frame not in global_frames:
            try:
                tf_to_odom = self._tf_buffer.lookup_transform(
                    'odom', source_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
            except Exception as e:
                self.get_logger().warn(f'TF 查询失败: {source_frame} -> odom: {e}')
                return path, []

        self.get_logger().info(f'使用 {len(self.known_cones)} 个已知锥桶, frame={source_frame}')
        left_cones = []  # 蓝锥桶
        right_cones = []  # 黄锥桶

        colors_seen = set()
        for cone in self.known_cones:
            c = cone.color.lower().strip().strip("'\"")
            colors_seen.add(c)

            if tf_to_odom is None:
                cx = cone.position.x
                cy = cone.position.y
            else:
                tx = tf_to_odom.transform.translation.x
                ty = tf_to_odom.transform.translation.y
                q = tf_to_odom.transform.rotation
                yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
                cx = tx + cone.position.x * math.cos(yaw) - cone.position.y * math.sin(yaw)
                cy = ty + cone.position.x * math.sin(yaw) + cone.position.y * math.cos(yaw)

            # 创建变换后的锥桶（浅拷贝+改坐标）
            tc = Cone()
            tc.position.x = cx
            tc.position.y = cy
            tc.color = cone.color
            local_x, local_y = self._odom_to_vehicle(cx, cy, vehicle_x, vehicle_y, vehicle_yaw)

            if c in ('blue', 'b'):
                left_cones.append((local_x, local_y, tc))
            elif c in ('yellow', 'y', 'red', 'r'):
                right_cones.append((local_x, local_y, tc))
            else:
                if local_y >= 0.0:
                    left_cones.append((local_x, local_y, tc))
                else:
                    right_cones.append((local_x, local_y, tc))

        self.get_logger().info(f'颜色值: {colors_seen} | 左={len(left_cones)} 右={len(right_cones)}')
        if len(left_cones) < 1 or len(right_cones) < 1:
            return path, []

        # 只使用车辆附近和前方的锥桶，并按车辆前向坐标排序。
        # 这样路径从车头前方开始生成，而不是按世界原点距离乱序。
        left_cones = [item for item in left_cones if item[0] > -1.0]
        right_cones = [item for item in right_cones if item[0] > -1.0]
        left_cones.sort(key=lambda item: item[0])
        right_cones.sort(key=lambda item: item[0])

        if len(left_cones) < 1 or len(right_cones) < 1:
            return path, []

        # 生成中心线：按车辆前向坐标配对左右锥桶。
        # 不能简单用"左第 i 个 + 右第 i 个"，弯道内外侧锥桶数量/间距不同，
        # 下标配对会把不相邻的锥桶连在一起，中心线就会突然拐大弯。
        center_points = self.pair_cones_to_centerline(left_cones, right_cones)
        if len(center_points) < 1:
            return path, []

        # 路径平滑：增加插值点
        smoothed_points = self.smooth_path(center_points)
        if len(smoothed_points) == 1:
            # 只有一对锥桶时，沿车辆当前朝向补一个短前瞻点，让控制器能收到有效 Path。
            x, y = smoothed_points[0]
            smoothed_points.append((
                x + 2.0 * math.cos(vehicle_yaw),
                y + 2.0 * math.sin(vehicle_yaw),
            ))

        # 路径末端延伸：当前方可视锥桶不足时，沿最后一段切向继续延伸，
        # 避免控制器因为路径太短而追逐近点导致震荡或冲出赛道。
        smoothed_points = self.extend_path(smoothed_points, extend_length=4.0,
                                           step=0.15)

        # 计算曲率
        curvatures = self.calculate_curvatures(smoothed_points)

        # 转换为 Path 消息
        for i, (x, y) in enumerate(smoothed_points):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # 计算航向角
            if i < len(smoothed_points) - 1:
                next_x, next_y = smoothed_points[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = self._yaw_from_quaternion(
                    path.poses[-1].pose.orientation.x if path.poses else 0.0,
                    path.poses[-1].pose.orientation.y if path.poses else 0.0,
                    path.poses[-1].pose.orientation.z if path.poses else 0.0,
                    path.poses[-1].pose.orientation.w if path.poses else 1.0,
                )

            # 四元数
            q = Quaternion()
            q.x = 0.0
            q.y = 0.0
            q.z = math.sin(yaw / 2)
            q.w = math.cos(yaw / 2)
            pose.pose.orientation = q

            path.poses.append(pose)

        return path, curvatures

    def merge_known_cones(self, cones, frame_id):
        """把新锥桶转换到 odom 并合并到规划用的全局缓存。

        改进点：
          - 新增时间戳，定期清理长时间未观测到的旧锥桶，防止旧误检累积。
          - 合并时更新对应锥桶的时间戳。
        """
        source_frame = (frame_id or 'base_link').strip().lstrip('/')
        global_frames = {'odom', 'world', 'map'}
        tf_to_odom = None
        if source_frame not in global_frames:
            try:
                tf_to_odom = self._tf_buffer.lookup_transform(
                    'odom', source_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2))
            except Exception as e:
                self.get_logger().warn(f'合并锥桶失败，TF 查询失败: {source_frame} -> odom: {e}')
                return

        now = self.get_clock().now().nanoseconds / 1e9

        # 1. 剔除过期锥桶
        fresh_indices = [
            i for i, t in enumerate(self.known_cones_time)
            if now - t < self.cone_max_age
        ]
        self.known_cones = [self.known_cones[i] for i in fresh_indices]
        self.known_cones_time = [self.known_cones_time[i] for i in fresh_indices]

        for cone in cones:
            if tf_to_odom is None:
                cx = cone.position.x
                cy = cone.position.y
            else:
                tx = tf_to_odom.transform.translation.x
                ty = tf_to_odom.transform.translation.y
                q = tf_to_odom.transform.rotation
                yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
                cx = tx + cone.position.x * math.cos(yaw) - cone.position.y * math.sin(yaw)
                cy = ty + cone.position.x * math.sin(yaw) + cone.position.y * math.cos(yaw)

            c = cone.color.lower().strip().strip("'\"")
            merged = False
            for i, old in enumerate(self.known_cones):
                if old.color.lower().strip().strip("'\"") != c:
                    continue
                if math.hypot(old.position.x - cx, old.position.y - cy) < 0.5:
                    old.position.x = 0.8 * old.position.x + 0.2 * cx
                    old.position.y = 0.8 * old.position.y + 0.2 * cy
                    self.known_cones_time[i] = now
                    merged = True
                    break

            if not merged:
                item = Cone()
                item.position.x = cx
                item.position.y = cy
                item.position.z = 0.0
                item.color = cone.color
                item.pose_confidence = cone.pose_confidence
                item.color_confidence = cone.color_confidence
                self.known_cones.append(item)
                self.known_cones_time.append(now)

    @staticmethod
    def pair_cones_to_centerline(left_cones, right_cones, default_width=3.0):
        """将左右锥桶配对为中心线点。

        输入元素格式为 (local_x, local_y, cone_in_odom)。local_x 是车辆前向
        坐标，用它做配对主依据；local_y 用于过滤异常宽度。

        改进点：
          - 先生成所有合法配对并按综合代价排序，再做全局一对一匹配，避免
            贪心顺序导致的错配。
          - 加大对纵向错位 x_gap 的惩罚，减少弯道中把不相邻锥桶配在一起的
            情况。
          - 赛道宽度阈值收紧，并用可配置的 default_width 做 soft penalty。
        """
        candidates = []

        for lx_local, ly_local, left_cone in left_cones:
            for rx_local, ry_local, right_cone in right_cones:
                x_gap = abs(lx_local - rx_local)
                track_width = abs(ly_local - ry_local)

                # 过滤明显不是同一赛道截面的锥桶对
                if x_gap > 2.5:
                    continue
                if track_width < 1.0 or track_width > 6.0:
                    continue

                # 代价：优先保证左右锥桶在同一车辆纵向截面，其次偏好合理宽度
                score = 2.0 * x_gap + 0.3 * abs(track_width - default_width)
                center_local_x = 0.5 * (lx_local + rx_local)
                center_x = 0.5 * (left_cone.position.x + right_cone.position.x)
                center_y = 0.5 * (left_cone.position.y + right_cone.position.y)
                candidates.append((score, center_local_x, center_x, center_y))

        # 按代价升序排序，然后做全局一对一选择
        candidates.sort(key=lambda item: item[0])
        used_left = set()
        used_right = set()
        center_items = []

        # 需要回溯到原始索引才能保证一对一，因此重新建立带索引的候选
        indexed_candidates = []
        for li, (lx_local, ly_local, left_cone) in enumerate(left_cones):
            for ri, (rx_local, ry_local, right_cone) in enumerate(right_cones):
                x_gap = abs(lx_local - rx_local)
                track_width = abs(ly_local - ry_local)
                if x_gap > 2.5:
                    continue
                if track_width < 1.0 or track_width > 6.0:
                    continue
                score = 2.0 * x_gap + 0.3 * abs(track_width - default_width)
                center_local_x = 0.5 * (lx_local + rx_local)
                center_x = 0.5 * (left_cone.position.x + right_cone.position.x)
                center_y = 0.5 * (left_cone.position.y + right_cone.position.y)
                indexed_candidates.append(
                    (score, li, ri, center_local_x, center_x, center_y))

        indexed_candidates.sort(key=lambda item: item[0])
        for score, li, ri, clx, cx, cy in indexed_candidates:
            if li in used_left or ri in used_right:
                continue
            used_left.add(li)
            used_right.add(ri)
            center_items.append((clx, cx, cy))

        center_items.sort(key=lambda item: item[0])
        return [(x, y) for _, x, y in center_items]

    @staticmethod
    def _yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _odom_to_vehicle(x, y, vehicle_x, vehicle_y, vehicle_yaw):
        dx = x - vehicle_x
        dy = y - vehicle_y
        local_x = dx * math.cos(vehicle_yaw) + dy * math.sin(vehicle_yaw)
        local_y = -dx * math.sin(vehicle_yaw) + dy * math.cos(vehicle_yaw)
        return local_x, local_y

    def smooth_path(self, points: list, step: float = 0.12,
                    window_size: int = 5) -> list:
        """路径平滑：先按固定弧长密化，再做高斯加权移动平均。

        改进点：
          - 旧的线性插值只是"加密度"，并没有真正平滑，弯中仍是折线。
          - 新的方法在密化后用局部加权平均抑制噪声，同时保留路径整体形状。
        """
        if len(points) < 2:
            return points

        pts = np.array(points, dtype=np.float64)

        # 1. 按弧长密化到约 step 间隔
        diffs = np.diff(pts, axis=0)
        seg_lens = np.hypot(diffs[:, 0], diffs[:, 1])
        cum_len = np.concatenate(([0.0], np.cumsum(seg_lens)))
        total_len = cum_len[-1]
        if total_len < 1e-6:
            return points

        num_samples = max(int(round(total_len / step)), 2)
        s_samples = np.linspace(0.0, total_len, num_samples)

        dense = np.empty((num_samples, 2), dtype=np.float64)
        for i, s in enumerate(s_samples):
            idx = int(np.searchsorted(cum_len, s)) - 1
            idx = np.clip(idx, 0, len(pts) - 2)
            t = (s - cum_len[idx]) / max(seg_lens[idx], 1e-9)
            dense[i] = pts[idx] + t * (pts[idx + 1] - pts[idx])

        if len(dense) < 3:
            return [(float(x), float(y)) for x, y in dense]

        # 2. 计算局部曲率，只在弯道段启用高斯平滑
        curvatures = np.zeros(len(dense))
        for i in range(2, len(dense) - 2):
            v1 = dense[i] - dense[i - 2]
            v2 = dense[i + 2] - dense[i]
            cross = abs(v1[0] * v2[1] - v1[1] * v2[0])
            denom = max(np.hypot(v1[0], v1[1]) * np.hypot(v2[0], v2[1]), 1e-9)
            curvatures[i] = cross / denom

        curve_threshold = 0.03  # 低于此曲率的视为直道，跳过平滑
        smoothed = np.copy(dense)
        half = window_size // 2
        for i in range(len(dense)):
            if curvatures[i] < curve_threshold:
                continue  # 直道：保持密化后的原始点
            # 弯道：高斯加权移动平均
            start = max(0, i - half)
            end = min(len(dense), i + half + 1)
            dists = np.arange(start, end) - i
            weights = np.exp(-0.5 * (dists / (half + 1e-9)) ** 2)
            weights = weights / np.sum(weights)
            smoothed[i] = np.sum(dense[start:end] * weights[:, None], axis=0)

        return [(float(x), float(y)) for x, y in smoothed]

    @staticmethod
    def extend_path(points: list, extend_length: float = 4.0,
                    step: float = 0.15) -> list:
        """沿路径末端切向延伸，防止因可视距离不足导致控制器目标点过近。

        改进：
          - 提高触发阈值至 8m，减少不必要的延伸
          - 当路径末端有曲率时沿圆弧延伸，避免虚构直道引导车辆冲出弯道
        """
        if len(points) < 2:
            return points

        pts = np.array(points, dtype=np.float64)
        diffs = np.diff(pts, axis=0)
        seg_lens = np.hypot(diffs[:, 0], diffs[:, 1])
        total_len = np.sum(seg_lens)
        if total_len > 8.0:
            return points

        # 取最后一段方向作为延伸方向
        last_dir = pts[-1] - pts[-2]
        last_len = np.hypot(last_dir[0], last_dir[1])
        if last_len < 1e-6:
            return points
        last_dir = last_dir / last_len

        num_extra = max(1, int(round(extend_length / step)))
        extended = list(points)

        # 取最后几个点估算局部曲率，决定直线延伸还是弧线延伸
        if len(pts) >= 5:
            recent = pts[-5:]
            # 用三点法估算平均曲率
            r0, r1, r2 = recent[0], recent[len(recent)//2], recent[-1]
            v1x, v1y = r1[0] - r0[0], r1[1] - r0[1]
            v2x, v2y = r2[0] - r0[0], r2[1] - r0[1]
            cross = v1x * v2y - v1y * v2x
            dot = v1x * v2x + v1y * v2y
            mid_arc = np.hypot(r2[0] - r0[0], r2[1] - r0[1])
            if mid_arc > 0.3 and abs(cross) > 1e-6:
                curvature = 2.0 * cross / (mid_arc * mid_arc * mid_arc + 1e-9)
                if abs(curvature) > 0.01:
                    # 有曲率：沿圆弧延伸
                    R = 1.0 / curvature
                    heading = math.atan2(last_dir[1], last_dir[0])
                    # 圆心在路径的哪一侧：根据叉积符号
                    side = 1.0 if cross > 0 else -1.0
                    cx = pts[-1, 0] - side * R * math.sin(heading)
                    cy = pts[-1, 1] + side * R * math.cos(heading)
                    for i in range(1, num_extra + 1):
                        arc_angle = heading + side * i * step / R
                        new_pt = np.array([
                            cx + R * math.sin(arc_angle),
                            cy - R * math.cos(arc_angle),
                        ])
                        extended.append((float(new_pt[0]), float(new_pt[1])))
                    return extended

        # 无曲率或路径太短：退化为直线延伸（原逻辑）
        for i in range(1, num_extra + 1):
            new_pt = pts[-1] + i * step * last_dir
            extended.append((float(new_pt[0]), float(new_pt[1])))

        return extended

    def calculate_curvatures(self, points: list) -> list:
        """计算路径曲率列表"""
        if len(points) < 3:
            return [0.0] * len(points)

        curvatures = []
        for i in range(len(points)):
            if i == 0:
                # 第一个点用前两个点计算
                p0 = points[0]
                p1 = points[1]
                kappa = self.compute_curvature_at_point(p0, p0, p1)
            elif i == len(points) - 1:
                # 最后一个点用后两个点计算
                p_prev = points[-2]
                p_curr = points[-1]
                kappa = self.compute_curvature_at_point(p_curr, p_prev, p_curr)
            else:
                # 中间点用三点曲率公式
                p_prev = points[i - 1]
                p_curr = points[i]
                p_next = points[i + 1]
                kappa = self.compute_curvature_at_point(p_curr, p_prev, p_next)
            curvatures.append(kappa)

        return curvatures

    def compute_curvature_at_point(self, point, p_prev, p_next) -> float:
        """计算三点形成的曲率"""
        x0, y0 = point
        x1, y1 = p_prev
        x2, y2 = p_next

        # 向量
        v1x, v1y = x1 - x0, y1 - y0
        v2x, v2y = x2 - x0, y2 - y0

        # 叉积（2D情况下为 z 分量）
        cross = v1x * v2y - v1y * v2x

        # 点积
        dot = v1x * v2x + v1y * v2y

        # 长度
        len1 = math.sqrt(v1x**2 + v1y**2)
        len2 = math.sqrt(v2x**2 + v2y**2)

        if len1 < 0.01 or len2 < 0.01:
            return 0.0

        # 夹角余弦
        cos_angle = dot / (len1 * len2)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        angle = math.acos(cos_angle)

        # 曲率 = 夹角 / 弧长近似
        return angle / ((len1 + len2) / 2 + 0.01)

    def calculate_speed(self, curvature: float) -> float:
        """根据曲率计算速度 - 弯道减速"""
        max_speed = 3.0    # 直道最大速度 m/s
        min_speed = 0.8     # 弯道最小速度 m/s
        curvature_threshold = 0.3  # 曲率阈值

        if curvature < curvature_threshold:
            # 直道或缓弯
            speed = max_speed - curvature * 5.0
        else:
            # 直角弯等急弯
            speed = min_speed + (curvature_threshold / curvature) * (max_speed - min_speed)

        return max(min_speed, min(max_speed, speed))

    def calculate_steering(self, path: Path) -> float:
        """根据路径计算转向角"""
        if len(path.poses) < 2:
            return 0.0

        # 简单比例控制
        # 计算前方的期望方向
        lookahead = 5  # 前看距离（点数）
        if len(path.poses) > lookahead:
            target = path.poses[lookahead]
            current = path.poses[0]

            dx = target.pose.position.x - current.pose.position.x
            dy = target.pose.position.y - current.pose.position.y

            target_yaw = math.atan2(dy, dx)
            current_yaw = math.atan2(
                current.pose.position.y,
                current.pose.position.x
            )

            # 误差
            error = target_yaw - current_yaw

            # 归一化到 [-pi, pi]
            while error > math.pi:
                error -= 2 * math.pi
            while error < -math.pi:
                error += 2 * math.pi

            return error * 0.5  # 比例增益
        return 0.0


def main():
    rclpy.init()
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
