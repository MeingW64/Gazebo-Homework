"""
Pure Pursuit 横向控制器。

输入：路径点列 (N,2)、车辆位姿 (x, y, yaw)、当前速度
输出：转向角 steering_angle (rad)

算法步骤：
  1. 自适应前瞻距离 ld = max(min_lookahead, base + speed * ratio)
  2. 在路径上找到距离最接近 ld 且在车辆前方的点
  3. 转到车辆局部坐标系，计算曲率 k = 2 * local_y / dist^2
  4. 转向角 delta = atan(k * wheelbase)
  5. 缩放 * steering_scale，限幅输出
"""

import math
import numpy as np


class LateralController:
    # PurePursuit横向控制器，与纵向控制完全独立。
    # 只负责转向，只操纵前轮转向角

    def __init__(self, wheelbase: float = 0.3):
        self._wheelbase = wheelbase

    def compute(
        self,
        path_xy: np.ndarray,
        vehicle_x: float,
        vehicle_y: float,
        vehicle_yaw: float,
        current_speed: float,
        lookahead_distance: float = 2.0,
        max_steering_angle: float = 0.5,
        min_lookahead: float = 1.5,
        lookahead_ratio: float = 1.0,
        steering_scale: float = 1.0,
    ) -> float:
        """
        计算转向角。
        参数:
            path_xy: 路径点 shape (N, 2)
            vehicle_x, vehicle_y: 车辆世界坐标
            vehicle_yaw: 航向角 (rad)
            current_speed: 当前车速 (m/s)
            lookahead_distance: 基础前瞻距离 (m)
            max_steering_angle: 转向角限幅 (rad)
            min_lookahead: 最小前瞻距离 (m)
            lookahead_ratio: 速度对前瞻的贡献系数(可调)
            steering_scale: 转向角整体缩放系数

        返回:
            steering_angle: 转向角(rad),>0就左转
        """
        if len(path_xy) < 2:    #如果现在规划出来的路径点小于2个，就判断不在转向
            return 0.0

        # 当前的速度越大，选用的前瞻距离就约大，防止冲出去。
        ld = max(min_lookahead, lookahead_distance + current_speed * lookahead_ratio)

        # 寻找前瞻点
        goal = self._find_lookahead_point(path_xy, vehicle_x, vehicle_y, vehicle_yaw, ld)
        if goal is None:
            return 0.0

        # 将点的从全局坐标系转到车辆局部坐标系
        dx = goal[0] - vehicle_x
        dy = goal[1] - vehicle_y
        local_x =  dx * math.cos(vehicle_yaw) + dy * math.sin(vehicle_yaw)
        local_y = -dx * math.sin(vehicle_yaw) + dy * math.cos(vehicle_yaw)

        # 曲率（使用实际距离而非期望 ld，防止弯道内外切）
        goal_dist2 = local_x * local_x + local_y * local_y
        if goal_dist2 < 1e-6:
            return 0.0
        curvature = 2.0 * local_y / goal_dist2

        # Ackermann 几何：曲率转转向角zh 
        steering = math.atan(curvature * self._wheelbase)

        # 限幅
        steering = max(-max_steering_angle, min(max_steering_angle, steering))
        return steering

    @staticmethod
    def _find_lookahead_point(
        path_xy: np.ndarray,
        vx: float, vy: float, vyaw: float,
        lookahead: float,
    ):
        """
        在路径上寻找距离最接近前瞻距离且在车辆前方的点。

        利用上一帧最近点做局部连续搜索，防止 nearest_idx 跳到路径其他位置。
        在所有车前方点中选择距离最接近 lookahead 的点。
        """
        dists = np.hypot(path_xy[:, 0] - vx, path_xy[:, 1] - vy)

        # 在上次最近点附近的窗口内查找
        if hasattr(LateralController, '_prev_nearest_idx'):
            prev = LateralController._prev_nearest_idx
            search_start = max(0, prev - 5)
            search_end = min(len(path_xy), prev + 30)
            local_dists = dists[search_start:search_end]
            start_idx = search_start + int(np.argmin(local_dists))
        else:
            start_idx = int(np.argmin(dists))
        LateralController._prev_nearest_idx = start_idx

        cos_yaw = math.cos(vyaw)
        sin_yaw = math.sin(vyaw)

        best_point = None
        best_diff = float('inf')
        best_forward = None
        best_forward_dist = -1.0

        for i in range(start_idx, len(path_xy)):
            px, py = path_xy[i]
            d = math.hypot(px - vx, py - vy)
            # 过滤车后方的点
            dot = (px - vx) * cos_yaw + (py - vy) * sin_yaw
            if dot <= 0:
                continue
            # 记录最远点作为回退
            if d > best_forward_dist:
                best_forward = (px, py)
                best_forward_dist = d
            # 选择距离最接近 lookahead 的点
            diff = abs(d - lookahead)
            if diff < best_diff:
                best_diff = diff
                best_point = (px, py)

        if best_point is not None:
            return best_point
        return best_forward

    @property
    def wheelbase(self) -> float:
        return self._wheelbase

    @wheelbase.setter
    def wheelbase(self, value: float):
        self._wheelbase = value
