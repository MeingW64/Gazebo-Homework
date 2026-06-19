"""
MPC 采样优化求解器。

每个控制周期：
  1. 投影：车位置投影到路径弧长坐标
  2. 候选：生成 v (6档) x omega (11档) = 66 条控制候选
  3. 模拟：用运动学模型批量前向模拟 horizon 步
  4. 代价：跟踪 + 终端 + 航向 + 平滑 + 前进奖励
  5. 选优：argmin 返回最优 (v, omega)
"""

import numpy as np
from .kinematic_model import simulate_batch


def _cumulative_path(path_xy: np.ndarray) -> np.ndarray:
    """计算路径累积弧长。"""
    diffs = np.diff(path_xy, axis=0)
    cum = np.zeros(len(path_xy), dtype=np.float64)
    cum[1:] = np.cumsum(np.hypot(diffs[:, 0], diffs[:, 1]))
    return cum


def _project_to_path(cx: float, cy: float,
                     path_xy: np.ndarray, cum: np.ndarray) -> float:
    """将车辆位置投影到路径上，返回弧长坐标。"""
    best_d2 = float('inf')
    best_s = 0.0
    for i in range(len(path_xy) - 1):
        p0, p1 = path_xy[i], path_xy[i + 1]
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-9:
            continue
        t = ((cx - p0[0]) * dx + (cy - p0[1]) * dy) / seg2
        t = max(0.0, min(1.0, t))
        px = p0[0] + t * dx
        py = p0[1] + t * dy
        d2 = (cx - px) ** 2 + (cy - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_s = cum[i] + t * (cum[i + 1] - cum[i])
    return best_s


def _interp_xy(path_xy: np.ndarray, cum: np.ndarray,
               target_dist: np.ndarray):
    """沿路径弧长插值，返回 (B, 2) 坐标。"""
    M = len(path_xy)
    target_dist = np.clip(target_dist, 0.0, cum[-1])
    idx = np.searchsorted(cum, target_dist, side='right') - 1
    idx = np.clip(idx, 0, M - 2)
    seg_len = cum[idx + 1] - cum[idx]
    t = (target_dist - cum[idx]) / np.maximum(seg_len, 1e-9)
    t = np.clip(t, 0.0, 1.0)
    px = path_xy[idx, 0] + t * (path_xy[idx + 1, 0] - path_xy[idx, 0])
    py = path_xy[idx, 1] + t * (path_xy[idx + 1, 1] - path_xy[idx, 1])
    return np.column_stack([px, py])


def _local_heading(path_xy: np.ndarray, cum: np.ndarray,
                   dist: np.ndarray):
    """返回弧长 dist 处的路径切向角 (rad)。"""
    M = len(path_xy)
    dist = np.clip(dist, 0.0, cum[-1])
    idx = np.searchsorted(cum, dist, side='right') - 1
    idx = np.clip(idx, 0, M - 2)
    dx = path_xy[idx + 1, 0] - path_xy[idx, 0]
    dy = path_xy[idx + 1, 1] - path_xy[idx, 1]
    return np.arctan2(dy, dx)


def solve(
    current_state: tuple,
    current_control: tuple,
    path_xy: np.ndarray,
    target_speed: float = 2.0,
    horizon: int = 20,
    dt: float = 0.025,
    weight_tracking: float = 1.0,
    weight_heading: float = 2.0,
    weight_smooth: float = 0.1,
    weight_progress: float = 0.5,
) -> tuple:
    """
    MPC 采样优化求解。

    Args:
        current_state: (x, y, theta) 车辆当前状态
        current_control: (v_prev, omega_prev) 上一周期控制量
        path_xy: (M, 2) 路径点
        target_speed: 目标巡航速度
        horizon: 预测步数
        dt: 离散时间步长
        weight_tracking: 跟踪代价权重
        weight_heading: 航向对齐权重
        weight_smooth: 控制平滑权重
        weight_progress: 前进奖励权重

    Returns:
        (v_opt, omega_opt) 最优控制量
    """
    v_prev, omega_prev = current_control
    M = len(path_xy)

    # 路径预处理
    cum = _cumulative_path(path_xy)
    total_len = cum[-1]

    # 车在路径上的弧长投影
    cx, cy, _ = current_state
    start_dist = _project_to_path(cx, cy, path_xy, cum)

    # 候选控制量网格
    v_min = max(0.5, target_speed - 1.5)
    v_candidates = np.linspace(v_min, target_speed, 6, dtype=np.float64)
    omega_candidates = np.arange(-0.75, 0.76, 0.15, dtype=np.float64)

    V, Omega = np.meshgrid(v_candidates, omega_candidates)
    V = V.ravel()
    Omega = Omega.ravel()
    B = len(V)

    # 批量前向模拟
    states = np.tile(np.array(current_state, dtype=np.float64), (B, 1))
    traj = simulate_batch(states, V, Omega, dt, horizon)

    # 代价函数
    cost = np.zeros(B, dtype=np.float64)

    # 跟踪代价：所有候选统一以 target_speed 为参考速度
    for k in range(1, horizon):
        tx = traj[:, k, 0]
        ty = traj[:, k, 1]
        target_dist = np.clip(start_dist + target_speed * k * dt, 0.0, total_len)
        target_xy = _interp_xy(path_xy, cum, target_dist)
        cost += weight_tracking * (
            (tx - target_xy[:, 0]) ** 2 + (ty - target_xy[:, 1]) ** 2
        )

    # 终端跟踪代价：双倍权重防止切出弯道
    end_k = horizon - 1
    target_dist = np.clip(start_dist + target_speed * end_k * dt, 0.0, total_len)
    target_xy = _interp_xy(path_xy, cum, target_dist)
    cost += 2.0 * weight_tracking * (
        (traj[:, end_k, 0] - target_xy[:, 0]) ** 2
        + (traj[:, end_k, 1] - target_xy[:, 1]) ** 2
    )

    # 航向代价
    end_dist = np.clip(start_dist + target_speed * (horizon - 1) * dt, 0.0, total_len)
    local_heading = _local_heading(path_xy, cum, end_dist)
    heading_err = traj[:, -1, 2] - local_heading
    heading_err = np.arctan2(np.sin(heading_err), np.cos(heading_err))
    cost += weight_heading * heading_err * heading_err

    # 平滑代价：抑制控制量突变
    cost += weight_smooth * ((V - v_prev) ** 2 + (Omega - omega_prev) ** 2)

    # 前进奖励：鼓励较高速度，打破 v=0 局部最优
    cost -= weight_progress * V

    best_idx = int(np.argmin(cost))
    return float(V[best_idx]), float(Omega[best_idx])
