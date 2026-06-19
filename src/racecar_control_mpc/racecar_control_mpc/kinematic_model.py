"""
运动学自行车模型，批量前向模拟。

状态: [x, y, theta]
控制: [v, omega]

离散化:
  x_{k+1} = x_k + v * cos(theta_k) * dt
  y_{k+1} = y_k + v * sin(theta_k) * dt
  theta_{k+1} = theta_k + omega * dt
"""

import numpy as np


def simulate_batch(
    states: np.ndarray,
    v: np.ndarray,
    omega: np.ndarray,
    dt: float,
    steps: int,
) -> np.ndarray:
    """
    批量前向模拟。

    给定 B 条初始状态和对应的恒定控制量，模拟 steps 步。

    Args:
        states: (B, 3) 初始状态 [x, y, theta]
        v: (B,) 线速度
        omega: (B,) 角速度
        dt: 时间步长
        steps: 模拟步数

    Returns:
        (B, steps, 3) 轨迹数组
    """
    B = len(states)
    traj = np.empty((B, steps, 3), dtype=np.float64)
    x = states[:, 0].copy()
    y = states[:, 1].copy()
    theta = states[:, 2].copy()

    for k in range(steps):
        traj[:, k, 0] = x
        traj[:, k, 1] = y
        traj[:, k, 2] = theta
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += omega * dt

    return traj
