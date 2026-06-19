"""
通用工具函数模块

这里放一些所有节点都会用到的辅助函数，比如：
- 四元数转欧拉角（yaw/pitch/roll）
- 欧拉角转四元数
- 角度归一化到 [-pi, pi]
"""

import math


def quaternion_to_yaw(x, y, z, w):
    """
    把四元数 (x, y, z, w) 转换成绕 Z 轴的旋转角 yaw。

    在二维平面上开车，我们最关心的就是 yaw（车头朝向）。
    公式来源：四元数到欧拉角的转换，取 roll=0, pitch=0 时的 yaw。
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def yaw_to_quaternion(yaw):
    """
    把 yaw 角转换回四元数 (x, y, z, w)。
    这里默认 roll=0, pitch=0，只绕 Z 轴旋转。
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = 1.0  # cos(pitch/2)，pitch=0
    sp = 0.0  # sin(pitch/2)，pitch=0
    cr = 1.0  # cos(roll/2)，roll=0
    sr = 0.0  # sin(roll/2)，roll=0

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def normalize_angle(angle):
    """
    把任意角度归一化到 [-pi, pi] 区间。
    处理角度相减时很有用，比如计算两个航向角的差。
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def magnetic_field_to_yaw(mx, my, mz, reference_x, reference_y, reference_z):
    """
    根据地磁场测量值和世界坐标系下的参考磁场，估算 yaw 航向角。

    思路：
    - 当车身水平时（roll≈0, pitch≈0），磁力计水平面分量 (mx, my) 的指向
      就是世界磁场在水平面的投影旋转了 -yaw 后的结果。
    - 参考磁场 (reference_x, reference_y, reference_z) 是世界坐标系下的。
    - 测量磁场是参考磁场被车体坐标系反向旋转后的结果，所以需要取
      “参考方向相对测量方向”的角度，才和 ROS/Gazebo 的 yaw 正方向一致。

    参数：
        mx, my, mz: 磁力计读数，单位特斯拉（T）
        reference_x, reference_y, reference_z: 世界坐标系下地磁场参考向量

    返回：
        yaw: 车头朝向，单位弧度
    """
    # 参考磁场水平分量
    ref_h = math.hypot(reference_x, reference_y)
    if ref_h < 1e-12:
        return 0.0

    # 测量磁场水平分量
    meas_h = math.hypot(mx, my)
    if meas_h < 1e-12:
        return 0.0

    # 参考磁场水平方向的单位向量
    ref_nx = reference_x / ref_h
    ref_ny = reference_y / ref_h

    # 测量磁场水平方向的单位向量
    meas_nx = mx / meas_h
    meas_ny = my / meas_h

    # 参考向量相对于测量向量的角度。
    # 推导：设世界磁场水平方向为 ref_n = (ref_nx, ref_ny)，
    # 车体坐标系下的测量方向为 meas_n = (meas_nx, meas_ny)。
    # 当车体逆时针旋转 yaw 时，世界向量在车体坐标系中的投影为
    # meas_n = R(-yaw) * ref_n，即 ref_n 相对 meas_n 旋转了 +yaw。
    # 因此 atan2(cross(ref_n, meas_n), dot(ref_n, meas_n)) 给出的是 ref 到 meas
    # 的有向角，正好等于 yaw。
    # 注意：早期实现使用 cross(meas, ref) 会得到 -yaw，导致 TF 朝向反了。
    yaw = math.atan2(
        ref_ny * meas_nx - ref_nx * meas_ny,
        ref_nx * meas_nx + ref_ny * meas_ny
    )

    return yaw
