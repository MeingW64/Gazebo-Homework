"""
PID 纵向控制器，含弯道自适应减速。

输入：目标速度、当前速度、前方曲率、时间步长
输出：速度指令 speed_cmd (m/s)

策略：
  - 直道（曲率 < deadband）：以 target_speed 巡航
  - 弯道：v_safe = target_speed / (1 + curve_gain * (|k| - deadband))
  - PID 平滑过渡，加速度限幅，条件积分抗饱和，微分先行
"""


class LongitudinalController:
    """PID 纵向控制器，与横向控制完全独立。"""

    def __init__(
        self,
        kp: float = 1.5,
        ki: float = 0.05,
        kd: float = 0.02,
        max_accel: float = 2.0,
    ):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._max_accel = max_accel

        self._integral = 0.0
        self._prev_speed = 0.0

    def compute(
        self,
        target_speed: float,
        current_speed: float,
        curvature: float,
        dt: float,
        min_speed: float = 0.0,
        curve_gain: float = 2.0,
    ) -> float:
        """
        计算速度指令。

        Args:
            target_speed: 直道巡航速度 (m/s)
            current_speed: 当前车速 (m/s)
            curvature: 路径局部曲率 (1/m)
            dt: 时间间隔 (s)
            min_speed: 最低速度 (m/s)
            curve_gain: 弯道减速强度，越大弯道越慢

        Returns:
            speed_cmd (m/s)，不低于 min_speed
        """
        # 弯道安全速度，含曲率死区
        abs_curv = abs(curvature)
        deadband = 0.02  # R > 50m 视为直道，不减速
        if abs_curv < deadband:
            v_safe = target_speed
        else:
            v_safe = target_speed / (1.0 + curve_gain * (abs_curv - deadband))

        # PID 加速度
        error = v_safe - current_speed

        # 条件积分：加速度未饱和 或 误差反向时才累加
        accel_unclamped = self._kp * error + self._ki * self._integral
        if (accel_unclamped < self._max_accel and accel_unclamped > -self._max_accel) \
                or (error * self._integral < 0):
            self._integral += error * dt
        self._integral = max(-3.0, min(3.0, self._integral))

        # 微分先行：只对测量值求导，避免 v_safe 跳变冲击
        speed_derivative = -(current_speed - self._prev_speed) / dt \
            if dt > 1e-6 else 0.0
        self._prev_speed = current_speed

        accel = self._kp * error + self._ki * self._integral + self._kd * speed_derivative

        # 加速度限幅
        accel = max(-self._max_accel, min(self._max_accel, accel))

        # 积分速度
        speed = current_speed + accel * dt
        speed = max(min_speed, speed)
        return speed

    def reset(self):
        """重置 PID 内部状态。"""
        self._integral = 0.0
        self._prev_speed = 0.0

    @property
    def kp(self) -> float:
        return self._kp

    @kp.setter
    def kp(self, value: float):
        self._kp = value

    @property
    def ki(self) -> float:
        return self._ki

    @ki.setter
    def ki(self, value: float):
        self._ki = value

    @property
    def kd(self) -> float:
        return self._kd

    @kd.setter
    def kd(self, value: float):
        self._kd = value
