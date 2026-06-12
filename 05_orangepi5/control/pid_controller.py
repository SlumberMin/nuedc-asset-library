"""
PID 控制器 — 支持位置式/增量式，带微分滤波与抗饱和
适用于 Orange Pi 5 (RK3588S) 上的电机、舵机等闭环控制
"""
import time
import enum


class PIDMode(enum.Enum):
    POSITION = "position"   # 位置式
    INCREMENTAL = "incremental"  # 增量式


class AntiWindupMethod(enum.Enum):
    NONE = "none"
    CLAMPING = "clamping"       # 积分限幅
    BACK_CALCULATION = "back_calculation"  # 反馈计算


class PIDController:
    """
    通用 PID 控制器

    Parameters
    ----------
    kp, ki, kd : float
        PID 增益
    mode : PIDMode
        位置式或增量式
    output_min, output_max : float
        输出限幅
    integral_min, integral_max : float
        积分限幅 (抗饱和 clamping)
    deadband : float
        死区范围 (误差绝对值小于此值时输出为0)
    derivative_filter_alpha : float
        微分低通滤波系数 (0~1, 越小滤波越强, 1=无滤波)
    anti_windup : AntiWindupMethod
        抗饱和策略
    back_calculation_kw : float
        反馈计算抗饱和增益 (仅 back_calculation 模式)
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        mode: PIDMode = PIDMode.POSITION,
        output_min: float = -100.0,
        output_max: float = 100.0,
        integral_min: float = -50.0,
        integral_max: float = 50.0,
        deadband: float = 0.0,
        derivative_filter_alpha: float = 0.3,
        anti_windup: AntiWindupMethod = AntiWindupMethod.CLAMPING,
        back_calculation_kw: float = 0.1,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.mode = mode
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.deadband = deadband
        self.derivative_filter_alpha = derivative_filter_alpha
        self.anti_windup = anti_windup
        self.back_calculation_kw = back_calculation_kw

        # 内部状态
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._prev_output = 0.0
        self._last_time = None

        # 用于增量式的上上次误差
        self._prev_prev_error = 0.0

    def reset(self):
        """重置所有内部状态"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._prev_output = 0.0
        self._prev_prev_error = 0.0
        self._last_time = None

    def set_gains(self, kp: float, ki: float, kd: float):
        """在线修改 PID 增益"""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def _clamp(self, value, lo, hi):
        return max(lo, min(hi, value))

    def compute(self, error: float, dt: float = None) -> float:
        """
        计算 PID 输出

        Parameters
        ----------
        error : float
            目标值 - 实际值
        dt : float, optional
            时间间隔(秒)。若为 None 则自动计算。

        Returns
        -------
        float : PID 输出 (已限幅)
        """
        now = time.monotonic()
        if dt is None:
            if self._last_time is None:
                dt = 0.01  # 首次调用的默认 dt
            else:
                dt = now - self._last_time
        self._last_time = now

        if dt <= 0:
            dt = 1e-6

        # 死区处理
        if abs(error) < self.deadband:
            error = 0.0

        if self.mode == PIDMode.POSITION:
            output = self._compute_position(error, dt)
        else:
            output = self._compute_incremental(error, dt)

        # 输出限幅
        output = self._clamp(output, self.output_min, self.output_max)

        # 反馈计算抗饱和
        if self.anti_windup == AntiWindupMethod.BACK_CALCULATION:
            saturated_output = self._clamp(
                output, self.output_min, self.output_max
            )
            self._integral += self.ki * error * dt + \
                self.back_calculation_kw * (saturated_output - output)

        self._prev_error = error
        self._prev_output = output
        self._last_time = now
        return output

    def _compute_position(self, error: float, dt: float) -> float:
        """位置式 PID"""
        # 比例项
        p_term = self.kp * error

        # 积分项 (带 clamping 抗饱和)
        self._integral += error * dt
        if self.anti_windup == AntiWindupMethod.CLAMPING:
            self._integral = self._clamp(
                self._integral, self.integral_min, self.integral_max
            )
        i_term = self.ki * self._integral

        # 微分项 (带一阶低通滤波)
        raw_derivative = (error - self._prev_error) / dt
        filtered_derivative = (
            self.derivative_filter_alpha * raw_derivative +
            (1 - self.derivative_filter_alpha) * self._prev_derivative
        )
        self._prev_derivative = filtered_derivative
        d_term = self.kd * filtered_derivative

        return p_term + i_term + d_term

    def _compute_incremental(self, error: float, dt: float) -> float:
        """增量式 PID"""
        delta_p = self.kp * (error - self._prev_error)
        delta_i = self.ki * error * dt
        delta_d = self.kd * ((error - self._prev_error) - (self._prev_error - self._prev_prev_error)) / dt \
            if dt > 0 else 0.0

        # 微分滤波
        delta_d = (
            self.derivative_filter_alpha * delta_d +
            (1 - self.derivative_filter_alpha) * self._prev_derivative
        )
        self._prev_derivative = delta_d
        self._prev_prev_error = self._prev_error

        delta_output = delta_p + delta_i + delta_d
        return self._prev_output + delta_output

    @property
    def state(self) -> dict:
        """获取当前 PID 内部状态 (用于调试/日志)"""
        return {
            'kp': self.kp, 'ki': self.ki, 'kd': self.kd,
            'integral': self._integral,
            'prev_error': self._prev_error,
            'prev_output': self._prev_output,
        }

    def __repr__(self):
        return (f"PIDController(Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, "
                f"mode={self.mode.value})")
