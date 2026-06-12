"""
pid_orangepi5.py - Orange Pi 5 Python版PID控制库

适用于 Orange Pi 5 (RK3588S) 上的Python控制应用
支持 GPIO/PWM/ADC 控制，可与 OpenCV 配合做视觉伺服

使用方法:
    from pid_orangepi5 import PIDController, FuzzyPID, MotorController
    
    pid = PIDController(kp=10, ki=2, kd=0.5)
    output = pid.update(measurement)
"""

import time
import math
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable, List, Tuple
from enum import Enum, auto


# ============================================================
# PID控制器
# ============================================================

class PIDMode(Enum):
    POSITION = auto()
    INCREMENT = auto()


class PIDFeature(Enum):
    NORMAL = auto()
    INTEGRAL_SEP = auto()
    DERIVATIVE_LPF = auto()
    SEGMENTED = auto()


@dataclass
class PIDConfig:
    """PID配置"""
    kp: float = 10.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = -1000.0
    output_max: float = 1000.0
    integral_min: float = -500.0
    integral_max: float = 500.0
    integral_sep_threshold: float = 100.0
    derivative_filter_alpha: float = 0.3
    mode: PIDMode = PIDMode.POSITION
    feature: PIDFeature = PIDFeature.NORMAL


class PIDController:
    """全功能PID控制器"""
    
    def __init__(self, config: Optional[PIDConfig] = None, **kwargs):
        if config:
            self.cfg = config
        else:
            self.cfg = PIDConfig(**kwargs)
        
        self.target = 0.0
        self.output = 0.0
        self._error = 0.0
        self._error_last = 0.0
        self._error_prev = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._output_last = 0.0
        self._measurement_last = 0.0
        self._segments: List[Tuple[float, float, float, float]] = []
    
    def set_target(self, target: float):
        self.target = target
    
    def set_gains(self, kp: float, ki: float, kd: float):
        self.cfg.kp = kp
        self.cfg.ki = ki
        self.cfg.kd = kd
    
    def set_output_limit(self, out_min: float, out_max: float):
        self.cfg.output_min = out_min
        self.cfg.output_max = out_max
    
    def set_segments(self, segments: List[Tuple[float, float, float, float]]):
        """设置分段PID: [(threshold, kp, ki, kd), ...]"""
        self._segments = sorted(segments, key=lambda x: -x[0])
    
    def update(self, measurement: float) -> float:
        """PID计算主函数"""
        self._error = self.target - measurement
        
        # 分段PID
        if self.cfg.feature == PIDFeature.SEGMENTED and self._segments:
            self._update_segments()
        
        if self.cfg.mode == PIDMode.INCREMENT:
            return self._increment_update(measurement)
        else:
            return self._position_update(measurement)
    
    def _position_update(self, measurement: float) -> float:
        kp, ki, kd = self.cfg.kp, self.cfg.ki, self.cfg.kd
        
        # 积分分离
        if self.cfg.feature == PIDFeature.INTEGRAL_SEP:
            if abs(self._error) > self.cfg.integral_sep_threshold:
                ki = 0
        
        p_term = kp * self._error
        
        self._integral += self._error
        self._integral = max(self.cfg.integral_min, min(self.cfg.integral_max, self._integral))
        i_term = ki * self._integral
        
        # 微分项
        if self.cfg.feature == PIDFeature.DERIVATIVE_LPF:
            d_raw = -kd * (measurement - self._measurement_last)
            self._measurement_last = measurement
        else:
            d_raw = kd * (self._error - self._error_last)
        
        self._derivative = (self.cfg.derivative_filter_alpha * d_raw + 
                           (1 - self.cfg.derivative_filter_alpha) * self._derivative)
        d_term = self._derivative
        
        self._error_last = self._error
        self.output = max(self.cfg.output_min, min(self.cfg.output_max, p_term + i_term + d_term))
        return self.output
    
    def _increment_update(self, measurement: float) -> float:
        increment = (self.cfg.kp * (self._error - self._error_last) +
                    self.cfg.ki * self._error +
                    self.cfg.kd * (self._error - 2 * self._error_last + self._error_prev))
        
        self._error_prev = self._error_last
        self._error_last = self._error
        self._output_last += increment
        self.output = max(self.cfg.output_min, min(self.cfg.output_max, self._output_last))
        self._output_last = self.output
        return self.output
    
    def _update_segments(self):
        abs_error = abs(self._error)
        for threshold, kp, ki, kd in self._segments:
            if abs_error >= threshold:
                self.cfg.kp = kp
                self.cfg.ki = ki
                self.cfg.kd = kd
                return
    
    def reset(self):
        self._error = self._error_last = self._error_prev = 0
        self._integral = self._derivative = 0
        self.output = self._output_last = 0
    
    @property
    def status(self) -> dict:
        return {
            'target': self.target, 'error': self._error,
            'output': self.output, 'integral': self._integral,
            'kp': self.cfg.kp, 'ki': self.cfg.ki, 'kd': self.cfg.kd,
        }


# ============================================================
# 模糊PID
# ============================================================

class FuzzyPID:
    """模糊PID控制器"""
    
    # 默认7×7规则表
    RULE_KP = [
        [6, 6, 5, 5, 4, 3, 3],
        [6, 6, 5, 4, 4, 3, 2],
        [5, 5, 5, 4, 3, 2, 2],
        [5, 5, 4, 3, 2, 1, 1],
        [4, 4, 3, 2, 2, 1, 1],
        [4, 3, 2, 1, 1, 1, 0],
        [3, 3, 1, 1, 1, 0, 0],
    ]
    RULE_KI = [
        [0, 0, 1, 1, 2, 3, 3],
        [0, 0, 1, 2, 2, 3, 3],
        [0, 1, 2, 2, 3, 4, 4],
        [1, 1, 2, 3, 4, 5, 5],
        [1, 2, 3, 4, 4, 5, 6],
        [2, 3, 4, 4, 5, 6, 6],
        [3, 3, 4, 5, 5, 6, 6],
    ]
    RULE_KD = [
        [4, 2, 0, 0, 0, 1, 4],
        [4, 2, 0, 1, 1, 2, 3],
        [3, 2, 1, 1, 2, 2, 3],
        [3, 2, 1, 2, 2, 2, 3],
        [3, 3, 2, 2, 3, 3, 3],
        [6, 2, 3, 3, 4, 4, 6],
        [6, 5, 5, 5, 4, 4, 6],
    ]
    
    def __init__(self, kp_base, ki_base, kd_base, delta_kp=None, delta_ki=None, delta_kd=None):
        self.kp_base = kp_base
        self.ki_base = ki_base
        self.kd_base = kd_base
        self.kp = kp_base
        self.ki = ki_base
        self.kd = kd_base
        self.delta_kp_max = delta_kp or kp_base * 0.5
        self.delta_ki_max = delta_ki or ki_base * 0.5
        self.delta_kd_max = delta_kd or kd_base * 0.5
        self.e_scale = 1.0
        self.ec_scale = 1.0
        self.target = 0
        self.output = 0
        self._error_last = 0
        self._integral = 0
        self._d_filter = 0
        self.output_min = -1000
        self.output_max = 1000
    
    def _fuzzify(self, value):
        x = max(-3, min(3, value * self.e_scale))
        centers = [-3, -2, -1, 0, 1, 2, 3]
        width = 1.5
        result = []
        for i, c in enumerate(centers):
            m = max(0, 1 - abs(x - c) / width)
            if m > 0.01:
                result.append((i, m))
        return result
    
    def _defuzzify(self, rule_table, e_sets, ec_sets, delta_max):
        num = den = 0
        for ei, em in e_sets:
            for ej, ec in ec_sets:
                w = min(em, ec)
                out = (rule_table[ei][ej] - 3) * delta_max / 3
                num += w * out
                den += w
        return num / den if den > 1e-6 else 0
    
    def update(self, measurement):
        error = self.target - measurement
        error_dot = error - self._error_last
        
        e_sets = self._fuzzify(error)
        ec_sets = self._fuzzify(error_dot)
        
        dkp = self._defuzzify(self.RULE_KP, e_sets, ec_sets, self.delta_kp_max)
        dki = self._defuzzify(self.RULE_KI, e_sets, ec_sets, self.delta_ki_max)
        dkd = self._defuzzify(self.RULE_KD, e_sets, ec_sets, self.delta_kd_max)
        
        self.kp = max(0, self.kp_base + dkp)
        self.ki = max(0, self.ki_base + dki)
        self.kd = max(0, self.kd_base + dkd)
        
        p = self.kp * error
        self._integral += error
        self._integral = max(-500, min(500, self._integral))
        i = self.ki * self._integral
        d = self.kd * error_dot
        self._d_filter = 0.3 * d + 0.7 * self._d_filter
        
        self._error_last = error
        self.output = max(self.output_min, min(self.output_max, p + i + self._d_filter))
        return self.output


# ============================================================
# 卡尔曼滤波
# ============================================================

class KalmanFilter:
    """一维卡尔曼滤波器(位置+速度)"""
    
    def __init__(self, dt=0.01, process_noise=0.1, measure_noise=1.0):
        self.dt = dt
        self.x = [0, 0]  # [位置, 速度]
        self.P = [[1, 0], [0, 1]]
        self.Q = [[process_noise, 0], [0, process_noise]]
        self.R = measure_noise
    
    def update(self, measurement):
        dt = self.dt
        # 预测
        x0 = self.x[0] + dt * self.x[1]
        x1 = self.x[1]
        p00 = self.P[0][0] + dt * (self.P[1][0] + self.P[0][1] + dt * self.P[1][1]) + self.Q[0][0]
        p01 = self.P[0][1] + dt * self.P[1][1]
        p10 = self.P[1][0] + dt * self.P[1][1]
        p11 = self.P[1][1] + self.Q[1][1]
        
        # 更新
        S = p00 + self.R
        K0 = p00 / S
        K1 = p10 / S
        y = measurement - x0
        self.x = [x0 + K0 * y, x1 + K1 * y]
        self.P = [
            [(1 - K0) * p00, (1 - K0) * p01],
            [p10 - K1 * p00, p11 - K1 * p01]
        ]
        return self.x[0]


# ============================================================
# 电机控制器(Orange Pi 5 GPIO/PWM)
# ============================================================

class MotorController:
    """
    Orange Pi 5 电机控制器
    需要安装: pip install gpiod  (或 OPi.GPIO)
    """
    
    def __init__(self, pwm_pin: int, dir_pin: int = None, 
                 encoder_a: int = None, encoder_b: int = None,
                 ppr: int = 1024, control_freq: int = 1000):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.encoder_a = encoder_a
        self.encoder_b = encoder_b
        self.ppr = ppr
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        
        self.pid = PIDController(kp=10, ki=2, kd=0.5,
                                 output_min=-100, output_max=100)
        self.kf = KalmanFilter(dt=self.dt, process_noise=0.1, measure_noise=1.0)
        
        self._encoder_count = 0
        self._running = False
        self._thread = None
        
        # GPIO初始化(需要根据实际库修改)
        # import gpiod
        # self.chip = gpiod.Chip('0')
        # ...
    
    def set_target(self, speed_rpm: float):
        self.pid.set_target(speed_rpm)
    
    def get_speed_rpm(self, delta_count: int) -> float:
        return (delta_count / self.ppr) * (60.0 / self.dt)
    
    def update(self) -> float:
        """单步控制更新"""
        speed = self.get_speed_rpm(self._encoder_count)
        self._encoder_count = 0
        
        filtered_speed = self.kf.update(speed)
        output = self.pid.update(filtered_speed)
        
        self._set_pwm(output)
        return output
    
    def start(self):
        """启动控制循环"""
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
    
    def _control_loop(self):
        interval = 1.0 / self.control_freq
        while self._running:
            self.update()
            time.sleep(interval)
    
    def _set_pwm(self, duty_percent: float):
        """设置PWM(需要根据实际库实现)"""
        # duty = max(0, min(100, abs(duty_percent)))
        # direction = duty_percent >= 0
        pass


# ============================================================
# 视觉伺服PID(配合OpenCV)
# ============================================================

class VisionServoPID:
    """视觉伺服PID: 根据图像误差控制执行器"""
    
    def __init__(self, image_width: int, image_height: int,
                 kp: float = 0.5, ki: float = 0.01, kd: float = 0.1):
        self.cx = image_width / 2
        self.cy = image_height / 2
        self.pid_x = PIDController(kp=kp, ki=ki, kd=kd,
                                   output_min=-45, output_max=45)
        self.pid_y = PIDController(kp=kp, ki=ki, kd=kd,
                                   output_min=-45, output_max=45)
    
    def set_target(self, x: float, y: float):
        self.pid_x.set_target(x)
        self.pid_y.set_target(y)
    
    def update(self, detected_x: float, detected_y: float) -> Tuple[float, float]:
        """返回(pan_angle, tilt_angle)"""
        pan = self.pid_x.update(detected_x)
        tilt = self.pid_y.update(detected_y)
        return pan, tilt


# ============================================================
# 使用示例
# ============================================================

if __name__ == '__main__':
    # 简单仿真测试
    pid = PIDController(kp=10, ki=2, kd=0.5)
    pid.set_target(100)
    
    value = 0
    for i in range(500):
        output = pid.update(value)
        # 模拟一阶系统
        value += (output - value) * 0.05
        if i % 50 == 0:
            s = pid.status
            print(f"t={i:4d} | target={s['target']:.1f} val={value:.1f} "
                  f"err={s['error']:.2f} out={s['output']:.1f} "
                  f"Kp={s['kp']:.2f} Ki={s['ki']:.2f} Kd={s['kd']:.2f}")
    
    print(f"\n最终值: {value:.2f}, 目标值: 100.00")
