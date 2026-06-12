#!/usr/bin/env python3
"""
数字PID仿真 - 量化效应分析

研究ADC分辨率、DAC分辨率、定点化精度对PID控制性能的影响。
包含:
  1. ADC量化噪声对控制精度的影响
  2. 定点化(定点vs浮点)误差分析
  3. PWM分辨率对输出的影响
  4. 采样周期对离散化误差的影响
  5. 时域/频域对比图
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import Optional

# ============================================================
# 被控对象: 二阶系统 (电机/RLC)
# ============================================================
@dataclass
class SecondOrderPlant:
    """二阶传递函数: wn^2 / (s^2 + 2*zeta*wn*s + wn^2)"""
    wn: float = 20.0        # 自然频率 (rad/s)
    zeta: float = 0.3       # 阻尼比
    dt: float = 0.001       # 仿真步长
    x1: float = 0.0         # 状态1
    x2: float = 0.0         # 状态2

    def update(self, u: float) -> float:
        """状态空间离散化 (前向欧拉)"""
        dx1 = self.x2
        dx2 = -self.wn**2 * self.x1 - 2*self.zeta*self.wn*self.x2 + self.wn**2 * u
        self.x1 += dx1 * self.dt
        self.x2 += dx2 * self.dt
        return self.x1

    def reset(self):
        self.x1 = 0.0
        self.x2 = 0.0


# ============================================================
# 浮点PID (理想参考)
# ============================================================
@dataclass
class FloatPID:
    kp: float = 2.0
    ki: float = 5.0
    kd: float = 0.1
    out_min: float = -1.0
    out_max: float = 1.0
    integral: float = 0.0
    prev_error: float = 0.0
    first_run: bool = True

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        self.integral += self.ki * error * dt
        d_term = 0.0 if self.first_run else self.kd * (error - self.prev_error) / dt
        self.first_run = False
        output = self.kp * error + self.integral + d_term
        output = np.clip(output, self.out_min, self.out_max)
        self.prev_error = error
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True


# ============================================================
# 定点PID (Q16.16)
# ============================================================
class FixedPointPID:
    Q_SHIFT = 16
    Q_ONE = 1 << Q_SHIFT

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = -1.0, out_max: float = 1.0):
        self.kp_q = int(kp * self.Q_ONE)
        self.ki_q = int(ki * self.Q_ONE)
        self.kd_q = int(kd * self.Q_ONE)
        self.out_min_q = int(out_min * self.Q_ONE)
        self.out_max_q = int(out_max * self.Q_ONE)
        self.integral_q = 0
        self.prev_error_q = 0
        self.first_run = True

    def _clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    def _q_mul(self, a, b):
        return int((a * b) >> self.Q_SHIFT)

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        # 转为定点
        dt_q = int(dt * self.Q_ONE)
        error_q = int((setpoint - measurement) * self.Q_ONE)

        # P
        p = self._q_mul(self.kp_q, error_q)

        # I
        self.integral_q += self._q_mul(self._q_mul(self.ki_q, error_q), dt_q)
        i = self.integral_q

        # D
        if self.first_run:
            d = 0
            self.first_run = False
        else:
            d_error_q = error_q - self.prev_error_q
            # 微分: kd * d_error / dt
            if dt_q > 0:
                d = self._q_mul(self.kd_q, (d_error_q << self.Q_SHIFT) // dt_q)
            else:
                d = 0

        output_q = p + i + d
        output_q = self._clamp(output_q, self.out_min_q, self.out_max_q)

        self.prev_error_q = error_q
        return output_q / self.Q_ONE

    def reset(self):
        self.integral_q = 0
        self.prev_error_q = 0
        self.first_run = True


# ============================================================
# 量化器
# ============================================================
def quantize(value: float, bits: int, vref: float = 3.3) -> float:
    """ADC/DAC量化: 将连续值量化为离散级别"""
    levels = 2 ** bits
    step = vref / levels
    return round(value / step) * step


def quantize_signed(value: float, bits: int, range_val: float = 1.0) -> float:
    """有符号量化"""
    levels = 2 ** bits
    step = 2 * range_val / levels
    return round(value / step) * step


# ============================================================
# 仿真主函数
# ============================================================
def simulate(pid_type: str, adc_bits: int, dac_bits: int,
             setpoint: float = 1.0, duration: float = 2.0,
             dt: float = 0.001, noise_std: float = 0.0):
    """
    运行一次仿真
    pid_type: 'float' | 'fixed'
    adc_bits: ADC分辨率
    dac_bits: DAC分辨率
    """
    plant = SecondOrderPlant(wn=20.0, zeta=0.3, dt=dt)
    kp, ki, kd = 2.0, 5.0, 0.1

    if pid_type == 'float':
        pid = FloatPID(kp, ki, kd)
    else:
        pid = FixedPointPID(kp, ki, kd)

    steps = int(duration / dt)
    t = np.zeros(steps)
    y = np.zeros(steps)
    u = np.zeros(steps)
    sp = np.full(steps, setpoint)

    for i in range(steps):
        # 读取传感器值
        y_raw = plant.x1
        # ADC量化
        y_q = quantize(y_raw, adc_bits, vref=3.3)
        # 加噪声
        if noise_std > 0:
            y_q += np.random.normal(0, noise_std)

        # PID计算
        u_val = pid.update(setpoint, y_q, dt)

        # DAC量化 (输出)
        u_q = quantize_signed(u_val, dac_bits, range_val=1.0)

        # 驱动被控对象
        plant.update(u_q)

        t[i] = i * dt
        y[i] = y_raw
        u[i] = u_q

    return t, y, u, sp


# ============================================================
# 绘图
# ============================================================
def run_all_simulations():
    """运行所有仿真实验并绘制结果"""
    plt.rcParams['font.size'] = 10
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # ---- 实验1: ADC位数对控制精度的影响 ----
    ax = axes[0, 0]
    for bits in [8, 10, 12, 16]:
        t, y, _, sp = simulate('float', adc_bits=bits, dac_bits=16)
        ax.plot(t, y, label=f'{bits}-bit ADC')
    ax.plot(t, sp, 'k--', alpha=0.5, label='设定值')
    ax.set_title('ADC分辨率对控制精度的影响')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ---- 实验2: 定点 vs 浮点 ----
    ax = axes[0, 1]
    t, y_f, _, sp = simulate('float', adc_bits=16, dac_bits=16)
    t, y_fx, _, _ = simulate('fixed', adc_bits=16, dac_bits=16)
    ax.plot(t, y_f, label='浮点PID')
    ax.plot(t, y_fx, '--', label='定点PID(Q16.16)')
    ax.plot(t, sp, 'k--', alpha=0.5, label='设定值')
    ax.set_title('浮点 vs 定点PID')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ---- 实验3: 定点误差 ----
    ax = axes[1, 0]
    error = np.abs(y_f - y_fx)
    ax.plot(t, error * 1000)
    ax.set_title('定点化量化误差')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('|误差| × 10³')
    ax.grid(True, alpha=0.3)

    # ---- 实验4: DAC分辨率 ----
    ax = axes[1, 1]
    for bits in [8, 10, 12, 16]:
        t, y, _, sp = simulate('float', adc_bits=16, dac_bits=bits)
        ax.plot(t, y, label=f'{bits}-bit DAC')
    ax.plot(t, sp, 'k--', alpha=0.5, label='设定值')
    ax.set_title('DAC/PWM分辨率的影响')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ---- 实验5: 采样周期影响 ----
    ax = axes[2, 0]
    for dt_ms in [0.5, 1.0, 2.0, 5.0, 10.0]:
        dt = dt_ms / 1000.0
        t, y, _, sp = simulate('float', adc_bits=12, dac_bits=12, dt=dt)
        ax.plot(t, y, label=f'Ts={dt_ms}ms')
    ax.plot(t, sp, 'k--', alpha=0.5, label='设定值')
    ax.set_title('采样周期对性能的影响')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ---- 实验6: 传感器噪声 ----
    ax = axes[2, 1]
    for noise in [0.0, 0.001, 0.005, 0.01]:
        t, y, _, sp = simulate('float', adc_bits=12, dac_bits=12, noise_std=noise)
        ax.plot(t, y, label=f'σ={noise}')
    ax.plot(t, sp, 'k--', alpha=0.5, label='设定值')
    ax.set_title('传感器噪声对控制的影响')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('digital_pid_simulation.png', dpi=150, bbox_inches='tight')
    print("图表已保存: digital_pid_simulation.png")
    plt.close('all')


def print_quantization_analysis():
    """打印量化效应分析报告"""
    print("=" * 60)
    print("数字PID量化效应分析报告")
    print("=" * 60)

    print("\n1. ADC量化精度:")
    for bits in [8, 10, 12, 14, 16]:
        levels = 2 ** bits
        step_3v3 = 3.3 / levels * 1000  # mV
        print(f"   {bits:2d}-bit: {levels:6d} 级, 分辨率 {step_3v3:.3f} mV (Vref=3.3V)")

    print("\n2. 定点Q16.16精度:")
    step = 1.0 / (1 << 16)
    print(f"   分辨率: {step:.8f}")
    print(f"   整数范围: ±{32767}")
    print(f"   动态范围: {20*np.log10(32767/step):.1f} dB")

    print("\n3. 定点Q8.24精度(常用于DSP):")
    step = 1.0 / (1 << 24)
    print(f"   分辨率: {step:.10f}")
    print(f"   整数范围: ±127")

    print("\n4. PWM分辨率 (假设1kHz PWM):")
    for bits in [8, 10, 12]:
        freq = 1000
        levels = 2 ** bits
        actual_freq = freq * levels
        print(f"   {bits}-bit: {levels} 级, 定时器频率 {actual_freq/1e6:.1f} MHz")


if __name__ == '__main__':
    print_quantization_analysis()
    run_all_simulations()
