#!/usr/bin/env python3
"""
PID参数自动整定仿真
====================
仿真两种自动整定方法:
  1. 继电反馈法 (Relay Feedback)
  2. ZN阶跃响应法

被控对象: 二阶系统 G(s) = K / (s^2 + a*s + b)

用法:
  python pid_auto_tune_simulation.py
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass, field

# ============================================================
#  被控对象模型
# ============================================================

@dataclass
class SecondOrderPlant:
    """二阶被控对象: G(s) = K / (s^2 + a1*s + a0)"""
    K: float = 1.0
    a1: float = 2.0
    a0: float = 10.0
    dt: float = 0.001
    x1: float = 0.0
    x2: float = 0.0

    def reset(self):
        self.x1 = self.x2 = 0.0

    def update(self, u: float) -> float:
        dx1 = self.x2
        dx2 = -self.a0 * self.x1 - self.a1 * self.x2 + self.K * u
        self.x1 += dx1 * self.dt
        self.x2 += dx2 * self.dt
        return self.x1


# ============================================================
#  继电反馈自动整定
# ============================================================

def relay_feedback_tune(plant: SecondOrderPlant, relay_amp: float = 1.0,
                        hysteresis: float = 0.01, timeout: float = 30.0,
                        n_periods: int = 5):
    """
    继电反馈法整定
    返回: Ku, Tu, 以及仿真数据
    """
    dt = plant.dt
    relay_out = relay_amp
    y = 0.0
    prev_y = 0.0

    times, outputs, inputs = [], [], []
    zero_cross_times = []
    peak_max, peak_min = -1e30, 1e30
    t = 0.0
    rising = True
    zc_time = 0.0
    periods, amplitudes = [], []

    while t < timeout:
        # 继电逻辑
        if y > hysteresis:
            relay_out = -relay_amp
        elif y < -hysteresis:
            relay_out = relay_amp

        y = plant.update(relay_out)
        times.append(t)
        outputs.append(y)
        inputs.append(relay_out)

        # 检测过零上升沿
        if prev_y < 0 and y >= 0:
            if rising and zc_time > 0:
                period = t - zc_time
                amp = (peak_max - peak_min) / 2.0
                if period > 0 and amp > 0:
                    periods.append(period)
                    amplitudes.append(amp)
            zc_time = t
            rising = True
            peak_max = peak_min = y
        else:
            peak_max = max(peak_max, y)
            peak_min = min(peak_min, y)

        prev_y = y
        t += dt

        if len(periods) >= n_periods:
            break

    if len(periods) < 2:
        print("继电反馈法: 未检测到足够振荡周期!")
        return None, None, times, outputs, inputs

    Tu = np.mean(periods)
    a = np.mean(amplitudes)
    Ku = 4 * relay_amp / (np.pi * a)

    return Ku, Tu, times, outputs, inputs


def compute_zn_pid(Ku, Tu, rule='classic'):
    """ZN规则计算PID参数"""
    rules = {
        'classic':     (0.6, 0.5, 0.125),
        'pessen':      (0.7, 0.4, 0.15),
        'some_overshoot': (0.33, 0.5, 0.33),
        'no_overshoot':   (0.2, 0.5, 0.33),
    }
    kc, ti_ratio, td_ratio = rules[rule]
    Kp = kc * Ku
    Ti = ti_ratio * Tu
    Td = td_ratio * Tu
    Ki = Kp / Ti if Ti > 0 else 0
    Kd = Kp * Td
    return Kp, Ki, Kd


# ============================================================
#  ZN阶跃响应法
# ============================================================

def zn_step_response_tune(plant: SecondOrderPlant, step_amp: float = 1.0,
                          timeout: float = 10.0):
    """
    ZN阶跃响应法: 从阶跃响应中提取 L (滞后) 和 T (时间常数)
    """
    dt = plant.dt
    plant.reset()
    baseline = plant.x1

    times, outputs = [], []
    t632 = t283 = None
    thresh_283 = baseline + 0.283 * step_amp
    thresh_632 = baseline + 0.632 * step_amp
    t = 0.0
    start_t = 0.0
    started = False

    while t < timeout:
        u = step_amp
        y = plant.update(u)

        if not started:
            start_t = t
            started = True

        times.append(t)
        outputs.append(y)

        if t283 is None and y >= thresh_283:
            t283 = t - start_t
        if t632 is None and y >= thresh_632:
            t632 = t - start_t

        if y >= baseline + 0.99 * step_amp:
            break

        t += dt

    if t632 is None or t283 is None:
        print("阶跃响应法: 未检测到足够响应!")
        return None, None, None, times, outputs

    T_est = 1.5 * (t632 - t283)
    L_est = max(t632 - T_est, dt)
    K_proc = (outputs[-1] - baseline) / step_amp

    return K_proc, L_est, T_est, times, outputs


# ============================================================
#  PID控制仿真
# ============================================================

def simulate_pid(plant: SecondOrderPlant, Kp, Ki, Kd, setpoint=1.0,
                 duration=5.0, disturbance=0.0, dist_time=2.0):
    """使用整定后的PID参数进行闭环仿真"""
    dt = plant.dt
    plant.reset()

    integral = 0.0
    prev_error = 0.0
    y = 0.0

    times, outputs, controls, setpoints = [], [], [], []

    for i in range(int(duration / dt)):
        t = i * dt
        error = setpoint - y

        integral += error * dt
        integral = max(-100, min(100, integral))  # 抗饱和
        derivative = (error - prev_error) / dt if dt > 0 else 0

        u = Kp * error + Ki * integral + Kd * derivative

        # 施加扰动
        if abs(t - dist_time) < dt:
            pass  # 脉冲扰动已注入

        d = disturbance if t >= dist_time else 0.0
        y = plant.update(u + d)

        prev_error = error
        times.append(t)
        outputs.append(y)
        controls.append(u)
        setpoints.append(setpoint)

    return np.array(times), np.array(outputs), np.array(controls), np.array(setpoints)


# ============================================================
#  主程序
# ============================================================

def main():
    # 创建被控对象
    plant = SecondOrderPlant(K=5.0, a1=3.0, a0=20.0, dt=0.001)

    print("=" * 60)
    print("PID参数自动整定仿真")
    print("=" * 60)
    print(f"被控对象: G(s) = {plant.K} / (s^2 + {plant.a1}*s + {plant.a0})")

    # ---------- 方法1: 继电反馈法 ----------
    print("\n--- 方法1: 继电反馈法 ---")
    plant_copy = SecondOrderPlant(K=5.0, a1=3.0, a0=20.0, dt=0.001)
    Ku, Tu, t_relay, y_relay, u_relay = relay_feedback_tune(
        plant_copy, relay_amp=2.0, hysteresis=0.01, timeout=30.0
    )

    if Ku is not None:
        print(f"  临界增益 Ku = {Ku:.4f}")
        print(f"  临界周期 Tu = {Tu:.4f} s")

        for rule_name in ['classic', 'pessen', 'some_overshoot', 'no_overshoot']:
            Kp, Ki, Kd = compute_zn_pid(Ku, Tu, rule_name)
            print(f"  [{rule_name:15s}] Kp={Kp:.4f}, Ki={Ki:.4f}, Kd={Kd:.4f}")

        Kp, Ki, Kd = compute_zn_pid(Ku, Tu, 'classic')
        t_pid, y_pid, u_pid, sp = simulate_pid(
            plant, Kp, Ki, Kd, setpoint=1.0, duration=3.0, disturbance=0.5, dist_time=1.5
        )
    else:
        t_relay = t_pid = y_relay = y_pid = u_pid = sp = None

    # ---------- 方法2: 阶跃响应法 ----------
    print("\n--- 方法2: ZN阶跃响应法 ---")
    plant_copy2 = SecondOrderPlant(K=5.0, a1=3.0, a0=20.0, dt=0.001)
    K_proc, L_est, T_est, t_step, y_step = zn_step_response_tune(
        plant_copy2, step_amp=1.0, timeout=10.0
    )

    if K_proc is not None:
        print(f"  过程增益 K = {K_proc:.4f}")
        print(f"  纯滞后 L   = {L_est:.4f} s")
        print(f"  时间常数 T  = {T_est:.4f} s")
        Ku_s = T_est / (K_proc * L_est) if (K_proc * L_est) > 0 else 0
        Tu_s = 3.33 * L_est
        print(f"  估算 Ku = {Ku_s:.4f}, Tu = {Tu_s:.4f} s")

        Kp2, Ki2, Kd2 = compute_zn_pid(Ku_s, Tu_s, 'classic')
        print(f"  [经典ZN] Kp={Kp2:.4f}, Ki={Ki2:.4f}, Kd={Kd2:.4f}")

        t_pid2, y_pid2, u_pid2, sp2 = simulate_pid(
            plant, Kp2, Ki2, Kd2, setpoint=1.0, duration=3.0, disturbance=0.5, dist_time=1.5
        )
    else:
        t_pid2 = y_pid2 = u_pid2 = sp2 = None

    # ---------- 绘图 ----------
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('PID参数自动整定仿真', fontsize=14)

    # 图1: 继电反馈响应
    if t_relay is not None:
        axes[0, 0].plot(t_relay, y_relay, 'b-', linewidth=0.8)
        axes[0, 0].set_title('继电反馈法 - 系统响应')
        axes[0, 0].set_xlabel('时间 (s)')
        axes[0, 0].set_ylabel('输出')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].annotate(f'Ku={Ku:.2f}, Tu={Tu:.3f}s', xy=(0.05, 0.95),
                           xycoords='axes fraction', va='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # 图2: 阶跃响应
    if t_step is not None:
        axes[0, 1].plot(t_step, y_step, 'r-', linewidth=1.0)
        axes[0, 1].axhline(y=0.632, color='gray', linestyle='--', alpha=0.5)
        axes[0, 1].axhline(y=0.283, color='gray', linestyle=':', alpha=0.5)
        axes[0, 1].set_title('阶跃响应法')
        axes[0, 1].set_xlabel('时间 (s)')
        axes[0, 1].set_ylabel('输出')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].annotate(f'L={L_est:.3f}s, T={T_est:.3f}s', xy=(0.05, 0.95),
                           xycoords='axes fraction', va='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # 图3: PID闭环响应对比
    if t_pid is not None:
        axes[1, 0].plot(t_pid, sp, 'k--', label='设定值', linewidth=1)
        axes[1, 0].plot(t_pid, y_pid, 'b-', label='继电法整定', linewidth=1)
        if y_pid2 is not None:
            axes[1, 0].plot(t_pid2, y_pid2, 'r-', label='阶跃法整定', linewidth=1)
        axes[1, 0].axvline(x=1.5, color='green', linestyle=':', alpha=0.5, label='扰动施加')
        axes[1, 0].set_title('PID闭环响应对比')
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('输出')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

    # 图4: 控制量对比
    if u_pid is not None:
        axes[1, 1].plot(t_pid, u_pid, 'b-', label='继电法整定', linewidth=0.8)
        if u_pid2 is not None:
            axes[1, 1].plot(t_pid2, u_pid2, 'r-', label='阶跃法整定', linewidth=0.8)
        axes[1, 1].set_title('控制量对比')
        axes[1, 1].set_xlabel('时间 (s)')
        axes[1, 1].set_ylabel('控制量')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('pid_auto_tune_result.png', dpi=150, bbox_inches='tight')
    print("\n仿真结果已保存: pid_auto_tune_result.png")
    plt.close('all')


if __name__ == '__main__':
    main()
