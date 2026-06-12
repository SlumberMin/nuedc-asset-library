#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID 性能基准测试
================
标准化测试指标:
  1. 阶跃响应: 上升时间、峰值时间、超调量、调节时间
  2. 扰动抑制: 恢复时间、最大偏差
  3. 跟踪性能: 正弦跟踪误差 (RMS)
  4. 鲁棒性: 参数摄动下的性能退化
  5. 计算耗时: 单次PID更新的平均时间

作者: nuedc-asset-library
"""

import numpy as np
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ── 基础组件 ──────────────────────────────────────────
class PID:
    def __init__(self, Kp, Ki, Kd, dt, u_min=-100, u_max=100):
        self.Kp, self.Ki, self.Kd, self.dt = Kp, Ki, Kd, dt
        self.u_min, self.u_max = u_min, u_max
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return np.clip(u, self.u_min, self.u_max)


class SecondOrderPlant:
    """二阶振荡系统"""
    def __init__(self, wn=10.0, zeta=0.3, dt=0.001):
        self.wn, self.zeta, self.dt = wn, zeta, dt
        self.x1 = 0.0  # 位置
        self.x2 = 0.0  # 速度

    def reset(self):
        self.x1 = self.x2 = 0.0

    def update(self, u, dist=0.0):
        dx2 = -2 * self.zeta * self.wn * self.x2 - self.wn**2 * self.x1 + self.wn**2 * (u + dist)
        self.x2 += dx2 * self.dt
        self.x1 += self.x2 * self.dt
        return self.x1


# ── 指标计算 ──────────────────────────────────────────
def calc_step_metrics(t, y, ref=1.0):
    """计算阶跃响应指标"""
    n = len(y)
    # 上升时间 (10%~90%)
    idx_10 = np.argmax(y >= 0.1 * ref)
    idx_90 = np.argmax(y >= 0.9 * ref)
    t_rise = t[idx_90] - t[idx_10] if idx_90 > idx_10 else float('inf')

    # 峰值时间和超调量
    idx_peak = np.argmax(y)
    t_peak = t[idx_peak]
    overshoot = (y[idx_peak] - ref) / ref * 100 if ref != 0 else 0

    # 调节时间 (2%准则)
    settling_idx = n - 1
    for i in range(n - 1, 0, -1):
        if abs(y[i] - ref) > 0.02 * abs(ref):
            settling_idx = min(i + 1, n - 1)
            break
    t_settle = t[settling_idx]

    # 稳态误差
    ss_error = abs(np.mean(y[-200:]) - ref)

    return {
        't_rise': t_rise,
        't_peak': t_peak,
        'overshoot': overshoot,
        't_settle': t_settle,
        'ss_error': ss_error,
    }


def calc_sine_tracking(t, y, ref_func):
    """正弦跟踪RMS误差"""
    ref = ref_func(t)
    return np.sqrt(np.mean((y - ref)**2))


# ── 测试1: 阶跃响应 ──────────────────────────────────
def test_step_response(Kp, Ki, Kd, dt=0.001, T=2.0):
    n = int(T / dt)
    t = np.arange(n) * dt
    plant = SecondOrderPlant(wn=10, zeta=0.3, dt=dt)
    pid = PID(Kp, Ki, Kd, dt)
    y = np.zeros(n)
    for i in range(n):
        error = 1.0 - plant.x1
        u = pid.update(error)
        y[i] = plant.update(u)
    return t, y, calc_step_metrics(t, y)


# ── 测试2: 扰动抑制 ──────────────────────────────────
def test_disturbance(Kp, Ki, Kd, dt=0.001, T=3.0):
    n = int(T / dt)
    t = np.arange(n) * dt
    plant = SecondOrderPlant(wn=10, zeta=0.3, dt=dt)
    pid = PID(Kp, Ki, Kd, dt)
    y = np.zeros(n)
    dist_mag = 0.5
    for i in range(n):
        error = 1.0 - plant.x1
        dist = dist_mag if int(1.0/dt) <= i < int(1.5/dt) else 0.0
        u = pid.update(error)
        y[i] = plant.update(u, dist=dist)
    # 恢复指标
    dist_start, dist_end = int(1.0/dt), int(1.5/dt)
    max_deviation = np.max(np.abs(y[dist_start:] - 1.0))
    # 恢复到2%的时间
    recovery_time = T
    for i in range(dist_end, n):
        if abs(y[i] - 1.0) < 0.02:
            recovery_time = t[i] - t[dist_end]
            break
    return t, y, {'max_deviation': max_deviation, 'recovery_time': recovery_time}


# ── 测试3: 正弦跟踪 ──────────────────────────────────
def test_sine_tracking(Kp, Ki, Kd, dt=0.001, T=5.0, freq=0.5):
    n = int(T / dt)
    t = np.arange(n) * dt
    ref_func = lambda tt: np.sin(2 * np.pi * freq * tt)
    plant = SecondOrderPlant(wn=10, zeta=0.3, dt=dt)
    pid = PID(Kp, Ki, Kd, dt)
    y = np.zeros(n)
    for i in range(n):
        error = ref_func(t[i]) - plant.x1
        u = pid.update(error)
        y[i] = plant.update(u)
    rms = calc_sine_tracking(t, y, ref_func)
    return t, y, ref_func(t), rms


# ── 测试4: 计算耗时 ──────────────────────────────────
def test_timing(Kp, Ki, Kd, dt, iterations=100000):
    pid = PID(Kp, Ki, Kd, dt)
    errors = np.random.randn(iterations) * 0.1
    start = time.perf_counter()
    for e in errors:
        pid.update(e)
    elapsed = time.perf_counter() - start
    return elapsed / iterations * 1e6  # μs


# ── 测试5: 鲁棒性 ────────────────────────────────────
def test_robustness(Kp, Ki, Kd, dt=0.001):
    """改变被控对象参数,观察性能退化"""
    results = []
    for wn in [5, 8, 10, 12, 15]:
        for zeta in [0.1, 0.2, 0.3, 0.5, 0.7]:
            n = int(2.0 / dt)
            t = np.arange(n) * dt
            plant = SecondOrderPlant(wn=wn, zeta=zeta, dt=dt)
            pid = PID(Kp, Ki, Kd, dt)
            y = np.zeros(n)
            for i in range(n):
                error = 1.0 - plant.x1
                u = pid.update(error)
                y[i] = plant.update(u)
            m = calc_step_metrics(t, y)
            results.append({'wn': wn, 'zeta': zeta, **m})
    return results


# ── 主程序 ──────────────────────────────────────────
if __name__ == '__main__':
    Kp, Ki, Kd = 3.0, 8.0, 0.5
    dt = 0.001

    # ── 1. 阶跃响应 ──
    t, y, step_m = test_step_response(Kp, Ki, Kd, dt)
    print('=== 阶跃响应指标 ===')
    for k, v in step_m.items():
        print(f'  {k:<15s}: {v:.4f}')

    # ── 2. 扰动抑制 ──
    t2, y2, dist_m = test_disturbance(Kp, Ki, Kd, dt)
    print('\n=== 扰动抑制指标 ===')
    for k, v in dist_m.items():
        print(f'  {k:<15s}: {v:.4f}')

    # ── 3. 正弦跟踪 ──
    t3, y3, ref3, rms = test_sine_tracking(Kp, Ki, Kd, dt)
    print(f'\n=== 正弦跟踪 RMS 误差: {rms:.4f} ===')

    # ── 4. 计算耗时 ──
    t_us = test_timing(Kp, Ki, Kd, dt)
    print(f'\n=== 单次PID更新耗时: {t_us:.2f} μs ===')

    # ── 5. 鲁棒性 ──
    rob = test_robustness(Kp, Ki, Kd, dt)
    print('\n=== 鲁棒性矩阵 (超调量%) ===')
    header = "wn\\zeta"
    print(f'  {header:<8s}', end='')
    zetas = sorted(set(r['zeta'] for r in rob))
    for z in zetas:
        print(f'{z:<8.1f}', end='')
    print()
    for wn in sorted(set(r['wn'] for r in rob)):
        print(f'  {wn:<8.0f}', end='')
        for z in zetas:
            val = [r for r in rob if r['wn'] == wn and r['zeta'] == z][0]['overshoot']
            print(f'{val:<8.1f}', end='')
        print()

    # ── 绘图 ──
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 阶跃响应
    ax = axes[0, 0]
    ax.plot(t, y, 'b-', linewidth=1.2)
    ax.axhline(1.0, color='k', linestyle='--', alpha=0.3)
    ax.set_title(f'阶跃响应 (超调={step_m["overshoot"]:.1f}%, ts={step_m["t_settle"]:.3f}s)')
    ax.set_ylabel('输出')
    ax.grid(True, alpha=0.3)

    # 扰动抑制
    ax = axes[0, 1]
    ax.plot(t2, y2, 'b-', linewidth=1.0)
    ax.axhline(1.0, color='k', linestyle='--', alpha=0.3)
    ax.axvspan(1.0, 1.5, alpha=0.1, color='red', label='扰动区间')
    ax.set_title(f'扰动抑制 (最大偏差={dist_m["max_deviation"]:.3f})')
    ax.set_ylabel('输出')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 正弦跟踪
    ax = axes[1, 0]
    ax.plot(t3, ref3, 'k--', label='参考', alpha=0.5)
    ax.plot(t3, y3, 'b-', label='跟踪', linewidth=0.8)
    ax.set_title(f'正弦跟踪 (RMS={rms:.4f})')
    ax.set_ylabel('输出')
    ax.set_xlabel('时间 (s)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 鲁棒性热力图
    ax = axes[1, 1]
    zetas_arr = sorted(set(r['zeta'] for r in rob))
    wns_arr = sorted(set(r['wn'] for r in rob))
    heatmap = np.zeros((len(wns_arr), len(zetas_arr)))
    for r in rob:
        wi = wns_arr.index(r['wn'])
        zi = zetas_arr.index(r['zeta'])
        heatmap[wi, zi] = r['overshoot']
    im = ax.imshow(heatmap, aspect='auto', cmap='RdYlGn_r',
                   extent=[zetas_arr[0], zetas_arr[-1], wns_arr[-1], wns_arr[0]])
    ax.set_xlabel('阻尼比 ζ')
    ax.set_ylabel('固有频率 ωn')
    ax.set_title('鲁棒性热力图 (超调量%)')
    plt.colorbar(im, ax=ax)

    plt.tight_layout()
    plt.savefig('pid_benchmark.png', dpi=150)
    print('\n[OK] 图像已保存: pid_benchmark.png')
    plt.close('all')
