"""
传感器噪声仿真
对比不同滤波器对传感器噪声的处理效果:
  - 无滤波 (原始信号)
  - 简单滑动平均 (SMA)
  - 指数滑动平均 (EMA)
  - 卡尔曼滤波器 (Kalman)
  - 中值滤波
使用wrappers.py中的滤波器类
"""

import os
import sys
import math
import random
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tests'))
from wrappers import SimpleMA, EMA, IntMA, KalmanFilter


class MedianFilter:
    """中值滤波器"""

    def __init__(self, window=5):
        self.window = window
        self.buffer = []

    def update(self, val):
        self.buffer.append(val)
        if len(self.buffer) > self.window:
            self.buffer.pop(0)
        sorted_buf = sorted(self.buffer)
        n = len(sorted_buf)
        if n % 2 == 1:
            return sorted_buf[n // 2]
        else:
            return (sorted_buf[n // 2 - 1] + sorted_buf[n // 2]) / 2.0

    def reset(self):
        self.buffer = []


def generate_signal(n, signal_type='step', noise_std=1.0):
    """生成测试信号 + 噪声"""
    random.seed(42)
    np.random.seed(42)

    if signal_type == 'step':
        signal = np.array([10.0 if i > n // 3 else 0.0 for i in range(n)])
    elif signal_type == 'sine':
        t = np.arange(n) * 0.01
        signal = 5.0 * np.sin(2 * np.pi * 0.5 * t) + 10.0
    elif signal_type == 'ramp':
        signal = np.linspace(0, 20, n)
    elif signal_type == 'square':
        signal = np.array([10.0 if (i // 50) % 2 == 0 else 0.0 for i in range(n)])
    else:
        signal = np.ones(n) * 10.0

    noise = np.random.normal(0, noise_std, n)
    noisy = signal + noise
    return signal, noisy


def apply_filter(noisy, filter_obj):
    """对带噪信号应用滤波器"""
    filtered = []
    for val in noisy:
        if hasattr(filter_obj, 'step'):
            pos, vel = filter_obj.step(float(val))
            filtered.append(pos)
        elif hasattr(filter_obj, 'update'):
            filtered.append(filter_obj.update(float(val)))
        else:
            filtered.append(float(val))
    return np.array(filtered)


def calc_metrics(original, filtered, skip=50):
    """计算滤波指标"""
    o = original[skip:]
    f = filtered[skip:]
    mse = np.mean((o - f) ** 2)
    mae = np.mean(np.abs(o - f))
    return mse, mae


def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False

    n = 1000
    noise_std = 2.0

    signal_types = ['step', 'sine', 'ramp', 'square']
    titles = ['阶跃信号', '正弦信号', '斜坡信号', '方波信号']

    fig, axes = plt.subplots(4, 2, figsize=(16, 16))
    fig.suptitle('传感器噪声仿真 — 不同滤波器效果对比', fontsize=16, fontweight='bold')

    for idx, (sig_type, title) in enumerate(zip(signal_types, titles)):
        signal, noisy = generate_signal(n, sig_type, noise_std)

        # 应用各种滤波器
        filters = {
            '无滤波': noisy,
            'SMA(w=10)': apply_filter(noisy, SimpleMA(10)),
            'EMA(α=0.1)': apply_filter(noisy, EMA(0.1)),
            'EMA(α=0.3)': apply_filter(noisy, EMA(0.3)),
            'Kalman': apply_filter(noisy, KalmanFilter(dt=0.01, proc_noise=1.0, meas_noise=noise_std ** 2)),
            '中值(w=7)': apply_filter(noisy, MedianFilter(7)),
        }

        # 左图: 时域对比
        ax = axes[idx, 0]
        t = np.arange(n)
        ax.plot(t, signal, 'k--', linewidth=1.5, alpha=0.5, label='真实信号')
        ax.plot(t, noisy, 'r-', alpha=0.2, linewidth=0.5, label='带噪声')
        colors = ['#2196F3', '#4CAF50', '#FF9800', '#9C27B0', '#00BCD4']
        for i, (name, filtered) in enumerate(filters.items()):
            if name == '无滤波':
                continue
            ax.plot(t, filtered, color=colors[(i - 1) % len(colors)],
                    linewidth=1.0, alpha=0.8, label=name)
        ax.set_title(f'{title} - 滤波效果')
        ax.set_xlabel('样本')
        ax.set_ylabel('幅值')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.3)

        # 右图: MSE/MAE对比
        ax = axes[idx, 1]
        names = []
        mses = []
        maes = []
        for name, filtered in filters.items():
            mse, mae = calc_metrics(signal, filtered)
            names.append(name)
            mses.append(mse)
            maes.append(mae)

        x = np.arange(len(names))
        width = 0.35
        bars1 = ax.bar(x - width / 2, mses, width, label='MSE', color='#2196F3', alpha=0.7)
        ax2 = ax.twinx()
        bars2 = ax2.bar(x + width / 2, maes, width, label='MAE', color='#FF9800', alpha=0.7)
        ax.set_xticks(x)
        ax.set_xticklabels(names, fontsize=7, rotation=30, ha='right')
        ax.set_ylabel('MSE')
        ax2.set_ylabel('MAE')
        ax.set_title(f'{title} - 滤波指标')
        ax.legend(loc='upper left', fontsize=7)
        ax2.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(out_dir, 'sensor_noise_filtering.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[OK] 图表已保存: {out_path}")

    # ═══════════════════════════════════════════════════════
    # 额外: 不同噪声强度下的滤波器性能
    # ═══════════════════════════════════════════════════════
    fig2, axes2 = plt.subplots(1, 2, figsize=(14, 6))
    fig2.suptitle('滤波器在不同噪声强度下的性能', fontsize=14, fontweight='bold')

    noise_levels = [0.5, 1.0, 2.0, 5.0, 10.0]
    filter_configs = [
        ('SMA(w=10)', lambda: SimpleMA(10)),
        ('EMA(α=0.1)', lambda: EMA(0.1)),
        ('EMA(α=0.3)', lambda: EMA(0.3)),
        ('Kalman', lambda: KalmanFilter(dt=0.01, proc_noise=1.0, meas_noise=4.0)),
        ('中值(w=7)', lambda: MedianFilter(7)),
    ]

    for name, factory in filter_configs:
        mses = []
        for ns in noise_levels:
            signal, noisy = generate_signal(n, 'sine', ns)
            filt = factory()
            filtered = apply_filter(noisy, filt)
            mse, _ = calc_metrics(signal, filtered)
            mses.append(mse)
        axes2[0].plot(noise_levels, mses, 'o-', label=name)

    axes2[0].set_title('MSE vs 噪声标准差')
    axes2[0].set_xlabel('噪声标准差')
    axes2[0].set_ylabel('MSE')
    axes2[0].legend()
    axes2[0].grid(True, alpha=0.3)

    # 延迟对比
    for name, factory in filter_configs:
        delays = []
        for ns in [2.0]:  # 固定噪声
            signal, noisy = generate_signal(n, 'step', ns)
            filt = factory()
            filtered = apply_filter(noisy, filt)
            # 找到上升沿中点
            mid = (signal.max() + signal.min()) / 2
            true_idx = np.argmax(signal > mid)
            filt_idx = np.argmax(filtered > mid)
            delay = max(0, filt_idx - true_idx)
            delays.append(delay)
        axes2[1].bar(name, delays[0], alpha=0.7)

    axes2[1].set_title('阶跃响应延迟 (样本数)')
    axes2[1].set_xlabel('滤波器')
    axes2[1].set_ylabel('延迟 (样本)')
    axes2[1].tick_params(axis='x', rotation=30)
    axes2[1].grid(True, alpha=0.3)

    plt.tight_layout()
    out_path2 = os.path.join(out_dir, 'sensor_noise_performance.png')
    plt.savefig(out_path2, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[OK] 图表已保存: {out_path2}")


if __name__ == '__main__':
    main()
