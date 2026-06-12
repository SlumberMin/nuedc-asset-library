#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
二阶滤波器 (Biquad) 仿真演示

演示内容：
1. 各类型滤波器频率响应
2. 实际信号滤波效果
3. 级联滤波器
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal as sig

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


def demo1_all_types():
    """演示1：各类 Biquad 滤波器频率响应"""
    fs = 1000.0

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))

    configs = [
        ('低通 (LPF)', sig.butter(2, 100, btype='low', fs=fs)),
        ('高通 (HPF)', sig.butter(2, 100, btype='high', fs=fs)),
        ('带通 (BPF)', sig.butter(2, [80, 120], btype='band', fs=fs)),
        ('带阻/陷波 (Notch)', sig.iirnotch(50, 10, fs=fs)),
    ]

    for idx, (title, (b, a)) in enumerate(configs):
        ax = axes[idx // 2][idx % 2]
        w, h = sig.freqz(b, a, worN=4096, fs=fs)
        ax.plot(w, 20 * np.log10(np.abs(h)), 'b-', linewidth=1.5)
        ax.set_xlabel('频率 (Hz)')
        ax.set_ylabel('增益 (dB)')
        ax.set_title(title)
        ax.set_xlim(0, fs / 2)
        ax.set_ylim(-60, 5)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=-3, color='r', linestyle='--', alpha=0.5, linewidth=0.8)

    plt.suptitle('Biquad 滤波器各类型频率响应', fontsize=14)
    plt.tight_layout()
    plt.savefig('biquad_freq_response.png', dpi=150, bbox_inches='tight')
    plt.close('all')


def demo2_signal_filtering():
    """演示2：实际信号滤波"""
    fs = 1000.0
    dt = 1.0 / fs
    T = 0.5
    t = np.arange(0, T, dt)

    # 构造信号：5Hz + 50Hz + 200Hz + 噪声
    sig_clean = np.sin(2 * np.pi * 5 * t)
    s50 = 0.3 * np.sin(2 * np.pi * 50 * t)
    s200 = 0.2 * np.sin(2 * np.pi * 200 * t)
    noise = 0.05 * np.random.randn(len(t))
    mixed = sig_clean + s50 + s200 + noise

    # 低通 20Hz
    b_lpf, a_lpf = sig.butter(4, 20, btype='low', fs=fs)
    out_lpf = sig.filtfilt(b_lpf, a_lpf, mixed)

    # 带通 3~8Hz
    b_bpf, a_bpf = sig.butter(4, [3, 8], btype='band', fs=fs)
    out_bpf = sig.filtfilt(b_bpf, a_bpf, mixed)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8))

    axes[0].plot(t, mixed, 'r-', alpha=0.6, linewidth=0.5)
    axes[0].plot(t, sig_clean, 'b--', linewidth=1, label='原始5Hz信号')
    axes[0].set_title('混合信号（5Hz + 50Hz + 200Hz + 噪声）')
    axes[0].set_ylabel('幅值')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, out_lpf, 'g-', linewidth=1.5, label='LPF 20Hz 滤波')
    axes[1].plot(t, sig_clean, 'b--', linewidth=1, alpha=0.5, label='参考')
    axes[1].set_title('低通滤波器（截止20Hz）')
    axes[1].set_ylabel('幅值')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, out_bpf, 'm-', linewidth=1.5, label='BPF 3-8Hz 滤波')
    axes[2].plot(t, sig_clean, 'b--', linewidth=1, alpha=0.5, label='参考')
    axes[2].set_title('带通滤波器（3-8Hz）')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('幅值')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('biquad_filtering.png', dpi=150, bbox_inches='tight')
    plt.close('all')


def demo3_cascade():
    """演示3：级联滤波器（高阶滤波）"""
    fs = 1000.0
    dt = 1.0 / fs
    T = 1.0
    t = np.arange(0, T, dt)

    signal = np.sin(2 * np.pi * 10 * t)
    noise = 0.3 * np.random.randn(len(t))
    noisy = signal + noise

    # 2阶低通
    b2, a2 = sig.butter(1, 30, btype='low', fs=fs)
    out_2 = sig.lfilter(b2, a2, noisy)

    # 4阶（级联2个2阶）
    b4, a4 = sig.butter(2, 30, btype='low', fs=fs)
    out_4 = sig.lfilter(b4, a4, noisy)

    # 8阶（级联4个2阶）
    b8, a8 = sig.butter(4, 30, btype='low', fs=fs)
    out_8 = sig.lfilter(b8, a8, noisy)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, noisy, 'gray', alpha=0.3, linewidth=0.5, label='含噪信号')
    ax.plot(t, out_2, 'r-', linewidth=1, label='2阶 (1级Biquad)')
    ax.plot(t, out_4, 'g-', linewidth=1, label='4阶 (2级Biquad)')
    ax.plot(t, out_8, 'b-', linewidth=1.5, label='8阶 (4级Biquad)')
    ax.plot(t, signal, 'k--', linewidth=1, alpha=0.5, label='原始信号')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('幅值')
    ax.set_title('级联 Biquad 滤波器效果对比')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('biquad_cascade.png', dpi=150, bbox_inches='tight')
    plt.close('all')


if __name__ == '__main__':
    print("=== Biquad 滤波器仿真演示 ===")
    print("演示1: 各类型频率响应...")
    demo1_all_types()
    print("演示2: 信号滤波...")
    demo2_signal_filtering()
    print("演示3: 级联滤波...")
    demo3_cascade()
    print("仿真完成！")
