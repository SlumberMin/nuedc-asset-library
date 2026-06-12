#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
陷波滤波器 (Notch Filter) 仿真演示

演示内容：
1. 陷波滤波器频率响应
2. 抑制特定频率干扰
3. 不同 Q 值的陷波效果
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal as sig

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


def design_notch(freq, Q, fs):
    """设计陷波滤波器，返回 (b, a) 系数"""
    w0 = freq / (fs / 2)  # 归一化频率
    b, a = sig.iirnotch(w0, Q)
    return b, a


def demo1_frequency_response():
    """演示1：不同 Q 值的频率响应"""
    fs = 1000.0

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))

    for Q in [1, 5, 10, 30]:
        b, a = design_notch(50, Q, fs)
        w, h = sig.freqz(b, a, worN=2048, fs=fs)
        ax1.plot(w, 20 * np.log10(np.abs(h)), label=f'Q={Q}')
    ax1.set_xlabel('频率 (Hz)')
    ax1.set_ylabel('增益 (dB)')
    ax1.set_title('陷波滤波器频率响应 (中心频率 50Hz)')
    ax1.set_xlim(0, 200)
    ax1.set_ylim(-60, 5)
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 不同中心频率
    for freq in [20, 50, 100, 200]:
        b, a = design_notch(freq, 10, fs)
        w, h = sig.freqz(b, a, worN=2048, fs=fs)
        ax2.plot(w, 20 * np.log10(np.abs(h)), label=f'f={freq}Hz')
    ax2.set_xlabel('频率 (Hz)')
    ax2.set_ylabel('增益 (dB)')
    ax2.set_title('不同中心频率的陷波滤波器 (Q=10)')
    ax2.set_xlim(0, 400)
    ax2.set_ylim(-60, 5)
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('notch_freq_response.png', dpi=150, bbox_inches='tight')
    plt.close('all')


def demo2_interference_suppression():
    """演示2：抑制特定频率干扰"""
    fs = 1000.0
    dt = 1.0 / fs
    T = 1.0
    t = np.arange(0, T, dt)

    # 有用信号：10Hz 低频信号
    signal = np.sin(2 * np.pi * 10 * t)

    # 干扰：50Hz 工频干扰
    interference = 0.5 * np.sin(2 * np.pi * 50 * t + 0.3)

    # 噪声
    noise = 0.1 * np.random.randn(len(t))

    # 合成信号
    noisy = signal + interference + noise

    # 陷波滤波
    b, a = design_notch(50, 10, fs)
    filtered = sig.lfilter(b, a, noisy)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8))

    axes[0].plot(t, signal, 'b-', linewidth=1)
    axes[0].set_ylabel('幅值')
    axes[0].set_title('原始有用信号 (10Hz)')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, noisy, 'r-', alpha=0.6, linewidth=0.5)
    axes[1].set_ylabel('幅值')
    axes[1].set_title('含 50Hz 干扰 + 噪声的信号')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, filtered, 'g-', linewidth=1, label='滤波后')
    axes[2].plot(t, signal, 'b--', linewidth=1, alpha=0.5, label='原始信号')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('幅值')
    axes[2].set_title('陷波滤波后（50Hz 干扰被抑制）')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('notch_suppression.png', dpi=150, bbox_inches='tight')
    plt.close('all')


def demo3_spectrum():
    """演示3：滤波前后频谱对比"""
    fs = 1000.0
    dt = 1.0 / fs
    T = 2.0
    t = np.arange(0, T, dt)

    signal = np.sin(2 * np.pi * 10 * t)
    interference = 0.5 * np.sin(2 * np.pi * 50 * t)
    noisy = signal + interference + 0.1 * np.random.randn(len(t))

    b, a = design_notch(50, 10, fs)
    filtered = sig.lfilter(b, a, noisy)

    N = len(t)
    freqs = np.fft.rfftfreq(N, dt)
    fft_noisy = np.abs(np.fft.rfft(noisy)) / N * 2
    fft_filtered = np.abs(np.fft.rfft(filtered)) / N * 2

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(freqs, fft_noisy, 'r-', alpha=0.5, label='滤波前')
    ax.plot(freqs, fft_filtered, 'g-', label='滤波后')
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('幅值')
    ax.set_title('陷波滤波前后频谱对比')
    ax.set_xlim(0, 100)
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('notch_spectrum.png', dpi=150, bbox_inches='tight')
    plt.close('all')


if __name__ == '__main__':
    print("=== 陷波滤波器仿真演示 ===")
    print("演示1: 频率响应...")
    demo1_frequency_response()
    print("演示2: 干扰抑制...")
    demo2_interference_suppression()
    print("演示3: 频谱对比...")
    demo3_spectrum()
    print("仿真完成！")
