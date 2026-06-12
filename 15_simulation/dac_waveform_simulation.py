#!/usr/bin/env python3
"""
DAC波形生成仿真
功能：正弦波/方波/三角波生成、量化效应分析、重建滤波器仿真
适用：电赛DAC类题目（信号发生器、波形合成等）
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ============== DAC核心参数 ==============
DAC_BITS = 12          # DAC分辨率（位）
VREF = 3.3             # 参考电压 (V)
FS_DAC = 1e6           # DAC更新率 (Hz)
F_SIGNAL = 1e3         # 信号频率 (Hz)
N_CYCLES = 5           # 显示周期数

def dac_quantize(signal_v, bits=DAC_BITS, vref=VREF):
    """DAC量化：将连续电压映射到离散阶梯"""
    levels = 2 ** bits
    lsb = vref / levels
    code = np.clip(np.round(signal_v / lsb), 0, levels - 1).astype(int)
    return code * lsb  # 重建电压


def generate_waveforms(t, freq, vref):
    """生成三种标准波形（连续值）"""
    # 正弦波: 0 ~ VREF
    sine = (vref / 2) * (1 + np.sin(2 * np.pi * freq * t))
    # 方波
    square = np.where(np.sin(2 * np.pi * freq * t) >= 0, vref, 0.0)
    # 三角波
    triangle = vref * (2 * np.abs(2 * (freq * t % 1) - 1))
    return sine, square, triangle


def reconstruction_filter(dac_output, cutoff_hz, fs):
    """简单一阶RC重建低通滤波器仿真"""
    dt = 1.0 / fs
    rc = 1.0 / (2 * np.pi * cutoff_hz)
    alpha = dt / (rc + dt)
    filtered = np.zeros_like(dac_output)
    filtered[0] = dac_output[0]
    for i in range(1, len(dac_output)):
        filtered[i] = filtered[i - 1] + alpha * (dac_output[i] - filtered[i - 1])
    return filtered


def compute_sinad_db(signal, noise_plus_distortion):
    """计算SINAD (dB)"""
    ps = np.mean(signal ** 2)
    pn = np.mean(noise_plus_distortion ** 2)
    if pn == 0:
        return float('inf')
    return 10 * np.log10(ps / pn)


def compute_enob(sinad_db):
    """由SINAD计算有效位数 ENOB"""
    return (sinad_db - 1.76) / 6.02


def run_simulation():
    print("=" * 60)
    print("DAC波形生成仿真系统")
    print("=" * 60)
    print(f"DAC分辨率: {DAC_BITS}位 | 满量程: {VREF}V")
    print(f"LSB = {VREF / 2**DAC_BITS * 1000:.3f} mV")
    print(f"量化级数: {2**DAC_BITS}")
    print(f"DAC更新率: {FS_DAC/1e3:.0f} kHz | 信号频率: {F_SIGNAL:.0f} Hz")

    # ---- 时间轴 ----
    n_points = int(N_CYCLES * FS_DAC / F_SIGNAL)
    t = np.arange(n_points) / FS_DAC

    # ---- 生成连续波形 ----
    sine_c, square_c, triangle_c = generate_waveforms(t, F_SIGNAL, VREF)

    # ---- DAC量化 ----
    sine_dac = dac_quantize(sine_c)
    square_dac = dac_quantize(square_c)
    triangle_dac = dac_quantize(triangle_c)

    # ---- 重建滤波 ----
    cutoff = F_SIGNAL * 10  # 滤波器截止频率 = 10×信号频率
    sine_filt = reconstruction_filter(sine_dac, cutoff, FS_DAC)
    square_filt = reconstruction_filter(square_dac, cutoff, FS_DAC)
    triangle_filt = reconstruction_filter(triangle_dac, cutoff, FS_DAC)

    # ---- 性能指标 ----
    for name, ideal, dac_out in [("正弦波", sine_c, sine_dac),
                                  ("方波", square_c, square_dac),
                                  ("三角波", triangle_c, triangle_dac)]:
        error = ideal - dac_out
        sinad = compute_sinad_db(ideal - np.mean(ideal), error)
        enob = compute_enob(sinad)
        print(f"\n[{name}] SINAD={sinad:.1f} dB | ENOB={enob:.2f} bits | "
              f"量化误差RMS={np.std(error)*1e6:.1f} µV")

    # ---- 不同分辨率对比 ----
    print("\n--- 不同DAC分辨率对比 (正弦波) ---")
    for bits in [8, 10, 12, 14, 16]:
        dq = dac_quantize(sine_c, bits=bits)
        err = sine_c - dq
        sinad = compute_sinad_db(sine_c - np.mean(sine_c), err)
        print(f"  {bits}位: SINAD={sinad:.1f} dB  ENOB={compute_enob(sinad):.2f} bits  "
              f"LSB={VREF/2**bits*1e6:.1f} µV")

    # ---- 绘图 ----
    fig = plt.figure(figsize=(18, 14))
    fig.suptitle("DAC波形生成仿真", fontsize=16, fontweight='bold')
    gs = GridSpec(4, 3, figure=fig, hspace=0.4, wspace=0.3)

    show = min(n_points, int(5 * FS_DAC / F_SIGNAL))  # 显示5个周期
    t_ms = t[:show] * 1e3

    waveforms = [
        ("正弦波", sine_c, sine_dac, sine_filt),
        ("方波", square_c, square_dac, square_filt),
        ("三角波", triangle_c, triangle_dac, triangle_filt),
    ]

    for col, (name, ideal, dac, filt) in enumerate(waveforms):
        # 行1: 理想 vs DAC输出
        ax = fig.add_subplot(gs[0, col])
        ax.plot(t_ms, ideal[:show], 'b-', alpha=0.5, label='理想')
        ax.step(t_ms, dac[:show], 'r-', linewidth=0.8, label='DAC输出', where='post')
        ax.set_title(f'{name} — DAC量化输出')
        ax.set_xlabel('时间 (ms)')
        ax.set_ylabel('电压 (V)')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        # 行2: 量化误差
        ax = fig.add_subplot(gs[1, col])
        err = ideal[:show] - dac[:show]
        ax.plot(t_ms, err * 1e3, 'g-', linewidth=0.5)
        ax.set_title(f'{name} — 量化误差')
        ax.set_xlabel('时间 (ms)')
        ax.set_ylabel('误差 (mV)')
        ax.grid(True, alpha=0.3)

        # 行3: 重建滤波
        ax = fig.add_subplot(gs[2, col])
        ax.plot(t_ms, ideal[:show], 'b-', alpha=0.5, label='理想')
        ax.plot(t_ms, filt[:show], 'r-', linewidth=1.0, label='重建滤波')
        ax.set_title(f'{name} — 重建滤波后')
        ax.set_xlabel('时间 (ms)')
        ax.set_ylabel('电压 (V)')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # 行4: 频谱分析 + 误差分布
    ax = fig.add_subplot(gs[3, 0])
    N_fft = 4096
    freqs = np.fft.rfftfreq(N_fft, 1.0 / FS_DAC) / 1e3
    spectrum = np.abs(np.fft.rfft(sine_dac[:N_fft])) / N_fft
    spectrum_ideal = np.abs(np.fft.rfft(sine_c[:N_fft])) / N_fft
    ax.plot(freqs, 20 * np.log10(spectrum + 1e-20), 'r-', label='DAC输出')
    ax.plot(freqs, 20 * np.log10(spectrum_ideal + 1e-20), 'b--', alpha=0.5, label='理想')
    ax.set_title('正弦波频谱')
    ax.set_xlabel('频率 (kHz)')
    ax.set_ylabel('幅度 (dB)')
    ax.set_xlim(0, 50)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[3, 1])
    error_all = sine_c - sine_dac
    ax.hist(error_all * 1e6, bins=50, color='steelblue', edgecolor='k', alpha=0.7)
    ax.set_title('量化误差分布')
    ax.set_xlabel('误差 (µV)')
    ax.set_ylabel('计数')
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[3, 2])
    bits_range = [8, 10, 12, 14, 16]
    sinads = []
    for b in bits_range:
        dq = dac_quantize(sine_c, bits=b)
        sinads.append(compute_sinad_db(sine_c - np.mean(sine_c), sine_c - dq))
    ax.plot(bits_range, sinads, 'bo-', markersize=8)
    ax.axhline(y=6.02 * 12 + 1.76, color='r', linestyle='--', alpha=0.5, label='理论极限')
    ax.set_title('SINAD vs DAC分辨率')
    ax.set_xlabel('DAC位数')
    ax.set_ylabel('SINAD (dB)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('dac_waveform_simulation.png', dpi=150, bbox_inches='tight')
    print("\n图像已保存: dac_waveform_simulation.png")
    plt.show()


if __name__ == '__main__':
    run_simulation()
