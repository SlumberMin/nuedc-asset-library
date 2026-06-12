#!/usr/bin/env python3
"""
ADC采样仿真
功能：过采样/抽取/噪声整形/Sigma-Delta调制器仿真
适用：电赛ADC类题目（高精度测量、Sigma-Delta ADC等）
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ============== 系统参数 ==============
OSR = 64               # 过采样率
N_BITS_SD = 1          # Sigma-Delta量化器位数
FC_SIGNAL = 1e3        # 信号频率 (Hz)
FS_BASE = 48e3         # 基础采样率 (Hz) = 奈奎斯特率
FS_HIGH = FS_BASE * OSR  # 过采样率
N_SAMPLES = 8192       # 仿真样本数
VREF = 1.0


def sigma_delta_modulator_1st(signal_input, vref=VREF):
    """一阶Sigma-Delta调制器"""
    n = len(signal_input)
    output = np.zeros(n)
    integrator = 0.0
    dac_fb = 0.0

    for i in range(n):
        integrator += (signal_input[i] - dac_fb)
        # 1-bit量化
        if integrator >= 0:
            output[i] = 1.0
        else:
            output[i] = -1.0
        dac_fb = output[i] * vref

    return output


def sigma_delta_modulator_2nd(signal_input, vref=VREF):
    """二阶Sigma-Delta调制器"""
    n = len(signal_input)
    output = np.zeros(n)
    int1 = 0.0
    int2 = 0.0
    dac_fb = 0.0

    for i in range(n):
        x = signal_input[i] - dac_fb
        int1 += x
        int2 += int1 - dac_fb * 0  # 简化
        int2 = int2 + int1
        # 1-bit量化
        if int2 >= 0:
            output[i] = 1.0
        else:
            output[i] = -1.0
        dac_fb = output[i] * vref

    return output


def sinc_decimator(raw_output, decimation_ratio):
    """Sinc³抽取滤波器（CIC）"""
    # 第一级积分
    stage1 = np.cumsum(raw_output)
    # 第二级积分
    stage2 = np.cumsum(stage1)
    # 第三级积分
    stage3 = np.cumsum(stage2)
    # 抽取
    decimated = stage3[::decimation_ratio] / (decimation_ratio ** 3)
    return decimated


def compute_snr_db(signal, noise):
    """计算SNR"""
    ps = np.mean(signal ** 2)
    pn = np.mean(noise ** 2)
    if pn == 0:
        return 100.0
    return 10 * np.log10(ps / pn)


def ideal_adc(signal_v, bits=16, vref=VREF):
    """理想ADC量化"""
    levels = 2 ** bits
    lsb = vref / levels
    code = np.clip(np.round(signal_v / lsb), 0, levels - 1).astype(int)
    return code * lsb


def run_simulation():
    print("=" * 60)
    print("ADC采样仿真系统")
    print("=" * 60)

    # ---- 生成测试信号 ----
    t_high = np.arange(N_SAMPLES) / FS_HIGH
    # 信号：正弦波 + 小噪声
    signal_clean = 0.8 * VREF * np.sin(2 * np.pi * FC_SIGNAL * t_high)
    signal_clean += 0.5 * VREF  # 偏置到 0~1V
    noise_floor = 0.01 * VREF * np.random.randn(N_SAMPLES)
    signal_in = signal_clean + noise_floor

    print(f"过采样率: {OSR}x | 信号频率: {FC_SIGNAL/1e3:.1f} kHz")
    print(f"基础采样率: {FS_BASE/1e3:.1f} kHz | 过采样率: {FS_HIGH/1e3:.1f} kHz")
    print(f"采样点数: {N_SAMPLES}")

    # ---- 方案1：直接Nyquist采样 (16位ADC) ----
    nyquist_samples = signal_in[::OSR]
    nyquist_ideal = signal_clean[::OSR]
    snr_nyquist = compute_snr_db(nyquist_ideal, nyquist_samples - nyquist_ideal)

    # ---- 方案2：一阶Sigma-Delta + 过采样 ----
    sd1_out = sigma_delta_modulator_1st(signal_in / VREF)
    sd1_dec = sinc_decimator(sd1_out, OSR)
    sd1_ideal = signal_clean[::OSR] / VREF
    sd1_noise = sd1_dec[:len(sd1_ideal)] - sd1_ideal
    snr_sd1 = compute_snr_db(sd1_ideal, sd1_noise)

    # ---- 方案3：二阶Sigma-Delta + 过采样 ----
    sd2_out = sigma_delta_modulator_2nd(signal_in / VREF)
    sd2_dec = sinc_decimator(sd2_out, OSR)
    sd2_ideal = signal_clean[::OSR] / VREF
    sd2_noise = sd2_dec[:len(sd2_ideal)] - sd2_ideal
    snr_sd2 = compute_snr_db(sd2_ideal, sd2_noise)

    # ---- 不同OSR对比 ----
    print("\n--- SNR vs 过采样率 ---")
    osr_list = [1, 4, 16, 64, 256]
    snr_list = []
    for osr in osr_list:
        n_pts = min(N_SAMPLES, 8192)
        sig = signal_in[:n_pts]
        sd_test = sigma_delta_modulator_1st(sig / VREF)
        dec_test = sinc_decimator(sd_test, osr)
        ideal_test = signal_clean[:n_pts:osr] / VREF
        noise_test = dec_test[:len(ideal_test)] - ideal_test
        snr_val = compute_snr_db(ideal_test, noise_test)
        snr_list.append(snr_val)
        enob = (snr_val - 1.76) / 6.02
        print(f"  OSR={osr:4d}x: SNR={snr_val:6.1f} dB  ENOB={enob:5.2f} bits")

    print(f"\n[Nyquist 16bit] SNR = {snr_nyquist:.1f} dB")
    print(f"[1阶ΣΔ OSR={OSR}] SNR = {snr_sd1:.1f} dB")
    print(f"[2阶ΣΔ OSR={OSR}] SNR = {snr_sd2:.1f} dB")

    # ---- 绘图 ----
    fig = plt.figure(figsize=(18, 16))
    fig.suptitle("ADC采样仿真系统", fontsize=16, fontweight='bold')
    gs = GridSpec(5, 2, figure=fig, hspace=0.45, wspace=0.3)

    show_n = min(500, N_SAMPLES)
    t_show = t_high[:show_n] * 1e3

    # (0,0) 输入信号
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(t_show, signal_in[:show_n], 'b-', linewidth=0.5, label='输入(含噪声)')
    ax.plot(t_show, signal_clean[:show_n], 'r-', linewidth=1.5, alpha=0.7, label='纯净信号')
    ax.set_title('输入模拟信号')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('电压 (V)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # (0,1) Sigma-Delta调制输出
    ax = fig.add_subplot(gs[0, 1])
    show_sd = min(200, N_SAMPLES)
    ax.step(t_high[:show_sd] * 1e3, sd1_out[:show_sd], 'g-', linewidth=0.8, where='post')
    ax.set_title('1阶ΣΔ调制器输出')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('量化输出')
    ax.set_ylim(-1.5, 1.5)
    ax.grid(True, alpha=0.3)

    # (1,0) ΣΔ噪声整形频谱
    ax = fig.add_subplot(gs[1, 0])
    N_fft = 4096
    freqs = np.fft.rfftfreq(N_fft, 1.0 / FS_HIGH) / 1e3
    sd1_spectrum = np.abs(np.fft.rfft(sd1_out[:N_fft])) / N_fft
    ax.semilogy(freqs, sd1_spectrum + 1e-20, 'r-', label='1阶ΣΔ')
    sd2_spectrum = np.abs(np.fft.rfft(sd2_out[:N_fft])) / N_fft
    ax.semilogy(freqs, sd2_spectrum + 1e-20, 'b-', alpha=0.7, label='2阶ΣΔ')
    ax.set_title('ΣΔ调制输出频谱（噪声整形）')
    ax.set_xlabel('频率 (kHz)')
    ax.set_ylabel('幅度')
    ax.set_xlim(0, FS_HIGH / 2 / 1e3)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3, which='both')

    # (1,1) 抽取后输出对比
    ax = fig.add_subplot(gs[1, 1])
    n_dec = min(200, len(sd1_dec), len(sd2_dec))
    t_dec = np.arange(n_dec) / (FS_HIGH / OSR) * 1e3
    ax.plot(t_dec, sd1_dec[:n_dec], 'r-', linewidth=1.0, label='1阶ΣΔ抽取')
    ax.plot(t_dec, sd2_dec[:n_dec], 'b-', linewidth=1.0, label='2阶ΣΔ抽取')
    ax.plot(t_dec, sd1_ideal[:n_dec], 'k--', linewidth=1.5, alpha=0.5, label='理想')
    ax.set_title(f'抽取滤波输出 (OSR={OSR})')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('归一化幅度')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # (2,0) SNR vs OSR
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(osr_list, snr_list, 'bo-', markersize=8, label='1阶ΣΔ仿真')
    # 理论曲线
    osr_theory = np.array(osr_list, dtype=float)
    snr_1st = 20 * np.log10(osr_theory) + 10 * np.log10(3 / 2) - 3.41
    ax.plot(osr_list, snr_1st, 'r--', label='1阶理论')
    snr_2nd = 20 * np.log10(osr_theory ** 2) + 10 * np.log10(5 / 2) - 12.9
    ax.plot(osr_list, snr_2nd, 'g--', label='2阶理论')
    ax.set_title('SNR vs 过采样率')
    ax.set_xlabel('OSR')
    ax.set_ylabel('SNR (dB)')
    ax.set_xscale('log')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3, which='both')

    # (2,1) 量化噪声分布
    ax = fig.add_subplot(gs[2, 1])
    ax.hist(sd1_noise, bins=80, color='steelblue', alpha=0.7, edgecolor='k',
            density=True, label='1阶ΣΔ')
    ax.hist(sd2_noise, bins=80, color='coral', alpha=0.5, edgecolor='k',
            density=True, label='2阶ΣΔ')
    ax.set_title('抽取后量化噪声分布')
    ax.set_xlabel('误差')
    ax.set_ylabel('概率密度')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # (3,0) CIC抽取滤波器频率响应
    ax = fig.add_subplot(gs[3, 0])
    n_taps = OSR * 3
    cic_impulse = np.zeros(n_taps)
    cic_impulse[:OSR] = 1.0
    cic_h = np.convolve(np.convolve(cic_impulse, cic_impulse), cic_impulse)
    H = np.abs(np.fft.rfft(cic_h, n=4096))
    H /= H[0]
    f_cic = np.linspace(0, 0.5, len(H))
    ax.plot(f_cic, 20 * np.log10(H + 1e-20), 'b-')
    ax.set_title('Sinc³ CIC滤波器频率响应')
    ax.set_xlabel('归一化频率 (×Fs)')
    ax.set_ylabel('增益 (dB)')
    ax.set_ylim(-100, 5)
    ax.grid(True, alpha=0.3)

    # (3,1) ENOB vs OSR
    ax = fig.add_subplot(gs[3, 1])
    enobs = [(s - 1.76) / 6.02 for s in snr_list]
    ax.plot(osr_list, enobs, 'mo-', markersize=8)
    ax.set_title('有效位数 vs 过采样率')
    ax.set_xlabel('OSR')
    ax.set_ylabel('ENOB (bits)')
    ax.set_xscale('log')
    ax.grid(True, alpha=0.3)

    # (4,0) 眼图-style 重叠
    ax = fig.add_subplot(gs[4, 0])
    period_samples = int(FS_HIGH / FC_SIGNAL)
    n_periods = min(50, N_SAMPLES // period_samples)
    for p in range(n_periods):
        start = p * period_samples
        end = start + period_samples
        if end > len(signal_in):
            break
        phase = np.linspace(0, 360, period_samples)
        ax.plot(phase, signal_in[start:end], 'b-', alpha=0.1, linewidth=0.3)
    ax.set_title('信号重叠图（眼图风格）')
    ax.set_xlabel('相位 (°)')
    ax.set_ylabel('电压 (V)')
    ax.grid(True, alpha=0.3)

    # (4,1) 功率谱密度对比
    ax = fig.add_subplot(gs[4, 1])
    n_fft_dec = min(2048, len(sd1_dec))
    psd_dec = np.abs(np.fft.rfft(sd1_dec[:n_fft_dec])) ** 2 / n_fft_dec
    freq_dec = np.fft.rfftfreq(n_fft_dec, 1.0 / (FS_HIGH / OSR)) / 1e3
    ax.semilogy(freq_dec, psd_dec + 1e-20, 'r-', label='1阶ΣΔ')
    psd_dec2 = np.abs(np.fft.rfft(sd2_dec[:n_fft_dec])) ** 2 / n_fft_dec
    ax.semilogy(freq_dec, psd_dec2 + 1e-20, 'b-', label='2阶ΣΔ')
    ax.set_title('抽取后功率谱密度')
    ax.set_xlabel('频率 (kHz)')
    ax.set_ylabel('PSD')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3, which='both')

    plt.tight_layout()
    plt.savefig('adc_sampling_simulation.png', dpi=150, bbox_inches='tight')
    print("\n图像已保存: adc_sampling_simulation.png")
    plt.show()


if __name__ == '__main__':
    run_simulation()
