#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
信号调理仿真 - 放大/滤波/检波/采样/数字化
==========================================
功能：
  - 信号源生成（正弦、调幅、调频、脉冲、噪声）
  - 放大器仿真（运放增益、带宽、压摆率、噪声）
  - 滤波器设计与仿真（低通/高通/带通/带阻、IIR/FIR）
  - 包络检波（峰值检波、同步检波）
  - 采样与量化（奈奎斯特、混叠、量化噪声）
  - 信号链端到端分析

适用场景：传感器前端设计、ADC前端信号调理、通信接收机
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal as sig
from scipy.fft import fft, fftfreq
from dataclasses import dataclass
from typing import Tuple, Optional
import warnings
warnings.filterwarnings('ignore')

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ======================== 信号源 ========================

class SignalGenerator:
    """信号发生器"""

    def __init__(self, fs: float = 10000, duration: float = 0.05):
        self.fs = fs
        self.duration = duration
        self.t = np.arange(0, duration, 1/fs)

    def sine(self, freq: float, amplitude: float = 1.0, phase: float = 0) -> np.ndarray:
        return amplitude * np.sin(2*np.pi*freq*self.t + phase)

    def amplitude_modulated(self, carrier_freq: float, mod_freq: float,
                            mod_depth: float = 0.8) -> np.ndarray:
        """调幅信号"""
        modulating = np.sin(2*np.pi*mod_freq*self.t)
        carrier = np.sin(2*np.pi*carrier_freq*self.t)
        return (1 + mod_depth * modulating) * carrier

    def frequency_modulated(self, carrier_freq: float, mod_freq: float,
                            freq_dev: float = 500) -> np.ndarray:
        """调频信号"""
        phase = 2*np.pi*carrier_freq*self.t + freq_dev/mod_freq * np.sin(2*np.pi*mod_freq*self.t)
        return np.sin(phase)

    def pulse_train(self, freq: float, duty: float = 0.5, amplitude: float = 1.0) -> np.ndarray:
        """脉冲串"""
        period = 1.0 / freq
        return amplitude * ((self.t % period) < duty * period).astype(float)

    def gaussian_noise(self, std: float = 0.1) -> np.ndarray:
        return np.random.randn(len(self.t)) * std

    def colored_noise(self, std: float = 0.1, alpha: float = 1.0) -> np.ndarray:
        """1/f^alpha 有色噪声"""
        white = np.random.randn(len(self.t))
        if alpha > 0:
            # 简化：通过滤波实现
            b, a = sig.butter(1, 0.1)
            colored = sig.filtfilt(b, a, white)
            return colored / np.std(colored) * std
        return white * std


# ======================== 放大器模型 ========================

class Amplifier:
    """运算放大器模型"""

    def __init__(self, gain: float = 100.0, bandwidth: float = 1e6,
                 slew_rate: float = 10e6, noise_density: float = 5e-9,
                 input_offset: float = 0.0, cmrr: float = 100.0):
        self.gain = gain                     # 电压增益 (V/V)
        self.bandwidth = bandwidth           # -3dB带宽 (Hz)
        self.slew_rate = slew_rate           # 压摆率 (V/s)
        self.noise_density = noise_density   # 输入噪声密度 (V/√Hz)
        self.input_offset = input_offset     # 输入失调电压 (V)
        self.cmrr = cmrr                     # 共模抑制比 (dB)

    def process(self, x: np.ndarray, fs: float) -> np.ndarray:
        """处理信号"""
        # 1. 添加输入噪声
        noise_rms = self.noise_density * np.sqrt(fs/2)
        x_noisy = x + np.random.randn(len(x)) * noise_rms

        # 2. 添加失调
        x_offset = x_noisy + self.input_offset

        # 3. 增益+带宽限制（一阶低通）
        fc = self.bandwidth / self.gain  # 增益带宽积
        if fc < fs/2:
            b, a = sig.butter(1, fc/(fs/2), 'low')
            x_filtered = sig.filtfilt(b, a, x_offset)
        else:
            x_filtered = x_offset

        # 4. 放大
        y = self.gain * x_filtered

        # 5. 压摆率限制
        max_step = self.slew_rate / fs
        y_slew = np.zeros_like(y)
        y_slew[0] = y[0]
        for i in range(1, len(y)):
            dy = y[i] - y_slew[i-1]
            if abs(dy) > max_step:
                y_slew[i] = y_slew[i-1] + np.sign(dy) * max_step
            else:
                y_slew[i] = y[i]

        return y_slew


# ======================== 滤波器设计 ========================

class FilterDesigner:
    """滤波器设计工具"""

    @staticmethod
    def design_lowpass(order: int, cutoff: float, fs: float) -> Tuple:
        return sig.butter(order, cutoff/(fs/2), 'low', output='ba')

    @staticmethod
    def design_highpass(order: int, cutoff: float, fs: float) -> Tuple:
        return sig.butter(order, cutoff/(fs/2), 'high', output='ba')

    @staticmethod
    def design_bandpass(order: int, low: float, high: float, fs: float) -> Tuple:
        return sig.butter(order, [low/(fs/2), high/(fs/2)], 'band', output='ba')

    @staticmethod
    def design_bandstop(order: int, low: float, high: float, fs: float) -> Tuple:
        return sig.butter(order, [low/(fs/2), high/(fs/2)], 'bandstop', output='ba')

    @staticmethod
    def design_chebyshev(order: int, cutoff: float, fs: float,
                         ripple: float = 1.0) -> Tuple:
        return sig.cheby1(order, ripple, cutoff/(fs/2), 'low', output='ba')

    @staticmethod
    def fir_window(order: int, cutoff: float, fs: float,
                   window: str = 'hamming') -> Tuple:
        nyq = fs / 2
        return sig.firwin(order+1, cutoff/nyq, window=window), [1.0]

    @staticmethod
    def frequency_response(b, a, fs: float, n_points: int = 1024) -> Tuple:
        w, h = sig.freqz(b, a, worN=n_points, fs=fs)
        return w, h


# ======================== 包络检波 ========================

class EnvelopeDetector:
    """包络检波器"""

    @staticmethod
    def peak_detector(x: np.ndarray, attack: float = 0.01,
                      release: float = 0.1, fs: float = 10000) -> np.ndarray:
        """峰值包络检波"""
        att_coeff = 1 - np.exp(-2*np.pi*attack*fs) if attack > 0 else 1.0
        rel_coeff = 1 - np.exp(-2*np.pi*release*fs) if release > 0 else 1.0

        envelope = np.zeros_like(x)
        envelope[0] = abs(x[0])
        for i in range(1, len(x)):
            val = abs(x[i])
            if val > envelope[i-1]:
                envelope[i] = envelope[i-1] + att_coeff * (val - envelope[i-1])
            else:
                envelope[i] = envelope[i-1] + rel_coeff * (val - envelope[i-1])
        return envelope

    @staticmethod
    def synchronous_detector(x: np.ndarray, carrier_freq: float,
                              fs: float, bandwidth: float = 100) -> np.ndarray:
        """同步检波"""
        t = np.arange(len(x)) / fs
        lo = np.sin(2*np.pi*carrier_freq*t)
        mixed = x * lo
        # 低通滤波
        b, a = sig.butter(4, bandwidth/(fs/2), 'low')
        return sig.filtfilt(b, a, mixed) * 2  # ×2补偿混频衰减

    @staticmethod
    def hilbert_envelope(x: np.ndarray) -> np.ndarray:
        """希尔伯特变换包络"""
        analytic = sig.hilbert(x)
        return np.abs(analytic)


# ======================== 采样与量化 ========================

class ADCSimulator:
    """ADC采样与量化仿真"""

    def __init__(self, resolution: int = 12, vref: float = 3.3,
                 fs_adc: float = 1000, dither: bool = True):
        self.resolution = resolution
        self.vref = vref
        self.fs_adc = fs_adc
        self.dither = dither  # 抖动减少量化失真
        self.lsb = vref / (2**resolution)

    def sample(self, x_continuous: np.ndarray, fs_continuous: float) -> np.ndarray:
        """降采样"""
        ratio = int(fs_continuous / self.fs_adc)
        if ratio <= 1:
            return x_continuous
        return x_continuous[::ratio]

    def quantize(self, x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """量化"""
        # 抖动
        if self.dither:
            x = x + np.random.uniform(-0.5*self.lsb, 0.5*self.lsb, len(x))
        # 量化
        levels = 2**self.resolution
        x_clipped = np.clip(x, 0, self.vref)
        x_quantized = np.round(x_clipped / self.lsb) * self.lsb
        quantization_error = x - x_quantized
        return x_quantized, quantization_error

    def snr_theoretical(self) -> float:
        """理论信噪比 (dB)"""
        return 6.02 * self.resolution + 1.76  # 理想ADC

    def process(self, x: np.ndarray, fs_continuous: float) -> dict:
        """完整ADC处理链"""
        x_sampled = self.sample(x + self.vref/2, fs_continuous)  # 偏置到单极性
        x_quantized, q_error = self.quantize(x_sampled)

        # 实际SNR
        if len(x_sampled) > 0:
            signal_power = np.var(x_sampled)
            noise_power = np.var(q_error)
            snr_actual = 10*np.log10(signal_power/noise_power) if noise_power > 0 else np.inf
        else:
            snr_actual = 0

        return {
            'sampled': x_sampled,
            'quantized': x_quantized,
            'quant_error': q_error,
            'snr_theoretical': self.snr_theoretical(),
            'snr_actual': snr_actual
        }


# ======================== 频谱分析 ========================

def compute_spectrum(x: np.ndarray, fs: float) -> Tuple:
    """计算功率谱"""
    N = len(x)
    X = fft(x)
    freqs = fftfreq(N, 1/fs)
    # 只取正频率
    pos_mask = freqs >= 0
    freqs = freqs[pos_mask]
    magnitude = 2.0/N * np.abs(X[pos_mask])
    psd = magnitude**2
    return freqs, magnitude, psd


# ======================== 仿真与可视化 ========================

def run_signal_conditioning():
    """信号调理完整仿真"""
    print("=" * 60)
    print("信号调理仿真系统")
    print("=" * 60)

    fs = 100000  # 100kHz采样率
    duration = 0.01  # 10ms
    gen = SignalGenerator(fs=fs, duration=duration)

    # 信号生成
    print("\n[1] 信号源生成...")
    f_signal = 1000  # 1kHz信号
    f_noise = 15000  # 15kHz干扰
    t = gen.t

    clean_signal = gen.sine(f_signal, amplitude=0.02)
    interference = gen.sine(f_noise, amplitude=0.005)
    noise = gen.gaussian_noise(std=0.003)
    raw_signal = clean_signal + interference + noise

    # AM信号
    am_signal = gen.amplitude_modulated(carrier_freq=20000, mod_freq=500, mod_depth=0.8)

    print(f"   信号频率: {f_signal} Hz")
    print(f"   干扰频率: {f_noise} Hz")
    print(f"   采样率: {fs} Hz")
    print(f"   信号长度: {len(t)} 点")

    # 放大
    print("\n[2] 放大器处理...")
    amp = Amplifier(gain=100, bandwidth=50000, slew_rate=5e6, noise_density=8e-9)
    amplified = amp.process(raw_signal, fs)
    print(f"   增益: {amp.gain} V/V ({20*np.log10(amp.gain):.1f} dB)")
    print(f"   -3dB带宽: {amp.bandwidth} Hz")

    # 滤波
    print("\n[3] 滤波器设计...")
    fd = FilterDesigner()

    # 低通: 截止5kHz
    b_lp, a_lp = fd.design_lowpass(order=4, cutoff=5000, fs=fs)
    filtered_lp = sig.filtfilt(b_lp, a_lp, amplified)

    # 带通: 500-2000Hz
    b_bp, a_bp = fd.design_bandpass(order=4, low=500, high=2000, fs=fs)
    filtered_bp = sig.filtfilt(b_bp, a_bp, amplified)

    # 带阻(陷波): 去除15kHz
    b_bs, a_bs = fd.design_bandstop(order=2, low=14500, high=15500, fs=fs)
    filtered_bs = sig.filtfilt(b_bs, a_bs, amplified)

    # FIR
    b_fir, a_fir = fd.fir_window(order=50, cutoff=5000, fs=fs)
    filtered_fir = sig.filtfilt(b_fir, a_fir, amplified)

    # 检波
    print("\n[4] 包络检波...")
    ed = EnvelopeDetector()
    envelope_peak = ed.peak_detector(amplified, attack=0.001, release=0.005, fs=fs)
    envelope_hilbert = ed.hilbert_envelope(amplified)

    # AM同步检波
    am_amplified = amp.process(am_signal, fs)
    envelope_sync = ed.synchronous_detector(am_amplified, 20000, fs, bandwidth=1000)

    # ADC
    print("\n[5] ADC采样与量化...")
    adc_configs = [
        ADCSimulator(resolution=8, vref=3.3, fs_adc=20000),
        ADCSimulator(resolution=10, vref=3.3, fs_adc=20000),
        ADCSimulator(resolution=12, vref=3.3, fs_adc=20000),
        ADCSimulator(resolution=16, vref=3.3, fs_adc=20000),
    ]

    adc_results = []
    for adc in adc_configs:
        result = adc.process(filtered_lp, fs)
        adc_results.append(result)
        print(f"   {adc.resolution}bit: 理论SNR={result['snr_theoretical']:.1f}dB, "
              f"实际SNR={result['snr_actual']:.1f}dB")

    # 频谱分析
    print("\n[6] 频谱分析...")
    freqs_raw, mag_raw, _ = compute_spectrum(amplified, fs)
    freqs_filt, mag_filt, _ = compute_spectrum(filtered_lp, fs)

    # ======================== 绘图 ========================
    fig, axes = plt.subplots(3, 3, figsize=(18, 14))
    fig.suptitle('信号调理仿真系统', fontsize=16, fontweight='bold')

    step = 3  # 降采样显示

    # (1) 原始信号 vs 放大信号
    ax = axes[0, 0]
    ax.plot(t[::step]*1000, raw_signal[::step]*1000, 'b-', alpha=0.5, label='原始信号')
    ax.plot(t[::step]*1000, amplified[::step], 'r-', alpha=0.7, label=f'放大后(×{amp.gain})')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('幅值')
    ax.set_title('信号放大')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (2) 滤波效果对比
    ax = axes[0, 1]
    ax.plot(t[::step]*1000, amplified[::step], 'b-', alpha=0.3, label='放大后')
    ax.plot(t[::step]*1000, filtered_lp[::step], 'r-', linewidth=1.5, label='低通5kHz')
    ax.plot(t[::step]*1000, filtered_bp[::step], 'g-', linewidth=1.5, label='带通0.5-2kHz')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('幅值')
    ax.set_title('滤波效果对比')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (3) 频谱
    ax = axes[0, 2]
    ax.semilogy(freqs_raw[:len(freqs_raw)//4], mag_raw[:len(freqs_raw)//4]+1e-12,
                'b-', alpha=0.5, label='滤波前')
    ax.semilogy(freqs_filt[:len(freqs_filt)//4], mag_filt[:len(freqs_filt)//4]+1e-12,
                'r-', linewidth=1.5, label='低通滤波后')
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('幅值')
    ax.set_title('频谱分析')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (4) 包络检波
    ax = axes[1, 0]
    ax.plot(t[::step]*1000, am_amplified[::step], 'b-', alpha=0.3, label='AM信号')
    ax.plot(t[::step]*1000, envelope_peak[::step], 'r-', linewidth=1.5, label='峰值检波')
    ax.plot(t[::step]*1000, envelope_hilbert[::step], 'g--', linewidth=1, label='希尔伯特包络')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('幅值')
    ax.set_title('包络检波')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (5) 同步检波
    ax = axes[1, 1]
    t_ds = np.arange(len(envelope_sync)) / fs * 1000
    ax.plot(t_ds, envelope_sync, 'r-', linewidth=1.5, label='同步检波输出')
    # 理想包络
    mod_env = np.abs(np.sin(2*np.pi*500*np.arange(len(envelope_sync))/fs))
    ax.plot(t_ds, mod_env * amp.gain * 0.02, 'k--', alpha=0.3, label='理想包络')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('幅值')
    ax.set_title('AM同步检波')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (6) 陷波滤波效果
    ax = axes[1, 2]
    freqs_notch, mag_notch, _ = compute_spectrum(filtered_bs, fs)
    ax.semilogy(freqs_raw[:len(freqs_raw)//2], mag_raw[:len(freqs_raw)//2]+1e-12,
                'b-', alpha=0.5, label='滤波前')
    ax.semilogy(freqs_notch[:len(freqs_notch)//2], mag_notch[:len(freqs_notch)//2]+1e-12,
                'r-', linewidth=1.5, label='带阻15kHz')
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('幅值')
    ax.set_title('陷波滤波 (去除15kHz干扰)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (7) ADC量化对比
    ax = axes[2, 0]
    adc_sub = 1  # 显示第2个ADC (10bit) 的一小段
    n_show = 200
    ax.plot(adc_results[adc_sub]['quantized'][:n_show], 'ro-', markersize=3,
            label=f'{adc_configs[adc_sub].resolution}bit量化')
    ax.plot(adc_results[adc_sub]['sampled'][:n_show], 'b-', alpha=0.5, label='模拟信号')
    ax.set_xlabel('样本')
    ax.set_ylabel('电压 (V)')
    ax.set_title(f'ADC {adc_configs[adc_sub].resolution}bit 量化')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (8) 量化噪声
    ax = axes[2, 1]
    ax.plot(adc_results[adc_sub]['quant_error'][:n_show], 'r-', linewidth=0.8)
    ax.axhline(y=adc_configs[adc_sub].lsb/2, color='k', linestyle='--',
               alpha=0.3, label=f'±½LSB={adc_configs[adc_sub].lsb/2*1000:.2f}mV')
    ax.set_xlabel('样本')
    ax.set_ylabel('量化误差 (V)')
    ax.set_title('量化噪声')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (9) SNR vs 分辨率
    ax = axes[2, 2]
    resolutions = [r[0] for r in [(8, adc_results[0]), (10, adc_results[1]),
                                    (12, adc_results[2]), (16, adc_results[3])]]
    snr_theo = [r['snr_theoretical'] for r in adc_results]
    snr_actual = [r['snr_actual'] for r in adc_results]
    x_pos = range(len(resolutions))
    ax.bar([x-0.15 for x in x_pos], snr_theo, 0.3, label='理论SNR', color='skyblue')
    ax.bar([x+0.15 for x in x_pos], snr_actual, 0.3, label='实际SNR', color='salmon')
    ax.set_xticks(list(x_pos))
    ax.set_xticklabels([f'{r}bit' for r in resolutions])
    ax.set_ylabel('SNR (dB)')
    ax.set_title('ADC SNR vs 分辨率')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = './nuedc-asset-library/15_simulation/signal_conditioning_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n图表已保存: {save_path}")

    # 打印滤波器参数
    print(f"\n滤波器参数:")
    w_lp, h_lp = fd.frequency_response(b_lp, a_lp, fs)
    w_bp, h_bp = fd.frequency_response(b_bp, a_bp, fs)
    print(f"   低通(-3dB): {w_lp[np.where(20*np.log10(np.abs(h_lp)+1e-12) <= -3)[0][0]]:.0f} Hz")
    print(f"   带通中心频率: 1250 Hz")
    print(f"   带阻中心频率: 15000 Hz")

    plt.show()
    print("\n仿真完成！")


if __name__ == '__main__':
    run_signal_conditioning()
