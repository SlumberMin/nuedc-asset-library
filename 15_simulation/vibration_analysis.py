#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
振动分析仿真模块
================
FFT频谱分析 | 模态分析 | 共振检测 | 时频分析

适用于电赛中振动传感器信号处理、结构健康监测等场景。
"""

import numpy as np
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass, field


# ============================================================
# 数据结构
# ============================================================
@dataclass
class ModalResult:
    """模态分析结果"""
    frequencies: np.ndarray      # 固有频率 (Hz)
    damping_ratios: np.ndarray   # 阻尼比
    mode_shapes: np.ndarray      # 振型矩阵 [n_modes x n_points]
    participation_factors: np.ndarray  # 参与因子


@dataclass
class FFTResult:
    """FFT分析结果"""
    frequencies: np.ndarray      # 频率轴 (Hz)
    magnitude: np.ndarray        # 幅值谱
    phase: np.ndarray            # 相位谱 (rad)
    power_spectrum: np.ndarray   # 功率谱
    peak_freq: float             # 主频率 (Hz)
    peak_magnitude: float        # 主频率幅值
    thd: float = 0.0             # 总谐波畸变率 (%)


@dataclass
class ResonanceResult:
    """共振检测结果"""
    resonance_freqs: List[float]      # 共振频率
    resonance_amplitudes: List[float] # 共振幅值
    quality_factors: List[float]      # 品质因数
    is_resonating: bool               # 是否处于共振状态
    safety_margin: float              # 安全裕度 (dB)


# ============================================================
# FFT 频谱分析
# ============================================================
class FFTAnalyzer:
    """FFT频谱分析器"""

    @staticmethod
    def compute_fft(signal: np.ndarray, fs: float,
                    window: str = 'hanning') -> FFTResult:
        """
        计算信号的FFT频谱

        Args:
            signal: 时域信号
            fs: 采样频率 (Hz)
            window: 窗函数类型 ('hanning', 'hamming', 'blackman', 'rectangular')

        Returns:
            FFTResult 包含频谱信息
        """
        N = len(signal)

        # 应用窗函数
        if window == 'hanning':
            win = np.hanning(N)
        elif window == 'hamming':
            win = np.hamming(N)
        elif window == 'blackman':
            win = np.blackman(N)
        else:
            win = np.ones(N)

        # 窗函数校正系数
        coherent_gain = np.sum(win) / N
        noise_gain = np.sqrt(np.sum(win**2) / N)

        windowed = signal * win
        spectrum = np.fft.rfft(windowed)
        freqs = np.fft.rfftfreq(N, 1.0 / fs)

        magnitude = np.abs(spectrum) / (N * coherent_gain) * 2
        magnitude[0] /= 2  # DC分量不乘2
        if N % 2 == 0:
            magnitude[-1] /= 2  # Nyquist分量

        phase = np.angle(spectrum)
        power = magnitude ** 2

        # 找主频率（排除DC）
        idx = np.argmax(magnitude[1:]) + 1
        peak_freq = freqs[idx]
        peak_mag = magnitude[idx]

        # 计算THD（总谐波畸变率）
        thd = FFTAnalyzer._compute_thd(magnitude, freqs, peak_freq)

        return FFTResult(
            frequencies=freqs, magnitude=magnitude, phase=phase,
            power_spectrum=power, peak_freq=peak_freq,
            peak_magnitude=peak_mag, thd=thd
        )

    @staticmethod
    def _compute_thd(magnitude: np.ndarray, freqs: np.ndarray,
                     fundamental_freq: float) -> float:
        """计算总谐波畸变率"""
        if fundamental_freq <= 0:
            return 0.0

        fund_idx = np.argmin(np.abs(freqs - fundamental_freq))
        fund_amp = magnitude[fund_idx]
        if fund_amp < 1e-12:
            return 0.0

        harmonic_power = 0.0
        for h in range(2, 10):
            h_freq = fundamental_freq * h
            if h_freq > freqs[-1]:
                break
            h_idx = np.argmin(np.abs(freqs - h_freq))
            harmonic_power += magnitude[h_idx] ** 2

        return np.sqrt(harmonic_power) / fund_amp * 100

    @staticmethod
    def spectrogram(signal: np.ndarray, fs: float,
                    nperseg: int = 256, noverlap: int = None,
                    window: str = 'hanning') -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        短时傅里叶变换 (STFT) 时频分析

        Returns:
            (frequencies, times, Sxx) - 频率轴, 时间轴, 时频矩阵
        """
        if noverlap is None:
            noverlap = nperseg // 2

        hop = nperseg - noverlap
        n_windows = (len(signal) - nperseg) // hop + 1
        n_freqs = nperseg // 2 + 1

        if window == 'hanning':
            win = np.hanning(nperseg)
        elif window == 'hamming':
            win = np.hamming(nperseg)
        else:
            win = np.ones(nperseg)

        Sxx = np.zeros((n_freqs, n_windows))
        times = np.arange(n_windows) * hop / fs
        freqs = np.fft.rfftfreq(nperseg, 1.0 / fs)

        for i in range(n_windows):
            start = i * hop
            segment = signal[start:start + nperseg] * win
            spectrum = np.fft.rfft(segment)
            Sxx[:, i] = np.abs(spectrum) ** 2

        return freqs, times, Sxx

    @staticmethod
    def psd_welch(signal: np.ndarray, fs: float,
                  nperseg: int = 1024, noverlap: int = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Welch法功率谱密度估计

        Returns:
            (frequencies, Pxx) - 频率轴和PSD (V²/Hz)
        """
        if noverlap is None:
            noverlap = nperseg // 2

        hop = nperseg - noverlap
        win = np.hanning(nperseg)
        scale = 1.0 / (fs * np.sum(win**2))

        segments = []
        start = 0
        while start + nperseg <= len(signal):
            seg = signal[start:start + nperseg] * win
            segments.append(np.abs(np.fft.rfft(seg))**2)
            start += hop

        if not segments:
            return np.array([]), np.array([])

        Pxx = np.mean(segments, axis=0) * scale
        Pxx[0] /= 2
        if nperseg % 2 == 0:
            Pxx[-1] /= 2

        freqs = np.fft.rfftfreq(nperseg, 1.0 / fs)
        return freqs, Pxx


# ============================================================
# 模态分析
# ============================================================
class ModalAnalyzer:
    """模态分析器 - 频域分解法 (EFDD / 峰值拾取)"""

    @staticmethod
    def peak_picking(freqs: np.ndarray, psd: np.ndarray,
                     n_modes: int = 5, min_distance_hz: float = 2.0,
                     prominence_ratio: float = 0.1) -> ModalResult:
        """
        峰值拾取法提取模态参数

        Args:
            freqs: 频率轴
            psd: 功率谱密度
            n_modes: 提取模态数
            min_distance_hz: 峰点最小频率间隔
            prominence_ratio: 突出度阈值(相对最大值)

        Returns:
            ModalResult 模态参数
        """
        # 简单峰值检测
        threshold = np.max(psd) * prominence_ratio
        peaks = []
        for i in range(1, len(psd) - 1):
            if (psd[i] > psd[i-1] and psd[i] > psd[i+1] and
                    psd[i] > threshold):
                peaks.append(i)

        # 按幅值排序
        peaks.sort(key=lambda k: psd[k], reverse=True)

        # 合并过近的峰
        selected = []
        for p in peaks:
            if all(abs(freqs[p] - freqs[s]) >= min_distance_hz for s in selected):
                selected.append(p)
            if len(selected) >= n_modes:
                break

        if not selected:
            return ModalResult(
                frequencies=np.array([]), damping_ratios=np.array([]),
                mode_shapes=np.array([]), participation_factors=np.array([])
            )

        # 半功率带宽法估算阻尼比
        freqs_arr = np.array([freqs[s] for s in selected])
        damping = np.zeros(len(selected))
        participation = np.zeros(len(selected))

        for idx, s in enumerate(selected):
            half_power = psd[s] / 2
            # 找半功率带宽
            f1, f2 = freqs[s], freqs[s]
            for j in range(s, 0, -1):
                if psd[j] <= half_power:
                    f1 = freqs[j]
                    break
            for j in range(s, len(psd) - 1):
                if psd[j] <= half_power:
                    f2 = freqs[j]
                    break
            bw = f2 - f1
            damping[idx] = bw / (2 * freqs[s]) if freqs[s] > 0 else 0
            participation[idx] = psd[s] / np.max(psd)

        # 生成示意振型
        n_points = 100
        x = np.linspace(0, 1, n_points)
        mode_shapes = np.zeros((len(selected), n_points))
        for i, s in enumerate(selected):
            mode_shapes[i] = np.sin((i + 1) * np.pi * x) * participation[i]

        return ModalResult(
            frequencies=freqs_arr,
            damping_ratios=damping,
            mode_shapes=mode_shapes,
            participation_factors=participation
        )

    @staticmethod
    def frequency_response_function(excitation: np.ndarray,
                                     response: np.ndarray,
                                     fs: float,
                                     nperseg: int = 1024) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        计算频率响应函数 (FRF)

        Returns:
            (freqs, H_mag_dB, H_phase_deg)
        """
        hop = nperseg // 2
        win = np.hanning(nperseg)

        Gxx = np.zeros(nperseg // 2 + 1)  # 输入自谱
        Gyy = np.zeros(nperseg // 2 + 1)  # 输出自谱
        Gyx = np.zeros(nperseg // 2 + 1, dtype=complex)  # 互谱

        n_avg = 0
        start = 0
        while start + nperseg <= len(excitation):
            x_seg = excitation[start:start + nperseg] * win
            y_seg = response[start:start + nperseg] * win

            X = np.fft.rfft(x_seg)
            Y = np.fft.rfft(y_seg)

            Gxx += np.abs(X)**2
            Gyy += np.abs(Y)**2
            Gyx += Y * np.conj(X)

            n_avg += 1
            start += hop

        if n_avg == 0:
            return np.array([]), np.array([]), np.array([])

        Gxx /= n_avg
        Gyy /= n_avg
        Gyx /= n_avg

        freqs = np.fft.rfftfreq(nperseg, 1.0 / fs)

        # H1估计 = Gyx / Gxx (噪声在输出端)
        H = Gyx / (Gxx + 1e-30)
        H_mag_dB = 20 * np.log10(np.abs(H) + 1e-30)
        H_phase_deg = np.degrees(np.angle(H))

        return freqs, H_mag_dB, H_phase_deg


# ============================================================
# 共振检测
# ============================================================
class ResonanceDetector:
    """共振检测器"""

    @staticmethod
    def detect_resonance(signal: np.ndarray, fs: float,
                         excitation_freq: float = None,
                         freq_band: Tuple[float, float] = (1, 1000),
                         threshold_db: float = 6.0) -> ResonanceResult:
        """
        检测共振峰

        Args:
            signal: 响应信号
            fs: 采样频率
            excitation_freq: 激励频率 (None则自动检测)
            freq_band: 分析频带
            threshold_db: 共振判定阈值 (dB above mean)

        Returns:
            ResonanceResult
        """
        fft_result = FFTAnalyzer.compute_fft(signal, fs)

        # 限制频带
        mask = (fft_result.frequencies >= freq_band[0]) & \
               (fft_result.frequencies <= freq_band[1])
        freqs = fft_result.frequencies[mask]
        mag = fft_result.magnitude[mask]
        mag_db = 20 * np.log10(mag + 1e-30)

        mean_db = np.mean(mag_db)
        threshold = mean_db + threshold_db

        # 检测共振峰
        resonance_freqs = []
        resonance_amps = []
        quality_factors = []

        for i in range(1, len(mag_db) - 1):
            if mag_db[i] > threshold and mag_db[i] > mag_db[i-1] and \
               mag_db[i] > mag_db[i+1]:
                resonance_freqs.append(freqs[i])
                resonance_amps.append(mag[i])

                # 半功率带宽法计算Q因子
                half_power_db = mag_db[i] - 3.0
                f1, f2 = freqs[i], freqs[i]
                for j in range(i, 0, -1):
                    if mag_db[j] <= half_power_db:
                        f1 = freqs[j]
                        break
                for j in range(i, len(mag_db) - 1):
                    if mag_db[j] <= half_power_db:
                        f2 = freqs[j]
                        break
                bw = f2 - f1
                Q = freqs[i] / bw if bw > 0 else 1000
                quality_factors.append(Q)

        # 判断是否正在共振
        is_resonating = False
        safety_margin = 40.0  # dB

        if excitation_freq is not None and resonance_freqs:
            closest_idx = np.argmin([abs(f - excitation_freq) for f in resonance_freqs])
            closest_freq = resonance_freqs[closest_idx]
            freq_error_pct = abs(closest_freq - excitation_freq) / excitation_freq * 100
            if freq_error_pct < 5:  # 5%以内认为可能共振
                is_resonating = True
                safety_margin = -20 * np.log10(quality_factors[closest_idx] / 100) if quality_factors[closest_idx] > 0 else 40

        return ResonanceResult(
            resonance_freqs=resonance_freqs,
            resonance_amplitudes=resonance_amps,
            quality_factors=quality_factors,
            is_resonating=is_resonating,
            safety_margin=safety_margin
        )

    @staticmethod
    def campbell_diagram(signals: List[np.ndarray], fs: float,
                         rpm_range: Tuple[float, float],
                         n_orders: int = 10) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        坎贝尔图 (转速-频率图)

        Args:
            signals: 不同转速下的振动信号列表
            fs: 采样频率
            rpm_range: (min_rpm, max_rpm)
            n_orders: 显示的阶次线数

        Returns:
            (rpms, freqs, amplitude_matrix)
        """
        n_signals = len(signals)
        rpms = np.linspace(rpm_range[0], rpm_range[1], n_signals)

        # 对每个信号做FFT
        amp_matrix = []
        common_freqs = None

        for sig in signals:
            result = FFTAnalyzer.compute_fft(sig, fs)
            if common_freqs is None:
                common_freqs = result.frequencies
            amp_matrix.append(result.magnitude[:len(common_freqs)])

        amp_matrix = np.array(amp_matrix)

        return rpms, common_freqs, amp_matrix


# ============================================================
# 辅助工具
# ============================================================
class VibrationUtils:
    """振动分析工具函数"""

    @staticmethod
    def generate_vibration_signal(frequencies: List[float],
                                   amplitudes: List[float],
                                   fs: float, duration: float,
                                   noise_level: float = 0.01,
                                   damping: float = 0.0) -> np.ndarray:
        """
        生成仿真振动信号

        Args:
            frequencies: 频率分量列表 (Hz)
            amplitudes: 对应幅值
            fs: 采样频率
            duration: 持续时间 (s)
            noise_level: 噪声水平
            damping: 阻尼衰减系数

        Returns:
            振动信号
        """
        t = np.arange(0, duration, 1.0 / fs)
        signal = np.zeros_like(t)

        for freq, amp in zip(frequencies, amplitudes):
            if damping > 0:
                signal += amp * np.exp(-damping * t) * np.sin(2 * np.pi * freq * t)
            else:
                signal += amp * np.sin(2 * np.pi * freq * t)

        signal += noise_level * np.random.randn(len(t))
        return signal

    @staticmethod
    def compute_rms(signal: np.ndarray) -> float:
        """计算均方根值"""
        return np.sqrt(np.mean(signal**2))

    @staticmethod
    def compute_crest_factor(signal: np.ndarray) -> float:
        """计算波峰因子"""
        rms = np.sqrt(np.mean(signal**2))
        peak = np.max(np.abs(signal))
        return peak / rms if rms > 0 else 0

    @staticmethod
    def compute_kurtosis(signal: np.ndarray) -> float:
        """计算峭度 (轴承故障指标)"""
        n = len(signal)
        mean = np.mean(signal)
        std = np.std(signal)
        if std < 1e-30:
            return 0.0
        return np.sum(((signal - mean) / std) ** 4) / n

    @staticmethod
    def envelope_analysis(signal: np.ndarray, fs: float,
                          band_center: float, band_width: float = 1000) -> Tuple[np.ndarray, np.ndarray]:
        """
        包络分析 (用于轴承故障诊断)

        Args:
            signal: 振动信号
            fs: 采样频率
            band_center: 带通中心频率 (Hz)
            band_width: 带通带宽 (Hz)

        Returns:
            (freqs, envelope_spectrum)
        """
        N = len(signal)
        freqs = np.fft.rfftfreq(N, 1.0 / fs)
        spectrum = np.fft.rfft(signal)

        # 带通滤波
        f_low = band_center - band_width / 2
        f_high = band_center + band_width / 2
        mask = (freqs >= f_low) & (freqs <= f_high)
        filtered_spectrum = np.zeros_like(spectrum)
        filtered_spectrum[mask] = spectrum[mask]

        filtered = np.fft.irfft(filtered_spectrum, n=N)

        # Hilbert变换求包络
        analytic = filtered + 1j * np.imag(np.fft.ifft(
            -1j * np.sign(freqs) * np.fft.rfft(filtered), n=N
        ))
        envelope = np.abs(analytic)

        # 包络谱
        env_fft = np.abs(np.fft.rfft(envelope)) / len(envelope)
        env_freqs = np.fft.rfftfreq(len(envelope), 1.0 / fs)

        return env_freqs, env_fft


# ============================================================
# 快捷函数
# ============================================================
def quick_fft(signal: np.ndarray, fs: float) -> Dict:
    """快速FFT分析，返回主要结果字典"""
    result = FFTAnalyzer.compute_fft(signal, fs)
    return {
        'peak_freq': result.peak_freq,
        'peak_magnitude': result.peak_magnitude,
        'thd': result.thd,
        'rms': np.sqrt(np.mean(signal**2)),
        'frequencies': result.frequencies,
        'magnitude': result.magnitude,
    }


def quick_modal(signal: np.ndarray, fs: float, n_modes: int = 5) -> ModalResult:
    """快速模态分析"""
    freqs, Pxx = FFTAnalyzer.psd_welch(signal, fs)
    return ModalAnalyzer.peak_picking(freqs, Pxx, n_modes=n_modes)


# ============================================================
# 演示 / 测试
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("  振动分析仿真模块 - 功能演示")
    print("=" * 60)

    # 1. 生成仿真信号
    print("\n[1] 生成多频振动信号...")
    fs = 10000  # 10kHz采样率
    duration = 1.0
    t = np.arange(0, duration, 1.0 / fs)

    signal = VibrationUtils.generate_vibration_signal(
        frequencies=[50, 120, 350, 800],
        amplitudes=[1.0, 0.5, 0.3, 0.1],
        fs=fs, duration=duration,
        noise_level=0.05
    )
    print(f"  采样点数: {len(signal)}, 采样率: {fs} Hz")
    print(f"  RMS: {VibrationUtils.compute_rms(signal):.4f}")
    print(f"  波峰因子: {VibrationUtils.compute_crest_factor(signal):.4f}")
    print(f"  峭度: {VibrationUtils.compute_kurtosis(signal):.4f}")

    # 2. FFT分析
    print("\n[2] FFT频谱分析...")
    fft_result = FFTAnalyzer.compute_fft(signal, fs)
    print(f"  主频率: {fft_result.peak_freq:.1f} Hz")
    print(f"  主频率幅值: {fft_result.peak_magnitude:.4f}")
    print(f"  THD: {fft_result.thd:.2f}%")

    # 3. 模态分析
    print("\n[3] 模态分析 (峰值拾取法)...")
    modal = quick_modal(signal, fs, n_modes=4)
    for i, (f, z) in enumerate(zip(modal.frequencies, modal.damping_ratios)):
        print(f"  模态 {i+1}: f={f:.1f} Hz, ζ={z:.4f}, 参与因子={modal.participation_factors[i]:.3f}")

    # 4. 共振检测
    print("\n[4] 共振检测...")
    res_result = ResonanceDetector.detect_resonance(
        signal, fs, excitation_freq=50.0,
        freq_band=(10, 2000), threshold_db=6.0
    )
    print(f"  检测到 {len(res_result.resonance_freqs)} 个共振峰")
    for i, (f, Q) in enumerate(zip(res_result.resonance_freqs, res_result.quality_factors)):
        print(f"  共振 {i+1}: f={f:.1f} Hz, Q={Q:.1f}")
    print(f"  是否共振: {res_result.is_resonating}")

    # 5. Welch PSD
    print("\n[5] Welch功率谱密度估计...")
    psd_freqs, psd = FFTAnalyzer.psd_welch(signal, fs, nperseg=2048)
    print(f"  频率分辨率: {psd_freqs[1] - psd_freqs[0]:.2f} Hz")
    print(f"  最大PSD频率: {psd_freqs[np.argmax(psd)]:.1f} Hz")

    print("\n" + "=" * 60)
    print("  振动分析仿真完成!")
    print("=" * 60)
