#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生物信号处理仿真 - ECG/EEG/EMG 滤波 + 特征提取
===============================================
功能:
  1. ECG信号仿真+QRS检测+心率变异性分析
  2. EEG信号仿真+频段分析(α/β/θ/δ)
  3. EMG信号仿真+肌力估算+疲劳检测
  4. 自适应滤波+陷波去工频干扰+小波去噪
  5. 特征提取(RMS/MAV/过零率/功率谱)

依赖: numpy, scipy, matplotlib (可选)
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum


# ============================================================
# 1. 信号类型定义
# ============================================================

class SignalType(Enum):
    ECG = "ECG"
    EEG = "EEG"
    EMG = "EMG"


@dataclass
class BioSignal:
    """生物信号容器"""
    signal_type: SignalType
    data: np.ndarray
    fs: float  # 采样率
    t: np.ndarray = field(default_factory=lambda: np.array([]))
    annotations: Dict = field(default_factory=dict)

    def __post_init__(self):
        if len(self.t) == 0:
            self.t = np.arange(len(self.data)) / self.fs


# ============================================================
# 2. 信号生成器
# ============================================================

class SignalGenerator:
    """生物信号仿真生成器"""

    @staticmethod
    def generate_ecg(duration: float = 5.0, fs: float = 500.0,
                     heart_rate: float = 72.0, noise_level: float = 0.05) -> BioSignal:
        """生成仿真ECG信号 (基于PQRST模型)"""
        t = np.arange(0, duration, 1/fs)
        n = len(t)
        ecg = np.zeros(n)

        # 心跳周期
        beat_period = 60.0 / heart_rate
        beat_times = np.arange(0, duration, beat_period)

        for bt in beat_times:
            # P波
            p_center = bt + 0.08 * beat_period
            p_amp = 0.15
            p_width = 0.04 * beat_period
            ecg += p_amp * np.exp(-((t - p_center) / p_width) ** 2)

            # Q波
            q_center = bt + 0.16 * beat_period
            q_amp = -0.1
            q_width = 0.015 * beat_period
            ecg += q_amp * np.exp(-((t - q_center) / q_width) ** 2)

            # R波 (主峰)
            r_center = bt + 0.20 * beat_period
            r_amp = 1.0
            r_width = 0.012 * beat_period
            ecg += r_amp * np.exp(-((t - r_center) / r_width) ** 2)

            # S波
            s_center = bt + 0.24 * beat_period
            s_amp = -0.2
            s_width = 0.015 * beat_period
            ecg += s_amp * np.exp(-((t - s_center) / s_width) ** 2)

            # T波
            t_center = bt + 0.45 * beat_period
            t_amp = 0.3
            t_width = 0.08 * beat_period
            ecg += t_amp * np.exp(-((t - t_center) / t_width) ** 2)

        # 添加噪声
        ecg += noise_level * np.random.randn(n)

        # 添加基线漂移
        baseline = 0.1 * np.sin(2 * np.pi * 0.15 * t)
        ecg += baseline

        signal = BioSignal(SignalType.ECG, ecg, fs)
        signal.annotations = {"heart_rate": heart_rate, "beat_times": beat_times}
        return signal

    @staticmethod
    def generate_eeg(duration: float = 10.0, fs: float = 256.0,
                     state: str = "relaxed", noise_level: float = 0.1) -> BioSignal:
        """生成仿真EEG信号 (多频段叠加)"""
        t = np.arange(0, duration, 1/fs)
        n = len(t)
        eeg = np.zeros(n)

        # 不同状态的频段配置
        band_configs = {
            "deep_sleep": {"delta": (1.5, 0.8), "theta": (5, 0.2), "alpha": (10, 0.05), "beta": (20, 0.02)},
            "relaxed":    {"delta": (2, 0.1), "theta": (6, 0.15), "alpha": (10, 0.5), "beta": (20, 0.05)},
            "alert":      {"delta": (2, 0.05), "theta": (6, 0.1), "alpha": (10, 0.1), "beta": (20, 0.4)},
            "focused":    {"delta": (2, 0.03), "theta": (6, 0.2), "alpha": (10, 0.08), "beta": (25, 0.5)},
        }

        config = band_configs.get(state, band_configs["relaxed"])

        for band_name, (freq, amp) in config.items():
            # 每个频段有多个频率分量
            for i in range(3):
                f = freq + np.random.uniform(-1, 1)
                phase = np.random.uniform(0, 2*np.pi)
                mod = 1 + 0.3 * np.sin(2 * np.pi * 0.05 * t)  # 调幅
                eeg += amp * mod * np.sin(2 * np.pi * f * t + phase) / (i + 1)

        # 瞬态事件 (K复合波/睡眠纺锤波)
        if state == "deep_sleep":
            for _ in range(3):
                sp_idx = np.random.randint(0, n - 200)
                sp_t = np.arange(200) / fs
                spindle = 0.5 * np.sin(2 * np.pi * 14 * sp_t) * np.exp(-sp_t / 0.3)
                eeg[sp_idx:sp_idx+200] += spindle

        eeg += noise_level * np.random.randn(n)

        signal = BioSignal(SignalType.EEG, eeg, fs)
        signal.annotations = {"state": state, "channels": 1}
        return signal

    @staticmethod
    def generate_emg(duration: float = 5.0, fs: float = 1000.0,
                     contraction_level: float = 0.5,
                     fatigue: bool = False) -> BioSignal:
        """生成仿真EMG信号"""
        t = np.arange(0, duration, 1/fs)
        n = len(t)

        # 基础EMG - 多个运动单元放电
        emg = np.zeros(n)
        n_units = int(50 * contraction_level)  # 激活的运动单元数

        for _ in range(n_units):
            # 随机放电率 (20-50 Hz)
            firing_rate = np.random.uniform(20, 50)
            spike_times = np.cumsum(np.random.exponential(1/firing_rate, int(duration * firing_rate * 2)))
            spike_times = spike_times[spike_times < duration]

            for st in spike_times:
                idx = int(st * fs)
                width = int(0.005 * fs)
                if idx + width < n:
                    spike = np.random.randn() * contraction_level
                    # 运动单元动作电位波形
                    t_spike = np.arange(width) / fs
                    waveform = spike * np.sin(2 * np.pi * 200 * t_spike) * np.exp(-t_spike / 0.003)
                    emg[idx:idx+width] += waveform

        # 疲劳效应: 频率下降 + 幅值变化
        if fatigue:
            freq_shift = np.linspace(1.0, 0.7, n)
            amp_mod = np.linspace(1.0, 1.3, n)
            emg *= amp_mod
            # 添加低频调制
            emg *= (1 + 0.3 * np.sin(2 * np.pi * 5 * t * freq_shift))

        # 背景噪声
        emg += 0.02 * np.random.randn(n)

        signal = BioSignal(SignalType.EMG, emg, fs)
        signal.annotations = {
            "contraction_level": contraction_level,
            "fatigue": fatigue,
            "n_motor_units": n_units,
        }
        return signal


# ============================================================
# 3. 滤波器组
# ============================================================

class BioFilters:
    """生物信号滤波器"""

    @staticmethod
    def bandpass_filter(signal: np.ndarray, fs: float,
                        low: float, high: float, order: int = 4) -> np.ndarray:
        """Butterworth带通滤波器 (纯numpy实现)"""
        # 简化的IIR滤波器实现
        nyq = fs / 2
        w_low = low / nyq
        w_high = high / nyq

        # 使用FFT实现频域滤波
        n = len(signal)
        freqs = np.fft.rfftfreq(n, 1/fs)
        fft_vals = np.fft.rfft(signal)

        # 构建带通响应
        response = np.zeros_like(freqs)
        mask = (freqs >= low) & (freqs <= high)
        # Butterworth近似的平滑过渡
        for i, f in enumerate(freqs):
            if low <= f <= high:
                response[i] = 1.0
            elif f < low:
                ratio = f / low if low > 0 else 0
                response[i] = ratio ** order
            elif f > high:
                ratio = high / f if f > 0 else 0
                response[i] = ratio ** order

        filtered = np.fft.irfft(fft_vals * response, n=n)
        return filtered

    @staticmethod
    def notch_filter(signal: np.ndarray, fs: float,
                     freq: float = 50.0, Q: float = 30.0) -> np.ndarray:
        """陷波滤波器 - 去除工频干扰"""
        n = len(signal)
        freqs = np.fft.rfftfreq(n, 1/fs)
        fft_vals = np.fft.rfft(signal)

        # 多谐波陷波
        for harmonic in [1, 2, 3]:
            f0 = freq * harmonic
            for i, f in enumerate(freqs):
                if abs(f - f0) < f0 / Q:
                    notch_depth = 1.0 - np.exp(-((f - f0) / (f0 / (2 * Q))) ** 2)
                    fft_vals[i] *= notch_depth

        return np.fft.irfft(fft_vals, n=n)

    @staticmethod
    def wavelet_denoise(signal: np.ndarray, wavelet: str = "db4",
                        level: int = 4, threshold_method: str = "soft") -> np.ndarray:
        """简化小波去噪 (使用滑动窗口模拟)"""
        denoised = signal.copy()

        for _ in range(level):
            # 简化的多分辨率分析
            n = len(denoised)
            if n < 4:
                break

            # 分解
            approx = (denoised[0::2] + denoised[1::2]) / 2
            detail = (denoised[0::2] - denoised[1::2]) / 2

            # 阈值处理
            sigma = np.median(np.abs(detail)) / 0.6745
            threshold = sigma * np.sqrt(2 * np.log(len(detail)))

            if threshold_method == "soft":
                detail = np.sign(detail) * np.maximum(np.abs(detail) - threshold, 0)
            else:  # hard
                detail[np.abs(detail) < threshold] = 0

            # 重构
            reconstructed = np.zeros(n)
            reconstructed[0::2] = approx + detail
            reconstructed[1::2] = approx - detail

            if n % 2 == 1:
                reconstructed[-1] = approx[-1]

            denoised = reconstructed

        return denoised

    @staticmethod
    def adaptive_filter_lms(signal: np.ndarray, noise_ref: np.ndarray,
                            mu: float = 0.01, filter_order: int = 32) -> Tuple[np.ndarray, np.ndarray]:
        """LMS自适应滤波器"""
        n = len(signal)
        w = np.zeros(filter_order)
        output = np.zeros(n)
        error = np.zeros(n)

        for i in range(filter_order, n):
            x = noise_ref[i-filter_order:i][::-1]
            y = np.dot(w, x)
            output[i] = y
            error[i] = signal[i] - y
            # LMS更新
            w += 2 * mu * error[i] * x

        return output, error


# ============================================================
# 4. 特征提取器
# ============================================================

class FeatureExtractor:
    """生物信号特征提取"""

    @staticmethod
    def time_domain_features(signal: np.ndarray) -> Dict[str, float]:
        """时域特征"""
        return {
            "mean": float(np.mean(signal)),
            "std": float(np.std(signal)),
            "rms": float(np.sqrt(np.mean(signal**2))),
            "mav": float(np.mean(np.abs(signal))),  # 平均绝对值
            "zero_crossings": int(np.sum(np.diff(np.sign(signal)) != 0)),
            "waveform_length": float(np.sum(np.abs(np.diff(signal)))),
            "slope_sign_changes": int(np.sum(np.diff(np.sign(np.diff(signal))) != 0)),
            "peak_to_peak": float(np.max(signal) - np.min(signal)),
            "variance": float(np.var(signal)),
            "kurtosis": float(np.mean((signal - np.mean(signal))**4) / (np.std(signal)**4 + 1e-10)),
            "skewness": float(np.mean((signal - np.mean(signal))**3) / (np.std(signal)**3 + 1e-10)),
        }

    @staticmethod
    def frequency_domain_features(signal: np.ndarray, fs: float) -> Dict[str, float]:
        """频域特征"""
        n = len(signal)
        freqs = np.fft.rfftfreq(n, 1/fs)
        psd = np.abs(np.fft.rfft(signal))**2 / n

        total_power = np.sum(psd) + 1e-10

        # 质心频率
        mdf_idx = np.searchsorted(np.cumsum(psd), total_power / 2)

        return {
            "total_power": float(total_power),
            "mean_frequency": float(np.sum(freqs * psd) / total_power),
            "median_frequency": float(freqs[min(mdf_idx, len(freqs)-1)]),
            "peak_frequency": float(freqs[np.argmax(psd[1:])+1]),
            "spectral_entropy": float(-np.sum((psd/total_power) * np.log2(psd/total_power + 1e-10))),
            "bandwidth": float(np.sqrt(np.sum(((freqs - np.sum(freqs*psd)/total_power)**2) * psd) / total_power)),
            "power_below_4hz": float(np.sum(psd[freqs < 4]) / total_power),
            "power_4_8hz": float(np.sum(psd[(freqs >= 4) & (freqs < 8)]) / total_power),
            "power_8_13hz": float(np.sum(psd[(freqs >= 8) & (freqs < 13)]) / total_power),
            "power_13_30hz": float(np.sum(psd[(freqs >= 13) & (freqs < 30)]) / total_power),
            "power_above_30hz": float(np.sum(psd[freqs >= 30]) / total_power),
        }

    @staticmethod
    def ecg_features(ecg: np.ndarray, fs: float) -> Dict[str, float]:
        """ECG专用特征: QRS检测 + HRV"""
        # 简化QRS检测: 差分+阈值
        diff_ecg = np.diff(ecg)
        energy = diff_ecg ** 2

        # 滑动窗口能量
        win_size = int(0.08 * fs)
        if win_size < 1:
            win_size = 1
        energy_smooth = np.convolve(energy, np.ones(win_size)/win_size, mode='same')

        # 自适应阈值检测R峰
        threshold = 0.4 * np.max(energy_smooth)
        r_peaks = []
        min_distance = int(0.3 * fs)  # 最小300ms间隔

        i = 0
        while i < len(energy_smooth):
            if energy_smooth[i] > threshold:
                peak_idx = i + np.argmax(energy_smooth[i:min(i+min_distance, len(energy_smooth))])
                r_peaks.append(peak_idx)
                i = peak_idx + min_distance
            else:
                i += 1

        r_peaks = np.array(r_peaks)

        result = {"n_beats": len(r_peaks)}

        if len(r_peaks) > 2:
            # RR间期
            rr_intervals = np.diff(r_peaks) / fs  # 秒
            rr_ms = rr_intervals * 1000  # 毫秒

            # 心率
            heart_rates = 60.0 / rr_intervals
            result.update({
                "mean_hr": float(np.mean(heart_rates)),
                "std_hr": float(np.std(heart_rates)),
                "mean_rr": float(np.mean(rr_ms)),
                "sdnn": float(np.std(rr_ms)),  # RR标准差
                "rmssd": float(np.sqrt(np.mean(np.diff(rr_ms)**2))),  # 相邻RR差的RMS
                "pnn50": float(np.sum(np.abs(np.diff(rr_ms)) > 50) / (len(rr_ms) - 1) * 100),
                "nn50": int(np.sum(np.abs(np.diff(rr_ms)) > 50)),
                "triangular_index": float(len(rr_ms) / (np.max(np.histogram(rr_ms, bins=128)[0]) + 1e-10)),
            })

            # 频域HRV
            if len(rr_intervals) > 10:
                rr_interp = np.interp(
                    np.linspace(0, len(rr_intervals)*np.mean(rr_intervals), 256),
                    np.arange(len(rr_intervals)) * np.mean(rr_intervals),
                    rr_intervals
                )
                rr_freqs = np.fft.rfftfreq(len(rr_interp), np.mean(rr_intervals))
                rr_psd = np.abs(np.fft.rfft(rr_interp - np.mean(rr_interp)))**2

                lf_mask = (rr_freqs >= 0.04) & (rr_freqs < 0.15)
                hf_mask = (rr_freqs >= 0.15) & (rr_freqs < 0.4)

                lf_power = np.sum(rr_psd[lf_mask])
                hf_power = np.sum(rr_psd[hf_mask])

                result.update({
                    "lf_power": float(lf_power),
                    "hf_power": float(hf_power),
                    "lf_hf_ratio": float(lf_power / (hf_power + 1e-10)),
                })

        return result

    @staticmethod
    def eeg_bandpower(eeg: np.ndarray, fs: float) -> Dict[str, float]:
        """EEG频段功率分析"""
        freq_features = FeatureExtractor.frequency_domain_features(eeg, fs)

        bands = {
            "delta": (0.5, 4),
            "theta": (4, 8),
            "alpha": (8, 13),
            "beta": (13, 30),
            "gamma": (30, 100),
        }

        n = len(eeg)
        freqs = np.fft.rfftfreq(n, 1/fs)
        psd = np.abs(np.fft.rfft(eeg))**2 / n

        band_powers = {}
        for name, (f_low, f_high) in bands.items():
            mask = (freqs >= f_low) & (freqs < f_high)
            band_powers[f"{name}_power"] = float(np.sum(psd[mask]))
            band_powers[f"{name}_relative"] = float(np.sum(psd[mask]) / (np.sum(psd) + 1e-10))

        band_powers.update(freq_features)

        # 主导频段
        band_abs = {k: v for k, v in band_powers.items() if k.endswith("_power")}
        dominant = max(band_abs, key=band_abs.get)
        band_powers["dominant_band"] = dominant.replace("_power", "")

        return band_powers

    @staticmethod
    def emg_features(emg: np.ndarray, fs: float,
                     window_ms: float = 250) -> Dict[str, float]:
        """EMG特征: 分窗RMS + 频率特征"""
        win_samples = int(window_ms / 1000 * fs)

        # 分窗RMS
        n_windows = len(emg) // win_samples
        rms_windows = []
        for i in range(n_windows):
            segment = emg[i*win_samples:(i+1)*win_samples]
            rms_windows.append(np.sqrt(np.mean(segment**2)))

        rms_windows = np.array(rms_windows)

        # 疲劳指标: 中位频率斜率
        mdf_list = []
        for i in range(n_windows):
            segment = emg[i*win_samples:(i+1)*win_samples]
            freq_feat = FeatureExtractor.frequency_domain_features(segment, fs)
            mdf_list.append(freq_feat["median_frequency"])

        mdf_slope = 0
        if len(mdf_list) > 1:
            x = np.arange(len(mdf_list))
            coeffs = np.polyfit(x, mdf_list, 1)
            mdf_slope = coeffs[0]

        time_feat = FeatureExtractor.time_domain_features(emg)

        result = {
            **time_feat,
            "mean_rms": float(np.mean(rms_windows)),
            "std_rms": float(np.std(rms_windows)),
            "max_rms": float(np.max(rms_windows)),
            "mdf_slope": float(mdf_slope),
            "estimated_force": float(np.mean(rms_windows) * 10),  # 粗略力估计
            "fatigue_index": float(-mdf_slope),  # 正值=疲劳
            "n_windows": n_windows,
        }

        return result


# ============================================================
# 5. 综合分析管线
# ============================================================

class BioSignalPipeline:
    """生物信号处理管线"""

    def __init__(self, fs: float = 500.0):
        self.fs = fs
        self.filters = BioFilters()
        self.features = FeatureExtractor()
        self.generator = SignalGenerator()

    def process_ecg(self, ecg_signal: Optional[np.ndarray] = None,
                    duration: float = 10.0) -> Dict:
        """完整ECG处理流程"""
        if ecg_signal is None:
            sig = self.generator.generate_ecg(duration=duration, fs=self.fs)
            ecg_signal = sig.data

        # 去基线漂移
        ecg_hp = self.filters.bandpass_filter(ecg_signal, self.fs, 0.5, 40)

        # 去工频干扰
        ecg_clean = self.filters.notch_filter(ecg_hp, self.fs, 50.0)

        # 小波去噪
        ecg_denoised = self.filters.wavelet_denoise(ecg_clean)

        # QRS检测 + HRV分析
        ecg_feat = self.features.ecg_features(ecg_denoised, self.fs)

        # 时域特征
        time_feat = self.features.time_domain_features(ecg_denoised)

        return {
            "raw": ecg_signal,
            "filtered": ecg_denoised,
            "ecg_features": ecg_feat,
            "time_features": time_feat,
        }

    def process_eeg(self, eeg_signal: Optional[np.ndarray] = None,
                    duration: float = 10.0, state: str = "relaxed") -> Dict:
        """完整EEG处理流程"""
        if eeg_signal is None:
            sig = self.generator.generate_eeg(duration=duration, fs=self.fs, state=state)
            eeg_signal = sig.data

        # 带通滤波 (0.5-100Hz)
        eeg_bp = self.filters.bandpass_filter(eeg_signal, self.fs, 0.5, 100)

        # 去工频
        eeg_clean = self.filters.notch_filter(eeg_bp, self.fs, 50.0)

        # 频段分析
        band_powers = self.features.eeg_bandpower(eeg_clean, self.fs)

        # 时域特征
        time_feat = self.features.time_domain_features(eeg_clean)

        return {
            "raw": eeg_signal,
            "filtered": eeg_clean,
            "band_powers": band_powers,
            "time_features": time_feat,
        }

    def process_emg(self, emg_signal: Optional[np.ndarray] = None,
                    duration: float = 5.0, contraction: float = 0.5,
                    fatigue: bool = False) -> Dict:
        """完整EMG处理流程"""
        if emg_signal is None:
            sig = self.generator.generate_emg(
                duration=duration, fs=self.fs,
                contraction_level=contraction, fatigue=fatigue
            )
            emg_signal = sig.data

        # 带通20-500Hz
        emg_bp = self.filters.bandpass_filter(emg_signal, self.fs, 20, 500)

        # 去工频
        emg_clean = self.filters.notch_filter(emg_bp, self.fs, 50.0)

        # 全波整流
        emg_rect = np.abs(emg_clean)

        # 特征
        emg_feat = self.features.emg_features(emg_clean, self.fs)

        return {
            "raw": emg_signal,
            "filtered": emg_clean,
            "rectified": emg_rect,
            "emg_features": emg_feat,
        }

    def full_analysis(self) -> Dict:
        """三种信号的完整分析"""
        return {
            "ecg": self.process_ecg(),
            "eeg": self.process_eeg(),
            "emg": self.process_emg(fatigue=True),
        }


# ============================================================
# 6. 可视化 (可选)
# ============================================================

def plot_results(results: Dict, save_path: Optional[str] = None):
    """绘制分析结果"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib未安装, 跳过绘图")
        return

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("生物信号处理仿真结果", fontsize=14)

    # ECG
    if "ecg" in results:
        ecg = results["ecg"]
        ax = axes[0, 0]
        t = np.arange(len(ecg["raw"])) / 500
        ax.plot(t[:1000], ecg["raw"][:1000], alpha=0.5, label="原始")
        ax.plot(t[:1000], ecg["filtered"][:1000], label="滤波后")
        ax.set_title("ECG信号")
        ax.legend()
        ax.set_xlabel("时间 (s)")

        ax = axes[0, 1]
        feat = ecg["ecg_features"]
        info = f"心率: {feat.get('mean_hr', 'N/A'):.1f} bpm\n"
        info += f"SDNN: {feat.get('sdnn', 'N/A'):.1f} ms\n"
        info += f"RMSSD: {feat.get('rmssd', 'N/A'):.1f} ms\n"
        info += f"LF/HF: {feat.get('lf_hf_ratio', 'N/A'):.2f}"
        ax.text(0.1, 0.5, info, transform=ax.transAxes, fontsize=12,
                verticalalignment='center', fontfamily='monospace')
        ax.set_title("ECG特征 (HRV)")
        ax.axis("off")

    # EEG
    if "eeg" in results:
        eeg = results["eeg"]
        ax = axes[1, 0]
        t = np.arange(len(eeg["raw"])) / 256
        ax.plot(t[:1000], eeg["raw"][:1000], alpha=0.5, label="原始")
        ax.plot(t[:1000], eeg["filtered"][:1000], label="滤波后")
        ax.set_title("EEG信号")
        ax.legend()

        ax = axes[1, 1]
        bp = eeg["band_powers"]
        bands = ["delta", "theta", "alpha", "beta", "gamma"]
        powers = [bp.get(f"{b}_relative", 0) for b in bands]
        ax.bar(bands, powers, color=["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"])
        ax.set_title(f"EEG频段功率 (主导: {bp.get('dominant_band', 'N/A')})")
        ax.set_ylabel("相对功率")

    # EMG
    if "emg" in results:
        emg = results["emg"]
        ax = axes[2, 0]
        t = np.arange(len(emg["raw"])) / 1000
        ax.plot(t[:2000], emg["raw"][:2000], alpha=0.5, label="原始")
        ax.plot(t[:2000], emg["filtered"][:2000], label="滤波后")
        ax.set_title("EMG信号")
        ax.legend()

        ax = axes[2, 1]
        feat = emg["emg_features"]
        info = f"RMS: {feat.get('mean_rms', 'N/A'):.4f}\n"
        info += f"MDF斜率: {feat.get('mdf_slope', 'N/A'):.4f}\n"
        info += f"疲劳指数: {feat.get('fatigue_index', 'N/A'):.4f}\n"
        info += f"估计肌力: {feat.get('estimated_force', 'N/A'):.2f}"
        ax.text(0.1, 0.5, info, transform=ax.transAxes, fontsize=12,
                verticalalignment='center', fontfamily='monospace')
        ax.set_title("EMG特征")
        ax.axis("off")

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[OK] 图像已保存: {save_path}")
    plt.show()


# ============================================================
# 7. 主程序
# ============================================================

def main():
    print("=" * 60)
    print("  生物信号处理仿真 - ECG/EEG/EMG")
    print("=" * 60)

    pipeline = BioSignalPipeline(fs=500.0)

    # ECG分析
    print("\n[1] ECG信号分析...")
    ecg_result = pipeline.process_ecg(duration=10.0)
    feat = ecg_result["ecg_features"]
    print(f"  心率: {feat.get('mean_hr', 0):.1f} bpm")
    print(f"  SDNN: {feat.get('sdnn', 0):.1f} ms")
    print(f"  RMSSD: {feat.get('rmssd', 0):.1f} ms")
    print(f"  LF/HF: {feat.get('lf_hf_ratio', 0):.2f}")
    print(f"  检测到 {feat.get('n_beats', 0)} 个心跳")

    # EEG分析
    print("\n[2] EEG信号分析...")
    eeg_result = pipeline.process_eeg(duration=10.0, state="relaxed")
    bp = eeg_result["band_powers"]
    print(f"  主导频段: {bp.get('dominant_band', 'N/A')}")
    for band in ["delta", "theta", "alpha", "beta", "gamma"]:
        print(f"  {band:8s}: {bp.get(f'{band}_relative', 0)*100:5.1f}%")

    # EMG分析
    print("\n[3] EMG信号分析 (含疲劳)...")
    emg_result = pipeline.process_emg(duration=5.0, fatigue=True)
    ef = emg_result["emg_features"]
    print(f"  RMS: {ef.get('mean_rms', 0):.4f}")
    print(f"  MDF斜率: {ef.get('mdf_slope', 0):.4f}")
    print(f"  疲劳指数: {ef.get('fatigue_index', 0):.4f}")
    print(f"  估计肌力: {ef.get('estimated_force', 0):.2f}")

    # 滤波效果演示
    print("\n[4] 滤波器演示...")
    gen = SignalGenerator()
    raw_ecg = gen.generate_ecg(duration=2.0, noise_level=0.2).data
    filt = BioFilters()

    denoised = filt.wavelet_denoise(raw_ecg)
    snr_before = 10 * np.log10(np.var(raw_ecg) / (np.var(raw_ecg - denoised) + 1e-10))
    print(f"  小波去噪 SNR提升: {snr_before:.1f} dB")

    noise_ref = np.random.randn(len(raw_ecg))
    _, error = filt.adaptive_filter_lms(raw_ecg, noise_ref, mu=0.001)
    print(f"  自适应滤波 收敛误差: {np.mean(error[-100:]**2):.6f}")

    print("\n" + "=" * 60)
    print("  分析完成!")
    print("=" * 60)

    # 可选绘图
    try:
        results = {"ecg": ecg_result, "eeg": eeg_result, "emg": emg_result}
        plot_results(results)
    except Exception as e:
        print(f"[INFO] 绘图跳过: {e}")

    return {"ecg": ecg_result, "eeg": eeg_result, "emg": emg_result}


if __name__ == "__main__":
    main()
