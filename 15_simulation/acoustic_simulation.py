#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
声学仿真模块
=============
声压级计算 | 混响仿真 | 麦克风阵列波束形成 | 声源定位

适用于电赛中声学传感器、麦克风阵列、噪声检测等场景。
"""

import numpy as np
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass


# ============================================================
# 物理常数
# ============================================================
SPEED_OF_SOUND = 343.0       # m/s (20°C空气)
AIR_DENSITY = 1.225          # kg/m³
REFERENCE_PRESSURE = 20e-6   # Pa (听阈声压)


# ============================================================
# 数据结构
# ============================================================
@dataclass
class SoundFieldResult:
    """声场计算结果"""
    positions: np.ndarray       # 位置坐标
    pressure: np.ndarray        # 声压 (Pa)
    spl_db: np.ndarray          # 声压级 (dB)
    intensity: np.ndarray       # 声强 (W/m²)


@dataclass
class ReverberationResult:
    """混响计算结果"""
    rt60: float                 # 混响时间 (s)
    edt: float                  # 早期衰减时间 (s)
    clarity_c50: float          # 明晰度 C50 (dB)
    d50: float                  # 直达声能量比
    impulse_response: np.ndarray  # 房间脉冲响应
    time_axis: np.ndarray       # 时间轴


@dataclass
class BeamformingResult:
    """波束形成结果"""
    steering_angles: np.ndarray  # 扫描角度
    beam_pattern: np.ndarray     # 波束图案 (dB)
    estimated_doa: float         # 估计到达角 (degrees)
    array_gain_db: float         # 阵列增益 (dB)
    spatial_spectrum: np.ndarray # 空间谱


# ============================================================
# 声压级计算
# ============================================================
class SPLCalculator:
    """声压级(SPL)计算器"""

    @staticmethod
    def pressure_to_spl(pressure: float, p_ref: float = REFERENCE_PRESSURE) -> float:
        """声压转声压级 (dB SPL)"""
        if pressure <= 0:
            return -np.inf
        return 20 * np.log10(pressure / p_ref)

    @staticmethod
    def spl_to_pressure(spl_db: float, p_ref: float = REFERENCE_PRESSURE) -> float:
        """声压级转声压 (Pa)"""
        return p_ref * 10 ** (spl_db / 20)

    @staticmethod
    def add_spl_levels(spl_list: List[float]) -> float:
        """叠加多个声压级 (非相干叠加)"""
        total_power = sum(10 ** (s / 10) for s in spl_list)
        return 10 * np.log10(total_power)

    @staticmethod
    def a_weighting(frequencies: np.ndarray) -> np.ndarray:
        """
        A计权曲线 (IEC 61672)

        Args:
            frequencies: 频率数组 (Hz)

        Returns:
            A计权值 (dB)
        """
        f2 = frequencies ** 2
        ra = (12194**2 * f2**2) / (
            (f2 + 20.6**2) *
            np.sqrt((f2 + 107.7**2) * (f2 + 737.9**2)) *
            (f2 + 12194**2)
        )
        a_weight = 20 * np.log10(ra + 1e-30) + 2.0
        return a_weight

    @staticmethod
    def apply_a_weighting(spl_values: np.ndarray, frequencies: np.ndarray) -> np.ndarray:
        """应用A计权"""
        aw = SPLCalculator.a_weighting(frequencies)
        return spl_values + aw

    @staticmethod
    def octave_bands(center_freqs: Optional[np.ndarray] = None) -> Dict:
        """
        标准1/1倍频程频带

        Returns:
            {'center': [...], 'lower': [...], 'upper': [...]}
        """
        if center_freqs is None:
            center_freqs = np.array([31.5, 63, 125, 250, 500, 1000, 2000, 4000, 8000, 16000])
        factor = 2 ** 0.5
        return {
            'center': center_freqs,
            'lower': center_freqs / factor,
            'upper': center_freqs * factor
        }


# ============================================================
# 混响仿真
# ============================================================
class ReverberationSimulator:
    """室内声学混响仿真"""

    @staticmethod
    def sabine_rt60(volume: float, surface_area: float,
                    absorption_coeff: float) -> float:
        """
        Sabine混响时间公式

        Args:
            volume: 房间体积 (m³)
            surface_area: 总表面积 (m²)
            absorption_coeff: 平均吸声系数

        Returns:
            RT60 (s)
        """
        total_absorption = surface_area * absorption_coeff
        if total_absorption < 1e-10:
            return float('inf')
        return 0.161 * volume / total_absorption

    @staticmethod
    def eyring_rt60(volume: float, surface_area: float,
                    absorption_coeff: float) -> float:
        """
        Eyring混响时间公式 (高吸声系数时更准确)

        Args:
            volume: 房间体积 (m³)
            surface_area: 总表面积 (m²)
            absorption_coeff: 平均吸声系数

        Returns:
            RT60 (s)
        """
        if absorption_coeff >= 1.0:
            return 0.0
        return 0.161 * volume / (-surface_area * np.log(1 - absorption_coeff))

    @staticmethod
    def generate_rir(room_dims: Tuple[float, float, float],
                     source_pos: Tuple[float, float, float],
                     receiver_pos: Tuple[float, float, float],
                     rt60: float = 0.5,
                     fs: int = 16000,
                     max_order: int = 10) -> Tuple[np.ndarray, np.ndarray]:
        """
        用镜像源法生成房间脉冲响应 (RIR)

        Args:
            room_dims: 房间尺寸 (x, y, z) m
            source_pos: 声源位置 (x, y, z) m
            receiver_pos: 接收位置 (x, y, z) m
            rt60: 混响时间 (s)
            fs: 采样率
            max_order: 最大镜像源阶数

        Returns:
            (rir, time_axis)
        """
        lx, ly, lz = room_dims
        xs, ys, zs = source_pos
        xr, yr, zr = receiver_pos

        # 计算每面墙的反射系数 (简化为均匀)
        # RT60 ≈ 0.161*V / (-S*ln(1-α)), 所以 α = 1 - exp(-0.161*V/(S*RT60))
        S = 2 * (lx*ly + ly*lz + lx*lz)
        V = lx * ly * lz
        if rt60 > 0 and S > 0:
            alpha = 1 - np.exp(-0.161 * V / (S * rt60))
        else:
            alpha = 0.5
        refl_coeff = np.sqrt(1 - alpha)

        # 镜像源法
        duration = rt60 * 1.5
        n_samples = int(duration * fs)
        rir = np.zeros(n_samples)
        time_axis = np.arange(n_samples) / fs

        for mx in range(-max_order, max_order + 1):
            for my in range(-max_order, max_order + 1):
                for mz in range(-max_order, max_order + 1):
                    # 镜像源位置
                    if mx % 2 == 0:
                        img_x = mx * lx + xs
                    else:
                        img_x = mx * lx + (lx - xs)

                    if my % 2 == 0:
                        img_y = my * ly + ys
                    else:
                        img_y = my * ly + (ly - ys)

                    if mz % 2 == 0:
                        img_z = mz * lz + zs
                    else:
                        img_z = mz * lz + (lz - zs)

                    # 距离
                    dist = np.sqrt((img_x - xr)**2 + (img_y - yr)**2 + (img_z - zr)**2)
                    if dist < 0.01:
                        dist = 0.01

                    # 到达时间
                    t_arrival = dist / SPEED_OF_SOUND
                    sample_idx = int(t_arrival * fs)

                    if sample_idx >= n_samples:
                        continue

                    # 反射次数
                    n_reflections = abs(mx) + abs(my) + abs(mz)
                    amplitude = refl_coeff ** n_reflections / (4 * np.pi * dist)

                    # 空气吸收衰减
                    air_absorption = np.exp(-0.005 * dist)
                    amplitude *= air_absorption

                    rir[sample_idx] += amplitude

        # 归一化
        rir /= np.max(np.abs(rir)) + 1e-30

        return rir, time_axis

    @staticmethod
    def measure_rt60(rir: np.ndarray, fs: int) -> Dict:
        """
        从脉冲响应测量RT60、EDT、C50

        Returns:
            {'rt60': ..., 'edt': ..., 'c50': ..., 'd50': ...}
        """
        # 能量衰减曲线
        energy = rir ** 2
        cum_energy = np.cumsum(energy[::-1])[::-1]  # 反向累积

        # 转dB
        edc = 10 * np.log10(cum_energy / (np.max(cum_energy) + 1e-30) + 1e-30)

        # RT60: -5dB到-65dB的斜率
        t_axis = np.arange(len(rir)) / fs

        try:
            idx_5 = np.where(edc <= -5)[0][0]
            idx_35 = np.where(edc <= -35)[0][0]
            slope = (t_axis[idx_35] - t_axis[idx_5]) / 30.0  # s/dB
            rt60 = slope * 60
        except (IndexError, ZeroDivisionError):
            rt60 = 0.0

        # EDT: 0到-10dB
        try:
            idx_0 = 0
            idx_10 = np.where(edc <= -10)[0][0]
            edt = (t_axis[idx_10] - t_axis[idx_0]) * 6  # 扩展到-60dB
        except (IndexError):
            edt = 0.0

        # C50: 前50ms能量与后50ms能量之比
        idx_50ms = int(0.05 * fs)
        early = np.sum(energy[:idx_50ms])
        late = np.sum(energy[idx_50ms:])
        c50 = 10 * np.log10((early + 1e-30) / (late + 1e-30))
        d50 = early / (early + late + 1e-30)

        return {
            'rt60': max(rt60, 0),
            'edt': max(edt, 0),
            'c50': c50,
            'd50': d50
        }


# ============================================================
# 麦克风阵列波束形成
# ============================================================
class BeamformingProcessor:
    """麦克风阵列波束形成处理器"""

    @staticmethod
    def linear_array_positions(n_elements: int, spacing: float) -> np.ndarray:
        """
        生成均匀线阵坐标

        Args:
            n_elements: 阵元数
            spacing: 阵元间距 (m)

        Returns:
            阵元位置 (n x 3) [x, y, z]
        """
        positions = np.zeros((n_elements, 3))
        positions[:, 0] = np.arange(n_elements) * spacing
        # 中心化
        positions[:, 0] -= np.mean(positions[:, 0])
        return positions

    @staticmethod
    def circular_array_positions(n_elements: int, radius: float) -> np.ndarray:
        """生成均匀圆阵坐标"""
        positions = np.zeros((n_elements, 3))
        angles = np.linspace(0, 2 * np.pi, n_elements, endpoint=False)
        positions[:, 0] = radius * np.cos(angles)
        positions[:, 1] = radius * np.sin(angles)
        return positions

    @staticmethod
    def steering_vector(array_pos: np.ndarray, angle_deg: float,
                        frequency: float) -> np.ndarray:
        """
        计算导向矢量

        Args:
            array_pos: 阵元位置 (n x 3)
            angle_deg: 导向角 (degrees)
            frequency: 频率 (Hz)

        Returns:
            导向矢量 (n x 1) complex
        """
        angle_rad = np.radians(angle_deg)
        # 单位方向矢量 (假设信号来自x-y平面)
        direction = np.array([np.cos(angle_rad), np.sin(angle_rad), 0])

        # 相位延迟
        delays = array_pos @ direction / SPEED_OF_SOUND
        sv = np.exp(-1j * 2 * np.pi * frequency * delays)
        return sv

    @staticmethod
    def conventional_beamformer(array_signals: np.ndarray,
                                 array_pos: np.ndarray,
                                 scan_angles: np.ndarray,
                                 frequency: float,
                                 fs: float) -> BeamformingResult:
        """
        常规(延迟-求和)波束形成

        Args:
            array_signals: 各阵元信号 (n_elements x n_samples)
            array_pos: 阵元位置
            scan_angles: 扫描角度范围 (degrees)
            frequency: 分析频率 (Hz)
            fs: 采样率

        Returns:
            BeamformingResult
        """
        n_elements = array_signals.shape[0]
        spatial_spectrum = np.zeros(len(scan_angles))

        # 各阵元信号的频域表示
        fft_signals = np.fft.rfft(array_signals, axis=1)
        freqs = np.fft.rfftfreq(array_signals.shape[1], 1.0 / fs)
        freq_idx = np.argmin(np.abs(freqs - frequency))
        target_freq_data = fft_signals[:, freq_idx]

        for i, angle in enumerate(scan_angles):
            sv = BeamformingProcessor.steering_vector(array_pos, angle, frequency)
            # 延迟-求和
            output = np.conj(sv) @ target_freq_data / n_elements
            spatial_spectrum[i] = np.abs(output) ** 2

        # 转dB
        beam_pattern_db = 10 * np.log10(spatial_spectrum / (np.max(spatial_spectrum) + 1e-30) + 1e-30)

        # 估计DOA
        estimated_doa = scan_angles[np.argmax(spatial_spectrum)]

        # 阵列增益
        # 假设噪声为各向同性
        noise_power = 1.0
        array_gain = spatial_spectrum[np.argmax(spatial_spectrum)] / (noise_power / n_elements)
        array_gain_db = 10 * np.log10(array_gain + 1e-30)

        return BeamformingResult(
            steering_angles=scan_angles,
            beam_pattern=beam_pattern_db,
            estimated_doa=estimated_doa,
            array_gain_db=array_gain_db,
            spatial_spectrum=spatial_spectrum
        )

    @staticmethod
    def music_algorithm(array_signals: np.ndarray,
                        array_pos: np.ndarray,
                        n_sources: int,
                        scan_angles: np.ndarray,
                        frequency: float,
                        fs: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        MUSIC算法声源定位

        Args:
            array_signals: 阵元信号 (n_elements x n_samples)
            array_pos: 阵元位置
            n_sources: 声源数量
            scan_angles: 扫描角度
            frequency: 频率
            fs: 采样率

        Returns:
            (scan_angles, music_spectrum_dB)
        """
        n_elements = array_signals.shape[0]

        # 协方差矩阵
        fft_signals = np.fft.rfft(array_signals, axis=1)
        freqs = np.fft.rfftfreq(array_signals.shape[1], 1.0 / fs)
        freq_idx = np.argmin(np.abs(freqs - frequency))
        X = fft_signals[:, freq_idx:freq_idx+1]  # n_elements x 1

        # 使用多快拍估计协方差
        n_segments = min(10, array_signals.shape[1] // 256)
        if n_segments < 2:
            # 单快拍 - 用空间平滑
            R = X @ X.conj().T
        else:
            seg_len = array_signals.shape[1] // n_segments
            R = np.zeros((n_elements, n_elements), dtype=complex)
            for s in range(n_segments):
                seg = array_signals[:, s*seg_len:(s+1)*seg_len]
                X_seg = np.fft.rfft(seg, axis=1)
                x = X_seg[:, freq_idx]
                R += np.outer(x, x.conj())
            R /= n_segments

        # 特征分解
        eigenvalues, eigenvectors = np.linalg.eigh(R)

        # 噪声子空间
        sorted_idx = np.argsort(eigenvalues)[::-1]
        noise_subspace = eigenvectors[:, sorted_idx[n_sources:]]

        # MUSIC谱
        music_spectrum = np.zeros(len(scan_angles))
        for i, angle in enumerate(scan_angles):
            sv = BeamformingProcessor.steering_vector(array_pos, angle, frequency)
            denominator = sv.conj() @ noise_subspace @ noise_subspace.conj().T @ sv
            music_spectrum[i] = 1.0 / (np.abs(denominator) + 1e-30)

        music_dB = 10 * np.log10(music_spectrum / (np.max(music_spectrum) + 1e-30) + 1e-30)

        return scan_angles, music_dB

    @staticmethod
    def simulate_array_signals(array_pos: np.ndarray,
                                source_angles: List[float],
                                source_freqs: List[float],
                                source_amplitudes: List[float],
                                fs: float, duration: float,
                                noise_level: float = 0.1) -> np.ndarray:
        """
        仿真麦克风阵列接收到的信号

        Returns:
            阵列信号 (n_elements x n_samples)
        """
        n_elements = array_pos.shape[0]
        n_samples = int(duration * fs)
        t = np.arange(n_samples) / fs

        signals = np.zeros((n_elements, n_samples))

        for angle, freq, amp in zip(source_angles, source_freqs, source_amplitudes):
            # 导向矢量
            sv = BeamformingProcessor.steering_vector(array_pos, angle, freq)
            # 信号
            source_signal = amp * np.sin(2 * np.pi * freq * t)
            # 各阵元接收
            for m in range(n_elements):
                signals[m] += np.real(sv[m]) * source_signal

        # 加噪声
        signals += noise_level * np.random.randn(n_elements, n_samples)

        return signals


# ============================================================
# 声源定位 (TDOA)
# ============================================================
class SoundLocalizer:
    """基于TDOA的声源定位"""

    @staticmethod
    def gcc_phat(sig1: np.ndarray, sig2: np.ndarray,
                 fs: float, max_delay: float = None) -> Tuple[float, np.ndarray, np.ndarray]:
        """
        广义互相关-相位变换 (GCC-PHAT)

        Args:
            sig1, sig2: 两通道信号
            fs: 采样率
            max_delay: 最大延迟 (s)

        Returns:
            (估计延迟, 互相关函数, 延迟轴)
        """
        N = len(sig1) + len(sig2) - 1
        N_fft = 2 ** int(np.ceil(np.log2(N)))

        SIG1 = np.fft.rfft(sig1, N_fft)
        SIG2 = np.fft.rfft(sig2, N_fft)

        # PHAT加权
        cross_spectrum = SIG1 * np.conj(SIG2)
        magnitude = np.abs(cross_spectrum)
        magnitude[magnitude < 1e-10] = 1e-10
        weighted = cross_spectrum / magnitude

        cc = np.fft.irfft(weighted, N_fft)
        cc = np.concatenate([cc[-(N_fft//2):], cc[:N_fft//2]])

        if max_delay is not None:
            max_samples = int(max_delay * fs)
            center = N_fft // 2
            cc[:center - max_samples] = 0
            cc[center + max_samples:] = 0

        peak_idx = np.argmax(np.abs(cc))
        delay_samples = peak_idx - N_fft // 2
        delay_time = delay_samples / fs

        lags = (np.arange(N_fft) - N_fft // 2) / fs

        return delay_time, cc, lags

    @staticmethod
    def tdoa_localize_2d(mic_positions: np.ndarray,
                         tdoa_values: np.ndarray,
                         ref_index: int = 0) -> Tuple[float, float]:
        """
        基于TDOA的2D声源定位 (最小二乘法)

        Args:
            mic_positions: 麦克风位置 (n x 2)
            tdoa_values: 相对于参考麦克风的TDOA (n-1,)
            ref_index: 参考麦克风索引

        Returns:
            (x, y) 估计声源位置
        """
        n_mics = mic_positions.shape[0]
        ref_pos = mic_positions[ref_index]

        # 将TDOA转为距离差
        distances = tdoa_values * SPEED_OF_SOUND

        # 构建线性方程组 (Fang's method简化版)
        # 假设声源在远场，用角度定位
        A = []
        b = []
        for i in range(n_mics):
            if i == ref_index:
                continue
            mic_i = mic_positions[i]
            d_i = distances[i - (1 if i > ref_index else 0)]

            # 线性化方程
            A.append([
                mic_i[0] - ref_pos[0],
                mic_i[1] - ref_pos[1]
            ])
            b.append(d_i**2 / 2 + d_i * np.linalg.norm(ref_pos))

        A = np.array(A)
        b = np.array(b)

        # 最小二乘解
        if A.shape[0] >= 2 and np.linalg.matrix_rank(A) >= 2:
            result = np.linalg.lstsq(A, b, rcond=None)
            return result[0][0], result[0][1]
        else:
            return 0.0, 0.0


# ============================================================
# 演示 / 测试
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("  声学仿真模块 - 功能演示")
    print("=" * 60)

    # 1. 声压级计算
    print("\n[1] 声压级计算...")
    for ref_level in [20e-6, 0.002, 0.02, 0.2, 2.0]:
        spl = SPLCalculator.pressure_to_spl(ref_level)
        print(f"  声压 {ref_level*1000:.3f} mPa -> SPL {spl:.1f} dB")

    # SPL叠加
    combined = SPLCalculator.add_spl_levels([70, 75, 80, 65])
    print(f"  多源叠加 (70+75+80+65 dB): {combined:.1f} dB")

    # A计权
    freqs = np.array([125, 250, 500, 1000, 2000, 4000, 8000])
    aw = SPLCalculator.a_weighting(freqs)
    print(f"  A计权: {dict(zip(freqs.astype(int), aw.round(1)))}")

    # 2. 混响仿真
    print("\n[2] 混响仿真...")
    # 教室参数
    volume = 10 * 8 * 3  # 240 m³
    S = 2 * (10*8 + 8*3 + 10*3)  # 268 m²
    for alpha in [0.1, 0.3, 0.5, 0.8]:
        rt60 = ReverberationSimulator.sabine_rt60(volume, S, alpha)
        print(f"  α={alpha:.1f}: RT60={rt60:.2f}s")

    # 生成RIR
    print("\n  生成房间脉冲响应...")
    rir, t_rir = ReverberationSimulator.generate_rir(
        room_dims=(10, 8, 3),
        source_pos=(2, 4, 1.5),
        receiver_pos=(8, 4, 1.5),
        rt60=0.5, fs=16000, max_order=5
    )
    rir_info = ReverberationSimulator.measure_rt60(rir, 16000)
    print(f"  测量RT60: {rir_info['rt60']:.3f}s, EDT: {rir_info['edt']:.3f}s")
    print(f"  C50: {rir_info['c50']:.1f}dB, D50: {rir_info['d50']:.3f}")

    # 3. 波束形成
    print("\n[3] 麦克风阵列波束形成...")
    n_mics = 8
    spacing = 0.04  # 4cm
    array_pos = BeamformingProcessor.linear_array_positions(n_mics, spacing)
    print(f"  {n_mics}元均匀线阵, 间距 {spacing*100:.0f}cm")

    # 仿真信号
    signals = BeamformingProcessor.simulate_array_signals(
        array_pos,
        source_angles=[30, -20],
        source_freqs=[1000, 1500],
        source_amplitudes=[1.0, 0.5],
        fs=16000, duration=0.1, noise_level=0.1
    )
    print(f"  信号矩阵: {signals.shape}")

    # 常规波束形成
    scan_angles = np.linspace(-90, 90, 181)
    bf_result = BeamformingProcessor.conventional_beamformer(
        signals, array_pos, scan_angles,
        frequency=1000, fs=16000
    )
    print(f"  估计DOA: {bf_result.estimated_doa:.1f}°")
    print(f"  阵列增益: {bf_result.array_gain_db:.1f} dB")

    # MUSIC算法
    angles, music_spec = BeamformingProcessor.music_algorithm(
        signals, array_pos, n_sources=2,
        scan_angles=scan_angles, frequency=1000, fs=16000
    )
    peak_angles = angles[np.argsort(music_spec)[-2:]]
    print(f"  MUSIC估计DOA: {sorted(peak_angles, reverse=True)}")

    # 4. TDOA定位
    print("\n[4] TDOA声源定位...")
    mic_pos_2d = np.array([[0, 0], [1, 0], [0.5, 0.866]])
    # 模拟TDOA
    source = np.array([3, 4])
    delays = []
    for i in range(1, len(mic_pos_2d)):
        d0 = np.linalg.norm(source - mic_pos_2d[0])
        di = np.linalg.norm(source - mic_pos_2d[i])
        delays.append((di - d0) / SPEED_OF_SOUND)
    tdoa = np.array(delays)
    est_x, est_y = SoundLocalizer.tdoa_localize_2d(mic_pos_2d, tdoa)
    print(f"  真实位置: ({source[0]:.1f}, {source[1]:.1f})")
    print(f"  估计位置: ({est_x:.2f}, {est_y:.2f})")

    print("\n" + "=" * 60)
    print("  声学仿真完成!")
    print("=" * 60)
