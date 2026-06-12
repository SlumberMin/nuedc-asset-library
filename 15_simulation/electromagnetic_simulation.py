#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电磁场仿真模块
===============
传输线仿真 | 串扰分析 | EMI/EMC | 屏蔽效能

适用于电赛中信号完整性、PCB布局、电磁兼容等场景。
"""

import numpy as np
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass


# ============================================================
# 物理常数
# ============================================================
MU_0 = 4 * np.pi * 1e-7       # 真空磁导率 (H/m)
EPS_0 = 8.854e-12              # 真空介电常数 (F/m)
C_LIGHT = 3e8                  # 光速 (m/s)
ETA_0 = 376.73                 # 自由空间波阻抗 (Ω)


# ============================================================
# 数据结构
# ============================================================
@dataclass
class TransmissionLineResult:
    """传输线仿真结果"""
    time: np.ndarray
    voltage: np.ndarray         # 沿线电压分布
    current: np.ndarray         # 沿线电流分布
    reflection_coeff: float     # 反射系数
    swr: float                  # 驻波比
    characteristic_impedance: float
    propagation_delay: float    # 传播延迟 (ns)


@dataclass
class CrosstalkResult:
    """串扰分析结果"""
    frequency: np.ndarray
    fext_near: np.ndarray       # 近端串扰 (NEXT) dB
    fext_far: np.ndarray        # 远端串扰 (FEXT) dB
    coupled_voltage: np.ndarray # 耦合电压
    peak_crosstalk_db: float


@dataclass
class ShieldingResult:
    """屏蔽效能结果"""
    frequency: np.ndarray
    se_total_db: np.ndarray     # 总屏蔽效能 (dB)
    se_absorption_db: np.ndarray  # 吸收损耗
    se_reflection_db: np.ndarray  # 反射损耗
    se_multiple_db: np.ndarray    # 多次反射修正


# ============================================================
# 传输线仿真
# ============================================================
class TransmissionLineSimulator:
    """传输线仿真器 - 基于电报方程"""

    @staticmethod
    def microstrip_impedance(width: float, height: float,
                             er: float = 4.4, thickness: float = 0) -> float:
        """
        微带线特性阻抗计算 (IPC-2141公式)

        Args:
            width: 线宽 (m)
            height: 介质厚度 (m)
            er: 相对介电常数 (FR4 ≈ 4.4)
            thickness: 铜箔厚度 (m)

        Returns:
            特性阻抗 (Ω)
        """
        w = width
        h = height
        t = thickness

        # 有效介电常数
        if w / h <= 1:
            er_eff = (er + 1) / 2 + (er - 1) / 2 * (
                (1 / np.sqrt(1 + 12 * h / w)) + 0.04 * (1 - w / h)**2
            )
            z0 = (60 / np.sqrt(er_eff)) * np.log(
                8 * h / w + w / (4 * h)
            )
        else:
            er_eff = (er + 1) / 2 + (er - 1) / 2 * (
                1 / np.sqrt(1 + 12 * h / w)
            )
            z0 = (120 * np.pi) / (
                np.sqrt(er_eff) * (w / h + 1.393 + 0.667 * np.log(w / h + 1.444))
            )

        # 铜箔厚度修正
        if t > 0:
            delta_w = (t / np.pi) * (1 + np.log(4 * np.pi * w / t))
            w_eff = w + delta_w
            z0 *= w / w_eff  # 简化修正

        return z0

    @staticmethod
    def stripline_impedance(width: float, height: float,
                             er: float = 4.4) -> float:
        """
        带状线特性阻抗

        Args:
            width: 线宽 (m)
            height: 上下参考层间距 (m)
            er: 介电常数

        Returns:
            特性阻抗 (Ω)
        """
        b = height
        w = width
        x = w / b
        ki = np.tanh(np.pi * x / 2)
        ki_prime = np.sqrt(1 - ki**2)

        # 使用椭圆积分近似
        from math import log as mlog
        if x < 0.7:
            K_ratio = np.pi / (mlog(2 * (1 + np.sqrt(ki_prime)) / (1 - np.sqrt(ki_prime))) + 1e-30)
        else:
            K_ratio = (mlog(2 * (1 + np.sqrt(ki)) / (1 - np.sqrt(ki))) + 1e-30) / np.pi

        z0 = 30 * np.pi / (np.sqrt(er) * K_ratio)
        return z0

    @staticmethod
    def propagation_params(z0: float, er_eff: float,
                            length: float) -> Dict:
        """
        传输线传播参数

        Returns:
            {'phase_velocity': ..., 'propagation_delay': ..., 'delay_per_m': ...}
        """
        vp = C_LIGHT / np.sqrt(er_eff)       # 相速度
        td = length / vp                       # 传播延迟
        td_per_m = 1.0 / vp                    # 延迟/m (s/m)

        return {
            'phase_velocity': vp,
            'propagation_delay': td,
            'delay_per_m': td_per_m * 1e9,     # ns/m
            'delay_ns': td * 1e9               # ns
        }

    @staticmethod
    def step_response(z0: float, zs: float, zl: float,
                      length: float, er_eff: float,
                      v_source: float = 1.0,
                      n_bounces: int = 10,
                      n_points: int = 1000) -> TransmissionLineResult:
        """
        传输线阶跃响应 (弹跳图)

        Args:
            z0: 特性阻抗
            zs: 源阻抗
            zl: 负载阻抗
            length: 线长
            er_eff: 有效介电常数
            v_source: 源电压
            n_bounces: 反弹次数
            n_points: 时间点数

        Returns:
            TransmissionLineResult
        """
        # 反射系数
        gamma_s = (zs - z0) / (zs + z0)
        gamma_l = (zl - z0) / (zl + z0)

        # 初始电压
        v_init = v_source * z0 / (zs + z0)

        # 传播延迟
        vp = C_LIGHT / np.sqrt(er_eff)
        td = length / vp

        # 时间轴 (覆盖多次反弹)
        t_end = td * (2 * n_bounces + 1) * 1.2
        t = np.linspace(0, t_end, n_points)
        voltage_load = np.zeros(n_points)
        voltage_source = np.zeros(n_points)

        # 逐步叠加各次反射
        v_forward = v_init
        for bounce in range(n_bounces + 1):
            # 到达负载的时刻
            t_load = td * (2 * bounce + 1)
            t_source = td * 2 * (bounce + 1)

            for i, ti in enumerate(t):
                if ti >= t_load:
                    voltage_load[i] += v_forward * (1 + gamma_l)
                if ti >= t_source:
                    v_reflected = v_forward * gamma_l
                    voltage_source[i] += v_reflected * (1 + gamma_s)

            # 更新下一次前向波
            v_forward = v_forward * gamma_l * gamma_s

        # 沿线稳态分布 (最终值)
        steady_state = voltage_load[-1]

        return TransmissionLineResult(
            time=t,
            voltage=voltage_load,
            current=voltage_load / zl if zl > 0 else voltage_load / 1e10,
            reflection_coeff=gamma_l,
            swr=(1 + abs(gamma_l)) / (1 - abs(gamma_l)) if abs(gamma_l) < 1 else float('inf'),
            characteristic_impedance=z0,
            propagation_delay=td * 1e9  # ns
        )

    @staticmethod
    def s_parameters(z0: float, zs: float, zl: float,
                     length: float, er_eff: float,
                     frequencies: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        计算二端口S参数

        Returns:
            (freqs, S11_dB, S21_dB, S11_phase)
        """
        vp = C_LIGHT / np.sqrt(er_eff)
        gamma_l = (zl - z0) / (zl + z0)
        gamma_s = (zs - z0) / (zs + z0)

        # 传播常数
        beta = 2 * np.pi * frequencies / vp

        # 电长度
        theta = beta * length

        # ABCD参数转S参数 (简化)
        # 假设理想传输线
        A = np.cos(theta)
        B = 1j * z0 * np.sin(theta)
        C = 1j * np.sin(theta) / z0
        D = np.cos(theta)

        denom = A + B / z0 + C * z0 + D
        S11 = (A + B / z0 - C * z0 - D) / denom
        S21 = 2.0 / denom

        S11_dB = 20 * np.log10(np.abs(S11) + 1e-30)
        S21_dB = 20 * np.log10(np.abs(S21) + 1e-30)
        S11_phase = np.degrees(np.angle(S11))

        return frequencies, S11_dB, S21_dB, S11_phase


# ============================================================
# 串扰分析
# ============================================================
class CrosstalkAnalyzer:
    """PCB传输线串扰分析器"""

    @staticmethod
    def coupled_line_crosstalk(coupling_length: float,
                                even_z0: float, odd_z0: float,
                                er_eff: float,
                                frequencies: np.ndarray) -> CrosstalkResult:
        """
        耦合传输线串扰分析

        Args:
            coupling_length: 耦合长度 (m)
            even_z0: 偶模阻抗
            odd_z0: 奇模阻抗
            er_eff: 有效介电常数
            frequencies: 频率数组

        Returns:
            CrosstalkResult
        """
        vp = C_LIGHT / np.sqrt(er_eff)

        # 耦合系数
        K = (even_z0 - odd_z0) / (even_z0 + odd_z0)

        # 电长度
        theta = 2 * np.pi * frequencies * coupling_length / vp

        # NEXT (近端串扰) - 与耦合长度成正比
        next_coupling = K * np.sin(theta) / theta if any(theta > 0) else np.zeros_like(frequencies)
        next_coupling = np.where(np.abs(theta) > 1e-10,
                                  K * np.abs(np.sin(theta)),
                                  K * np.abs(theta))
        next_db = 20 * np.log10(next_coupling + 1e-30)

        # FEXT (远端串扰) - 与频率和长度平方成正比
        fext_coupling = K * coupling_length * 2 * np.pi * frequencies / (2 * vp)
        fext_coupling *= np.sinc(theta / np.pi)  # sinc函数
        fext_db = 20 * np.log10(np.abs(fext_coupling) + 1e-30)

        # 耦合电压 (假设1V驱动)
        coupled_v = next_coupling * 1.0

        return CrosstalkResult(
            frequency=frequencies,
            fext_near=next_db,
            fext_far=fext_db,
            coupled_voltage=coupled_v,
            peak_crosstalk_db=np.max(next_db)
        )

    @staticmethod
    def edge_coupled_microstrip_crosstalk(width: float, spacing: float,
                                           height: float, er: float,
                                           length: float,
                                           frequencies: np.ndarray) -> CrosstalkResult:
        """
        边耦合微带线串扰计算

        Args:
            width: 线宽 (m)
            spacing: 线间距 (m)
            height: 介质厚度 (m)
            er: 介电常数
            length: 耦合长度 (m)
            frequencies: 频率数组

        Returns:
            CrosstalkResult
        """
        # 近似偶/奇模阻抗 (IPC简化公式)
        s = spacing / height
        w = width / height

        # 耦合系数近似
        K = np.exp(-1.2 * s)  # 间距越大耦合越小

        z0_single = TransmissionLineSimulator.microstrip_impedance(width, height, er)
        even_z0 = z0_single * (1 + K)
        odd_z0 = z0_single * (1 - K)

        er_eff = (er + 1) / 2  # 简化

        return CrosstalkAnalyzer.coupled_line_crosstalk(
            length, even_z0, odd_z0, er_eff, frequencies
        )

    @staticmethod
    def guard_trace_spacing(trace_width: float, height: float,
                             er: float, required_isolation_db: float) -> float:
        """
        计算防护走线间距以满足隔离度要求

        Returns:
            建议的走线间距 (m)
        """
        # 基于经验公式：每1倍高度增加约6dB隔离
        n_heights = required_isolation_db / 6.0
        return max(trace_width * 3, height * n_heights)


# ============================================================
# EMI / EMC 分析
# ============================================================
class EMIAnalyzer:
    """电磁干扰分析器"""

    @staticmethod
    def radiated_emission_pcb(trace_length: float, current: float,
                               frequency: float,
                               distance: float = 3.0) -> Dict:
        """
        PCB走线辐射发射估算 (远场近似)

        Args:
            trace_length: 走线长度 (m)
            current: 电流幅值 (A)
            frequency: 频率 (Hz)
            distance: 测量距离 (m)

        Returns:
            {'e_field_v_m': ..., 'e_field_dBuV_m': ...}
        """
        wavelength = C_LIGHT / frequency
        k = 2 * np.pi / wavelength

        # 短偶极子辐射 (远场)
        # E = (η₀ * k * I * L) / (4π * r)
        e_field = ETA_0 * k * current * trace_length / (4 * np.pi * distance)

        e_field_dBuVm = 20 * np.log10(e_field / 1e-6 + 1e-30)

        return {
            'e_field_v_m': e_field,
            'e_field_dBuV_m': e_field_dBuVm,
            'wavelength': wavelength,
            'is_short_dipole': trace_length < wavelength / 10
        }

    @staticmethod
    def differential_mode_emission(loop_area: float, current: float,
                                    frequency: float,
                                    distance: float = 3.0) -> float:
        """
        差模电流辐射发射

        Args:
            loop_area: 回路面积 (m²)
            current: 差模电流 (A)
            frequency: 频率 (Hz)
            distance: 测量距离 (m)

        Returns:
            电场强度 (dBμV/m)
        """
        wavelength = C_LIGHT / frequency
        # 小环天线辐射
        e_field = (ETA_0 * np.pi * loop_area * current * frequency) / (C_LIGHT * distance * wavelength)
        return 20 * np.log10(e_field / 1e-6 + 1e-30)

    @staticmethod
    def common_mode_emission(cable_length: float, current: float,
                              frequency: float,
                              distance: float = 3.0) -> float:
        """
        共模电流辐射发射 (通常是主要的EMI源)

        Args:
            cable_length: 电缆长度 (m)
            current: 共模电流 (A)
            frequency: 频率 (Hz)
            distance: 测量距离 (m)

        Returns:
            电场强度 (dBμV/m)
        """
        k = 2 * np.pi * frequency / C_LIGHT
        # 共模辐射比差模大得多
        e_field = ETA_0 * k * current * cable_length / (4 * np.pi * distance)
        return 20 * np.log10(e_field / 1e-6 + 1e-30)

    @staticmethod
    def fcc_class_b_limit(frequencies: np.ndarray,
                           measurement_distance: float = 3.0) -> np.ndarray:
        """
        FCC Class B 辐射发射限值 (3m距离)

        Returns:
            限值 (dBμV/m)
        """
        limits = np.zeros_like(frequencies)
        for i, f in enumerate(frequencies):
            if f < 30e6:
                limits[i] = 40.0  # 30-88 MHz
            elif f < 88e6:
                limits[i] = 40.0
            elif f < 216e6:
                limits[i] = 43.5  # 88-216 MHz
            elif f < 960e6:
                limits[i] = 46.0  # 216-960 MHz
            else:
                limits[i] = 54.0  # 960 MHz - 40 GHz
        return limits


# ============================================================
# 屏蔽效能
# ============================================================
class ShieldingEffectiveness:
    """屏蔽效能计算"""

    @staticmethod
    def se_solid_shield(frequency: np.ndarray, thickness: float,
                        sigma_r: float = 1.0, mu_r: float = 1.0) -> ShieldingResult:
        """
        实心金属屏蔽体屏蔽效能

        Args:
            frequency: 频率数组 (Hz)
            thickness: 屏蔽厚度 (m)
            sigma_r: 相对电导率 (铜=1)
            mu_r: 相对磁导率

        Returns:
            ShieldingResult
        """
        omega = 2 * np.pi * frequency

        # 集肤深度
        delta = np.sqrt(2 / (omega * MU_0 * mu_r * 5.8e7 * sigma_r + 1e-30))

        # 吸收损耗 (dB)
        se_absorption = 8.686 * thickness / (delta + 1e-30)

        # 反射损耗 (dB) - 平面波
        eta_shield = np.sqrt(1j * omega * MU_0 * mu_r / (5.8e7 * sigma_r + 1e-30))
        eta_shield_mag = np.abs(eta_shield)
        se_reflection = 20 * np.log10(ETA_0 / (4 * eta_shield_mag + 1e-30))

        # 多次反射修正
        m = (eta_shield_mag - ETA_0) / (eta_shield_mag + ETA_0 + 1e-30)
        se_multiple = 20 * np.log10(np.abs(1 - m**2 * np.exp(-2 * thickness / (delta + 1e-30) * (1 + 1j))) + 1e-30)

        se_total = se_absorption + se_reflection - np.abs(se_multiple)

        return ShieldingResult(
            frequency=frequency,
            se_total_db=se_total,
            se_absorption_db=se_absorption,
            se_reflection_db=se_reflection,
            se_multiple_db=-np.abs(se_multiple)
        )

    @staticmethod
    def se_aperture(frequency: np.ndarray, aperture_size: float,
                    n_apertures: int = 1) -> np.ndarray:
        """
        孔缝泄漏的屏蔽效能损失

        Args:
            frequency: 频率 (Hz)
            aperture_size: 孔缝最大尺寸 (m)
            n_apertures: 孔缝数量

        Returns:
            孔缝导致的SE损失 (dB, 正值=损耗)
        """
        wavelength = C_LIGHT / frequency
        # 单孔泄漏
        se_hole = 20 * np.log10(wavelength / (2 * aperture_size + 1e-30))

        # 多孔修正
        se_hole -= 10 * np.log10(n_apertures)

        return np.maximum(se_hole, 0)  # 不能为负

    @staticmethod
    def required_shielding(required_se_db: float, frequency: float,
                            material: str = 'aluminum') -> Dict:
        """
        计算满足屏蔽要求所需的材料厚度

        Returns:
            {'material': ..., 'thickness_mm': ..., 'skin_depth_mm': ...}
        """
        # 材料参数
        materials = {
            'copper': {'sigma_r': 1.0, 'mu_r': 1.0, 'name': '铜'},
            'aluminum': {'sigma_r': 0.61, 'mu_r': 1.0, 'name': '铝'},
            'steel': {'sigma_r': 0.10, 'mu_r': 1000, 'name': '钢'},
            'mu_metal': {'sigma_r': 0.03, 'mu_r': 30000, 'name': '坡莫合金'}
        }

        mat = materials.get(material, materials['aluminum'])

        omega = 2 * np.pi * frequency
        delta = np.sqrt(2 / (omega * MU_0 * mat['mu_r'] * 5.8e7 * mat['sigma_r']))

        # 需要多少倍集肤深度
        n_skin_depths = required_se_db / 8.686
        thickness = n_skin_depths * delta

        return {
            'material': mat['name'],
            'thickness_mm': thickness * 1000,
            'skin_depth_mm': delta * 1000,
            'skin_depths_needed': n_skin_depths
        }


# ============================================================
# 天线参数 (基础)
# ============================================================
class AntennaBasics:
    """基础天线参数计算"""

    @staticmethod
    def dipole_length(frequency: float) -> float:
        """半波偶极子长度"""
        return C_LIGHT / (2 * frequency)

    @staticmethod
    def patch_antenna_size(frequency: float, er: float = 4.4) -> Tuple[float, float]:
        """
        微带贴片天线尺寸估算

        Returns:
            (width, length) in meters
        """
        w = C_LIGHT / (2 * frequency) * np.sqrt(2 / (er + 1))
        er_eff = (er + 1) / 2 + (er - 1) / 2 * (1 + 12 * 0.0016 / w)**(-0.5)
        delta_l = 0.412 * 0.0016 * (er_eff + 0.3) * (w / 0.0016 + 0.264) / \
                  ((er_eff - 0.258) * (w / 0.0016 + 0.8) + 1e-30)
        L = C_LIGHT / (2 * frequency * np.sqrt(er_eff)) - 2 * delta_l
        return w, L


# ============================================================
# 演示 / 测试
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("  电磁场仿真模块 - 功能演示")
    print("=" * 60)

    # 1. 传输线阻抗
    print("\n[1] 微带线阻抗计算...")
    for w_um in [100, 150, 200, 300, 500]:
        w = w_um * 1e-6
        z0 = TransmissionLineSimulator.microstrip_impedance(w, 1.6e-3, 4.4)
        print(f"  线宽 {w_um}μm, 介质1.6mm FR4: Z0 = {z0:.1f} Ω")

    # 传播参数
    params = TransmissionLineSimulator.propagation_params(50, 3.5, 0.1)
    print(f"\n  50Ω线, 10cm: 传播延迟 = {params['delay_ns']:.2f} ns")

    # 2. 传输线阶跃响应
    print("\n[2] 传输线阶跃响应...")
    result = TransmissionLineSimulator.step_response(
        z0=50, zs=10, zl=1000, length=0.15, er_eff=3.5, n_bounces=8
    )
    print(f"  反射系数 ΓL = {result.reflection_coeff:.3f}")
    print(f"  驻波比 VSWR = {result.swr:.2f}")
    print(f"  传播延迟 = {result.propagation_delay:.2f} ns")
    print(f"  最终电压 = {result.voltage[-1]:.4f} V")

    # 3. S参数
    print("\n[3] S参数计算...")
    freqs = np.linspace(100e6, 10e9, 500)
    _, s11, s21, s11_ph = TransmissionLineSimulator.s_parameters(
        z0=50, zs=50, zl=50, length=0.05, er_eff=3.5, frequencies=freqs
    )
    # 找谐振频率
    s21_peaks = np.where(s21 > -1)[0]
    if len(s21_peaks) > 0:
        print(f"  S21 > -1dB 频率范围: {freqs[s21_peaks[0]]/1e9:.2f} - {freqs[s21_peaks[-1]]/1e9:.2f} GHz")
    print(f"  S11 @ 1GHz: {s11[50]:.1f} dB")

    # 4. 串扰分析
    print("\n[4] PCB串扰分析...")
    xt_freqs = np.linspace(100e6, 10e9, 200)
    xt = CrosstalkAnalyzer.edge_coupled_microstrip_crosstalk(
        width=150e-6, spacing=150e-6, height=1.6e-3,
        er=4.4, length=0.05, frequencies=xt_freqs
    )
    print(f"  峰值NEXT: {xt.peak_crosstalk_db:.1f} dB")
    # 某些频率点
    for freq in [100e6, 1e9, 5e9]:
        idx = np.argmin(np.abs(xt_freqs - freq))
        print(f"  @ {freq/1e9:.1f}GHz: NEXT={xt.fext_near[idx]:.1f}dB, FEXT={xt.fext_far[idx]:.1f}dB")

    # 5. 屏蔽效能
    print("\n[5] 屏蔽效能计算...")
    se_freqs = np.array([1e6, 10e6, 100e6, 1e9, 10e9])
    for material in ['copper', 'aluminum', 'steel']:
        se = ShieldingEffectiveness.se_solid_shield(se_freqs, thickness=1e-3,
                                                      sigma_r=1.0 if material == 'copper' else 0.61 if material == 'aluminum' else 0.1,
                                                      mu_r=1 if material != 'steel' else 1000)
        print(f"  {material} 1mm @ 1GHz: SE = {se.se_total_db[3]:.0f} dB")

    # 孔缝泄漏
    se_aperture = ShieldingEffectiveness.se_aperture(
        np.array([1e9, 10e9]), aperture_size=5e-3, n_apertures=1
    )
    print(f"  5mm孔缝 @ 1GHz: SE损失 = {se_aperture[0]:.1f} dB")

    # 6. 辐射发射
    print("\n[6] EMI辐射发射估算...")
    emi = EMIAnalyzer.radiated_emission_pcb(
        trace_length=0.05, current=0.01, frequency=500e6, distance=3.0
    )
    print(f"  E场: {emi['e_field_dBuV_m']:.1f} dBμV/m @ 3m")
    print(f"  波长: {emi['wavelength']:.3f} m")

    # 7. 天线尺寸
    print("\n[7] 天线参数...")
    for freq in [433e6, 2.4e9, 5.8e9]:
        dipole_l = AntennaBasics.dipole_length(freq)
        w, L = AntennaBasics.patch_antenna_size(freq, 4.4)
        print(f"  @ {freq/1e9 if freq > 1e9 else freq/1e6:.0f}{'GHz' if freq > 1e9 else 'MHz'}:")
        print(f"    偶极子长度: {dipole_l*100:.1f}cm")
        print(f"    贴片尺寸: {w*1000:.1f}mm x {L*1000:.1f}mm")

    print("\n" + "=" * 60)
    print("  电磁场仿真完成!")
    print("=" * 60)
