#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
结构分析仿真模块
=================
梁弯曲分析 | 应力集中 | 疲劳寿命 | 能量方法

适用于电赛中结构设计、应力校核、疲劳评估等场景。
基于材料力学解析方法，无需FEA。
"""

import numpy as np
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass


# ============================================================
# 数据结构
# ============================================================
@dataclass
class BeamResult:
    """梁分析结果"""
    x: np.ndarray              # 位置坐标 (m)
    shear_force: np.ndarray    # 剪力 (N)
    bending_moment: np.ndarray # 弯矩 (N·m)
    slope: np.ndarray          # 转角 (rad)
    deflection: np.ndarray     # 挠度 (m)
    max_deflection: float      # 最大挠度 (m)
    max_moment: float          # 最大弯矩 (N·m)
    max_stress: float          # 最大弯曲应力 (Pa)

@dataclass
class StressConcentrationResult:
    """应力集中结果"""
    nominal_stress: float      # 名义应力 (Pa)
    max_stress: float          # 最大应力 (Pa)
    Kt: float                  # 应力集中系数
    location: str              # 位置描述

@dataclass
class FatigueResult:
    """疲劳分析结果"""
    cycles_to_failure: float   # 寿命 (循环数)
    fatigue_life_hours: float  # 疲劳寿命 (小时)
    safety_factor: float       # 疲劳安全系数
    damage_per_cycle: float    # 每次循环损伤
    is_safe: bool              # 是否安全


# ============================================================
# 截面特性
# ============================================================
class CrossSection:
    """截面特性计算器"""

    @staticmethod
    def rectangle(width: float, height: float) -> Dict:
        """矩形截面"""
        A = width * height
        Ix = width * height**3 / 12
        Iy = height * width**3 / 12
        Sx = Ix / (height / 2)
        Sy = Iy / (width / 2)
        return {
            'A': A, 'Ix': Ix, 'Iy': Iy,
            'Sx': Sx, 'Sy': Sy,
            'c_top': height / 2, 'c_bot': height / 2,
            'type': 'rectangle'
        }

    @staticmethod
    def circle(radius: float) -> Dict:
        """圆形截面"""
        A = np.pi * radius**2
        Ix = np.pi * radius**4 / 4
        Sx = Ix / radius
        return {
            'A': A, 'Ix': Ix, 'Iy': Ix,
            'Sx': Sx, 'Sy': Sx,
            'c_top': radius, 'c_bot': radius,
            'type': 'circle'
        }

    @staticmethod
    def hollow_circle(outer_radius: float, inner_radius: float) -> Dict:
        """空心圆截面"""
        A = np.pi * (outer_radius**2 - inner_radius**2)
        Ix = np.pi * (outer_radius**4 - inner_radius**4) / 4
        Sx = Ix / outer_radius
        return {
            'A': A, 'Ix': Ix, 'Iy': Ix,
            'Sx': Sx, 'Sy': Sx,
            'c_top': outer_radius, 'c_bot': outer_radius,
            'type': 'hollow_circle'
        }

    @staticmethod
    def i_beam(width_flange: float, height_total: float,
               thickness_web: float, thickness_flange: float) -> Dict:
        """工字梁截面"""
        h = height_total
        tw = thickness_web
        tf = thickness_flange
        b = width_flange

        # 上翼缘 + 腹板 + 下翼缘
        A_top = b * tf
        A_web = (h - 2 * tf) * tw
        A_bot = b * tf

        A = A_top + A_web + A_bot

        # 形心 (对称截面, 在中间)
        y_c = h / 2

        # 惯性矩 (平行轴定理)
        I_flange = 2 * (b * tf**3 / 12 + A_top * ((h - tf) / 2)**2)
        I_web = tw * (h - 2 * tf)**3 / 12
        Ix = I_flange + I_web

        Sx = Ix / (h / 2)

        return {
            'A': A, 'Ix': Ix, 'Iy': Ix,  # Iy需要另外算
            'Sx': Sx, 'Sy': Sx,
            'c_top': h / 2, 'c_bot': h / 2,
            'type': 'i_beam'
        }

    @staticmethod
    def t_beam(width_flange: float, height_web: float,
               thickness_flange: float, thickness_web: float) -> Dict:
        """T型截面"""
        A_flange = width_flange * thickness_flange
        A_web = thickness_web * height_web
        A = A_flange + A_web

        # 形心位置 (从底部算起)
        y_flange = height_web + thickness_flange / 2
        y_web = height_web / 2
        y_bar = (A_flange * y_flange + A_web * y_web) / A

        # 惯性矩
        I_flange = (width_flange * thickness_flange**3 / 12 +
                    A_flange * (y_flange - y_bar)**2)
        I_web = (thickness_web * height_web**3 / 12 +
                 A_web * (y_web - y_bar)**2)
        Ix = I_flange + I_web

        h_total = height_web + thickness_flange
        c_top = h_total - y_bar
        c_bot = y_bar

        Sx_top = Ix / c_top if c_top > 0 else float('inf')
        Sx_bot = Ix / c_bot if c_bot > 0 else float('inf')

        return {
            'A': A, 'Ix': Ix,
            'Sx': min(Sx_top, Sx_bot),
            'c_top': c_top, 'c_bot': c_bot,
            'y_bar': y_bar,
            'type': 't_beam'
        }


# ============================================================
# 梁弯曲分析
# ============================================================
class BeamAnalyzer:
    """梁弯曲分析器 - 解析法"""

    @staticmethod
    def simply_supported_point_load(L: float, P: float, a: float,
                                     E: float, section: Dict,
                                     n_points: int = 200) -> BeamResult:
        """
        简支梁-集中载荷

        Args:
            L: 梁长 (m)
            P: 集中力 (N)
            a: 力到左端距离 (m)
            E: 弹性模量 (Pa)
            section: 截面特性字典

        Returns:
            BeamResult
        """
        b = L - a
        I = section['Ix']

        x = np.linspace(0, L, n_points)
        V = np.zeros(n_points)
        M = np.zeros(n_points)
        theta = np.zeros(n_points)
        delta = np.zeros(n_points)

        RA = P * b / L  # 左支反力
        RB = P * a / L  # 右支反力

        for i, xi in enumerate(x):
            if xi <= a:
                V[i] = RA
                M[i] = RA * xi
                theta[i] = P * b / (6 * E * I * L) * (3 * xi**2 - (L**2 - b**2))
                delta[i] = P * b * xi / (6 * E * I * L) * (L**2 - b**2 - xi**2)
            else:
                V[i] = RA - P
                M[i] = RA * xi - P * (xi - a)
                theta[i] = P * a / (6 * E * I * L) * (-3 * (L - xi)**2 + (L**2 - a**2))
                delta[i] = P * a * (L - xi) / (6 * E * I * L) * (2 * L * xi - xi**2 - a**2)

        max_moment = P * a * b / L
        max_stress = max_moment * section['c_top'] / I
        max_def = np.max(np.abs(delta))

        return BeamResult(
            x=x, shear_force=V, bending_moment=M,
            slope=theta, deflection=delta,
            max_deflection=max_def, max_moment=max_moment,
            max_stress=max_stress
        )

    @staticmethod
    def simply_supported_uniform_load(L: float, w: float,
                                       E: float, section: Dict,
                                       n_points: int = 200) -> BeamResult:
        """
        简支梁-均布载荷

        Args:
            L: 梁长 (m)
            w: 均布载荷 (N/m)
            E: 弹性模量 (Pa)
            section: 截面特性

        Returns:
            BeamResult
        """
        I = section['Ix']

        x = np.linspace(0, L, n_points)
        V = w * (L / 2 - x)
        M = w * x * (L - x) / 2
        theta = w / (24 * E * I) * (L**3 - 6 * L * x**2 + 4 * x**3)
        delta = w * x / (24 * E * I) * (L**3 - 2 * L * x**2 + x**3)

        max_moment = w * L**2 / 8
        max_stress = max_moment * section['c_top'] / I
        max_def = 5 * w * L**4 / (384 * E * I)

        return BeamResult(
            x=x, shear_force=V, bending_moment=M,
            slope=theta, deflection=delta,
            max_deflection=max_def, max_moment=max_moment,
            max_stress=max_stress
        )

    @staticmethod
    def cantilever_point_load(L: float, P: float,
                               E: float, section: Dict,
                               n_points: int = 200) -> BeamResult:
        """
        悬臂梁-端部集中力

        Args:
            L: 梁长 (m)
            P: 端部力 (N)
            E: 弹性模量 (Pa)
            section: 截面特性

        Returns:
            BeamResult
        """
        I = section['Ix']

        x = np.linspace(0, L, n_points)
        V = np.full(n_points, -P)
        M = -P * (L - x)
        theta = P / (2 * E * I) * (x**2 - 2 * L * x)
        delta = P / (6 * E * I) * (x**3 - 3 * L * x**2)

        max_moment = P * L
        max_stress = max_moment * section['c_top'] / I
        max_def = P * L**3 / (3 * E * I)

        return BeamResult(
            x=x, shear_force=V, bending_moment=M,
            slope=theta, deflection=delta,
            max_deflection=max_def, max_moment=max_moment,
            max_stress=max_stress
        )

    @staticmethod
    def cantilever_uniform_load(L: float, w: float,
                                 E: float, section: Dict,
                                 n_points: int = 200) -> BeamResult:
        """悬臂梁-均布载荷"""
        I = section['Ix']

        x = np.linspace(0, L, n_points)
        V = -w * (L - x)
        M = -w * (L - x)**2 / 2
        theta = w / (6 * E * I) * (3 * L**2 * x - 3 * L * x**2 + x**3)
        delta = w / (24 * E * I) * (x**4 - 4 * L * x**3 + 6 * L**2 * x**2)

        max_moment = w * L**2 / 2
        max_stress = max_moment * section['c_top'] / I
        max_def = w * L**4 / (8 * E * I)

        return BeamResult(
            x=x, shear_force=V, bending_moment=M,
            slope=theta, deflection=delta,
            max_deflection=max_def, max_moment=max_moment,
            max_stress=max_stress
        )

    @staticmethod
    def superpose_beams(beams: List[BeamResult],
                        weights: List[float] = None) -> BeamResult:
        """
        叠加多个梁分析结果

        Args:
            beams: 梁分析结果列表
            weights: 叠加权重 (默认全1)

        Returns:
            叠加后的BeamResult
        """
        if weights is None:
            weights = [1.0] * len(beams)

        n_points = len(beams[0].x)
        V = np.zeros(n_points)
        M = np.zeros(n_points)
        theta = np.zeros(n_points)
        delta = np.zeros(n_points)

        for beam, w in zip(beams, weights):
            # 重采样到相同长度
            if len(beam.x) != n_points:
                V += w * np.interp(beams[0].x, beam.x, beam.shear_force)
                M += w * np.interp(beams[0].x, beam.x, beam.bending_moment)
                theta += w * np.interp(beams[0].x, beam.x, beam.slope)
                delta += w * np.interp(beams[0].x, beam.x, beam.deflection)
            else:
                V += w * beam.shear_force
                M += w * beam.bending_moment
                theta += w * beam.slope
                delta += w * beam.deflection

        return BeamResult(
            x=beams[0].x, shear_force=V, bending_moment=M,
            slope=theta, deflection=delta,
            max_deflection=np.max(np.abs(delta)),
            max_moment=np.max(np.abs(M)),
            max_stress=0  # 需要截面参数才能计算
        )


# ============================================================
# 应力集中
# ============================================================
class StressConcentration:
    """应力集中系数计算"""

    @staticmethod
    def circular_hole_infinite_plate(Kt: float = None,
                                      a: float = None,
                                      b: float = None) -> float:
        """
        无限大板中心圆孔: Kt = 3

        或椭圆孔: Kt = 1 + 2*a/b
        """
        if a is not None and b is not None:
            return 1 + 2 * a / b
        return 3.0

    @staticmethod
    def shoulder_fillet(D: float, d: float, r: float) -> float:
        """
        阶梯轴圆角应力集中系数 (Peterson近似)

        Args:
            D: 大直径
            d: 小直径
            r: 圆角半径

        Returns:
            Kt (弯曲)
        """
        t = (D - d) / 2
        r_d = r / d
        t_d = t / d

        # Peterson公式近似
        C1 = 0.927 + 1.149 * np.sqrt(t_d) - 0.086 * t_d
        C2 = 0.024 - 3.386 * np.sqrt(t_d) + 0.949 * t_d
        C3 = 0.849 + 1.965 * np.sqrt(t_d) - 0.553 * t_d

        if r_d > 0:
            Kt = C1 + C2 * np.sqrt(1 / r_d) + C3 / r_d
        else:
            Kt = 10.0  # 极限值

        return max(Kt, 1.0)

    @staticmethod
    def groove(d: float, r: float, t: float) -> float:
        """
        U型槽应力集中系数

        Args:
            d: 净直径
            r: 槽根半径
            t: 槽深

        Returns:
            Kt (弯曲)
        """
        r_d = r / d
        t_d = t / d

        if r_d < 0.01:
            r_d = 0.01

        Kt = 1 + 2 * np.sqrt(t_d / r_d)
        return max(Kt, 1.0)

    @staticmethod
    def hole_in_finite_width(d: float, w: float) -> float:
        """
        有限宽度板中心孔

        Args:
            d: 孔径
            w: 板宽

        Returns:
            Kt
        """
        d_w = d / w
        # Howland公式近似
        Kt = 3 - 3.13 * d_w + 3.66 * d_w**2 - 1.53 * d_w**3
        return max(Kt, 1.0)

    @staticmethod
    def press_fit_shaft(d_shaft: float, d_hole: float,
                         E_shaft: float = 200e9,
                         E_hole: float = 200e9,
                         nu: float = 0.3) -> float:
        """
        过盈配合应力集中系数

        Returns:
            Kt
        """
        # 简化近似
        ratio = d_hole / d_shaft if d_shaft > 0 else 1.0
        return 2.0 + 0.5 * ratio  # 经验值

    @staticmethod
    def neuber_correction(Kt: float, Kf: float = None,
                           material_sensitivity: float = 0.5,
                           notch_radius: float = 1.0) -> float:
        """
        Neuber修正 (弹性应力集中到疲劳缺口系数)

        Kf = 1 + (Kt - 1) / (1 + √(a/ρ))

        Args:
            Kt: 理论应力集中系数
            material_sensitivity: 材料敏感度参数 a (mm)
            notch_radius: 缺口根部半径 (mm)

        Returns:
            Kf: 疲劳缺口系数
        """
        if notch_radius < 0.001:
            notch_radius = 0.001

        # Neuber材料常数 (简化)
        a = material_sensitivity  # 钢: 0.25~0.5mm, 铝: 0.5~1.0mm

        Kf = 1 + (Kt - 1) / (1 + np.sqrt(a / notch_radius))
        return max(Kf, 1.0)


# ============================================================
# 疲劳分析
# ============================================================
class FatigueAnalyzer:
    """疲劳寿命分析器"""

    @staticmethod
    def s_n_curve_steel(sigma_f: float, b: float,
                        N: np.ndarray) -> np.ndarray:
        """
        S-N曲线 (Basquin方程)

        σ_a = σ'_f * (2N)^b

        Args:
            sigma_f: 疲劳强度系数 (Pa)
            b: 疲劳强度指数 (-0.05 ~ -0.15)
            N: 循环次数数组

        Returns:
            应力幅值数组 (Pa)
        """
        return sigma_f * (2 * N) ** b

    @staticmethod
    def basquin_life(sigma_a: float, sigma_f: float,
                      b: float) -> float:
        """
        Basquin方程求寿命

        N = 0.5 * (σ_a / σ'_f)^(1/b)

        Returns:
            循环次数
        """
        if sigma_a <= 0 or sigma_f <= 0:
            return float('inf')
        ratio = sigma_a / sigma_f
        if ratio <= 0:
            return float('inf')
        N = 0.5 * ratio ** (1.0 / b)
        return max(N, 1)

    @staticmethod
    def goodman_correction(sigma_a: float, sigma_m: float,
                            sigma_u: float) -> float:
        """
        Goodman修正 (考虑平均应力)

        σ_a_corrected = σ_a / (1 - σ_m/σ_u)

        Returns:
            修正后的等效应力幅
        """
        if sigma_m >= sigma_u:
            return float('inf')
        return sigma_a / (1 - sigma_m / sigma_u)

    @staticmethod
    def gerber_correction(sigma_a: float, sigma_m: float,
                           sigma_u: float) -> float:
        """
        Gerber修正

        σ_a_corrected = σ_a / (1 - (σ_m/σ_u)²)
        """
        ratio = sigma_m / sigma_u
        if ratio >= 1.0:
            return float('inf')
        return sigma_a / (1 - ratio**2)

    @staticmethod
    def miner_rule(stress_history: np.ndarray,
                    S_N_func, params: Dict) -> FatigueResult:
        """
        Miner线性累积损伤准则

        D = Σ(n_i / N_i)

        Args:
            stress_history: 应力历程 (应力幅值数组)
            S_N_func: S-N函数 (接受应力幅, 返回寿命)
            params: 参数字典

        Returns:
            FatigueResult
        """
        total_damage = 0.0
        cycle_count = len(stress_history)

        for sigma_a in stress_history:
            if sigma_a <= 0:
                continue
            N_f = S_N_func(sigma_a)
            if N_f > 0 and np.isfinite(N_f):
                total_damage += 1.0 / N_f

        if total_damage > 0:
            cycles_to_failure = 1.0 / total_damage
        else:
            cycles_to_failure = float('inf')

        # 假设循环频率
        freq = 1.0  # Hz
        fatigue_life_hours = cycles_to_failure / (3600 * freq)

        return FatigueResult(
            cycles_to_failure=cycles_to_failure,
            fatigue_life_hours=fatigue_life_hours,
            safety_factor=1.0 / total_damage if total_damage > 0 else float('inf'),
            damage_per_cycle=total_damage / cycle_count if cycle_count > 0 else 0,
            is_safe=total_damage < 1.0
        )

    @staticmethod
    def rainflow_counting_simple(signal: np.ndarray) -> List[Tuple[float, float]]:
        """
        简化雨流计数法

        Args:
            signal: 应力/应变时程信号

        Returns:
            [(range, mean), ...] 循环范围和均值列表
        """
        # 简化版: 提取峰谷值后配对
        peaks = []
        valleys = []

        # 提取峰谷
        for i in range(1, len(signal) - 1):
            if signal[i] > signal[i-1] and signal[i] > signal[i+1]:
                peaks.append(signal[i])
            elif signal[i] < signal[i-1] and signal[i] < signal[i+1]:
                valleys.append(signal[i])

        # 简单配对
        cycles = []
        n = min(len(peaks), len(valleys))
        for i in range(n):
            stress_range = abs(peaks[i] - valleys[i])
            stress_mean = (peaks[i] + valleys[i]) / 2
            cycles.append((stress_range, stress_mean))

        return cycles

    @staticmethod
    def endurance_limit_steel(hardness_HB: float) -> float:
        """
        钢材疲劳极限估算

        σ_e ≈ 0.5 * σ_u (σ_u < 1400 MPa)
        或 σ_e ≈ 700 MPa (σ_u > 1400 MPa)

        Args:
            hardness_HB: 布氏硬度

        Returns:
            疲劳极限 (Pa)
        """
        # σ_u ≈ 3.45 * HB (MPa)
        sigma_u = 3.45 * hardness_HB
        if sigma_u < 1400:
            return sigma_u * 0.5 * 1e6  # Pa
        else:
            return 700e6  # Pa


# ============================================================
# 能量方法
# ============================================================
class EnergyMethods:
    """基于能量的结构分析方法"""

    @staticmethod
    def castigliano_deflection(L: float, P: float, E: float,
                                I: float, beam_type: str = 'simply_supported') -> float:
        """
        Castigliano定理求挠度

        δ = ∂U/∂P

        Args:
            L: 梁长
            P: 载荷
            E: 弹性模量
            I: 惯性矩
            beam_type: 'simply_supported' 或 'cantilever'

        Returns:
            挠度 (m)
        """
        if beam_type == 'simply_supported':
            # 集中力在中点
            return P * L**3 / (48 * E * I)
        elif beam_type == 'cantilever':
            return P * L**3 / (3 * E * I)
        else:
            return 0.0

    @staticmethod
    def strain_energy_bending(M: np.ndarray, x: np.ndarray,
                               E: float, I: float) -> float:
        """
        弯曲应变能

        U = ∫ M²/(2EI) dx

        Returns:
            应变能 (J)
        """
        integrand = M**2 / (2 * E * I)
        return np.trapz(integrand, x)

    @staticmethod
    def strain_energy_axial(N: float, L: float,
                             E: float, A: float) -> float:
        """
        轴向应变能

        U = N²L/(2EA)

        Returns:
            应变能 (J)
        """
        return N**2 * L / (2 * E * A)

    @staticmethod
    def strain_energy_shear(V: np.ndarray, x: np.ndarray,
                             G: float, A: float,
                             shape_factor: float = 6/5) -> float:
        """
        剪切应变能

        U = ∫ αV²/(2GA) dx

        Returns:
            应变能 (J)
        """
        integrand = shape_factor * V**2 / (2 * G * A)
        return np.trapz(integrand, x)


# ============================================================
# 复合载荷
# ============================================================
class CombinedLoading:
    """组合载荷分析"""

    @staticmethod
    def von_mises(sigma_x: float, sigma_y: float = 0,
                   sigma_z: float = 0,
                   tau_xy: float = 0) -> float:
        """
        Von Mises等效应力

        σ_vm = √(σx² + σy² + σz² - σxσy - σyσz - σzσx + 3(τxy² + τyz² + τxz²))
        """
        vm = (sigma_x**2 + sigma_y**2 + sigma_z**2 -
              sigma_x * sigma_y - sigma_y * sigma_z - sigma_z * sigma_x +
              3 * tau_xy**2)
        return np.sqrt(max(vm, 0))

    @staticmethod
    def mohr_circle_2d(sigma_x: float, sigma_y: float,
                        tau_xy: float) -> Dict:
        """
        二维莫尔圆

        Returns:
            {'sigma_1': ..., 'sigma_2': ..., 'tau_max': ..., 'theta_p': ..., 'theta_s': ...}
        """
        avg = (sigma_x + sigma_y) / 2
        R = np.sqrt(((sigma_x - sigma_y) / 2)**2 + tau_xy**2)

        sigma_1 = avg + R
        sigma_2 = avg - R
        tau_max = R

        theta_p = 0.5 * np.degrees(np.arctan2(2 * tau_xy, sigma_x - sigma_y))
        theta_s = theta_p + 45

        return {
            'sigma_1': sigma_1,
            'sigma_2': sigma_2,
            'tau_max': tau_max,
            'theta_p': theta_p,
            'theta_s': theta_s,
            'sigma_avg': avg,
            'radius': R
        }

    @staticmethod
    def combined_stress_bending_torsion(M: float, T: float,
                                         d: float) -> Dict:
        """
        弯扭组合应力

        Args:
            M: 弯矩 (N·m)
            T: 扭矩 (N·m)
            d: 轴径 (m)

        Returns:
            {'sigma_bending': ..., 'tau_torsion': ..., 'von_mises': ..., 'principal_stress': ...}
        """
        r = d / 2
        I = np.pi * d**4 / 64
        J = np.pi * d**4 / 32

        sigma_b = M * r / I
        tau_t = T * r / J

        # 主应力
        sigma_1 = sigma_b / 2 + np.sqrt((sigma_b / 2)**2 + tau_t**2)
        sigma_2 = sigma_b / 2 - np.sqrt((sigma_b / 2)**2 + tau_t**2)

        # Von Mises
        vm = np.sqrt(sigma_b**2 + 3 * tau_t**2)

        return {
            'sigma_bending': sigma_b,
            'tau_torsion': tau_t,
            'von_mises': vm,
            'sigma_1': sigma_1,
            'sigma_2': sigma_2,
            'tau_max': np.sqrt((sigma_b / 2)**2 + tau_t**2)
        }


# ============================================================
# 演示 / 测试
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("  结构分析仿真模块 - 功能演示")
    print("=" * 60)

    # 材料参数 (钢)
    E = 200e9  # 弹性模量 (Pa)
    sigma_y = 350e6  # 屈服强度 (Pa)
    sigma_u = 500e6  # 抗拉强度 (Pa)

    # 1. 截面特性
    print("\n[1] 截面特性计算...")
    rect = CrossSection.rectangle(0.05, 0.1)
    circle = CrossSection.circle(0.03)
    ibeam = CrossSection.i_beam(0.1, 0.15, 0.008, 0.012)
    print(f"  矩形 50x100mm: A={rect['A']*1e4:.2f}cm², Ix={rect['Ix']*1e8:.2f}cm⁴")
    print(f"  圆形 D=60mm:   A={circle['A']*1e4:.2f}cm², Ix={circle['Ix']*1e8:.2f}cm⁴")
    print(f"  工字梁:         A={ibeam['A']*1e4:.2f}cm², Ix={ibeam['Ix']*1e8:.2f}cm⁴")

    # 2. 梁弯曲分析
    print("\n[2] 梁弯曲分析...")
    L = 1.0  # 1m

    # 简支梁集中力
    beam1 = BeamAnalyzer.simply_supported_point_load(L, P=1000, a=0.4, E=E, section=rect)
    print(f"  简支梁-集中力 (1kN, a=0.4m):")
    print(f"    最大弯矩: {beam1.max_moment:.1f} N·m")
    print(f"    最大应力: {beam1.max_stress/1e6:.1f} MPa")
    print(f"    最大挠度: {beam1.max_deflection*1000:.3f} mm")

    # 简支梁均布载荷
    beam2 = BeamAnalyzer.simply_supported_uniform_load(L, w=500, E=E, section=rect)
    print(f"  简支梁-均布载荷 (500N/m):")
    print(f"    最大弯矩: {beam2.max_moment:.1f} N·m")
    print(f"    最大挠度: {beam2.max_deflection*1000:.3f} mm")

    # 悬臂梁
    beam3 = BeamAnalyzer.cantilever_point_load(L, P=500, E=E, section=rect)
    print(f"  悬臂梁-端部力 (500N):")
    print(f"    最大弯矩: {beam3.max_moment:.1f} N·m")
    print(f"    最大挠度: {beam3.max_deflection*1000:.3f} mm")

    # 3. 应力集中
    print("\n[3] 应力集中分析...")
    Kt_hole = StressConcentration.circular_hole_infinite_plate()
    print(f"  无限大板圆孔: Kt = {Kt_hole}")

    Kt_fillet = StressConcentration.shoulder_fillet(D=50, d=40, r=5)
    print(f"  阶梯轴圆角 (D50/d40/R5): Kt = {Kt_fillet:.2f}")

    Kt_hole2 = StressConcentration.hole_in_finite_width(d=10, w=50)
    print(f"  有限宽板孔 (d=10, w=50): Kt = {Kt_hole2:.2f}")

    Kf = StressConcentration.neuber_correction(Kt_fillet, material_sensitivity=0.3, notch_radius=5)
    print(f"  Neuber修正: Kt={Kt_fillet:.2f} → Kf={Kf:.2f}")

    # 4. 疲劳分析
    print("\n[4] 疲劳分析...")
    # 钢的S-N曲线参数
    sigma_f = 1000e6  # 疲劳强度系数
    b = -0.10  # 疲劳强度指数

    # 不同应力水平下的寿命
    for sigma_a_MPa in [200, 300, 400, 500]:
        sigma_a = sigma_a_MPa * 1e6
        N = FatigueAnalyzer.basquin_life(sigma_a, sigma_f, b)
        print(f"  σ_a={sigma_a_MPa}MPa: N_f = {N:.0f} cycles ({N/1e6:.2f}M)")

    # Goodman修正
    print("\n  Goodman平均应力修正:")
    sigma_a = 300e6
    for sigma_m_MPa in [0, 100, 200, 300]:
        sigma_a_eq = FatigueAnalyzer.goodman_correction(sigma_a, sigma_m_MPa * 1e6, sigma_u)
        N = FatigueAnalyzer.basquin_life(sigma_a_eq, sigma_f, b)
        print(f"    σ_m={sigma_m_MPa}MPa: σ_a_eq={sigma_a_eq/1e6:.0f}MPa, N={N:.0f}")

    # Miner累积损伤
    print("\n  Miner累积损伤准则:")
    stress_history = np.array([300, 350, 280, 320, 400, 290, 310, 350]) * 1e6

    def sn_func(sigma_a):
        return FatigueAnalyzer.basquin_life(sigma_a, sigma_f, b)

    fatigue_result = FatigueAnalyzer.miner_rule(stress_history, sn_func, {})
    print(f"    总循环数: {len(stress_history)}")
    print(f"    等效寿命: {fatigue_result.cycles_to_failure:.0f} cycles")
    print(f"    安全系数: {fatigue_result.safety_factor:.2f}")
    print(f"    是否安全: {fatigue_result.is_safe}")

    # 5. 组合应力
    print("\n[5] 组合应力分析...")
    # 弯扭组合
    combined = CombinedLoading.combined_stress_bending_torsion(M=500, T=300, d=0.03)
    print(f"  弯扭组合 (M=500N·m, T=300N·m, d=30mm):")
    print(f"    弯曲应力: {combined['sigma_bending']/1e6:.1f} MPa")
    print(f"    扭转剪力: {combined['tau_torsion']/1e6:.1f} MPa")
    print(f"    Von Mises: {combined['von_mises']/1e6:.1f} MPa")
    print(f"    主应力: {combined['sigma_1']/1e6:.1f}, {combined['sigma_2']/1e6:.1f} MPa")

    # 莫尔圆
    mohr = CombinedLoading.mohr_circle_2d(100e6, -50e6, 30e6)
    print(f"\n  莫尔圆 (σx=100, σy=-50, τxy=30 MPa):")
    print(f"    σ1={mohr['sigma_1']/1e6:.1f} MPa, σ2={mohr['sigma_2']/1e6:.1f} MPa")
    print(f"    τmax={mohr['tau_max']/1e6:.1f} MPa")
    print(f"    主方向: θp={mohr['theta_p']:.1f}°")

    # 6. 能量方法
    print("\n[6] 能量方法...")
    U_bend = EnergyMethods.strain_energy_bending(beam1.bending_moment, beam1.x, E, rect['Ix'])
    print(f"  弯曲应变能: {U_bend:.4f} J")

    delta_calc = EnergyMethods.castigliano_deflection(L, 1000, E, rect['Ix'])
    print(f"  Castigliano挠度: {delta_calc*1000:.3f} mm")

    print("\n" + "=" * 60)
    print("  结构分析仿真完成!")
    print("=" * 60)
