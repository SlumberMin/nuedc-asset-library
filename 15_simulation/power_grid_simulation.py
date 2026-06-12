#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电网仿真 - 三相/功率因数/谐波/无功补偿
========================================
功能:
  1. 三相电压/电流仿真 (平衡/不平衡)
  2. 功率计算 (有功/无功/视在/功率因数)
  3. 谐波分析 (FFT + THD计算)
  4. 无功补偿 (并联电容器/SVG/STATCOM)
  5. 三相不平衡分析 (对称分量法)
  6. 负载建模 (恒阻抗/恒电流/恒功率)
  7. 功率因数校正

依赖: numpy (必需), matplotlib (可选)
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum


# ============================================================
# 1. 基本参数
# ============================================================

class PhaseSequence(Enum):
    POSITIVE = "positive"  # 正序
    NEGATIVE = "negative"  # 负序
    ZERO = "zero"          # 零序


@dataclass
class ThreePhaseSignal:
    """三相信号"""
    Va: np.ndarray
    Vb: np.ndarray
    Vc: np.ndarray
    fs: float
    f0: float
    t: np.ndarray = field(default_factory=lambda: np.array([]))

    def __post_init__(self):
        if len(self.t) == 0:
            self.t = np.arange(len(self.Va)) / self.fs


@dataclass
class PowerMetrics:
    """功率指标"""
    P: float       # 有功功率 (W)
    Q: float       # 无功功率 (var)
    S: float       # 视在功率 (VA)
    pf: float      # 功率因数
    pf_type: str   # "leading" / "lagging"
    P_per_phase: List[float] = field(default_factory=list)
    Q_per_phase: List[float] = field(default_factory=list)


# ============================================================
# 2. 三相信号生成
# ============================================================

class ThreePhaseGenerator:
    """三相电源仿真"""

    def __init__(self, V_rms: float = 220.0, f0: float = 50.0, fs: float = 10000.0):
        self.V_rms = V_rms
        self.f0 = f0
        self.fs = fs
        self.V_peak = V_rms * np.sqrt(2)

    def generate_balanced(self, duration: float = 0.1) -> ThreePhaseSignal:
        """生成平衡三相电压"""
        t = np.arange(0, duration, 1/self.fs)
        omega = 2 * np.pi * self.f0

        Va = self.V_peak * np.sin(omega * t)
        Vb = self.V_peak * np.sin(omega * t - 2*np.pi/3)
        Vc = self.V_peak * np.sin(omega * t + 2*np.pi/3)

        return ThreePhaseSignal(Va, Vb, Vc, self.fs, self.f0, t)

    def generate_unbalanced(self, duration: float = 0.1,
                            imbalance: float = 0.1,
                            phase_shift_deg: float = 5.0) -> ThreePhaseSignal:
        """生成不平衡三相电压"""
        t = np.arange(0, duration, 1/self.fs)
        omega = 2 * np.pi * self.f0

        Va = self.V_peak * (1.0) * np.sin(omega * t)
        Vb = self.V_peak * (1.0 + imbalance) * np.sin(omega * t - 2*np.pi/3 + np.radians(phase_shift_deg))
        Vc = self.V_peak * (1.0 - imbalance) * np.sin(omega * t + 2*np.pi/3)

        return ThreePhaseSignal(Va, Vb, Vc, self.fs, self.f0, t)

    def generate_with_harmonics(self, duration: float = 0.1,
                                 harmonics: Dict[int, float] = None) -> ThreePhaseSignal:
        """生成含谐波的三相电压"""
        if harmonics is None:
            harmonics = {3: 0.05, 5: 0.03, 7: 0.02, 11: 0.01, 13: 0.005}

        t = np.arange(0, duration, 1/self.fs)
        omega = 2 * np.pi * self.f0

        Va = self.V_peak * np.sin(omega * t)
        Vb = self.V_peak * np.sin(omega * t - 2*np.pi/3)
        Vc = self.V_peak * np.sin(omega * t + 2*np.pi/3)

        for n, amp in harmonics.items():
            phase_shift = 2 * np.pi / 3
            Va += amp * self.V_peak * np.sin(n * omega * t)
            if n % 3 == 0:  # 零序谐波同相
                Vb += amp * self.V_peak * np.sin(n * omega * t)
                Vc += amp * self.V_peak * np.sin(n * omega * t)
            elif n % 3 == 1:  # 正序
                Vb += amp * self.V_peak * np.sin(n * (omega * t - phase_shift))
                Vc += amp * self.V_peak * np.sin(n * (omega * t + phase_shift))
            else:  # 负序
                Vb += amp * self.V_peak * np.sin(n * (omega * t + phase_shift))
                Vc += amp * self.V_peak * np.sin(n * (omega * t - phase_shift))

        return ThreePhaseSignal(Va, Vb, Vc, self.fs, self.f0, t)


# ============================================================
# 3. 负载模型
# ============================================================

class LoadType(Enum):
    CONSTANT_Z = "constant_impedance"   # 恒阻抗
    CONSTANT_I = "constant_current"     # 恒电流
    CONSTANT_P = "constant_power"       # 恒功率
    RL = "RL_load"                      # RL负载
    RLC = "RLC_load"                    # RLC负载


@dataclass
class LoadConfig:
    """负载配置"""
    load_type: LoadType
    P_rated: float  # 额定有功功率 (W)
    Q_rated: float = 0.0  # 额定无功功率 (var)
    V_rated: float = 220.0
    R: float = 0.0
    L: float = 0.0
    C: float = 0.0


class LoadModel:
    """负载模型"""

    @staticmethod
    def rl_load(voltage: np.ndarray, R: float, L: float,
                f0: float, fs: float) -> Tuple[np.ndarray, float]:
        """RL负载电流计算"""
        omega = 2 * np.pi * f0
        Z = np.sqrt(R**2 + (omega * L)**2)
        phi = np.arctan2(omega * L, R)

        I_peak = np.max(np.abs(voltage)) / Z
        # 简化: 假设稳态
        current = voltage / Z
        return current, phi

    @staticmethod
    def rlc_load(voltage: np.ndarray, R: float, L: float, C: float,
                 f0: float, fs: float) -> Tuple[np.ndarray, float]:
        """RLC负载电流计算"""
        omega = 2 * np.pi * f0
        XL = omega * L
        XC = 1 / (omega * C) if C > 0 else float('inf')
        X = XL - XC
        Z = np.sqrt(R**2 + X**2)
        phi = np.arctan2(X, R)

        current = voltage / Z
        return current, phi

    @staticmethod
    def constant_power_load(voltage: np.ndarray, P: float, Q: float,
                            V_rated: float = 220.0) -> Tuple[np.ndarray, float]:
        """恒功率负载"""
        S = np.sqrt(P**2 + Q**2)
        I_rated = S / V_rated
        phi = np.arctan2(Q, P)

        V_rms = np.sqrt(np.mean(voltage**2))
        I_actual = I_rated * V_rated / V_rms if V_rms > 0 else 0

        # 电流相位滞后电压phi
        current = I_actual * np.sqrt(2) * np.sin(2 * np.pi * 50 *
                  np.arange(len(voltage)) / len(voltage) * 100 - phi)
        return current, phi


# ============================================================
# 4. 功率分析
# ============================================================

class PowerAnalyzer:
    """功率分析器"""

    @staticmethod
    def instant_power(v: np.ndarray, i: np.ndarray) -> np.ndarray:
        """瞬时功率"""
        return v * i

    @staticmethod
    def rms(signal: np.ndarray) -> float:
        """有效值"""
        return float(np.sqrt(np.mean(signal**2)))

    @staticmethod
    def active_power(v: np.ndarray, i: np.ndarray) -> float:
        """有功功率"""
        return float(np.mean(v * i))

    @staticmethod
    def reactive_power(v: np.ndarray, i: np.ndarray, f0: float, fs: float) -> float:
        """无功功率 (使用Hilbert变换)"""
        n = len(v)
        # Hilbert变换计算无功
        from numpy.fft import fft, ifft
        V_h = fft(v)
        I_h = fft(i)

        # 90度相移
        h = np.zeros(n)
        if n % 2 == 0:
            h[0] = 1
            h[n//2] = 1
            h[1:n//2] = 2
        else:
            h[0] = 1
            h[1:(n+1)//2] = 2

        v_hilbert = np.real(ifft(V_h * (-1j * h)))
        return float(np.mean(v_hilbert * i))

    @staticmethod
    def apparent_power(P: float, Q: float) -> float:
        """视在功率"""
        return np.sqrt(P**2 + Q**2)

    @staticmethod
    def power_factor(P: float, S: float) -> Tuple[float, str]:
        """功率因数"""
        if S < 1e-10:
            return 1.0, "unity"
        pf = abs(P / S)
        pf = min(pf, 1.0)
        pf_type = "lagging" if P > 0 else "leading"
        return pf, pf_type

    @classmethod
    def analyze_three_phase(cls, signal: ThreePhaseSignal,
                            Ia: np.ndarray, Ib: np.ndarray, Ic: np.ndarray) -> PowerMetrics:
        """三相功率分析"""
        pa = cls.active_power(signal.Va, Ia)
        pb = cls.active_power(signal.Vb, Ib)
        pc = cls.active_power(signal.Vc, Ic)

        qa = cls.reactive_power(signal.Va, Ia, signal.f0, signal.fs)
        qb = cls.reactive_power(signal.Vb, Ib, signal.f0, signal.fs)
        qc = cls.reactive_power(signal.Vc, Ic, signal.f0, signal.fs)

        P = pa + pb + pc
        Q = qa + qb + qc
        S = cls.apparent_power(P, Q)
        pf, pf_type = cls.power_factor(P, S)

        return PowerMetrics(
            P=P, Q=Q, S=S, pf=pf, pf_type=pf_type,
            P_per_phase=[pa, pb, pc],
            Q_per_phase=[qa, qb, qc],
        )

    @classmethod
    def analyze_single_phase(cls, v: np.ndarray, i: np.ndarray,
                             f0: float = 50.0, fs: float = 10000.0) -> Dict:
        """单相功率详细分析"""
        V_rms = cls.rms(v)
        I_rms = cls.rms(i)
        P = cls.active_power(v, i)
        Q = cls.reactive_power(v, i, f0, fs)
        S = V_rms * I_rms
        pf, pf_type = cls.power_factor(P, S)

        return {
            "V_rms": V_rms,
            "I_rms": I_rms,
            "P": P,
            "Q": Q,
            "S": S,
            "pf": pf,
            "pf_type": pf_type,
            "P_per_phase": [P],
            "Q_per_phase": [Q],
        }


# ============================================================
# 5. 谐波分析
# ============================================================

class HarmonicAnalyzer:
    """谐波分析器"""

    def __init__(self, f0: float = 50.0, fs: float = 10000.0):
        self.f0 = f0
        self.fs = fs

    def fft_analysis(self, signal: np.ndarray, max_harmonic: int = 50) -> Dict:
        """FFT谐波分析"""
        n = len(signal)
        freqs = np.fft.rfftfreq(n, 1/self.fs)
        spectrum = np.fft.rfft(signal) * 2 / n

        magnitudes = np.abs(spectrum)
        phases = np.angle(spectrum)

        # 基波
        fund_idx = np.argmin(np.abs(freqs - self.f0))
        fund_mag = magnitudes[fund_idx]

        # 各次谐波
        harmonics = {}
        for h in range(1, max_harmonic + 1):
            h_freq = self.f0 * h
            h_idx = np.argmin(np.abs(freqs - h_freq))
            if h_idx < len(magnitudes):
                harmonics[h] = {
                    "frequency": float(freqs[h_idx]),
                    "magnitude": float(magnitudes[h_idx]),
                    "phase_deg": float(np.degrees(phases[h_idx])),
                    "thd_contribution": float(magnitudes[h_idx] / (fund_mag + 1e-10) * 100),
                }

        # THD
        harmonic_power = sum(harmonics[h]["magnitude"]**2 for h in harmonics if h > 1)
        thd = np.sqrt(harmonic_power) / (fund_mag + 1e-10) * 100

        return {
            "fundamental": {
                "frequency": float(freqs[fund_idx]),
                "magnitude": float(fund_mag),
                "rms": float(fund_mag / np.sqrt(2)),
            },
            "harmonics": harmonics,
            "thd_percent": float(thd),
            "total_rms": float(np.sqrt(np.mean(signal**2))),
        }

    def compute_thd(self, signal: np.ndarray) -> float:
        """计算总谐波畸变率"""
        result = self.fft_analysis(signal)
        return result["thd_percent"]

    def identify_harmonic_source(self, v_harmonics: Dict, i_harmonics: Dict) -> Dict:
        """谐波源识别"""
        sources = {}

        for h in range(2, 26):
            v_mag = v_harmonics.get("harmonics", {}).get(h, {}).get("magnitude", 0)
            i_mag = i_harmonics.get("harmonics", {}).get(h, {}).get("magnitude", 0)
            v_phase = v_harmonics.get("harmonics", {}).get(h, {}).get("phase_deg", 0)
            i_phase = i_harmonics.get("harmonics", {}).get(h, {}).get("phase_deg", 0)

            if v_mag > 0.01 or i_mag > 0.01:
                # 功率流向判断
                power_flow = v_mag * i_mag * np.cos(np.radians(v_phase - i_phase))
                direction = "load" if power_flow < 0 else "source"
                sources[h] = {
                    "harmonic": h,
                    "v_magnitude": v_mag,
                    "i_magnitude": i_mag,
                    "power_flow": float(power_flow),
                    "direction": direction,
                }

        return sources


# ============================================================
# 6. 三相不平衡分析
# ============================================================

class UnbalanceAnalyzer:
    """三相不平衡分析 (对称分量法)"""

    @staticmethod
    def symmetrical_components(Va: complex, Vb: complex, Vc: complex) -> Dict:
        """对称分量分解"""
        a = np.exp(1j * 2 * np.pi / 3)  # 120度旋转算子

        T = np.array([
            [1, 1, 1],
            [1, a, a**2],
            [1, a**2, a],
        ]) / 3

        V = np.array([Va, Vb, Vc])
        components = T @ V

        return {
            "V0": components[0],  # 零序
            "V1": components[1],  # 正序
            "V2": components[2],  # 负序
        }

    @staticmethod
    def voltage_unbalance_factor(negative_seq: float, positive_seq: float) -> float:
        """电压不平衡度 (VUF)"""
        if abs(positive_seq) < 1e-10:
            return 0.0
        return abs(negative_seq) / abs(positive_seq) * 100

    @classmethod
    def analyze(cls, Va: np.ndarray, Vb: np.ndarray, Vc: np.ndarray,
                f0: float = 50.0, fs: float = 10000.0) -> Dict:
        """完整三相不平衡分析"""
        n = len(Va)
        freqs = np.fft.rfftfreq(n, 1/fs)

        # 提取基波相量
        def get_fundamental(signal):
            spectrum = np.fft.rfft(signal) * 2 / n
            idx = np.argmin(np.abs(freqs - f0))
            return spectrum[idx]

        Va_ph = get_fundamental(Va)
        Vb_ph = get_fundamental(Vb)
        Vc_ph = get_fundamental(Vc)

        # 对称分量
        sc = cls.symmetrical_components(Va_ph, Vb_ph, Vc_ph)

        # 不平衡度
        vuf = cls.voltage_unbalance_factor(sc["V2"], sc["V1"])
        zero_seq_ratio = abs(sc["V0"]) / (abs(sc["V1"]) + 1e-10) * 100

        # 各相电压
        phase_voltages = {
            "Va_rms": float(np.sqrt(np.mean(Va**2))),
            "Vb_rms": float(np.sqrt(np.mean(Vb**2))),
            "Vc_rms": float(np.sqrt(np.mean(Vc**2))),
        }

        return {
            "symmetrical_components": {
                "V0": {"magnitude": float(abs(sc["V0"])), "phase_deg": float(np.degrees(np.angle(sc["V0"])))},
                "V1": {"magnitude": float(abs(sc["V1"])), "phase_deg": float(np.degrees(np.angle(sc["V1"])))},
                "V2": {"magnitude": float(abs(sc["V2"])), "phase_deg": float(np.degrees(np.angle(sc["V2"])))},
            },
            "voltage_unbalance_factor": float(vuf),
            "zero_sequence_ratio": float(zero_seq_ratio),
            "phase_voltages": phase_voltages,
            "is_balanced": vuf < 2.0,
        }


# ============================================================
# 7. 无功补偿
# ============================================================

class ReactiveCompensation:
    """无功补偿仿真"""

    @staticmethod
    def capacitor_bank(Q_target: float, V_rms: float, f0: float = 50.0) -> Dict:
        """并联电容器组设计"""
        omega = 2 * np.pi * f0
        C = Q_target / (omega * V_rms**2)
        Xc = 1 / (omega * C)

        return {
            "capacitance_F": float(C),
            "capacitance_uF": float(C * 1e6),
            "reactance_ohm": float(Xc),
            "rating_var": float(Q_target),
            "rating_kvar": float(Q_target / 1000),
        }

    @staticmethod
    def power_factor_correction(P: float, pf_before: float,
                                 pf_target: float = 0.95,
                                 V_rms: float = 220.0, f0: float = 50.0) -> Dict:
        """功率因数校正计算"""
        Q_before = P * np.tan(np.arccos(pf_before))
        Q_target = P * np.tan(np.arccos(pf_target))
        Q_compensation = Q_before - Q_target

        cap = ReactiveCompensation.capacitor_bank(Q_compensation, V_rms, f0)

        S_before = P / pf_before
        S_after = P / pf_target
        current_reduction = (1 - S_after / S_before) * 100

        return {
            "Q_before_var": float(Q_before),
            "Q_after_var": float(Q_target),
            "Q_compensation_var": float(Q_compensation),
            "Q_compensation_kvar": float(Q_compensation / 1000),
            "pf_before": float(pf_before),
            "pf_after": float(pf_target),
            "current_reduction_percent": float(current_reduction),
            "capacitor": cap,
        }

    @staticmethod
    def svg_compensation(target_pf: float = 0.95, P: float = 1000.0,
                         V_rms: float = 220.0) -> Dict:
        """SVG (静止无功发生器) 补偿方案"""
        Q_target = P * np.tan(np.arccos(target_pf))
        I_rating = Q_target / (V_rms * np.sqrt(3))

        return {
            "type": "SVG",
            "rating_var": float(Q_target),
            "rating_kvar": float(Q_target / 1000),
            "current_rating_A": float(I_rating),
            "response_time_ms": 5.0,
            "advantages": ["动态响应快", "无谐振风险", "可连续调节"],
            "disadvantages": ["成本较高", "需要电力电子器件"],
        }

    @staticmethod
    def simulate_compensation(voltage: np.ndarray, current: np.ndarray,
                               Q_compensation: float, f0: float, fs: float) -> Dict:
        """仿真补偿效果"""
        analyzer = PowerAnalyzer()

        # 补偿前
        P = analyzer.active_power(voltage, current)
        Q = analyzer.reactive_power(voltage, current, f0, fs)
        S = analyzer.apparent_power(P, Q)
        pf_before = analyzer.power_factor(P, S)[0]

        # 补偿后
        Q_after = Q - Q_compensation
        S_after = analyzer.apparent_power(P, Q_after)
        pf_after = analyzer.power_factor(P, S_after)[0]

        return {
            "before": {"P": P, "Q": Q, "S": S, "pf": pf_before},
            "after": {"P": P, "Q": Q_after, "S": S_after, "pf": pf_after},
            "improvement": {
                "Q_reduction": Q_compensation,
                "pf_improvement": pf_after - pf_before,
                "S_reduction": S - S_after,
            }
        }


# ============================================================
# 8. 综合电网仿真
# ============================================================

class PowerGridSimulation:
    """电网综合仿真"""

    def __init__(self, V_rms: float = 220.0, f0: float = 50.0, fs: float = 10000.0):
        self.V_rms = V_rms
        self.f0 = f0
        self.fs = fs
        self.gen = ThreePhaseGenerator(V_rms, f0, fs)
        self.analyzer = PowerAnalyzer()
        self.harmonic_analyzer = HarmonicAnalyzer(f0, fs)

    def simulate_balanced_system(self) -> Dict:
        """平衡系统仿真"""
        print("[1] 平衡三相系统仿真")

        # 生成平衡电压
        signal = self.gen.generate_balanced(duration=0.1)

        # RL负载
        R_load = 10.0
        L_load = 0.01
        Ia, phi = LoadModel.rl_load(signal.Va, R_load, L_load, self.f0, self.fs)
        Ib, _ = LoadModel.rl_load(signal.Vb, R_load, L_load, self.f0, self.fs)
        Ic, _ = LoadModel.rl_load(signal.Vc, R_load, L_load, self.f0, self.fs)

        # 功率分析
        power = self.analyzer.analyze_three_phase(signal, Ia, Ib, Ic)

        print(f"  有功功率: {power.P:.1f} W")
        print(f"  无功功率: {power.Q:.1f} var")
        print(f"  视在功率: {power.S:.1f} VA")
        print(f"  功率因数: {power.pf:.4f} ({power.pf_type})")

        return {
            "signal": signal,
            "currents": (Ia, Ib, Ic),
            "power": power,
        }

    def simulate_harmonic_system(self) -> Dict:
        """含谐波系统仿真"""
        print("\n[2] 谐波分析仿真")

        # 含谐波电压
        signal = self.gen.generate_with_harmonics(duration=0.1)

        # 非线性负载产生更多谐波
        R_load = 10.0
        Ia = signal.Va / R_load  # 基波电流
        # 添加谐波电流
        omega = 2 * np.pi * self.f0
        t = signal.t
        for h in [3, 5, 7, 11, 13]:
            Ia += 0.02 * np.sin(h * omega * t) * np.max(np.abs(signal.Va)) / R_load

        Ib = signal.Vb / R_load
        Ic = signal.Vc / R_load

        # 谐波分析
        v_harmonics = self.harmonic_analyzer.fft_analysis(signal.Va, max_harmonic=25)
        i_harmonics = self.harmonic_analyzer.fft_analysis(Ia, max_harmonic=25)

        print(f"  电压THD: {v_harmonics['thd_percent']:.2f}%")
        print(f"  电流THD: {i_harmonics['thd_percent']:.2f}%")

        # 主要谐波
        print("  电压谐波 (前5大):")
        sorted_h = sorted(v_harmonics["harmonics"].items(),
                         key=lambda x: x[1]["magnitude"], reverse=True)
        for h, info in sorted_h[:5]:
            if h > 1:
                print(f"    {h}次: {info['magnitude']:.3f} V ({info['thd_contribution']:.2f}%)")

        # 谐波源识别
        sources = self.harmonic_analyzer.identify_harmonic_source(v_harmonics, i_harmonics)
        n_load = sum(1 for s in sources.values() if s["direction"] == "load")
        print(f"  谐波源: {n_load}个负载侧谐波")

        return {
            "voltage_harmonics": v_harmonics,
            "current_harmonics": i_harmonics,
            "harmonic_sources": sources,
        }

    def simulate_unbalanced_system(self) -> Dict:
        """不平衡系统仿真"""
        print("\n[3] 三相不平衡仿真")

        # 不平衡电压
        signal = self.gen.generate_unbalanced(duration=0.1, imbalance=0.15, phase_shift_deg=8.0)

        # 不平衡分析
        unbalance = UnbalanceAnalyzer.analyze(signal.Va, signal.Vb, signal.Vc,
                                              self.f0, self.fs)

        print(f"  电压不平衡度 (VUF): {unbalance['voltage_unbalance_factor']:.2f}%")
        print(f"  零序分量比: {unbalance['zero_sequence_ratio']:.2f}%")
        print(f"  是否平衡: {'是' if unbalance['is_balanced'] else '否'}")

        sc = unbalance["symmetrical_components"]
        print(f"  正序: {sc['V1']['magnitude']:.2f} V")
        print(f"  负序: {sc['V2']['magnitude']:.2f} V")
        print(f"  零序: {sc['V0']['magnitude']:.2f} V")

        return unbalance

    def simulate_power_factor_correction(self) -> Dict:
        """功率因数校正仿真"""
        print("\n[4] 功率因数校正仿真")

        # 典型工业负载
        P = 100000  # 100kW
        pf_before = 0.65  # 感性负载

        # 计算补偿
        result = ReactiveCompensation.power_factor_correction(
            P, pf_before, pf_target=0.95, V_rms=self.V_rms, f0=self.f0)

        print(f"  负载功率: {P/1000:.0f} kW")
        print(f"  补偿前PF: {result['pf_before']:.4f}")
        print(f"  补偿后PF: {result['pf_after']:.4f}")
        print(f"  需补偿无功: {result['Q_compensation_kvar']:.1f} kvar")
        print(f"  电容量: {result['capacitor']['capacitance_uF']:.1f} μF")
        print(f"  电流降低: {result['current_reduction_percent']:.1f}%")

        # SVG方案
        svg = ReactiveCompensation.svg_compensation(0.95, P, self.V_rms)
        print(f"\n  SVG方案: {svg['rating_kvar']:.1f} kvar")

        return {"capacitor_correction": result, "svg": svg}

    def run_all(self) -> Dict:
        """运行所有仿真"""
        print("=" * 60)
        print("  电网仿真 - 三相/功率因数/谐波/无功补偿")
        print("=" * 60)

        results = {
            "balanced": self.simulate_balanced_system(),
            "harmonics": self.simulate_harmonic_system(),
            "unbalanced": self.simulate_unbalanced_system(),
            "pfc": self.simulate_power_factor_correction(),
        }

        print("\n" + "=" * 60)
        print("  仿真完成!")
        print("=" * 60)

        return results


def plot_results(results: Dict, save_path: Optional[str] = None):
    """绘制仿真结果"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib未安装, 跳过绘图")
        return

    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    fig.suptitle("电网仿真结果", fontsize=14)

    # 三相电压
    if "balanced" in results:
        sig = results["balanced"]["signal"]
        ax = axes[0, 0]
        n = min(500, len(sig.t))
        ax.plot(sig.t[:n]*1000, sig.Va[:n], label="Va")
        ax.plot(sig.t[:n]*1000, sig.Vb[:n], label="Vb")
        ax.plot(sig.t[:n]*1000, sig.Vc[:n], label="Vc")
        ax.set_title("三相电压 (平衡)")
        ax.set_xlabel("时间 (ms)")
        ax.legend()

    # 含谐波电压
    if "harmonics" in results:
        vh = results["harmonics"]["voltage_harmonics"]
        ax = axes[0, 1]
        harmonics = vh["harmonics"]
        h_nums = sorted([h for h in harmonics if h > 1])[:20]
        h_mags = [harmonics[h]["thd_contribution"] for h in h_nums]
        ax.bar(h_nums, h_mags, color='steelblue')
        ax.set_title(f"电压谐波 (THD={vh['thd_percent']:.2f}%)")
        ax.set_xlabel("谐波次数")
        ax.set_ylabel("THD贡献 (%)")

    # 电流THD
    if "harmonics" in results:
        ih = results["harmonics"]["current_harmonics"]
        ax = axes[0, 2]
        harmonics = ih["harmonics"]
        h_nums = sorted([h for h in harmonics if h > 1])[:20]
        h_mags = [harmonics[h]["thd_contribution"] for h in h_nums]
        ax.bar(h_nums, h_mags, color='coral')
        ax.set_title(f"电流谐波 (THD={ih['thd_percent']:.2f}%)")
        ax.set_xlabel("谐波次数")

    # 不平衡分析
    if "unbalanced" in results:
        ub = results["unbalanced"]
        ax = axes[1, 0]
        sc = ub["symmetrical_components"]
        labels = ["正序", "负序", "零序"]
        mags = [sc["V1"]["magnitude"], sc["V2"]["magnitude"], sc["V0"]["magnitude"]]
        colors = ["green", "red", "gray"]
        ax.bar(labels, mags, color=colors)
        ax.set_title(f"对称分量 (VUF={ub['voltage_unbalance_factor']:.2f}%)")

    # 功率因数校正
    if "pfc" in results:
        pfc = results["pfc"]["capacitor_correction"]
        ax = axes[1, 1]
        labels = ["补偿前", "补偿后"]
        pfs = [pfc["pf_before"], pfc["pf_after"]]
        ax.bar(labels, pfs, color=["red", "green"])
        ax.set_ylim(0, 1)
        ax.set_title("功率因数校正")
        for i, v in enumerate(pfs):
            ax.text(i, v + 0.02, f"{v:.3f}", ha='center')

    # 功率分解
    if "balanced" in results:
        power = results["balanced"]["power"]
        ax = axes[1, 2]
        ax.bar(["P (有功)", "Q (无功)", "S (视在)"],
               [power.P, abs(power.Q), power.S],
               color=["blue", "orange", "gray"])
        ax.set_title(f"功率分析 (PF={power.pf:.4f})")
        ax.set_ylabel("W / var / VA")

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[OK] 图像已保存: {save_path}")
    plt.show()


# ============================================================
# 主程序
# ============================================================

def main():
    sim = PowerGridSimulation()
    results = sim.run_all()

    try:
        plot_results(results)
    except Exception as e:
        print(f"[INFO] 绘图跳过: {e}")

    return results


if __name__ == "__main__":
    main()
