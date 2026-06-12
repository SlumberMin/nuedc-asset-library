#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轻量级流体仿真模块
===================
管道流动 | 伯努利方程 | 泵特性曲线 | 流量测量

适用于电赛中流量传感器标定、泵控系统设计等场景。
仅使用numpy，无需CFD网格，基于1D/解析方法。
"""

import numpy as np
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass


# ============================================================
# 物理常数
# ============================================================
GRAVITY = 9.81           # m/s²
WATER_DENSITY = 998.0    # kg/m³ (20°C)
WATER_VISCOSITY = 1.003e-3  # Pa·s (20°C 动力粘度)
AIR_DENSITY = 1.225      # kg/m³
AIR_VISCOSITY = 1.81e-5  # Pa·s


# ============================================================
# 数据结构
# ============================================================
@dataclass
class PipeFlowResult:
    """管道流动结果"""
    velocity: float          # 流速 (m/s)
    reynolds_number: float   # 雷诺数
    flow_regime: str         # 流态
    friction_factor: float   # 摩擦系数
    pressure_drop: float     # 压降 (Pa)
    flow_rate: float         # 体积流量 (m³/s)
    head_loss: float         # 水头损失 (m)

@dataclass
class BernoulliResult:
    """伯努利方程结果"""
    p1: float                # 位置1压力 (Pa)
    v1: float                # 位置1流速 (m/s)
    p2: float                # 位置2压力 (Pa)
    v2: float                # 位置2流速 (m/s)
    pressure_change: float   # 压力变化 (Pa)
    velocity_change: float   # 流速变化 (m/s)

@dataclass
class PumpCurveResult:
    """泵特性结果"""
    flow_rates: np.ndarray   # 流量数组 (m³/s)
    head: np.ndarray         # 扬程 (m)
    power: np.ndarray        # 功率 (W)
    efficiency: np.ndarray   # 效率 (%)
    npsh_required: np.ndarray  # NPSHr (m)
    operating_point: Tuple[float, float]  # 工作点 (Q, H)


# ============================================================
# 管道流动
# ============================================================
class PipeFlowCalculator:
    """管道流动计算器"""

    @staticmethod
    def reynolds_number(velocity: float, diameter: float,
                         density: float = WATER_DENSITY,
                         viscosity: float = WATER_VISCOSITY) -> float:
        """计算雷诺数 Re = ρvD/μ"""
        return density * velocity * diameter / viscosity

    @staticmethod
    def flow_regime(Re: float) -> str:
        """判断流态"""
        if Re < 2300:
            return '层流'
        elif Re < 4000:
            return '过渡区'
        else:
            return '湍流'

    @staticmethod
    def darcy_friction_factor(Re: float, roughness: float = 0,
                               diameter: float = 1.0) -> float:
        """
        Darcy摩擦系数

        Args:
            Re: 雷诺数
            roughness: 管壁粗糙度 (m)
            diameter: 管径 (m)

        Returns:
            摩擦系数 f
        """
        if Re < 2300:
            return 64.0 / Re if Re > 0 else 0.0

        # Colebrook-White方程 (迭代求解)
        rel_roughness = roughness / diameter if diameter > 0 else 0

        # 初始估计 (Swamee-Jain近似)
        if rel_roughness > 0:
            f = 0.25 / (np.log10(rel_roughness / 3.7 + 5.74 / Re**0.9))**2
        else:
            f = 0.316 / Re**0.25  # Blasius

        # Newton迭代求解Colebrook-White
        for _ in range(20):
            if f <= 0:
                f = 0.01
            sqrt_f = np.sqrt(f)
            rhs = -2 * np.log10(rel_roughness / 3.7 + 2.51 / (Re * sqrt_f + 1e-30))
            f_new = 1.0 / (rhs**2 + 1e-30)
            if abs(f_new - f) / (f + 1e-30) < 1e-8:
                break
            f = f_new

        return max(f, 0.001)

    @staticmethod
    def head_loss_darcy(diameter: float, length: float,
                         velocity: float,
                         roughness: float = 0,
                         density: float = WATER_DENSITY,
                         viscosity: float = WATER_VISCOSITY) -> float:
        """
        Darcy-Weisbach水头损失

        h_f = f * (L/D) * (v²/2g)

        Returns:
            水头损失 (m)
        """
        Re = PipeFlowCalculator.reynolds_number(velocity, diameter, density, viscosity)
        f = PipeFlowCalculator.darcy_friction_factor(Re, roughness, diameter)
        return f * (length / diameter) * (velocity**2 / (2 * GRAVITY))

    @staticmethod
    def minor_loss(velocity: float, K: float) -> float:
        """
        局部水头损失

        h_m = K * v²/2g

        Args:
            velocity: 流速 (m/s)
            K: 局部损失系数

        Returns:
            水头损失 (m)
        """
        return K * velocity**2 / (2 * GRAVITY)

    @staticmethod
    def pipe_flow(diameter: float, length: float,
                   flow_rate: float = None, velocity: float = None,
                   roughness: float = 0.0015e-3,
                   density: float = WATER_DENSITY,
                   viscosity: float = WATER_VISCOSITY,
                   minor_loss_coefficients: List[float] = None) -> PipeFlowResult:
        """
        完整管道流动计算

        Args:
            diameter: 管径 (m)
            length: 管长 (m)
            flow_rate: 体积流量 (m³/s), 与velocity二选一
            velocity: 流速 (m/s)
            roughness: 管壁粗糙度 (m), 钢管≈0.045mm, PVC≈0.0015mm
            density: 流体密度
            viscosity: 动力粘度
            minor_loss_coefficients: 局部损失系数列表

        Returns:
            PipeFlowResult
        """
        area = np.pi * diameter**2 / 4

        if flow_rate is not None:
            velocity = flow_rate / area
        elif velocity is not None:
            flow_rate = velocity * area
        else:
            raise ValueError("需要提供flow_rate或velocity")

        Re = PipeFlowCalculator.reynolds_number(velocity, diameter, density, viscosity)
        regime = PipeFlowCalculator.flow_regime(Re)
        f = PipeFlowCalculator.darcy_friction_factor(Re, roughness, diameter)

        # 沿程损失
        h_f = f * (length / diameter) * (velocity**2 / (2 * GRAVITY))

        # 局部损失
        if minor_loss_coefficients:
            h_m = sum(PipeFlowCalculator.minor_loss(velocity, K)
                      for K in minor_loss_coefficients)
        else:
            h_m = 0.0

        total_head_loss = h_f + h_m
        pressure_drop = total_head_loss * density * GRAVITY

        return PipeFlowResult(
            velocity=velocity,
            reynolds_number=Re,
            flow_regime=regime,
            friction_factor=f,
            pressure_drop=pressure_drop,
            flow_rate=flow_rate,
            head_loss=total_head_loss
        )

    @staticmethod
    def velocity_profile(r: np.ndarray, diameter: float,
                          velocity_mean: float,
                          Re: float) -> np.ndarray:
        """
        管道流速分布

        Args:
            r: 径向坐标数组 (从中心到壁面, 0~D/2)
            diameter: 管径
            velocity_mean: 平均流速
            Re: 雷诺数

        Returns:
            各点流速
        """
        R = diameter / 2

        if Re < 2300:
            # 层流: 抛物线分布
            v = 2 * velocity_mean * (1 - (r / R)**2)
        else:
            # 湍流: 1/7律
            n = 7
            v = velocity_mean * (n + 1) * (n + 2) / (2 * n) * (1 - np.abs(r / R))**(1/n)
            v = np.maximum(v, 0)

        return v


# ============================================================
# 伯努利方程
# ============================================================
class BernoulliSolver:
    """伯努利方程求解器"""

    @staticmethod
    def bernoulli_basic(p1: float, v1: float, z1: float,
                         p2: float, v2: float, z2: float,
                         density: float = WATER_DENSITY) -> BernoulliResult:
        """
        伯努利方程验证

        P1 + ½ρv1² + ρgz1 = P2 + ½ρv2² + ρgz2 + losses

        返回各项的值和平衡情况
        """
        e1 = p1 + 0.5 * density * v1**2 + density * GRAVITY * z1
        e2 = p2 + 0.5 * density * v2**2 + density * GRAVITY * z2

        return BernoulliResult(
            p1=p1, v1=v1, p2=p2, v2=v2,
            pressure_change=p2 - p1,
            velocity_change=v2 - v1
        )

    @staticmethod
    def venturi_flow(p1: float, p2: float,
                     d1: float, d2: float,
                     density: float = WATER_DENSITY,
                     Cd: float = 0.98) -> Dict:
        """
        文丘里流量计计算

        Args:
            p1: 上游压力 (Pa)
            p2: 喉部压力 (Pa)
            d1: 上游管径 (m)
            d2: 喉部直径 (m)
            density: 流体密度
            Cd: 流量系数

        Returns:
            {'flow_rate': ..., 'velocity_1': ..., 'velocity_2': ..., 'dp': ...}
        """
        a1 = np.pi * d1**2 / 4
        a2 = np.pi * d2**2 / 4

        dp = p1 - p2
        if dp < 0:
            dp = 0

        # Q = Cd * A2 / sqrt(1-(A2/A1)^2) * sqrt(2*dp/rho)
        beta = d2 / d1
        if beta >= 1:
            return {'flow_rate': 0, 'velocity_1': 0, 'velocity_2': 0, 'dp': 0}

        area_ratio = a2 / a1
        denominator = np.sqrt(1 - area_ratio**2)

        Q = Cd * a2 / denominator * np.sqrt(2 * dp / density)

        v1 = Q / a1
        v2 = Q / a2

        return {
            'flow_rate': Q,
            'velocity_1': v1,
            'velocity_2': v2,
            'dp': dp,
            'pressure_drop_pa': dp,
            'Re_throat': density * v2 * d2 / WATER_VISCOSITY
        }

    @staticmethod
    def orifice_flow(p1: float, p2: float,
                     pipe_d: float, orifice_d: float,
                     density: float = WATER_DENSITY,
                     Cd: float = 0.61) -> Dict:
        """
        孔板流量计计算

        Returns:
            {'flow_rate': ..., 'velocity_pipe': ...}
        """
        a_orifice = np.pi * orifice_d**2 / 4
        beta = orifice_d / pipe_d

        dp = max(p1 - p2, 0)

        Q = Cd * a_orifice / np.sqrt(1 - beta**4) * np.sqrt(2 * dp / density)
        v_pipe = Q / (np.pi * pipe_d**2 / 4)

        return {
            'flow_rate': Q,
            'velocity_pipe': v_pipe,
            'dp': dp,
            'beta': beta
        }

    @staticmethod
    def pitot_tube_velocity(p_total: float, p_static: float,
                             density: float = AIR_DENSITY) -> float:
        """
        皮托管测速

        v = sqrt(2*(P_total - P_static) / ρ)

        Returns:
            流速 (m/s)
        """
        dp = p_total - p_static
        if dp < 0:
            return 0.0
        return np.sqrt(2 * dp / density)


# ============================================================
# 泵特性曲线
# ============================================================
class PumpSimulator:
    """泵特性曲线仿真器"""

    @staticmethod
    def centrifugal_pump_curve(Q_rated: float, H_rated: float,
                                n_points: int = 50) -> Tuple[np.ndarray, np.ndarray]:
        """
        离心泵扬程-流量曲线 (二次模型)

        H = H_0 - k * Q²

        Args:
            Q_rated: 额定流量 (m³/s)
            H_rated: 额定扬程 (m)
            n_points: 数据点数

        Returns:
            (flow_rates, heads)
        """
        # H_0 ≈ 1.2 * H_rated (零流量扬程)
        H_0 = 1.2 * H_rated
        # k = (H_0 - H_rated) / Q_rated²
        k = (H_0 - H_rated) / (Q_rated**2 + 1e-30)

        Q_max = Q_rated * 1.5  # 最大流量
        Q = np.linspace(0, Q_max, n_points)
        H = H_0 - k * Q**2
        H = np.maximum(H, 0)

        return Q, H

    @staticmethod
    def pump_power_efficiency(Q: np.ndarray, H: np.ndarray,
                               density: float = WATER_DENSITY,
                               pump_type: str = 'centrifugal') -> Tuple[np.ndarray, np.ndarray]:
        """
        计算泵功率和效率

        Args:
            Q: 流量数组
            H: 扬程数组
            density: 流体密度

        Returns:
            (shaft_power, efficiency)
        """
        # 水力功率
        P_hydraulic = density * GRAVITY * Q * H  # W

        # 效率曲线 (典型离心泵)
        Q_rated_idx = len(Q) // 2
        Q_rated = Q[Q_rated_idx] if Q_rated_idx < len(Q) else Q[-1]

        if Q_rated > 0:
            q_ratio = Q / Q_rated
        else:
            q_ratio = Q

        # 效率模型: η = η_max * (1 - a*(q_ratio - 1)^2)
        eta_max = 0.85
        efficiency = eta_max * (1 - 0.5 * (q_ratio - 1.0)**2)
        efficiency = np.clip(efficiency, 0.1, eta_max)

        shaft_power = P_hydraulic / (efficiency + 1e-30)

        return shaft_power, efficiency * 100  # 效率转%

    @staticmethod
    def system_curve(H_static: float, pipe_diameter: float,
                     pipe_length: float, roughness: float = 0.0015e-3,
                     minor_K: float = 5.0,
                     n_points: int = 50,
                     density: float = WATER_DENSITY,
                     viscosity: float = WATER_VISCOSITY) -> Tuple[np.ndarray, np.ndarray]:
        """
        管路系统阻力曲线

        H_system = H_static + f(L/D)(Q/A)²/2g + K*(Q/A)²/2g

        Returns:
            (flow_rates, system_heads)
        """
        area = np.pi * pipe_diameter**2 / 4
        Q_max = area * 5.0  # 最大5m/s
        Q = np.linspace(0, Q_max, n_points)

        H_system = np.zeros_like(Q)
        for i, q in enumerate(Q):
            v = q / area
            Re = PipeFlowCalculator.reynolds_number(v, pipe_diameter, density, viscosity)
            f = PipeFlowCalculator.darcy_friction_factor(Re, roughness, pipe_diameter)
            h_friction = f * (pipe_length / pipe_diameter) * v**2 / (2 * GRAVITY)
            h_minor = minor_K * v**2 / (2 * GRAVITY)
            H_system[i] = H_static + h_friction + h_minor

        return Q, H_system

    @staticmethod
    def find_operating_point(Q_pump: np.ndarray, H_pump: np.ndarray,
                              Q_system: np.ndarray, H_system: np.ndarray) -> Tuple[float, float]:
        """
        求泵与系统曲线的交点 (工作点)

        Returns:
            (Q_operating, H_operating)
        """
        # 插值到相同Q轴
        Q_min = max(Q_pump[0], Q_system[0])
        Q_max = min(Q_pump[-1], Q_system[-1])
        Q_common = np.linspace(Q_min, Q_max, 200)

        H_p = np.interp(Q_common, Q_pump, H_pump)
        H_s = np.interp(Q_common, Q_system, H_system)

        # 找交点 (差值变号)
        diff = H_p - H_s
        crossings = np.where(np.diff(np.sign(diff)))[0]

        if len(crossings) > 0:
            idx = crossings[0]
            # 线性插值
            t = diff[idx] / (diff[idx] - diff[idx+1])
            Q_op = Q_common[idx] + t * (Q_common[idx+1] - Q_common[idx])
            H_op = H_p[idx] + t * (H_p[idx+1] - H_p[idx])
            return Q_op, H_op

        # 找最接近的点
        idx = np.argmin(np.abs(diff))
        return Q_common[idx], H_p[idx]

    @staticmethod
    def affinity_laws(Q1: float, H1: float, P1: float,
                      n1: float, n2: float) -> Tuple[float, float, float]:
        """
        泵相似定律

        Q2/Q1 = n2/n1
        H2/H1 = (n2/n1)²
        P2/P1 = (n2/n1)³

        Returns:
            (Q2, H2, P2)
        """
        ratio = n2 / n1
        return Q1 * ratio, H1 * ratio**2, P1 * ratio**3


# ============================================================
# 流量测量仿真
# ============================================================
class FlowMeasurement:
    """流量测量方法仿真"""

    @staticmethod
    def ultrasonic_flowmeter(signal_up: np.ndarray, signal_down: np.ndarray,
                              L_path: float, angle_deg: float,
                              fs: float) -> Dict:
        """
        超声波时差法流量计

        Args:
            signal_up: 上游超声波信号
            signal_down: 下游超声波信号
            L_path: 声程长度 (m)
            angle_deg: 声道角度 (degrees)
            fs: 采样率

        Returns:
            {'time_diff': ..., 'velocity': ..., 'flow_rate': ...}
        """
        angle_rad = np.radians(angle_deg)

        # 互相关求时差
        correlation = np.correlate(signal_down, signal_up, mode='full')
        lags = np.arange(-len(signal_up) + 1, len(signal_up))
        peak_lag = lags[np.argmax(np.abs(correlation))]
        time_diff = peak_lag / fs

        # 流速计算
        C_sound = SPEED_OF_SOUND if 'SPEED_OF_SOUND' in dir() else 343.0
        if abs(time_diff) > 1e-15:
            v_fluid = (L_path / (2 * np.cos(angle_rad))) * time_diff / (
                (L_path / (C_sound + 1e-30))**2 - time_diff**2 / 4 + 1e-30
            )
        else:
            v_fluid = 0.0

        return {
            'time_diff': time_diff,
            'velocity': v_fluid,
        }

    @staticmethod
    def electromagnetic_flowmeter(voltage: float, diameter: float,
                                    B_field: float) -> float:
        """
        电磁流量计

        Q = (π * D * V) / (4 * B)

        Args:
            voltage: 感应电压 (V)
            diameter: 管径 (m)
            B_field: 磁感应强度 (T)

        Returns:
            流量 (m³/s)
        """
        if B_field < 1e-10:
            return 0.0
        return np.pi * diameter * voltage / (4 * B_field)

    @staticmethod
    def turbine_flowmeter(frequency: float, K_factor: float) -> float:
        """
        涡轮流量计

        Q = f / K

        Args:
            frequency: 脉冲频率 (Hz)
            K_factor: 仪表系数 (pulse/L)

        Returns:
            流量 (L/s)
        """
        if K_factor < 1e-10:
            return 0.0
        return frequency / K_factor


# ============================================================
# 辅助工具
# ============================================================
SPEED_OF_SOUND = 343.0  # m/s (for ultrasonic flowmeter)


class FluidUtils:
    """流体工具函数"""

    @staticmethod
    def reynolds_number_to_regime(Re_array: np.ndarray) -> List[str]:
        """批量判断流态"""
        return [PipeFlowCalculator.flow_regime(Re) for Re in Re_array]

    @staticmethod
    def pressure_at_depth(depth: float, density: float = WATER_DENSITY,
                           p_atm: float = 101325.0) -> float:
        """计算某深度的绝对压力"""
        return p_atm + density * GRAVITY * depth

    @staticmethod
    def hydrostatic_force(width: float, depth: float,
                           density: float = WATER_DENSITY) -> float:
        """计算垂直矩形壁面上的静水压力"""
        return 0.5 * density * GRAVITY * depth**2 * width

    @staticmethod
    def settling_velocity(particle_diameter: float,
                           particle_density: float,
                           fluid_density: float = WATER_DENSITY,
                           fluid_viscosity: float = WATER_VISCOSITY) -> float:
        """
        Stokes沉降速度

        v = (ρp - ρf) * g * d² / (18μ)

        Returns:
            沉降速度 (m/s)
        """
        return ((particle_density - fluid_density) * GRAVITY *
                particle_diameter**2 / (18 * fluid_viscosity))


# ============================================================
# 演示 / 测试
# ============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("  轻量级流体仿真模块 - 功能演示")
    print("=" * 60)

    # 1. 管道流动
    print("\n[1] 管道流动计算...")
    for d_mm in [10, 25, 50, 100]:
        d = d_mm * 1e-3
        result = PipeFlowCalculator.pipe_flow(
            diameter=d, length=10, velocity=1.0,
            roughness=0.045e-3
        )
        print(f"  DN{d_mm} @ 1m/s: Re={result.reynolds_number:.0f}, "
              f"{result.flow_regime}, ΔP={result.pressure_drop:.1f}Pa, "
              f"f={result.friction_factor:.4f}")

    # 2. 伯努利/文丘里
    print("\n[2] 文丘里流量计...")
    for dp_pa in [100, 500, 1000, 5000]:
        result = BernoulliSolver.venturi_flow(
            p1=101325 + dp_pa, p2=101325,
            d1=0.05, d2=0.025
        )
        print(f"  ΔP={dp_pa}Pa: Q={result['flow_rate']*1000:.2f} L/s, "
              f"v_throat={result['velocity_2']:.2f} m/s")

    # 3. 皮托管
    print("\n[3] 皮托管测速...")
    for dp in [10, 50, 100, 500]:
        v = BernoulliSolver.pitot_tube_velocity(101325 + dp, 101325, AIR_DENSITY)
        print(f"  ΔP={dp}Pa (空气): v={v:.2f} m/s")

    # 4. 泵特性
    print("\n[4] 泵特性曲线...")
    Q_rated = 0.01  # 10 L/s
    H_rated = 30    # 30m
    Q_pump, H_pump = PumpSimulator.centrifugal_pump_curve(Q_rated, H_rated)
    shaft_power, eff = PumpSimulator.power_efficiency(Q_pump, H_pump)

    # 系统曲线
    Q_sys, H_sys = PumpSimulator.system_curve(
        H_static=10, pipe_diameter=0.08, pipe_length=100
    )

    # 工作点
    Q_op, H_op = PumpSimulator.find_operating_point(Q_pump, H_pump, Q_sys, H_sys)
    print(f"  额定: Q={Q_rated*1000:.0f}L/s, H={H_rated}m")
    print(f"  工作点: Q={Q_op*1000:.2f}L/s, H={H_op:.1f}m")

    # 效率
    idx = np.argmin(np.abs(Q_pump - Q_op))
    if idx < len(eff):
        print(f"  工作点效率: {eff[idx]:.1f}%")
        print(f"  轴功率: {shaft_power[idx]:.0f} W")

    # 5. 相似定律
    print("\n[5] 泵相似定律 (变速)...")
    Q2, H2, P2 = PumpSimulator.affinity_laws(Q_rated, H_rated, 3000, 2900, 2600)
    print(f"  2900rpm → 2600rpm:")
    print(f"    Q: {Q_rated*1000:.2f} → {Q2*1000:.2f} L/s")
    print(f"    H: {H_rated:.1f} → {H2:.1f} m")
    print(f"    P: 3000 → {P2:.0f} W")

    # 6. 流量测量
    print("\n[6] 流量测量仿真...")
    # 电磁流量计
    for V in [0.001, 0.005, 0.01]:
        Q = FlowMeasurement.electromagnetic_flowmeter(V, 0.05, 0.02)
        print(f"  电磁流量计 V={V*1000:.1f}mV: Q={Q*1000:.2f} L/s")

    # 涡轮流量计
    for freq in [100, 500, 1000]:
        Q = FlowMeasurement.turbine_flowmeter(freq, K_factor=100)
        print(f"  涡轮流量计 f={freq}Hz (K=100): Q={Q:.2f} L/s")

    # 7. 流速分布
    print("\n[7] 管道流速分布...")
    r = np.linspace(0, 0.025, 10)
    v_laminar = PipeFlowCalculator.velocity_profile(r, 0.05, 1.0, Re=1000)
    v_turbulent = PipeFlowCalculator.velocity_profile(r, 0.05, 1.0, Re=50000)
    print(f"  层流中心速度: {v_laminar[0]:.2f} m/s (平均 1.0)")
    print(f"  湍流中心速度: {v_turbulent[0]:.2f} m/s (平均 1.0)")

    print("\n" + "=" * 60)
    print("  流体仿真完成!")
    print("=" * 60)
