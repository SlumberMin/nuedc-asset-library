#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
热管理仿真 - 散热器/风扇/热管/液冷模型
========================================
适用于电赛电源/功率器件热设计题目
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. 散热器模型
# ============================================================
class Heatsink:
    def __init__(self, base_area_cm2=25, fin_height_mm=20, fin_thickness_mm=1,
                 n_fins=10, fin_spacing_mm=3, material='aluminum'):
        self.base_area = base_area_cm2 * 1e-4  # m^2
        self.fin_height = fin_height_mm * 1e-3
        self.fin_thickness = fin_thickness_mm * 1e-3
        self.n_fins = n_fins
        self.fin_spacing = fin_spacing_mm * 1e-3
        self.material = material

        k_materials = {'aluminum': 205, 'copper': 400, 'steel': 50}
        self.k = k_materials.get(material, 205)

    def natural_convection_resistance(self, T_junction, T_ambient=25):
        """自然对流热阻 (简化模型)"""
        # R_conv = 1/(h*A), h自然对流 5~25 W/(m²·K)
        A_total = self._total_surface_area()
        # 简化: h ≈ 1.32*(ΔT/fin_height)^0.25 (垂直平板)
        dT = max(T_junction - T_ambient, 1)
        h = 1.32 * (dT / self.fin_height)**0.25
        R_conv = 1 / (h * A_total)
        return R_conv, h

    def forced_convection_resistance(self, air_velocity_mps=2.0):
        """强制对流热阻"""
        A_total = self._total_surface_area()
        # h ≈ 10.45 - v + 10*v^0.5 (简化风冷模型)
        h = max(10.45 - air_velocity_mps + 10*np.sqrt(air_velocity_mps), 10)
        R_conv = 1 / (h * A_total)
        return R_conv, h

    def fin_efficiency(self, h):
        """翅片效率"""
        m = np.sqrt(2*h / (self.k * self.fin_thickness))
        eta = np.tanh(m * self.fin_height) / (m * self.fin_height)
        return min(eta, 1.0)

    def _total_surface_area(self):
        """总散热面积"""
        L_fin = self.fin_height
        W_fin = self.fin_thickness
        # 基座面积 + 翅片面积
        fin_area = 2 * L_fin * W_fin * self.n_fins  # 两侧
        base_exposed = self.base_area - self.n_fins * W_fin * np.sqrt(self.base_area)
        return max(fin_area + base_exposed, self.base_area)

    def total_thermal_resistance(self, T_junction, air_velocity=0, T_ambient=25):
        """总热阻"""
        if air_velocity > 0:
            R_conv, h = self.forced_convection_resistance(air_velocity)
        else:
            R_conv, h = self.natural_convection_resistance(T_junction, T_ambient)
        eta_fin = self.fin_efficiency(h)
        R_effective = R_conv / eta_fin
        return R_effective


# ============================================================
# 2. 热管模型
# ============================================================
class HeatPipe:
    def __init__(self, length_mm=100, diameter_mm=6, Q_max_W=30):
        self.length = length_mm * 1e-3
        self.diameter = diameter_mm * 1e-3
        self.Q_max = Q_max_W
        self.R_hp = 0.1  # 典型热管热阻 0.1~0.5 °C/W

    def thermal_resistance(self, power_W):
        """热管热阻（考虑功率限制）"""
        if power_W > self.Q_max:
            return float('inf')
        return self.R_hp * (1 + 0.1 * power_W / self.Q_max)


# ============================================================
# 3. 风扇模型
# ============================================================
class Fan:
    def __init__(self, max_airflow_cfm=50, max_pressure_pa=200, diameter_mm=80):
        self.max_airflow = max_airflow_cfm * 4.72e-4  # CFM -> m³/s
        self.max_pressure = max_pressure_pa
        self.area = np.pi * (diameter_mm*1e-3/2)**2

    def operating_point(self, system_resistance):
        """风扇工作点 (P-Q曲线交点)"""
        # 风扇P-Q: P = Pmax*(1 - Q/Qmax)
        # 系统: P = R*Q^2
        # 联立求解
        Q = np.linspace(0, self.max_airflow, 1000)
        P_fan = self.max_pressure * (1 - Q/self.max_airflow)
        P_system = system_resistance * Q**2

        idx = np.argmin(np.abs(P_fan - P_system))
        return Q[idx], P_fan[idx]


# ============================================================
# 4. 液冷系统模型
# ============================================================
class LiquidCooling:
    def __init__(self, cp=4186, rho=1000, flow_rate_lpm=1.0):
        self.cp = cp      # J/(kg·K) 水
        self.rho = rho     # kg/m³
        self.flow_rate = flow_rate_lpm / 60000  # L/min -> m³/s

    def thermal_resistance(self, area_m2=0.01):
        """液冷热阻"""
        # h_water ≈ 3000~10000 W/(m²·K)
        h = 5000
        return 1 / (h * area_m2)

    def temperature_rise(self, power_W):
        """冷却液温升 ΔT = Q/(ṁ*cp)"""
        mass_flow = self.rho * self.flow_rate
        return power_W / (mass_flow * self.cp)


# ============================================================
# 5. 瞬态热响应
# ============================================================
def transient_thermal_response(R_th, C_th, P_step, t_array, T_init=25):
    """一阶RC热模型阶跃响应"""
    tau = R_th * C_th  # 热时间常数
    T = T_init + P_step * R_th * (1 - np.exp(-t_array/tau))
    return T


# ============================================================
# 主仿真
# ============================================================
def main():
    fig, axes = plt.subplots(3, 2, figsize=(14, 16))
    fig.suptitle('热管理仿真综合', fontsize=16, fontweight='bold')

    hs = Heatsink(base_area_cm2=25, fin_height_mm=20, n_fins=12)
    fan = Fan(max_airflow_cfm=50, diameter_mm=80)

    # --- 1. 不同散热器热阻对比 ---
    ax = axes[0, 0]
    powers = np.linspace(1, 50, 50)
    T_junction_nat = []
    T_junction_forced = []
    for P in powers:
        R_nat = hs.total_thermal_resistance(100, air_velocity=0, T_ambient=25)
        R_for = hs.total_thermal_resistance(100, air_velocity=3, T_ambient=25)
        T_junction_nat.append(25 + P * R_nat)
        T_junction_forced.append(25 + P * R_for)
    ax.plot(powers, T_junction_nat, 'r-o', markersize=3, label='自然对流')
    ax.plot(powers, T_junction_forced, 'b-s', markersize=3, label='强制风冷 (3m/s)')
    ax.axhline(125, color='k', linestyle='--', alpha=0.5, label='Tj_max=125°C')
    ax.set_xlabel('功耗 (W)')
    ax.set_ylabel('结温 (°C)')
    ax.set_title('散热器温度 vs 功耗')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 2. 翅片效率 vs 风速 ---
    ax = axes[0, 1]
    velocities = np.linspace(0, 10, 50)
    fin_etas = []
    for v in velocities:
        _, h = hs.forced_convection_resistance(v)
        fin_etas.append(hs.fin_efficiency(h))
    ax.plot(velocities, fin_etas, 'g-', linewidth=2)
    ax.set_xlabel('风速 (m/s)')
    ax.set_ylabel('翅片效率')
    ax.set_title('翅片效率 vs 风速')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 1.05)

    # --- 3. 风扇工作点 ---
    ax = axes[1, 0]
    Q = np.linspace(0, fan.max_airflow*1.5, 100)
    P_fan = fan.max_pressure * (1 - Q/fan.max_airflow)
    P_fan = np.maximum(P_fan, 0)

    for R_sys, label in [(1e4, '低阻力'), (5e4, '中阻力'), (1e5, '高阻力')]:
        P_sys = R_sys * Q**2
        ax.plot(Q*60000, P_sys, '--', label=f'系统 ({label})')

    ax.plot(Q*60000, P_fan, 'r-', linewidth=2, label='风扇P-Q曲线')

    # 工作点
    for R_sys, color in [(1e4, 'green'), (5e4, 'orange'), (1e5, 'red')]:
        q_op, p_op = fan.operating_point(R_sys)
        ax.plot(q_op*60000, p_op, 'o', color=color, markersize=8)

    ax.set_xlabel('流量 (L/min)')
    ax.set_ylabel('压力 (Pa)')
    ax.set_title('风扇工作点分析')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 4. 瞬态热响应 ---
    ax = axes[1, 1]
    t = np.linspace(0, 300, 1000)  # 5分钟
    configs = [
        (0.5, 50, '小型散热器'),
        (1.0, 100, '中型散热器'),
        (2.0, 200, '大型散热器+热管'),
    ]
    for R, C, label in configs:
        T = transient_thermal_response(R, C, 10, t, T_init=25)
        ax.plot(t, T, linewidth=2, label=f'{label} (τ={R*C:.0f}s)')
    ax.axhline(85, color='r', linestyle='--', alpha=0.5, label='限值85°C')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('温度 (°C)')
    ax.set_title('瞬态热响应 (P=10W阶跃)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 5. 液冷 vs 风冷对比 ---
    ax = axes[2, 0]
    lc = LiquidCooling(flow_rate_lpm=1.0)
    powers = np.linspace(5, 200, 50)
    T_air = [25 + P * hs.total_thermal_resistance(100, air_velocity=3) for P in powers]
    T_liquid = [25 + P * lc.thermal_resistance(0.01) + lc.temperature_rise(P) for P in powers]
    T_heatpipe = [25 + P * 0.15 for P in powers]
    ax.plot(powers, T_air, 'r-', linewidth=2, label='强制风冷')
    ax.plot(powers, T_heatpipe, 'g-', linewidth=2, label='热管+散热片')
    ax.plot(powers, T_liquid, 'b-', linewidth=2, label='液冷')
    ax.axhline(125, color='k', linestyle='--', alpha=0.5)
    ax.set_xlabel('功耗 (W)')
    ax.set_ylabel('结温 (°C)')
    ax.set_title('不同冷却方案对比')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 6. 温度场示意 (2D热扩散) ---
    ax = axes[2, 1]
    nx, ny = 40, 40
    T_field = np.ones((ny, nx)) * 25
    # 热源在中心
    cx, cy = nx//2, ny//2
    T_field[cy-3:cy+3, cx-3:cx+3] = 80

    # 简化2D扩散迭代
    for _ in range(500):
        T_new = T_field.copy()
        T_new[1:-1, 1:-1] = 0.25 * (T_field[:-2, 1:-1] + T_field[2:, 1:-1] +
                                      T_field[1:-1, :-2] + T_field[1:-1, 2:])
        # 边界恒温
        T_new[0, :] = 25; T_new[-1, :] = 25
        T_new[:, 0] = 25; T_new[:, -1] = 25
        T_field = T_new

    im = ax.imshow(T_field, cmap='hot', vmin=25, vmax=80, origin='lower')
    plt.colorbar(im, ax=ax, label='温度 (°C)')
    ax.set_title('2D温度场分布')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    plt.tight_layout()
    out = r'./nuedc-asset-library\15_simulation\thermal_management_simulation_result.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'已保存: {out}')

    R_nat = hs.total_thermal_resistance(100, air_velocity=0)
    R_for = hs.total_thermal_resistance(100, air_velocity=3)
    print(f'\n=== 热管理仿真统计 ===')
    print(f'散热器热阻(自然对流): {R_nat:.2f} °C/W')
    print(f'散热器热阻(3m/s风冷): {R_for:.2f} °C/W')
    print(f'液冷热阻: {lc.thermal_resistance(0.01):.4f} °C/W')
    print(f'液冷温升@50W: {lc.temperature_rise(50):.2f} °C')


if __name__ == '__main__':
    main()
