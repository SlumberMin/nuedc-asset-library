#!/usr/bin/env python3
"""
PCB热仿真 - 铜箔宽度 + 散热过孔 + 热阻网络
适用于电赛PCB散热设计与铜箔载流能力评估
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib.patches import Rectangle, FancyBboxPatch
import matplotlib.patches as mpatches

rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
rcParams['axes.unicode_minus'] = False


class PCBThermalSim:
    """PCB热仿真器"""

    def __init__(self, params=None):
        p = params or {}
        self.copper_thickness = p.get('copper_thickness', 35e-6)  # 1oz = 35µm
        self.trace_width = p.get('trace_width', 1.0e-3)           # 线宽 1mm
        self.ambient_temp = p.get('ambient_temp', 25)              # 环境温度
        self.pcb_length = p.get('pcb_length', 50e-3)              # PCB尺寸
        self.pcb_width = p.get('pcb_width', 30e-3)
        self.pcb_thickness = p.get('pcb_thickness', 1.6e-3)
        self.conductivity_cu = 5.8e7    # 铜导热系数 W/(m·K) ~ 385
        self.k_cu = 385                 # 铜导热率
        self.k_fr4 = 0.3               # FR4导热率
        self.k_cu_in_plane = 385       # 面内
        self.convection_coeff = 10      # 自然对流换热系数 W/(m²·K)

    # ==================== IPC-2221 铜箔载流能力 ====================
    @staticmethod
    def ipc2221_current(trace_width_mm, thickness_um=35, temp_rise=10, external=True):
        """
        IPC-2221标准铜箔载流能力计算
        trace_width_mm: 线宽(mm)
        thickness_um: 铜厚(µm), 35=1oz, 70=2oz
        temp_rise: 允许温升(°C)
        external: 外层(True)还是内层(False)
        """
        # IPC-2221公式: I = k * ΔT^0.44 * A^0.725
        # A = 截面积 (mils²)
        width_mils = trace_width_mm / 0.0254
        thickness_mils = thickness_um / 25.4
        area = width_mils * thickness_mils  # mils²

        k = 0.048 if external else 0.024  # 外层/内层系数
        current = k * (temp_rise ** 0.44) * (area ** 0.725)
        return current

    @staticmethod
    def min_trace_width(current_a, thickness_um=35, temp_rise=10, external=True):
        """计算最小线宽"""
        k = 0.048 if external else 0.024
        thickness_mils = thickness_um / 25.4
        # I = k * ΔT^0.44 * (w * t)^0.725
        # w = (I / (k * ΔT^0.44))^(1/0.725) / t
        area = (current_a / (k * temp_rise ** 0.44)) ** (1 / 0.725)
        width_mils = area / thickness_mils
        return width_mils * 0.0254  # 返回mm

    # ==================== 散热过孔计算 ====================
    @staticmethod
    def via_thermal_resistance(drill_diameter_mm=0.3, pad_diameter_mm=0.6,
                                plating_thickness_um=25, num_vias=1):
        """
        散热过孔热阻计算
        单个过孔热阻: R_via = 1/(k_cu * π * d * t_plating / L)
        """
        k_cu = 385  # W/(m·K)
        d = drill_diameter_mm * 1e-3
        t = plating_thickness_um * 1e-6
        L = 1.6e-3  # PCB厚度 1.6mm

        # 单个过孔的截面积 (铜壁环形)
        A_via = np.pi * d * t  # 近似为圆柱壁

        # 热阻 R = L / (k * A)
        R_single = L / (k_cu * A_via)

        # 多个过孔并联
        R_total = R_single / num_vias
        return R_single, R_total

    # ==================== 铜箔温度升高仿真 ====================
    def trace_temperature_rise(self, current_a, trace_length_mm=None):
        """
        铜箔走线温升仿真
        考虑: 传导 + 对流 + 辐射
        """
        if trace_length_mm is None:
            trace_length_mm = 10

        w = self.trace_width
        t = self.copper_thickness
        L = trace_length_mm * 1e-3

        # 铜箔电阻
        rho_cu = 1.72e-8  # 铜电阻率
        R_trace = rho_cu * L / (w * t)

        # 功耗
        P = current_a**2 * R_trace

        # 散热面积 (上下两面)
        A_conv = 2 * w * L

        # 稳态温升 (简化: Q = h*A*ΔT)
        delta_T = P / (self.convection_coeff * A_conv) if A_conv > 0 else 0

        return delta_T, R_trace, P

    # ==================== 热阻网络仿真 ====================
    def thermal_resistance_network(self, power_dissipation_w, component_area_mm2=9,
                                     copper_pad_area_mm2=25, num_vias=4):
        """
        热阻网络模型: 芯片→焊盘→铜箔→PCB→环境
        """
        # 1) 芯片到焊盘 (R_junction-to-board)
        A_comp = component_area_mm2 * 1e-6
        R_jb = 1 / (self.convection_coeff * A_comp)  # 简化
        R_jb = min(R_jb, 50)  # 典型值限制

        # 2) 焊盘到铜皮 (通过过孔)
        _, R_via = self.via_thermal_resistance(num_vias=num_vias)

        # 3) 铜皮散热 (面内扩展)
        A_pad = copper_pad_area_mm2 * 1e-6
        R_copper = 1 / (self.convection_coeff * A_pad)

        # 4) FR4基板热阻
        A_pcb = self.pcb_length * self.pcb_width
        R_fr4 = self.pcb_thickness / (self.k_fr4 * A_pcb)

        # 5) PCB到环境 (对流)
        A_total = 2 * A_pcb  # 两面散热
        R_env = 1 / (self.convection_coeff * A_total)

        # 总热阻
        # 并联路径: 铜皮直接对流 || 过孔→背面散热
        R_parallel = 1 / (1/R_copper + 1/(R_via + R_env))

        # 串联: R_junction_to_board + R_parallel
        R_total = R_jb + R_parallel

        # 温升
        delta_T = power_dissipation_w * R_total
        T_junction = self.ambient_temp + delta_T

        return {
            'R_jb': R_jb,
            'R_via': R_via,
            'R_copper': R_copper,
            'R_fr4': R_fr4,
            'R_env': R_env,
            'R_total': R_total,
            'delta_T': delta_T,
            'T_junction': T_junction,
            'components': ['芯片→焊盘', '散热过孔', '铜皮散热', 'FR4基板', 'PCB→环境'],
            'R_values': [R_jb, R_via, R_copper, R_fr4, R_env]
        }

    # ==================== 综合绘图 ====================
    def plot_all(self):
        fig, axes = plt.subplots(2, 3, figsize=(18, 11))
        fig.suptitle('PCB热仿真 - 铜箔载流+散热过孔+热阻网络', fontsize=14)

        # 1) 铜箔载流能力 vs 线宽 (不同铜厚)
        ax = axes[0, 0]
        widths = np.linspace(0.2, 5, 100)  # mm
        for thickness, label in [(18, '0.5oz(18µm)'), (35, '1oz(35µm)'),
                                  (70, '2oz(70µm)'), (105, '3oz(105µm)')]:
            for dt in [10, 20]:
                currents = [self.ipc2221_current(w, thickness, dt) for w in widths]
                ls = '-' if dt == 10 else '--'
                ax.plot(widths, currents, ls, linewidth=1.5,
                        label=f'{label}, ΔT={dt}°C' if dt == 10 else '')
                if dt == 20:
                    ax.plot(widths, currents, ls, linewidth=1, alpha=0.4)

        ax.set_xlabel('线宽 (mm)')
        ax.set_ylabel('载流能力 (A)')
        ax.set_title('IPC-2221铜箔载流能力')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, 5)

        # 2) 最小线宽 vs 电流
        ax = axes[0, 1]
        currents = np.linspace(0.1, 10, 100)
        for thickness, label in [(35, '1oz'), (70, '2oz')]:
            for dt in [10, 20]:
                widths_min = [self.min_trace_width(I, thickness, dt) for I in currents]
                ax.plot(currents, widths_min, linewidth=2, label=f'{label}, ΔT={dt}°C')

        ax.set_xlabel('电流 (A)')
        ax.set_ylabel('最小线宽 (mm)')
        ax.set_title('所需最小线宽 vs 电流')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 3) 散热过孔热阻 vs 过孔数量
        ax = axes[0, 2]
        via_counts = np.arange(1, 21)
        for drill_d in [0.2, 0.3, 0.4]:
            R_singles = []
            R_totals = []
            for n in via_counts:
                rs, rt = self.via_thermal_resistance(drill_diameter_mm=drill_d, num_vias=n)
                R_singles.append(rs)
                R_totals.append(rt)
            ax.plot(via_counts, R_totals, 'o-', linewidth=2, markersize=4,
                    label=f'Ø{drill_d}mm过孔')

        ax.set_xlabel('过孔数量')
        ax.set_ylabel('热阻 (°C/W)')
        ax.set_title('散热过孔热阻 vs 数量')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 4) 热阻网络饼图
        ax = axes[1, 0]
        result = self.thermal_resistance_network(2.0, num_vias=4)
        R_vals = result['R_values']
        R_total = sum(R_vals)
        labels = [f"{c}\n{v:.1f}°C/W\n({v/R_total*100:.0f}%)"
                  for c, v in zip(result['components'], R_vals)]
        colors = plt.cm.RdYlBu_r(np.linspace(0.2, 0.8, len(R_vals)))
        wedges, texts = ax.pie(R_vals, labels=None, colors=colors, startangle=90,
                                wedgeprops=dict(width=0.5, edgecolor='white'))
        ax.legend(wedges, [f"{c}: {v:.1f}°C/W" for c, v in zip(result['components'], R_vals)],
                  loc='center left', bbox_to_anchor=(1, 0.5), fontsize=8)
        ax.set_title(f'热阻分布 (Rtotal={R_total:.1f}°C/W)')

        # 5) 温升 vs 散热过孔数量
        ax = axes[1, 1]
        via_counts = np.arange(0, 21)
        for power in [0.5, 1.0, 2.0, 3.0]:
            temps = []
            for n in via_counts:
                if n == 0:
                    r = self.thermal_resistance_network(power, num_vias=1)
                    temps.append(r['T_junction'])
                else:
                    r = self.thermal_resistance_network(power, num_vias=n)
                    temps.append(r['T_junction'])
            ax.plot(via_counts, temps, 'o-', linewidth=2, markersize=4, label=f'{power}W')

        ax.axhline(y=85, color='red', linestyle='--', linewidth=2, alpha=0.7, label='Tj限制(85°C)')
        ax.axhline(y=125, color='darkred', linestyle=':', linewidth=2, alpha=0.7, label='Tj限制(125°C)')
        ax.set_xlabel('散热过孔数量')
        ax.set_ylabel('结温 (°C)')
        ax.set_title(f'结温 vs 散热过孔数 (Ta={self.ambient_temp}°C)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 6) 铜皮温度分布热力图
        ax = axes[1, 2]
        nx, ny = 50, 30
        x = np.linspace(0, 50, nx)
        y = np.linspace(0, 30, ny)
        X, Y = np.meshgrid(x, y)

        # 模拟热源在中心
        heat_x, heat_y = 25, 15
        r = np.sqrt((X - heat_x)**2 + (Y - heat_y)**2)
        # 温度分布: T = T_ambient + P/(2π*k*t) * ln(r0/r)
        T = self.ambient_temp + 30 / (1 + r/3)
        # 加一个第二个热源
        T += 20 / (1 + np.sqrt((X-40)**2 + (Y-10)**2)/2)

        im = ax.contourf(X, Y, T, levels=20, cmap='hot')
        plt.colorbar(im, ax=ax, label='温度 (°C)')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_title('PCB表面温度分布')
        # 标注热源
        ax.plot(heat_x, heat_y, 'w*', markersize=15)
        ax.plot(40, 10, 'w*', markersize=15)

        plt.tight_layout()
        plt.savefig('pcb_thermal.png', dpi=150, bbox_inches='tight')
        plt.show()


def demo():
    print("=" * 60)
    print("PCB热仿真系统 - 铜箔载流+散热设计")
    print("=" * 60)

    sim = PCBThermalSim()

    # 载流能力示例
    print("\n=== 铜箔载流能力 (IPC-2221, ΔT=10°C, 外层1oz) ===")
    for w in [0.25, 0.5, 1.0, 2.0, 3.0]:
        I = sim.ipc2221_current(w, 35, 10)
        print(f"  线宽 {w:.2f}mm → 载流 {I:.2f}A")

    # 最小线宽
    print("\n=== 最小线宽 (1oz, ΔT=10°C) ===")
    for I in [0.5, 1.0, 2.0, 3.0, 5.0]:
        w = sim.min_trace_width(I, 35, 10)
        print(f"  电流 {I:.1f}A → 最小线宽 {w:.2f}mm")

    # 散热过孔
    print("\n=== 散热过孔热阻 (Ø0.3mm, 25µm镀铜) ===")
    for n in [1, 4, 9, 16]:
        rs, rt = sim.via_thermal_resistance(num_vias=n)
        print(f"  {n:2d}个过孔: 单个 {rs:.1f}°C/W, 并联 {rt:.1f}°C/W")

    # 热阻网络
    print("\n=== 热阻网络分析 (2W功耗, 4个过孔) ===")
    r = sim.thermal_resistance_network(2.0, num_vias=4)
    for comp, val in zip(r['components'], r['R_values']):
        print(f"  {comp}: {val:.1f}°C/W")
    print(f"  总热阻: {r['R_total']:.1f}°C/W")
    print(f"  温升: {r['delta_T']:.1f}°C, 结温: {r['T_junction']:.1f}°C")

    sim.plot_all()
    print("\n仿真完成！")


if __name__ == '__main__':
    demo()
