#!/usr/bin/env python3
"""
电源管理仿真 (Power Management Simulation)
============================================
仿真内容:
  - DC-DC Buck/Boost变换器仿真
  - 多路输出电源系统
  - 效率模型 (开关损耗+导通损耗+磁芯损耗)
  - 热设计仿真 (结温+散热器+热阻网络)
  - 保护功能 (OVP/OCP/OTP/UVLO/软启动)
  - 线性稳压器(LDO)对比

依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Dict, Tuple
from enum import Enum


class Topology(Enum):
    BUCK = "buck"
    BOOST = "boost"
    BUCK_BOOST = "buck-boost"
    LDO = "ldo"


@dataclass
class PowerStageConfig:
    """功率级配置"""
    topology: Topology = Topology.BUCK
    v_in: float = 12.0            # 输入电压
    v_out: float = 3.3            # 输出电压
    i_out_max: float = 3.0        # 最大输出电流
    f_sw: float = 500e3           # 开关频率 500kHz
    l_value: float = 10e-6        # 电感 10μH
    c_out: float = 100e-6         # 输出电容 100μF
    esr_cap: float = 10e-3        # 电容ESR 10mΩ
    rds_on: float = 50e-3         # MOSFET导通电阻 50mΩ
    r_l: float = 30e-3            # 电感DCR 30mΩ
    v_f_diode: float = 0.3        # 肖特基二极管正向压降
    qg: float = 10e-9             # MOSFET栅极电荷 10nC
    v_gate: float = 5.0           # 栅极驱动电压
    deadtime: float = 20e-9       # 死区时间 20ns


@dataclass
class ThermalConfig:
    """热设计配置"""
    rth_jc: float = 2.0           # 结到壳热阻 °C/W
    rth_cs: float = 0.5           # 壳到散热器热阻 °C/W
    rth_sa: float = 10.0          # 散热器到环境热阻 °C/W
    rth_ja: float = 40.0          # 结到环境热阻 (无散热器)
    t_ambient: float = 40         # 环境温度 °C
    t_j_max: float = 150          # 最高结温 °C
    thermal_mass: float = 0.01    # 热容 J/°C


@dataclass
class ProtectionConfig:
    """保护配置"""
    ovp_threshold: float = 3.6    # 过压保护阈值
    ocp_threshold: float = 4.0    # 过流保护阈值
    otp_threshold: float = 130    # 过温保护阈值
    uvlo_rising: float = 4.0      # 欠压锁定上升阈值
    uvlo_falling: float = 3.5     # 欠压锁定下降阈值
    soft_start_ms: float = 5      # 软启动时间


class PowerManagementSimulation:
    """电源管理仿真引擎"""

    def __init__(self, power: PowerStageConfig = None,
                 thermal: ThermalConfig = None,
                 protection: ProtectionConfig = None):
        self.power = power or PowerStageConfig()
        self.thermal = thermal or ThermalConfig()
        self.protection = protection or ProtectionConfig()

    # ── 效率模型 ──────────────────────────────────────────
    def calculate_efficiency(self, i_out: float, v_in: float = None,
                             temperature: float = 25) -> dict:
        """计算变换器效率"""
        p = self.power
        v_in = v_in or p.v_in
        duty = p.v_out / v_in if p.topology == Topology.BUCK else v_in / p.v_out
        duty = np.clip(duty, 0.01, 0.99)

        # 导通损耗
        i_rms_inductor = i_out * np.sqrt(duty + (1 - duty) * (1 - duty) / 12)  # 纹波修正
        p_conduction_mosfet = i_rms_inductor ** 2 * p.rds_on * duty
        p_conduction_diode = i_out * p.v_f_diode * (1 - duty)
        p_conduction_inductor = i_rms_inductor ** 2 * p.r_l

        # 开关损耗
        t_sw = p.deadtime + p.qg / (p.v_gate / 10)  # 简化开关时间
        p_switching = 0.5 * v_in * i_out * t_sw * p.f_sw

        # 驱动损耗
        p_gate = p.qg * p.v_gate * p.f_sw

        # 电容ESR损耗
        i_ripple = (p.v_out * (1 - duty)) / (p.l_value * p.f_sw)
        i_cap_rms = i_ripple / np.sqrt(12)
        p_cap_esr = i_cap_rms ** 2 * p.esr_cap

        # 总损耗
        p_total_loss = (p_conduction_mosfet + p_conduction_diode +
                        p_conduction_inductor + p_switching + p_gate + p_cap_esr)
        p_output = p.v_out * i_out
        p_input = p_output + p_total_loss

        efficiency = p_output / p_input if p_input > 0 else 0

        return {
            "efficiency": efficiency,
            "p_input": p_input,
            "p_output": p_output,
            "p_loss_total": p_total_loss,
            "p_loss_mosfet_cond": p_conduction_mosfet,
            "p_loss_diode": p_conduction_diode,
            "p_loss_inductor": p_conduction_inductor,
            "p_loss_switching": p_switching,
            "p_loss_gate": p_gate,
            "p_loss_cap_esr": p_cap_esr,
            "duty_cycle": duty,
            "current": i_out,
            "temperature": temperature,
        }

    # ── 热设计仿真 ──────────────────────────────────────
    def thermal_analysis(self, power_dissipation: float,
                         duration_s: float = 10) -> dict:
        """热仿真 (集总参数RC热模型)"""
        th = self.thermal
        p = self.power

        # 热阻网络: 结→壳→散热器→环境
        rth_total = th.rth_jc + th.rth_cs + th.rth_sa

        # 稳态结温
        t_j_steady = th.t_ambient + power_dissipation * rth_total
        t_hs_steady = th.t_ambient + power_dissipation * th.rth_sa
        t_case_steady = th.t_ambient + power_dissipation * (th.rth_cs + th.rth_sa)

        # 瞬态仿真 (RC时间常数)
        dt = 0.01  # 10ms步长
        n_steps = int(duration_s / dt)
        t_axis = np.linspace(0, duration_s, n_steps)

        tau_j = th.thermal_mass * th.rth_jc  # 结时间常数
        tau_hs = 0.5 * th.rth_sa  # 散热器时间常数 (假设热容0.5J/°C)

        t_j = np.zeros(n_steps)
        t_case = np.zeros(n_steps)
        t_hs = np.zeros(n_steps)

        t_j[0] = th.t_ambient
        t_case[0] = th.t_ambient
        t_hs[0] = th.t_ambient

        for i in range(1, n_steps):
            # 散热器温升
            t_hs[i] = t_hs[i - 1] + (dt / tau_hs) * (
                th.t_ambient + power_dissipation * th.rth_sa - t_hs[i - 1])
            # 壳温
            t_case[i] = t_case[i - 1] + (dt / tau_j * 0.5) * (
                t_hs[i] + power_dissipation * th.rth_cs - t_case[i - 1])
            # 结温
            t_j[i] = t_j[i - 1] + (dt / tau_j) * (
                t_case[i] + power_dissipation * th.rth_jc - t_j[i - 1])

        return {
            "time": t_axis,
            "t_junction": t_j,
            "t_case": t_case,
            "t_heatsink": t_hs,
            "t_j_steady": t_j_steady,
            "t_hs_steady": t_hs_steady,
            "power": power_dissipation,
            "rth_total": rth_total,
            "thermal_margin": th.t_j_max - t_j_steady,
        }

    # ── Buck变换器时域仿真 ───────────────────────────────
    def buck_transient(self, i_load: float = 1.0, load_step: float = 2.0,
                       step_time_ms: float = 1, sim_time_ms: float = 5,
                       n_points: int = 10000) -> dict:
        """Buck变换器时域仿真 (简化状态空间平均模型)"""
        p = self.power
        dt = sim_time_ms * 1e-3 / n_points
        t = np.linspace(0, sim_time_ms * 1e-3, n_points)

        # 状态变量: 电感电流, 输出电压
        i_l = np.zeros(n_points)
        v_out = np.zeros(n_points)
        v_out[0] = p.v_out

        duty = p.v_out / p.v_in
        step_sample = int(step_time_ms * 1e-3 / dt)

        for k in range(1, n_points):
            # 负载阶跃
            if k >= step_sample:
                i_load_now = i_load + load_step
            else:
                i_load_now = i_load

            # 简单PI补偿器 (电压模式)
            v_error = p.v_out - v_out[k - 1]
            kp = 0.5
            ki = 1000
            # 积分项 (简化)
            duty_adj = duty + kp * v_error / p.v_in
            duty_adj = np.clip(duty_adj, 0.05, 0.95)

            # 电感电流 (V_L = L * di/dt)
            v_l = duty_adj * p.v_in - v_out[k - 1] - i_l[k - 1] * p.r_l
            i_l[k] = i_l[k - 1] + (v_l / p.l_value) * dt

            # 输出电压 (I_C = C * dv/dt)
            i_cap = i_l[k] - i_load_now - v_out[k - 1] / 100  # 100Ω假负载
            v_out[k] = v_out[k - 1] + (i_cap / p.c_out) * dt
            # ESR影响
            v_out[k] += (i_cap * p.esr_cap - (k > 0 and i_cap * p.esr_cap or 0))

        return {
            "time": t * 1000,  # ms
            "voltage": v_out,
            "current": i_l,
            "load_step_time": step_time_ms,
            "load_current_before": i_load,
            "load_current_after": i_load + load_step,
        }

    # ── 多路输出系统 ─────────────────────────────────────
    def multi_rail_system(self, rails: List[PowerStageConfig] = None) -> dict:
        """多路输出电源系统仿真"""
        if rails is None:
            rails = [
                PowerStageConfig(topology=Topology.BUCK, v_in=12, v_out=3.3, i_out_max=3),
                PowerStageConfig(topology=Topology.BUCK, v_in=12, v_out=1.8, i_out_max=2),
                PowerStageConfig(topology=Topology.BUCK, v_in=12, v_out=5.0, i_out_max=1.5),
                PowerStageConfig(topology=Topology.LDO, v_in=3.3, v_out=1.2, i_out_max=0.5),
            ]

        rail_names = ["3.3V", "1.8V", "5.0V", "1.2V(LDO)"]
        results = []

        for i, rail in enumerate(rails):
            sim = PowerManagementSimulation(rail)
            # 典型负载 (50%最大)
            eff = sim.calculate_efficiency(rail.i_out_max * 0.5)
            thermal = sim.thermal_analysis(eff["p_loss_total"])
            results.append({
                "name": rail_names[i] if i < len(rail_names) else f"Rail{i}",
                "config": rail,
                "efficiency": eff,
                "thermal": thermal,
            })

        return {
            "rails": results,
            "total_input_power": sum(r["efficiency"]["p_input"] for r in results),
            "total_output_power": sum(r["efficiency"]["p_output"] for r in results),
            "total_loss": sum(r["efficiency"]["p_loss_total"] for r in results),
            "system_efficiency": (sum(r["efficiency"]["p_output"] for r in results) /
                                  sum(r["efficiency"]["p_input"] for r in results)),
        }

    # ── 保护功能仿真 ─────────────────────────────────────
    def protection_simulation(self, sim_time_ms: float = 20,
                              n_points: int = 5000) -> dict:
        """保护功能综合仿真"""
        p = self.power
        prot = self.protection
        dt = sim_time_ms * 1e-3 / n_points
        t = np.linspace(0, sim_time_ms * 1e-3, n_points)

        v_out = np.full(n_points, p.v_out)
        i_out = np.full(n_points, 1.0)
        t_junction = np.full(n_points, 50.0)
        enable = np.ones(n_points, dtype=bool)
        fault_code = np.zeros(n_points, dtype=int)  # 0=正常

        # 模拟事件序列
        # t=2ms: 输出过载
        for k in range(n_points):
            t_ms = t[k] * 1000
            if 2 <= t_ms <= 5:
                i_out[k] = 5.0  # 过流
                t_junction[k] = 80 + (t_ms - 2) * 20
            elif 8 <= t_ms <= 12:
                v_out[k] = 4.0  # 过压
            elif 15 <= t_ms <= 18:
                t_junction[k] = 140 + (t_ms - 15) * 5  # 过温

        # 保护逻辑
        ss_samples = int(prot.soft_start_ms * 1e-3 / dt)
        for k in range(n_points):
            if k < ss_samples:
                # 软启动期间不限流
                continue
            # OCP
            if i_out[k] > prot.ocp_threshold:
                fault_code[k] = 1  # OCP
                enable[k] = False
            # OVP
            if v_out[k] > prot.ovp_threshold:
                fault_code[k] = 2  # OVP
            # OTP
            if t_junction[k] > prot.otp_threshold:
                fault_code[k] = 3  # OTP
                enable[k] = False

        # 应用保护: 被保护的输出拉低
        for k in range(n_points):
            if not enable[k]:
                # 看门狗恢复 (1ms后重试)
                recovery = int(1e-3 / dt)
                if k + recovery < n_points and fault_code[k + recovery] == 0:
                    enable[k + recovery] = True

        return {
            "time": t * 1000,
            "v_out": v_out,
            "i_out": i_out,
            "t_junction": t_junction,
            "enable": enable,
            "fault_code": fault_code,
        }


def run_demo():
    """运行完整仿真演示"""
    print("=" * 70)
    print("电源管理仿真")
    print("=" * 70)

    sim = PowerManagementSimulation()
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    # ── 1. 效率曲线 (不同输入电压) ───────────────────────
    print("\n[1] 效率曲线...")
    ax = axes[0, 0]
    load_range = np.linspace(0.01, 3.0, 50)
    for v_in in [5, 9, 12, 24]:
        efficiencies = []
        for i_load in load_range:
            cfg = PowerStageConfig(v_in=v_in)
            s = PowerManagementSimulation(cfg)
            eff = s.calculate_efficiency(i_load)
            efficiencies.append(eff["efficiency"] * 100)
        ax.plot(load_range, efficiencies, label=f'Vin={v_in}V')

    ax.set_xlabel('负载电流 (A)')
    ax.set_ylabel('效率 (%)')
    ax.set_title('Buck变换器效率曲线 (Vout=3.3V)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(70, 100)

    # ── 2. 损耗分布 (饼图) ──────────────────────────────
    print("[2] 损耗分布...")
    eff = sim.calculate_efficiency(2.0)
    labels = ['MOSFET导通', '二极管', '电感DCR', '开关损耗', '栅极驱动', '电容ESR']
    sizes = [eff["p_loss_mosfet_cond"], eff["p_loss_diode"],
             eff["p_loss_inductor"], eff["p_loss_switching"],
             eff["p_loss_gate"], eff["p_loss_cap_esr"]]
    colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99', '#ff66b3', '#c2c2f0']

    ax = axes[0, 1]
    wedges, texts, autotexts = ax.pie(sizes, labels=labels, autopct='%1.1f%%',
                                       colors=colors, startangle=90)
    ax.set_title(f'损耗分布 (Iout=2A, η={eff["efficiency"]*100:.1f}%)\n总损耗={eff["p_loss_total"]*1000:.1f}mW')

    # ── 3. 热仿真 ──────────────────────────────────────
    print("[3] 热仿真...")
    thermal = sim.thermal_analysis(eff["p_loss_total"], duration_s=30)

    ax = axes[0, 2]
    ax.plot(thermal["time"], thermal["t_junction"], 'r-', linewidth=2, label='结温')
    ax.plot(thermal["time"], thermal["t_case"], 'orange', linewidth=2, label='壳温')
    ax.plot(thermal["time"], thermal["t_heatsink"], 'g-', linewidth=2, label='散热器温度')
    ax.axhline(150, color='red', linestyle='--', alpha=0.5, label='Tj_max=150°C')
    ax.axhline(thermal["t_j_steady"], color='gray', linestyle=':', alpha=0.5,
               label=f'Tj稳态={thermal["t_j_steady"]:.1f}°C')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('温度 (°C)')
    ax.set_title(f'热仿真 (Ploss={eff["p_loss_total"]*1000:.1f}mW)\n'
                 f'热裕量={thermal["thermal_margin"]:.1f}°C')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── 4. Buck瞬态响应 ────────────────────────────────
    print("[4] Buck瞬态响应...")
    transient = sim.buck_transient(i_load=0.5, load_step=2.0, step_time_ms=1)

    ax = axes[1, 0]
    ax.plot(transient["time"], transient["voltage"], 'b-', linewidth=1)
    ax.axhline(3.3, color='r', linestyle='--', alpha=0.5, label='目标=3.3V')
    ax.axvline(1, color='gray', linestyle=':', alpha=0.5, label='负载阶跃')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('输出电压 (V)')
    ax.set_title(f'负载瞬态响应 (0.5A→2.5A)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax2 = ax.twinx()
    ax2.plot(transient["time"], transient["current"], 'g-', alpha=0.5, label='电感电流')
    ax2.set_ylabel('电感电流 (A)', color='green')

    # ── 5. 多路输出系统 ────────────────────────────────
    print("[5] 多路输出系统...")
    multi = sim.multi_rail_system()

    ax = axes[1, 1]
    rail_names = [r["name"] for r in multi["rails"]]
    efficiencies = [r["efficiency"]["efficiency"] * 100 for r in multi["rails"]]
    losses = [r["efficiency"]["p_loss_total"] * 1000 for r in multi["rails"]]
    powers = [r["efficiency"]["p_output"] for r in multi["rails"]]

    x = np.arange(len(rail_names))
    width = 0.35
    bars1 = ax.bar(x - width / 2, efficiencies, width, label='效率 (%)', color='steelblue')
    ax.set_ylabel('效率 (%)')
    ax.set_ylim(70, 100)

    ax2 = ax.twinx()
    bars2 = ax2.bar(x + width / 2, losses, width, label='损耗 (mW)', color='coral')
    ax2.set_ylabel('损耗 (mW)')

    ax.set_xticks(x)
    ax.set_xticklabels(rail_names)
    ax.set_title(f'多路输出系统 (总效率={multi["system_efficiency"]*100:.1f}%)')
    ax.legend(loc='upper left')
    ax2.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # ── 6. 保护功能仿真 ────────────────────────────────
    print("[6] 保护功能仿真...")
    prot_sim = sim.protection_simulation()

    ax = axes[1, 2]
    # 电压
    ax.plot(prot_sim["time"], prot_sim["v_out"], 'b-', label='Vout', linewidth=1.5)
    ax.axhline(sim.protection.ovp_threshold, color='r', linestyle='--', alpha=0.5, label='OVP阈值')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('电压 (V)')
    ax.set_ylim(0, 5)

    # 电流
    ax2 = ax.twinx()
    ax2.plot(prot_sim["time"], prot_sim["i_out"], 'g-', alpha=0.7, label='Iout')
    ax2.axhline(sim.protection.ocp_threshold, color='orange', linestyle='--', alpha=0.5, label='OCP阈值')
    ax2.set_ylabel('电流 (A)')

    # 标注故障
    for fc, label in [(1, 'OCP'), (2, 'OVP'), (3, 'OTP')]:
        mask = prot_sim["fault_code"] == fc
        if np.any(mask):
            ax.fill_between(prot_sim["time"], 0, 5, where=mask,
                           alpha=0.15, label=f'{label}故障')

    ax.set_title('保护功能综合仿真')
    ax.legend(fontsize=6, loc='upper left')
    ax2.legend(fontsize=6, loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('power_management_simulation.png', dpi=150, bbox_inches='tight')
    plt.show()

    print(f"\n仿真完成!")
    print(f"  Buck效率 (12V→3.3V, 2A): {eff['efficiency']*100:.1f}%")
    print(f"  总损耗: {eff['p_loss_total']*1000:.1f}mW")
    print(f"  热裕量: {thermal['thermal_margin']:.1f}°C")
    print(f"  多路系统效率: {multi['system_efficiency']*100:.1f}%")
    print("  图表已保存为 power_management_simulation.png")


if __name__ == "__main__":
    run_demo()
