#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机动力学仿真 - 电气+机械+热模型+效率MAP
==========================================
功能：
  - 直流电机/永磁同步电机(PMSM)数学模型
  - 电气子系统（电枢电路、反电动势）
  - 机械子系统（转矩平衡、摩擦、负载）
  - 热模型（铜损、铁损、温升）
  - 效率MAP图绘制
  - 启动/调速/制动过程仿真

适用场景：电机选型、驱动器设计、控制策略验证
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from dataclasses import dataclass
from typing import Tuple, Optional
import warnings
warnings.filterwarnings('ignore')

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ======================== 电机参数 ========================

@dataclass
class DCMotorParams:
    """直流电机参数"""
    # 电气参数
    Ra: float = 1.0       # 电枢电阻 (Ω)
    La: float = 0.005     # 电枢电感 (H)
    Ke: float = 0.05      # 反电动势常数 (V/(rad/s))
    Kt: float = 0.05      # 转矩常数 (N·m/A)
    # 机械参数
    J: float = 0.01       # 转动惯量 (kg·m²)
    B: float = 0.001      # 粘性摩擦系数 (N·m·s/rad)
    Tfriction: float = 0.05  # 库仑摩擦 (N·m)
    # 热参数
    Rth_motor: float = 2.0   # 电机热阻 (°C/W)
    Rth_ambient: float = 5.0 # 到环境热阻 (°C/W)
    Cth: float = 100.0       # 热容 (J/°C)
    T_ambient: float = 25.0  # 环境温度
    # 限制
    V_max: float = 48.0   # 最大电压 (V)
    I_max: float = 20.0   # 最大电流 (A)
    omega_max: float = 3000 * 2*np.pi/60  # 最大转速 (rad/s)


@dataclass
class PMSMParams:
    """永磁同步电机参数"""
    Rs: float = 0.5       # 定子电阻 (Ω)
    Ld: float = 0.005     # d轴电感 (H)
    Lq: float = 0.008     # q轴电感 (H)
    psi_f: float = 0.1    # 永磁体磁链 (Wb)
    p: int = 4            # 极对数
    J: float = 0.005      # 转动惯量 (kg·m²)
    B: float = 0.0005     # 粘性摩擦系数
    T_ambient: float = 25.0


# ======================== 直流电机仿真器 ========================

class DCMotorSimulator:
    """直流电机完整仿真"""

    def __init__(self, params: DCMotorParams = None):
        self.p = params or DCMotorParams()

    def simulate(self, V_input: np.ndarray, T_load: np.ndarray,
                 dt: float = 1e-4, t_end: float = 2.0) -> dict:
        """
        仿真直流电机动态响应
        V_input: 输入电压序列 (V)
        T_load: 负载转矩序列 (N·m)
        """
        n_steps = int(t_end / dt)
        t = np.arange(n_steps) * dt

        # 插值输入到仿真步长
        t_input = np.linspace(0, t_end, len(V_input))
        t_load = np.linspace(0, t_end, len(T_load))
        V = np.interp(t, t_input, V_input)
        T_l = np.interp(t, t_load, T_load)

        # 状态变量
        i_a = np.zeros(n_steps)    # 电枢电流
        omega = np.zeros(n_steps)  # 角速度
        theta = np.zeros(n_steps)  # 角位置
        T_motor = np.zeros(n_steps)  # 电磁转矩
        P_elec = np.zeros(n_steps)   # 电功率
        P_mech = np.zeros(n_steps)   # 机械功率
        P_loss = np.zeros(n_steps)   # 损耗功率
        T_winding = np.zeros(n_steps)  # 绕组温度

        p = self.p
        T_winding[0] = p.T_ambient

        for k in range(n_steps - 1):
            # 反电动势
            e_bemf = p.Ke * omega[k]
            # 电磁转矩
            T_motor[k] = p.Kt * i_a[k]
            # 摩擦转矩
            T_fric = p.B * omega[k] + p.Tfriction * np.sign(omega[k]) if abs(omega[k]) > 0.01 else p.Tfriction * np.sign(V[k])
            # 电气方程: L*di/dt = V - R*i - Ke*omega
            di_dt = (V[k] - p.Ra * i_a[k] - e_bemf) / p.La
            # 机械方程: J*dω/dt = Kt*i - B*omega - T_load - T_fric
            domega_dt = (T_motor[k] - T_l[k] - T_fric) / p.J

            # 欧拉积分
            i_a[k+1] = i_a[k] + di_dt * dt
            omega[k+1] = omega[k] + domega_dt * dt
            theta[k+1] = theta[k] + omega[k] * dt

            # 限幅
            i_a[k+1] = np.clip(i_a[k+1], -p.I_max, p.I_max)
            omega[k+1] = np.clip(omega[k+1], -p.omega_max, p.omega_max)

            # 功率
            P_elec[k] = V[k] * i_a[k]
            P_mech[k] = T_motor[k] * omega[k]
            P_cu = i_a[k]**2 * p.Ra
            P_loss[k] = P_cu

            # 热模型: 简单一阶RC
            dT_dt = (P_cu - (T_winding[k] - p.T_ambient) / p.Rth_motor) / p.Cth
            T_winding[k+1] = T_winding[k] + dT_dt * dt

        # 最后一步
        T_motor[-1] = p.Kt * i_a[-1]
        P_elec[-1] = V[-1] * i_a[-1]
        P_mech[-1] = T_motor[-1] * omega[-1]

        rpm = omega * 60 / (2 * np.pi)
        efficiency = np.where(np.abs(P_elec) > 0.01,
                              np.abs(P_mech) / np.abs(P_elec) * 100, 0)
        efficiency = np.clip(efficiency, 0, 100)

        return {
            't': t, 'V': V, 'i_a': i_a, 'omega': omega, 'rpm': rpm,
            'theta': theta, 'T_motor': T_motor, 'T_load': T_l,
            'P_elec': P_elec, 'P_mech': P_mech, 'P_loss': P_loss,
            'efficiency': efficiency, 'T_winding': T_winding
        }

    def compute_efficiency_map(self, n_speed=50, n_torque=50) -> dict:
        """计算效率MAP图"""
        p = self.p
        speeds = np.linspace(0, p.omega_max, n_speed)
        torques = np.linspace(0, p.Kt * p.I_max, n_torque)
        SS, TT = np.meshgrid(speeds, torques)

        P_mech = TT * SS
        I_required = TT / p.Kt
        P_copper = I_required**2 * p.Ra
        P_iron = 0.01 * SS**2  # 简化铁损模型
        P_total_loss = P_copper + P_iron
        P_input = P_mech + P_total_loss

        eff = np.where(P_input > 0.1, P_mech / P_input * 100, 0)
        eff = np.clip(eff, 0, 100)

        # 电流限制线
        T_max_line = p.Kt * p.I_max * np.ones_like(speeds)
        # 功率限制线
        V_emf = p.Ke * speeds
        I_max_at_speed = (p.V_max - V_emf) / p.Ra
        I_max_at_speed = np.clip(I_max_at_speed, 0, p.I_max)
        T_max_power = p.Kt * I_max_at_speed
        T_envelope = np.minimum(T_max_line, T_max_power)

        return {
            'speeds_rpm': speeds * 60 / (2*np.pi),
            'torques': torques,
            'efficiency': eff,
            'SS_rpm': SS * 60 / (2*np.pi),
            'TT': TT,
            'T_envelope': T_envelope,
            'speeds_rpm_env': speeds * 60 / (2*np.pi)
        }


# ======================== PMSM仿真器 ========================

class PMSMSimulator:
    """永磁同步电机仿真（dq模型）"""

    def __init__(self, params: PMSMParams = None):
        self.p = params or PMSMParams()

    def id_controller(self, id_ref: float, id_meas: float,
                      integral: float, dt: float,
                      Kp: float = 10.0, Ki: float = 1000.0) -> Tuple[float, float]:
        """d轴电流PI控制器"""
        error = id_ref - id_meas
        integral += error * dt
        output = Kp * error + Ki * integral
        return output, integral

    def simulate_foc(self, omega_ref: float, T_load: float,
                     dt: float = 1e-5, t_end: float = 0.5) -> dict:
        """FOC控制下的PMSM仿真"""
        p = self.p
        n_steps = int(t_end / dt)
        t = np.arange(n_steps) * dt

        # 状态
        id = np.zeros(n_steps)
        iq = np.zeros(n_steps)
        omega = np.zeros(n_steps)
        theta_e = np.zeros(n_steps)
        T_e = np.zeros(n_steps)

        # 控制器状态
        id_int, iq_int = 0.0, 0.0
        speed_int = 0.0

        id_ref = 0.0  # MTPA: id=0

        for k in range(n_steps - 1):
            # 转速环PI
            speed_err = omega_ref - omega[k]
            speed_int += speed_err * dt
            iq_ref = 5.0 * speed_err + 50.0 * speed_int
            iq_ref = np.clip(iq_ref, -20, 20)

            # 电流环PI
            vd, id_int = self.id_controller(id_ref, id[k], id_int, dt)
            vq_err = iq_ref - iq[k]
            iq_int += vq_err * dt
            vq = 10.0 * vq_err + 1000.0 * iq_int

            # PMSM电压方程 (dq)
            did_dt = (vd - p.Rs * id[k] + p.p * omega[k] * p.Lq * iq[k]) / p.Ld
            diq_dt = (vq - p.Rs * iq[k] - p.p * omega[k] * (p.Ld * id[k] + p.psi_f)) / p.Lq

            # 电磁转矩
            T_e[k] = 1.5 * p.p * (p.psi_f * iq[k] + (p.Ld - p.Lq) * id[k] * iq[k])

            # 机械方程
            domega_dt = (T_e[k] - T_load - p.B * omega[k]) / p.J

            # 积分
            id[k+1] = id[k] + did_dt * dt
            iq[k+1] = iq[k] + diq_dt * dt
            omega[k+1] = omega[k] + domega_dt * dt
            theta_e[k+1] = theta_e[k] + p.p * omega[k] * dt

        rpm = omega * 60 / (2*np.pi)

        return {
            't': t, 'id': id, 'iq': iq, 'omega': omega, 'rpm': rpm,
            'theta_e': theta_e, 'T_e': T_e
        }


# ======================== 可视化 ========================

def run_dc_motor_sim():
    """直流电机完整仿真"""
    print("=" * 60)
    print("电机动力学仿真系统")
    print("=" * 60)

    params = DCMotorParams()
    sim = DCMotorSimulator(params)

    # 场景1：阶跃电压启动
    print("\n[1] 直流电机启动仿真...")
    n_input = 200
    V_step = np.concatenate([np.zeros(50), np.full(150, 24.0)])
    T_load_step = np.concatenate([np.zeros(100), np.full(100, 0.1)])
    result1 = sim.simulate(V_step, T_load_step, dt=1e-4, t_end=2.0)

    # 场景2：调速过程
    print("[2] 调速过程仿真...")
    V_ramp = np.concatenate([np.full(50, 24.0), np.linspace(24, 40, 50),
                              np.full(50, 40.0), np.linspace(40, 20, 50)])
    T_const = np.full(200, 0.05)
    result2 = sim.simulate(V_ramp, T_const, dt=1e-4, t_end=2.0)

    # 场景3：效率MAP
    print("[3] 计算效率MAP...")
    eff_map = sim.compute_efficiency_map(n_speed=60, n_torque=60)

    # PMSM仿真
    print("[4] PMSM FOC控制仿真...")
    pmsm_sim = PMSMSimulator()
    pmsm_result = pmsm_sim.simulate_foc(
        omega_ref=500*2*np.pi/60, T_load=0.3, dt=1e-5, t_end=0.3)

    # ======================== 绘图 ========================
    fig, axes = plt.subplots(3, 3, figsize=(18, 15))
    fig.suptitle('电机动力学仿真系统', fontsize=16, fontweight='bold')

    t1 = result1['t']

    # (1) 启动-转速
    ax = axes[0, 0]
    ax.plot(t1*1000, result1['rpm'], 'b-', linewidth=1.5, label='转速')
    ax2 = ax.twinx()
    ax2.plot(t1*1000, result1['V'], 'r--', alpha=0.5, label='电压')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('转速 (RPM)', color='b')
    ax2.set_ylabel('电压 (V)', color='r')
    ax.set_title('启动过程-转速响应')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left')
    ax2.legend(loc='upper right')

    # (2) 启动-电流
    ax = axes[0, 1]
    ax.plot(t1*1000, result1['i_a'], 'r-', linewidth=1)
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('电枢电流 (A)')
    ax.set_title('启动过程-电枢电流')
    ax.grid(True, alpha=0.3)

    # (3) 启动-转矩
    ax = axes[0, 2]
    ax.plot(t1*1000, result1['T_motor']*1000, 'g-', linewidth=1.5, label='电磁转矩')
    ax.plot(t1*1000, result1['T_load']*1000, 'r--', linewidth=1, label='负载转矩')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('转矩 (mN·m)')
    ax.set_title('转矩对比')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (4) 调速-转速
    ax = axes[1, 0]
    t2 = result2['t']
    ax.plot(t2*1000, result2['rpm'], 'b-', linewidth=1.5)
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('转速 (RPM)')
    ax.set_title('调速过程')
    ax.grid(True, alpha=0.3)

    # (5) 功率分析
    ax = axes[1, 1]
    ax.plot(t1*1000, result1['P_elec'], 'r-', linewidth=1, label='电功率')
    ax.plot(t1*1000, result1['P_mech'], 'b-', linewidth=1, label='机械功率')
    ax.plot(t1*1000, result1['P_loss'], 'g-', linewidth=1, label='铜损')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('功率 (W)')
    ax.set_title('功率分析')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (6) 温升
    ax = axes[1, 2]
    ax.plot(t1*1000, result1['T_winding'], 'r-', linewidth=1.5)
    ax.axhline(y=80, color='k', linestyle='--', alpha=0.3, label='限值80°C')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('绕组温度 (°C)')
    ax.set_title('绕组温升')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (7) 效率MAP
    ax = axes[2, 0]
    c = ax.contourf(eff_map['SS_rpm'], eff_map['TT'], eff_map['efficiency'],
                    levels=20, cmap='RdYlGn')
    fig.colorbar(c, ax=ax, label='效率 (%)')
    ax.plot(eff_map['speeds_rpm_env'], eff_map['T_envelope']*1000, 'k-', linewidth=2, label='工作包络')
    ax.set_xlabel('转速 (RPM)')
    ax.set_ylabel('转矩 (mN·m)')
    ax.set_title('效率MAP图')
    ax.legend()

    # (8) PMSM转速响应
    ax = axes[2, 1]
    t_pmsm = pmsm_result['t'] * 1000
    ax.plot(t_pmsm, pmsm_result['rpm'], 'b-', linewidth=1)
    ax.axhline(y=500, color='r', linestyle='--', alpha=0.5, label='参考500RPM')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('转速 (RPM)')
    ax.set_title('PMSM FOC转速响应')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (9) PMSM dq电流
    ax = axes[2, 2]
    ax.plot(t_pmsm, pmsm_result['id'], 'r-', linewidth=1, label='id')
    ax.plot(t_pmsm, pmsm_result['iq'], 'b-', linewidth=1, label='iq')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('电流 (A)')
    ax.set_title('PMSM dq轴电流')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = './nuedc-asset-library/15_simulation/motor_dynamics_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n图表已保存: {save_path}")

    # 打印关键指标
    rpm_final = result1['rpm'][-1]
    eff_avg = np.mean(result1['efficiency'][result1['efficiency'] > 5])
    T_w_max = np.max(result1['T_winding'])
    print(f"\n关键指标:")
    print(f"  稳态转速: {rpm_final:.1f} RPM")
    print(f"  平均效率: {eff_avg:.1f}%")
    print(f"  最高绕组温度: {T_w_max:.1f}°C")

    plt.show()
    print("\n仿真完成！")


if __name__ == '__main__':
    run_dc_motor_sim()
