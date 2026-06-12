#!/usr/bin/env python3
"""
步进电机仿真 - 微步控制 + 转矩特性 + 共振分析
适用于电赛步进电机驱动方案设计与优化
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
rcParams['axes.unicode_minus'] = False


class StepperMotorSim:
    """步进电机仿真器"""

    def __init__(self, params=None):
        p = params or {}
        self.num_phases = p.get('num_phases', 2)          # 相数
        self.num_poles = p.get('num_poles', 50)           # 转子齿数
        self.step_angle = 360.0 / (self.num_poles * self.num_phases)  # 步距角
        self.R = p.get('R', 1.2)           # 相电阻 (Ω)
        self.L = p.get('L', 0.003)         # 相电感 (H)
        self.holding_torque = p.get('holding_torque', 0.5)  # 保持转矩 (N·m)
        self.detent_torque = p.get('detent_torque', 0.02)   # 齿槽转矩
        self.inertia = p.get('inertia', 1e-5)               # 转子惯量 (kg·m²)
        self.damping = p.get('damping', 1e-4)               # 阻尼系数
        self.max_current = p.get('max_current', 2.0)        # 最大相电流 (A)

    # ==================== 微步控制仿真 ====================
    def microstepping_sim(self, microstep=16, cycles=2):
        """
        微步控制电流波形仿真
        Args:
            microstep: 微步细分数 (1=整步, 2=半步, 4/8/16/32/64/128/256)
            cycles: 展示几个完整电周期
        """
        steps_per_cycle = 4 * microstep  # 一个电周期的步数
        total_steps = steps_per_cycle * cycles
        theta = np.linspace(0, 2 * np.pi * cycles, total_steps)

        # 两相正弦/余弦电流 (微步本质是正弦驱动)
        Ia = self.max_current * np.sin(theta)
        Ib = self.max_current * np.cos(theta)

        # 合成转矩 (假设线性转矩-电流关系)
        torque = self.holding_torque * np.sqrt(Ia**2 + Ib**2) / self.max_current

        # 整步参考
        theta_full = np.linspace(0, 2 * np.pi * cycles, 4 * cycles)
        Ia_full = self.max_current * np.sign(np.sin(theta_full))
        Ib_full = self.max_current * np.sign(np.cos(theta_full))

        fig, axes = plt.subplots(3, 1, figsize=(12, 9))
        fig.suptitle(f'步进电机微步控制仿真 ({microstep}微步, 步距角={self.step_angle/microstep:.3f}°)', fontsize=14)

        # 电流波形
        ax = axes[0]
        ax.plot(theta * 180 / np.pi, Ia, 'b-', label='A相电流(微步)', linewidth=1.5)
        ax.plot(theta * 180 / np.pi, Ib, 'r-', label='B相电流(微步)', linewidth=1.5)
        ax.step(theta_full * 180 / np.pi, Ia_full, 'b--', alpha=0.4, label='A相(整步)', where='mid')
        ax.step(theta_full * 180 / np.pi, Ib_full, 'r--', alpha=0.4, label='B相(整步)', where='mid')
        ax.set_xlabel('电角度 (°)')
        ax.set_ylabel('电流 (A)')
        ax.set_title('相电流波形')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

        # 合成矢量轨迹
        ax = axes[1]
        t_circle = np.linspace(0, 2*np.pi, 100)
        ax.plot(np.cos(t_circle), np.sin(t_circle), 'k--', alpha=0.3, label='理想圆')
        ax.plot(Ia / self.max_current, Ib / self.max_current, 'g-', linewidth=0.5, label=f'{microstep}微步轨迹')
        # 整步轨迹(正方形)
        sq = np.array([(-1,-1),(1,-1),(1,1),(-1,1),(-1,-1)])
        ax.plot(sq[:,0], sq[:,1], 'r-', alpha=0.4, label='整步轨迹')
        ax.set_xlabel('A相电流 (标幺)')
        ax.set_ylabel('B相电流 (标幺)')
        ax.set_title('电流矢量轨迹 (越圆越好)')
        ax.set_aspect('equal')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 转矩波动
        ax = axes[2]
        ax.plot(theta * 180 / np.pi, torque * 1000, 'm-', linewidth=1.5)
        ax.axhline(y=self.holding_torque * 1000, color='k', linestyle='--', alpha=0.5, label=f'额定转矩 {self.holding_torque*1000:.0f} mN·m')
        ax.set_xlabel('电角度 (°)')
        ax.set_ylabel('转矩 (mN·m)')
        ax.set_title(f'合成转矩 (微步细分后的转矩波动)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('stepper_microstepping.png', dpi=150, bbox_inches='tight')
        plt.show()

        # 输出关键指标
        torque_ripple = (torque.max() - torque.min()) / torque.mean() * 100
        print(f"=== 微步控制分析 ({microstep}细分) ===")
        print(f"  步距角: {self.step_angle/microstep:.4f}°")
        print(f"  转矩波动率: {torque_ripple:.2f}%")
        print(f"  电流矢量圆度(RMS偏差): {self._vector_roundness(Ia, Ib):.4f}")
        return torque_ripple

    def _vector_roundness(self, Ia, Ib):
        """计算电流矢量轨迹的圆度偏差"""
        r = np.sqrt(Ia**2 + Ib**2)
        max_val = r.max()
        if max_val == 0:
            return 1.0
        r_norm = r / max_val
        return np.std(r_norm)

    # ==================== 转矩-速度特性 ====================
    def torque_speed_curve(self, voltage=24.0, max_rps=10):
        """
        转矩-速度特性曲线仿真
        基于简化电气模型: V = IR + L*dI/dt + Ke*ω
        """
        speeds_rps = np.linspace(0.01, max_rps, 200)  # 转/秒
        speeds_rpm = speeds_rps * 60

        # 反电动势系数 (V/rad/s)
        Ke = voltage / (max_rps * 2 * np.pi * 1.5)  # 估算

        # 不同驱动电压
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle('步进电机转矩-速度特性', fontsize=14)

        ax = axes[0]
        for V in [12, 24, 36, 48]:
            omega = speeds_rps * 2 * np.pi
            # 可用电压 = V - Ke*ω (简化)
            V_avail = np.maximum(V - Ke * omega, 0)
            # 电流受限于电阻
            I = V_avail / self.R
            I = np.minimum(I, self.max_current)
            # 转矩 (简化: 按整步模式，转矩随速度下降)
            # 加入电感效应: 高频时电流建立不完全
            tau_elec = self.L / self.R  # 电气时间常数
            step_freq = speeds_rps * self.num_poles * self.num_phases
            current_factor = 1.0 / np.sqrt(1 + (2 * np.pi * step_freq * tau_elec)**2)
            T = self.holding_torque * (I / self.max_current) * current_factor

            ax.plot(speeds_rpm, T * 1000, label=f'{V}V', linewidth=2)

        ax.set_xlabel('转速 (RPM)')
        ax.set_ylabel('转矩 (mN·m)')
        ax.set_title('不同驱动电压的转矩-速度曲线')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, max_rps * 60)
        ax.set_ylim(0, None)

        # 拉入/拉出转矩对比
        ax = axes[1]
        V = 24.0
        omega = speeds_rps * 2 * np.pi
        tau_elec = self.L / self.R
        step_freq = speeds_rps * self.num_poles * self.num_phases

        # 拉出转矩 (Pull-out, 运行中不失步的最大转矩)
        current_factor = 1.0 / np.sqrt(1 + (2 * np.pi * step_freq * tau_elec)**2)
        T_pullout = self.holding_torque * current_factor

        # 拉入转矩 (Pull-in, 能启动并同步的转矩，比拉出低)
        # 启动还需克服惯量，简化为拉出的60-80%
        inertia_factor = 1 - 0.4 * (speeds_rps / max_rps)
        inertia_factor = np.clip(inertia_factor, 0.1, 1.0)
        T_pullin = T_pullout * inertia_factor * 0.75

        ax.fill_between(speeds_rpm, T_pullin * 1000, T_pullout * 1000,
                        alpha=0.2, color='blue', label='可运行区域')
        ax.plot(speeds_rpm, T_pullout * 1000, 'b-', linewidth=2, label='拉出转矩')
        ax.plot(speeds_rpm, T_pullin * 1000, 'r-', linewidth=2, label='拉入转矩(启动)')
        ax.set_xlabel('转速 (RPM)')
        ax.set_ylabel('转矩 (mN·m)')
        ax.set_title(f'{V}V 拉入/拉出转矩特性')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('stepper_torque_speed.png', dpi=150, bbox_inches='tight')
        plt.show()

    # ==================== 共振分析 ====================
    def resonance_analysis(self):
        """
        步进电机共振特性分析
        包括: 低频共振、中频共振、高频失步
        """
        freqs = np.logspace(0, 5, 1000)  # 1Hz ~ 100kHz (步进频率)

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('步进电机共振分析', fontsize=14)

        # 1) 转子角度响应 (开环频率响应)
        ax = axes[0, 0]
        # 简化二阶系统: 共振频率 ~ sqrt(K/J)
        # 步进电机的"刚度" K = dT/dθ ≈ holding_torque / (step_angle_rad)
        step_angle_rad = self.step_angle * np.pi / 180
        K = self.holding_torque / step_angle_rad
        fn = np.sqrt(K / self.inertia) / (2 * np.pi)  # 自然频率 Hz
        zeta = self.damping / (2 * np.sqrt(K * self.inertia))

        omega = 2 * np.pi * freqs
        omega_n = 2 * np.pi * fn
        # 二阶传递函数幅值
        H_mag = 1.0 / np.sqrt((1 - (omega/omega_n)**2)**2 + (2*zeta*omega/omega_n)**2)
        H_mag_db = 20 * np.log10(H_mag)

        ax.semilogx(freqs, H_mag_db, 'b-', linewidth=2)
        ax.axvline(x=fn, color='r', linestyle='--', alpha=0.7, label=f'共振频率 {fn:.0f} Hz')
        ax.axhline(y=-3, color='gray', linestyle=':', alpha=0.5)
        ax.set_xlabel('步进频率 (Hz)')
        ax.set_ylabel('响应 (dB)')
        ax.set_title('开环频率响应')
        ax.legend()
        ax.grid(True, which='both', alpha=0.3)

        # 2) 阻尼对共振的影响
        ax = axes[0, 1]
        for zeta_ratio in [0.05, 0.1, 0.2, 0.5, 1.0]:
            z = zeta * zeta_ratio / 0.05  # 归一化不同阻尼
            z = max(z, 0.01)
            H = 1.0 / np.sqrt((1 - (omega/omega_n)**2)**2 + (2*z*omega/omega_n)**2)
            ax.semilogx(freqs, 20*np.log10(H), linewidth=1.5, label=f'ζ={z:.3f}')
        ax.axvline(x=fn, color='r', linestyle='--', alpha=0.5)
        ax.set_xlabel('步进频率 (Hz)')
        ax.set_ylabel('响应 (dB)')
        ax.set_title('不同阻尼比的频率响应')
        ax.legend(fontsize=8)
        ax.grid(True, which='both', alpha=0.3)
        ax.set_ylim(-30, 40)

        # 3) 速度波动 vs 步进频率
        ax = axes[1, 0]
        # 模拟不同频率下的速度波动(半步模式)
        step_freqs = np.logspace(1, 4, 200)
        velocity_ripple = []
        for sf in step_freqs:
            T_period = 1.0 / sf
            omega_ratio = T_period / (1.0 / fn)  # 步进周期与共振周期之比
            # 共振附近波动最大
            ripple = 1.0 / np.sqrt((1 - omega_ratio**2)**2 + (2*zeta*omega_ratio)**2)
            velocity_ripple.append(ripple * 100)

        ax.semilogx(step_freqs, velocity_ripple, 'g-', linewidth=2)
        ax.axvline(x=fn, color='r', linestyle='--', alpha=0.7, label=f'共振区 ~{fn:.0f} Hz')
        # 标注几个常见问题区域
        ax.axvspan(fn * 0.7, fn * 1.5, alpha=0.1, color='red')
        ax.set_xlabel('步进频率 (Hz)')
        ax.set_ylabel('速度波动 (%)')
        ax.set_title('速度波动 vs 步进频率')
        ax.legend()
        ax.grid(True, which='both', alpha=0.3)

        # 4) 共振抑制方案对比
        ax = axes[1, 1]
        methods = ['无阻尼器', '机械阻尼器', '微步驱动(16)', '微步驱动(64)', '电流衰减优化']
        # 共振峰值抑制效果 (dB衰减)
        reductions = [0, -12, -18, -24, -8]
        colors = ['red', 'orange', 'green', 'blue', 'purple']

        bars = ax.bar(methods, [-r for r in reductions], color=colors, alpha=0.7)
        ax.set_ylabel('共振峰值抑制 (dB)')
        ax.set_title('各种共振抑制方案效果对比')
        ax.grid(True, alpha=0.3, axis='y')
        for bar, val in zip(bars, reductions):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f'{val}dB', ha='center', va='bottom', fontweight='bold')

        plt.tight_layout()
        plt.savefig('stepper_resonance.png', dpi=150, bbox_inches='tight')
        plt.show()

        print(f"\n=== 共振分析结果 ===")
        print(f"  转子刚度 K = {K:.2f} N·m/rad")
        print(f"  自然频率 fn = {fn:.1f} Hz")
        print(f"  阻尼比 ζ = {zeta:.4f}")
        print(f"  建议: 避开 {fn*0.7:.0f}~{fn*1.5:.0f} Hz 步进频率区间")
        return fn, zeta


def demo():
    """综合演示"""
    print("=" * 60)
    print("步进电机仿真系统 - nuedc-asset-library")
    print("=" * 60)

    motor = StepperMotorSim({
        'num_phases': 2,
        'num_poles': 50,
        'R': 1.2,
        'L': 0.003,
        'holding_torque': 0.5,
        'inertia': 1e-5,
        'damping': 1e-4,
        'max_current': 2.0,
    })

    print(f"\n电机参数: {motor.num_phases}相, {motor.num_poles}极, 步距角={motor.step_angle}°")

    # 1) 微步控制
    for ms in [1, 2, 16, 64]:
        motor.microstepping_sim(microstep=ms, cycles=1)

    # 2) 转矩-速度
    motor.torque_speed_curve(voltage=24, max_rps=10)

    # 3) 共振分析
    motor.resonance_analysis()

    print("\n仿真完成！图表已保存。")


if __name__ == '__main__':
    demo()
