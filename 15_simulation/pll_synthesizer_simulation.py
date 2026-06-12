"""
PLL频率合成器仿真 - PLL Synthesizer Simulation
================================================
仿真内容: 环路滤波器设计 / 锁定时间 / 杂散分析 / 相位噪声传递
适用场景: 频率合成器设计、时钟恢复、本振信号生成

电赛应用: 通信系统本振设计、信号发生器频率合成
"""

import numpy as np
from scipy import signal as sig
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class PLLModel:
    """PLL频率合成器行为级模型"""

    def __init__(self, f_ref, N_div, K_pd, K_vco, C1, C2, R1, order=2):
        """
        参数:
            f_ref: 参考频率 (Hz)
            N_div: 分频比 N
            K_pd: 鉴相器增益 (A/rad)
            K_vco: VCO增益 (Hz/V) — 注意这里是Hz/V不是rad/s/V
            C1: 环路滤波器主电容 (F)
            C2: 环路滤波器旁路电容 (F, order>=2时)
            R1: 环路滤波器电阻 (Ω)
            order: 环路滤波器阶数 (1或2)
        """
        self.f_ref = f_ref
        self.N = N_div
        self.K_pd = K_pd
        self.K_vco = K_vco
        self.C1 = C1
        self.C2 = C2
        self.R1 = R1
        self.order = order
        self.f_out = f_ref * N_div

        # 开环增益常数 K = K_pd * K_vco / N
        self.K = K_pd * 2 * np.pi * K_vco / N_div

    @property
    def loop_bandwidth(self):
        """估算环路带宽 (Hz)"""
        if self.order == 1:
            omega_c = np.sqrt(self.K / self.C1)
        else:
            # 二阶无源超前-滞后滤波器
            tau1 = self.R1 * self.C1 * self.C2 / (self.C1 + self.C2)
            tau2 = self.R1 * self.C1
            omega_c = np.sqrt(self.K / (tau2 * (self.C1 + self.C2)))
        return omega_c / (2 * np.pi)

    @property
    def phase_margin(self):
        """估算相位裕度 (度)"""
        omega_c = 2 * np.pi * self.loop_bandwidth
        if self.order == 1:
            return 90.0
        tau2 = self.R1 * self.C1
        pm = np.arctan(tau2 * omega_c)
        return np.degrees(pm)

    def open_loop_transfer(self, f):
        """
        开环传递函数 G(s)H(s)
        G(s) = K_pd * Z(s) * K_vco / s
        H(s) = 1/N
        Z(s)为环路滤波器阻抗
        """
        s = 2j * np.pi * f

        # 环路滤波器阻抗
        if self.order == 1:
            Z = 1.0 / (s * self.C1)
        else:
            # 无源超前-滞后: Z(s) = (1 + s*R1*C1) / (s*(C1+C2)*(1 + s*R1*C1*C2/(C1+C2)))
            tau2 = self.R1 * self.C1
            tau1 = self.R1 * self.C1 * self.C2 / (self.C1 + self.C2)
            Z = (1 + s * tau2) / (s * (self.C1 + self.C2) * (1 + s * tau1))

        # 开环传递函数
        G_open = self.K_pd * Z * (2 * np.pi * self.K_vco) / s / self.N
        return G_open

    def closed_loop_transfer(self, f):
        """闭环传递函数 H(s) = G(s) / (1 + G(s)*H_fb(s))"""
        G = self.open_loop_transfer(f)
        return G / (1 + G)

    def noise_transfer_vco(self, f):
        """VCO噪声传递函数 (1/(1+G*H)) — 高通"""
        G = self.open_loop_transfer(f)
        return 1.0 / (1 + G)

    def noise_transfer_ref(self, f):
        """参考噪声传递函数 G/(1+G*H) — 低通"""
        G = self.open_loop_transfer(f)
        return G / (1 + G)


class PLLSimulator:
    """PLL时域仿真器"""

    def __init__(self, pll: PLLModel, f_sim=None, dt=None):
        self.pll = pll
        if dt is None:
            self.dt = 1.0 / (pll.f_ref * 100)  # 100倍过采样
        else:
            self.dt = dt
        if f_sim is None:
            self.f_sim = pll.f_ref
        else:
            self.f_sim = f_sim

    def simulate_frequency_step(self, N_new, duration=None, initial_phase=0):
        """
        仿真频率跳变后的锁定过程

        参数:
            N_new: 新的分频比
            duration: 仿真时长 (s)
            initial_phase: 初始相位 (rad)
        """
        if duration is None:
            # 估计锁定时间 ≈ 5/环路带宽
            duration = 10.0 / max(self.pll.loop_bandwidth, 1)

        n_steps = int(duration / self.dt)
        t = np.arange(n_steps) * self.dt

        # 状态变量
        phase_error = np.zeros(n_steps)
        v_control = np.zeros(n_steps)
        freq_out = np.zeros(n_steps)
        phase_out = np.zeros(n_steps)

        # 环路滤波器状态
        x_filter = 0.0  # 滤波器积分状态
        x_filter2 = 0.0

        N_old = self.pll.N
        K_vco_rad = 2 * np.pi * self.pll.K_vco

        phase_vco = initial_phase
        phase_div = 0.0

        for i in range(1, n_steps):
            # 线性N过渡 (简化)
            if i < n_steps // 10:
                N_current = N_old + (N_new - N_old) * i / (n_steps // 10)
            else:
                N_current = N_new

            # 鉴相器 (简化为相位差)
            pe = phase_div - phase_vco
            # 归一化到 [-pi, pi]
            pe = (pe + np.pi) % (2 * np.pi) - np.pi
            phase_error[i] = pe

            # 电荷泵 + 鉴相器输出电流
            I_cp = self.pll.K_pd * pe

            # 环路滤波器 (梯形积分)
            if self.pll.order == 1:
                v_control[i] = v_control[i-1] + I_cp * self.dt / self.pll.C1
            else:
                tau2 = self.pll.R1 * self.pll.C1
                # 电流通过R1产生电压, 同时对C1/C2充电
                v_r = I_cp * self.pll.R1
                x_filter += (I_cp - x_filter2) * self.dt / self.pll.C1
                x_filter2 += (v_r / self.pll.R1) * self.dt / self.pll.C2  # 简化
                v_control[i] = x_filter + v_r * self.pll.C2 / (self.pll.C1 + self.pll.C2)

            # VCO
            freq_out[i] = self.pll.f_out + v_control[i] * K_vco_rad / (2 * np.pi)
            phase_vco += (2 * np.pi * freq_out[i]) * self.dt
            phase_out[i] = phase_vco

            # 分频器
            phase_div = phase_vco / N_current

        # 检测锁定
        locked = np.abs(phase_error[-1000:]) < 0.1  # 最后1000个点的相位误差
        lock_ratio = np.mean(locked)

        return {
            't': t,
            'phase_error': phase_error,
            'v_control': v_control,
            'freq_out': freq_out,
            'freq_deviation': freq_out - self.pll.f_out,
            'locked': lock_ratio > 0.95,
            'lock_ratio': lock_ratio,
        }


def compute_spur_levels(f_ref, f_offset_range, pll: PLLModel):
    """
    计算PLL参考杂散水平

    参考杂散出现在 f_out ± n*f_ref 处
    """
    # 参考杂散主要由电荷泵泄漏和失配引起
    # 简化模型: 杂散水平与环路滤波器在f_ref处的抑制有关
    f_test = np.array(f_offset_range)
    G = pll.open_loop_transfer(f_test)
    # 环路滤波器对参考频率的抑制
    suppression_db = 20 * np.log10(np.abs(G) + 1e-20)
    # 杂散 ≈ 电荷泵泄漏经过滤波器衰减
    spur_level = suppression_db - 20  # 假设-20dBc的CP泄漏
    return {
        'f_offset': f_test,
        'suppression_db': suppression_db,
        'spur_level_dbc': spur_level,
    }


def plot_pll_results():
    """综合绘图: PLL仿真结果"""
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))

    # PLL参数
    pll = PLLModel(
        f_ref=10e6, N_div=100, K_pd=5e-3, K_vco=50e6,
        C1=1e-9, C2=100e-12, R1=10e3, order=2
    )

    # 1. 开环传递函数 Bode图
    f = np.logspace(1, 8, 2000)
    G_open = pll.open_loop_transfer(f)
    mag = 20 * np.log10(np.abs(G_open))
    phase = np.angle(G_open, deg=True)

    axes[0, 0].semilogx(f, mag, 'b-', linewidth=2)
    axes[0, 0].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    axes[0, 0].axvline(x=pll.loop_bandwidth, color='r', linestyle='--',
                        label=f'环路BW={pll.loop_bandwidth:.0f}Hz')
    axes[0, 0].set_ylabel('幅度 (dB)')
    axes[0, 0].set_title(f'PLL开环传递函数 (N={pll.N})')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    ax00_twin = axes[0, 0].twinx()
    ax00_twin.semilogx(f, phase, 'g-', linewidth=2)
    ax00_twin.set_ylabel('相位 (°)')
    ax00_twin.axhline(y=-180, color='r', linestyle=':', alpha=0.5)

    # 2. 噪声传递函数
    H_ref = pll.noise_transfer_ref(f)
    H_vco = pll.noise_transfer_vco(f)
    axes[0, 1].semilogx(f, 20 * np.log10(np.abs(H_ref)), 'b-', linewidth=2, label='参考噪声(低通)')
    axes[0, 1].semilogx(f, 20 * np.log10(np.abs(H_vco)), 'r-', linewidth=2, label='VCO噪声(高通)')
    axes[0, 1].axvline(x=pll.loop_bandwidth, color='g', linestyle='--', alpha=0.5)
    axes[0, 1].set_xlabel('频率 (Hz)')
    axes[0, 1].set_ylabel('增益 (dB)')
    axes[0, 1].set_title('PLL噪声传递函数')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()

    # 3. 频率阶跃锁定仿真
    sim = PLLSimulator(pll)
    result = sim.simulate_frequency_step(N_new=110, duration=50e-6)

    axes[1, 0].plot(result['t'] * 1e6, result['phase_error'], 'b-', linewidth=1)
    axes[1, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[1, 0].set_xlabel('时间 (μs)')
    axes[1, 0].set_ylabel('相位误差 (rad)')
    axes[1, 0].set_title(f'频率阶跃响应 (N: 100→110, 锁定={result["locked"]})')
    axes[1, 0].grid(True, alpha=0.3)

    # 4. 控制电压
    axes[1, 1].plot(result['t'] * 1e6, result['v_control'] * 1e3, 'r-', linewidth=1)
    axes[1, 1].set_xlabel('时间 (μs)')
    axes[1, 1].set_ylabel('控制电压 (mV)')
    axes[1, 1].set_title('VCO控制电压')
    axes[1, 1].grid(True, alpha=0.3)

    # 5. 参考杂散分析
    spur = compute_spur_levels(10e6, np.logspace(3, 7, 500), pll)
    axes[2, 0].semilogx(spur['f_offset'], spur['suppression_db'], 'b-', linewidth=2)
    axes[2, 0].set_xlabel('偏移频率 (Hz)')
    axes[2, 0].set_ylabel('开环增益 (dB)')
    axes[2, 0].set_title('环路滤波器参考杂散抑制')
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].axvline(x=10e6, color='r', linestyle='--', label='f_ref=10MHz')
    axes[2, 0].legend()

    # 6. 不同环路带宽的锁定时间对比
    bandwidths = []
    lock_times = []
    for R1_val in [1e3, 5e3, 10e3, 50e3, 100e3]:
        pll_test = PLLModel(10e6, 100, 5e-3, 50e6, 1e-9, 100e-12, R1_val, 2)
        sim_test = PLLSimulator(pll_test)
        res = sim_test.simulate_frequency_step(110, duration=200e-6)
        bw = pll_test.loop_bandwidth
        # 找到相位误差首次小于0.1rad的时间
        lock_idx = np.where(np.abs(res['phase_error']) < 0.1)[0]
        lt = res['t'][lock_idx[0]] * 1e6 if len(lock_idx) > 0 else 200
        bandwidths.append(bw)
        lock_times.append(lt)

    axes[2, 1].plot(bandwidths, lock_times, 'bo-', linewidth=2, markersize=8)
    axes[2, 1].set_xlabel('环路带宽 (Hz)')
    axes[2, 1].set_ylabel('锁定时间 (μs)')
    axes[2, 1].set_title('锁定时间 vs 环路带宽')
    axes[2, 1].grid(True, alpha=0.3)
    axes[2, 1].set_xscale('log')

    plt.tight_layout()
    save_path = './nuedc-asset-library/15_simulation/pll_synthesizer_simulation_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[OK] 图表已保存: {save_path}")


def demo():
    """演示: PLL频率合成器仿真"""
    print("=" * 60)
    print("PLL频率合成器仿真 - PLL Synthesizer Simulation")
    print("=" * 60)

    # 1. 基本PLL参数
    pll = PLLModel(
        f_ref=10e6, N_div=100, K_pd=5e-3, K_vco=50e6,
        C1=1e-9, C2=100e-12, R1=10e3, order=2
    )
    print(f"\n[1] PLL参数:")
    print(f"  参考频率: {pll.f_ref / 1e6:.0f} MHz")
    print(f"  分频比 N: {pll.N}")
    print(f"  输出频率: {pll.f_out / 1e6:.0f} MHz")
    print(f"  环路带宽: {pll.loop_bandwidth:.0f} Hz")
    print(f"  相位裕度: {pll.phase_margin:.1f}°")

    # 2. 不同环路滤波器参数对比
    print("\n[2] 环路滤波器参数扫描:")
    for R1_val in [1e3, 5e3, 10e3, 50e3]:
        p = PLLModel(10e6, 100, 5e-3, 50e6, 1e-9, 100e-12, R1_val, 2)
        print(f"  R1={R1_val/1e3:.0f}kΩ → BW={p.loop_bandwidth:.0f}Hz, PM={p.phase_margin:.1f}°")

    # 3. 频率切换锁定仿真
    print("\n[3] 频率切换仿真:")
    sim = PLLSimulator(pll)
    for N_new in [105, 110, 120, 150]:
        result = sim.simulate_frequency_step(N_new, duration=100e-6)
        f_new = pll.f_ref * N_new / 1e6
        print(f"  N: 100→{N_new} (f_out: 1000→{f_new:.0f}MHz), 锁定={result['locked']}")

    # 4. 绘图
    print("\n[4] 生成图表...")
    try:
        plot_pll_results()
    except Exception as e:
        print(f"  绘图跳过: {e}")

    print("\n仿真完成!")


if __name__ == '__main__':
    demo()
