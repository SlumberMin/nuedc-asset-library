"""
输入整形器仿真 - 对比ZV/ZVD/ZVDD整形效果
Input Shaper Simulation (ZV, ZVD, ZVDD comparison)
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class InputShaperSimulation:
    """输入整形器仿真类"""

    def __init__(self, wn=10.0, zeta=0.05):
        """
        参数:
            wn: 自然频率 (rad/s)
            zeta: 阻尼比
        """
        self.wn = wn
        self.zeta = zeta
        self.wd = wn * np.sqrt(1 - zeta**2)

    def _time_delay(self, delay):
        """创建纯延迟的脉冲响应序列"""
        return delay

    def make_zv_shaper(self):
        """ZV整形器 (Zero Vibration)"""
        K = np.exp(-self.zeta * np.pi / np.sqrt(1 - self.zeta**2))
        T = np.pi / self.wd

        # ZV: 2个脉冲
        A1 = 1.0 / (1.0 + K)
        A2 = K / (1.0 + K)
        t1 = 0.0
        t2 = T

        amps = [A1, A2]
        times = [t1, t2]
        return amps, times, 'ZV'

    def make_zvd_shaper(self):
        """ZVD整形器 (Zero Vibration and Derivative)"""
        K = np.exp(-self.zeta * np.pi / np.sqrt(1 - self.zeta**2))
        T = np.pi / self.wd

        # ZVD: ZV与自身的卷积
        A1 = 1.0 / (1.0 + K)**2
        A2 = 2.0 * K / (1.0 + K)**2
        A3 = K**2 / (1.0 + K)**2
        t1 = 0.0
        t2 = T
        t3 = 2 * T

        amps = [A1, A2, A3]
        times = [t1, t2, t3]
        return amps, times, 'ZVD'

    def make_zvdd_shaper(self):
        """ZVDD整形器 (Zero Vibration, Derivative, and Double Derivative)"""
        K = np.exp(-self.zeta * np.pi / np.sqrt(1 - self.zeta**2))
        T = np.pi / self.wd

        # ZVDD: ZVD与ZV的卷积
        A1 = 1.0 / (1.0 + K)**3
        A2 = 3.0 * K / (1.0 + K)**3
        A3 = 3.0 * K**2 / (1.0 + K)**3
        A4 = K**3 / (1.0 + K)**3
        t1 = 0.0
        t2 = T
        t3 = 2 * T
        t4 = 3 * T

        amps = [A1, A2, A3, A4]
        times = [t1, t2, t3, t4]
        return amps, times, 'ZVDD'

    def make_ei_shaper(self):
        """EI整形器 (Extra Insensitive) - 鲁棒性更强"""
        K = np.exp(-self.zeta * np.pi / np.sqrt(1 - self.zeta**2))
        T = np.pi / self.wd

        # EI允许5%残余振动以换取更宽的频率容差
        A1 = 1.0 / (1.0 + K + K**2)
        A2 = K / (1.0 + K + K**2)
        A3 = K**2 / (1.0 + K + K**2)
        t1 = 0.0
        t2 = T
        t3 = 2 * T

        amps = [A1, A2, A3]
        times = [t1, t2, t3]
        return amps, times, 'EI(5%)'

    def apply_shaper(self, t, u_raw, amps, times):
        """将整形器应用到输入信号"""
        dt = t[1] - t[0]
        u_shaped = np.zeros_like(t)

        for amp, td in zip(amps, times):
            # 延迟并缩放
            delay_samples = int(round(td / dt))
            if delay_samples == 0:
                u_shaped += amp * u_raw
            else:
                u_shaped[delay_samples:] += amp * u_raw[:-delay_samples]

        return u_shaped

    def simulate_system(self, t, u):
        """仿真二阶振荡系统"""
        # 传递函数: wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
        sys_num = [self.wn**2]
        sys_den = [1, 2 * self.zeta * self.wn, self.wn**2]
        sys_tf = signal.TransferFunction(sys_num, sys_den)
        _, y, _ = signal.lsim(sys_tf, u, t)
        return y

    def run_comparison(self):
        """运行对比仿真"""
        t = np.linspace(0, 5, 5000)
        dt = t[1] - t[0]

        # S曲线参考输入 (避免加速度突变)
        step_time = 0.5
        u_ref = np.zeros_like(t)
        rise = (t >= step_time) & (t <= step_time + 0.2)
        u_ref[t >= step_time] = 1.0
        # 平滑上升
        ramp_idx = (t >= step_time) & (t <= step_time + 0.1)
        u_ref[ramp_idx] = (t[ramp_idx] - step_time) / 0.1

        # 各整形器
        shapers = [
            self.make_zv_shaper(),
            self.make_zvd_shaper(),
            self.make_zvdd_shaper(),
            self.make_ei_shaper(),
        ]

        # ========== 图1: 整形信号与系统响应 ==========
        fig, axes = plt.subplots(3, 2, figsize=(14, 12))

        # 无整形的响应
        y_no_shaper = self.simulate_system(t, u_ref)
        axes[0, 0].plot(t, u_ref, 'k-', label='原始输入', linewidth=1.5)
        axes[0, 0].set_title('原始阶跃输入')
        axes[0, 0].set_xlabel('时间 (s)')
        axes[0, 0].set_ylabel('幅值')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)

        colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12']

        # 整形后的输入信号
        for i, (amps, times, name) in enumerate(shapers):
            u_shaped = self.apply_shaper(t, u_ref, amps, times)
            axes[0, 1].plot(t, u_shaped, color=colors[i], label=name, linewidth=1.5)

        axes[0, 1].set_title('整形后的输入信号')
        axes[0, 1].set_xlabel('时间 (s)')
        axes[0, 1].set_ylabel('幅值')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        # 系统响应对比
        axes[1, 0].plot(t, y_no_shaper, 'k-', label='无整形', linewidth=2, alpha=0.7)
        for i, (amps, times, name) in enumerate(shapers):
            u_shaped = self.apply_shaper(t, u_ref, amps, times)
            y = self.simulate_system(t, u_shaped)
            axes[1, 0].plot(t, y, color=colors[i], label=name, linewidth=1.5)

        axes[1, 0].set_title('系统响应对比')
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('输出')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        # 残余振动分析
        axes[1, 1].plot(t, y_no_shaper, 'k-', label='无整形', linewidth=2, alpha=0.7)
        for i, (amps, times, name) in enumerate(shapers):
            u_shaped = self.apply_shaper(t, u_ref, amps, times)
            y = self.simulate_system(t, u_shaped)
            # 残余振动 (减去稳态)
            residual = y - 1.0
            residual[t < step_time + 0.5] = 0
            axes[1, 1].plot(t, residual, color=colors[i], label=name, linewidth=1.5)

        axes[1, 1].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
        axes[1, 1].set_title('残余振动对比')
        axes[1, 1].set_xlabel('时间 (s)')
        axes[1, 1].set_ylabel('残余振动')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].set_xlim([1, 5])

        # 不同阻尼比下的效果
        zetas = [0.01, 0.05, 0.1, 0.15]
        for z in zetas:
            sim = InputShaperSimulation(wn=self.wn, zeta=z)
            amps, times, _ = sim.make_zvd_shaper()
            u_shaped = sim.apply_shaper(t, u_ref, amps, times)
            y = sim.simulate_system(t, u_shaped)
            axes[2, 0].plot(t, y, label=f'ζ={z}', linewidth=1.5)

        axes[2, 0].plot(t, y_no_shaper, 'k--', label='无整形', linewidth=1, alpha=0.5)
        axes[2, 0].set_title('不同阻尼比下ZVD整形效果')
        axes[2, 0].set_xlabel('时间 (s)')
        axes[2, 0].set_ylabel('输出')
        axes[2, 0].legend()
        axes[2, 0].grid(True, alpha=0.3)

        # 频率敏感性分析 (鲁棒性)
        freqs = np.linspace(0.5 * self.wn, 1.5 * self.wn, 50)
        residual_amps = {'无整形': [], 'ZV': [], 'ZVD': [], 'ZVDD': [], 'EI(5%)': []}

        for freq in freqs:
            # 无整形
            sim_test = InputShaperSimulation(wn=freq, zeta=self.zeta)
            u_test = np.ones_like(t) * (t >= 0.5).astype(float)
            y_test = sim_test.simulate_system(t, u_test)
            # 测量最终振动幅值
            residual_amps['无整形'].append(
                np.max(np.abs(y_test[-500:] - y_test[-1])))

            for amps_func, name in [
                (self.make_zv_shaper, 'ZV'),
                (self.make_zvd_shaper, 'ZVD'),
                (self.make_zvdd_shaper, 'ZVDD'),
                (self.make_ei_shaper, 'EI(5%)'),
            ]:
                amps, times, _ = amps_func()
                u_shaped = self.apply_shaper(t, u_test, amps)
                y_test = sim_test.simulate_system(t, u_shaped)
                residual_amps[name].append(
                    np.max(np.abs(y_test[-500:] - y_test[-1])))

        for name, color in zip(
            ['无整形', 'ZV', 'ZVD', 'ZVDD', 'EI(5%)'],
            ['black', '#e74c3c', '#3498db', '#2ecc71', '#f39c12']
        ):
            axes[2, 1].plot(freqs / self.wn, residual_amps[name],
                           color=color, label=name, linewidth=1.5)

        axes[2, 1].set_title('频率敏感性分析 (残余振动 vs 频率比)')
        axes[2, 1].set_xlabel('频率比 (ω/ωn)')
        axes[2, 1].set_ylabel('残余振动幅值')
        axes[2, 1].legend()
        axes[2, 1].grid(True, alpha=0.3)
        axes[2, 1].axvline(x=1.0, color='gray', linestyle='--', alpha=0.5)

        plt.suptitle('输入整形器效果对比', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('input_shaper_result.png', dpi=150, bbox_inches='tight')
        plt.close('all')

        # 打印整形器参数
        self._print_shaper_params()

    def _print_shaper_params(self):
        """打印整形器参数"""
        print("=" * 60)
        print(f"系统参数: ωn={self.wn} rad/s, ζ={self.zeta}")
        print(f"振荡周期: {2*np.pi/self.wd:.3f} s")
        print("=" * 60)

        for func in [self.make_zv_shaper, self.make_zvd_shaper,
                     self.make_zvdd_shaper, self.make_ei_shaper]:
            amps, times, name = func()
            print(f"\n{name} 整形器:")
            print(f"  脉冲数量: {len(amps)}")
            for i, (a, t) in enumerate(zip(amps, times)):
                print(f"  脉冲{i+1}: 幅值={a:.4f}, 时刻={t:.4f} s")
            print(f"  整形器长度: {times[-1]:.4f} s "
                  f"({times[-1]/(2*np.pi/self.wd)*100:.1f}% 周期)")
        print("=" * 60)


if __name__ == '__main__':
    sim = InputShaperSimulation(wn=10.0, zeta=0.05)
    sim.run_comparison()
    print("仿真完成! 结果已保存。")
