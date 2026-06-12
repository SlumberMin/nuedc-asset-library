"""
扰动观测器(DOB)仿真 - 对比有无DOB的抗扰性能
Disturbance Observer Simulation
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class DOBSimulation:
    """扰动观测器仿真类"""

    def __init__(self, J=0.01, b=0.1, Kt=1.0):
        """
        参数:
            J: 转动惯量
            b: 粘性摩擦系数
            Kt: 电机力矩常数
        """
        self.J = J
        self.b = b
        self.Kt = Kt

        # 实际被控对象: G(s) = Kt / (Js + b)
        self.plant_num = [Kt]
        self.plant_den = [J, b]

        # 标称模型 (与实际一致的理想情况)
        self.nominal_num = [Kt]
        self.nominal_den = [J, b]

    def make_dob_controller(self, Q_order=3, Q_cutoff=50):
        """
        构建DOB控制器
        Q(s): 低通滤波器, Q_cutoff: 截止频率(rad/s)
        使用n阶Butterworth低通滤波器
        """
        # Q滤波器 (低通)
        Q_butter = signal.butter(Q_order, Q_cutoff, btype='low', analog=True)
        self.Q_num = Q_butter[0]
        self.Q_den = Q_butter[1]

        # G_n^-1(s) = (Js + b) / Kt (标称模型的逆)
        self.Gn_inv_num = [self.J, self.b]
        self.Gn_inv_den = [self.Kt]

    def simulate_without_dob(self, t, ref, disturbance):
        """无DOB的闭环系统仿真 (仅PI控制器)"""
        # PI控制器
        Kp, Ki = 2.0, 10.0
        C_num = [Kp, Ki]
        C_den = [1, 0]

        # 开环: C(s)*G(s)
        ol_num = np.polymul(C_num, self.plant_num)
        ol_den = np.polymul(C_num, self.plant_den)

        # 闭环传递函数: CG/(1+CG)
        cl_num = ol_num
        cl_den = np.polyadd(ol_den, ol_num)

        # 干扰到输出: G/(1+CG)
        dist_num = self.plant_num
        dist_den = cl_den

        # 参考响应
        sys_ref = signal.TransferFunction(cl_num, cl_den)
        _, y_ref, _ = signal.lsim(sys_ref, ref, t)

        # 干扰响应
        sys_dist = signal.TransferFunction(dist_num, dist_den)
        _, y_dist, _ = signal.lsim(sys_dist, disturbance, t)

        return y_ref + y_dist

    def simulate_with_dob(self, t, ref, disturbance):
        """有DOB的闭环系统仿真"""
        # PI控制器
        Kp, Ki = 2.0, 10.0
        C_num = [Kp, Ki]
        C_den = [1, 0]

        # 构建DOB等效扰动抑制传递函数
        # 有DOB时, 干扰到输出: G*(1-Q)/(1+CG)
        # Q≈1时干扰被完全抑制

        # 分子: G_n^-1 * Q * G
        dob_feedback_num = np.polymul(
            np.polymul(self.Gn_inv_num, self.Q_num), self.plant_num)
        dob_feedback_den = np.polymul(
            np.polymul(self.Gn_inv_den, self.Q_den), self.plant_den)

        # 1 + G_n^-1*Q*G (DOB内环分母)
        dob_loop_den = np.polyadd(dob_feedback_den, dob_feedback_num)

        # 简化: 用等效传递函数计算
        # 参考到输出 (同无DOB): CG/(1+CG)
        ol_num = np.polymul(C_num, self.plant_num)
        ol_den = np.polymul(C_num, self.plant_den)
        cl_num = ol_num
        cl_den = np.polyadd(ol_den, ol_num)

        sys_ref = signal.TransferFunction(cl_num, cl_den)
        _, y_ref, _ = signal.lsim(sys_ref, ref, t)

        # 干扰到输出: G*(1-Q)/(1+CG)
        # 1-Q的分子和分母
        one_minus_Q_num = np.polyadd(self.Q_den, [-x for x in self.Q_num])
        one_minus_Q_den = self.Q_den

        dist_num_full = np.polymul(self.plant_num, one_minus_Q_num)
        dist_den_full = np.polymul(cl_den, one_minus_Q_den)

        sys_dist = signal.TransferFunction(dist_num_full, dist_den_full)
        _, y_dist, _ = signal.lsim(sys_dist, disturbance, t)

        return y_ref + y_dist

    def run_comparison(self):
        """运行对比仿真"""
        # 时间设置
        t = np.linspace(0, 2, 2000)

        # 阶跃参考信号
        ref = np.ones_like(t)

        # 阶跃扰动 (在t=0.5s时加入)
        disturbance = np.zeros_like(t)
        disturbance[t >= 0.5] = 0.5

        # 阶跃扰动 (在t=1.0s时移除, 方向相反)
        disturbance[t >= 1.0] = 0.0

        # 正弦扰动
        sinusoidal_dist = 0.2 * np.sin(2 * np.pi * 5 * t) * (t >= 0.5).astype(float)

        # 构建DOB
        Q_cutoffs = [20, 50, 100]

        # ========== 图1: 阶跃扰动对比 ==========
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # 阶跃扰动
        y_no_dob_step = self.simulate_without_dob(t, ref, disturbance)
        y_with_dob_step = self.simulate_with_dob(t, ref, disturbance)

        axes[0, 0].plot(t, ref, 'k--', label='参考信号', linewidth=1.5)
        axes[0, 0].plot(t, y_no_dob_step, 'r-', label='无DOB', linewidth=1.5)
        axes[0, 0].plot(t, y_with_dob_step, 'b-', label='有DOB', linewidth=1.5)
        axes[0, 0].axvspan(0.5, 1.0, alpha=0.1, color='red', label='扰动区间')
        axes[0, 0].set_title('阶跃扰动下的响应对比')
        axes[0, 0].set_xlabel('时间 (s)')
        axes[0, 0].set_ylabel('输出')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)

        # 正弦扰动
        y_no_dob_sine = self.simulate_without_dob(t, ref, sinusoidal_dist)
        y_with_dob_sine = self.simulate_with_dob(t, ref, sinusoidal_dist)

        axes[0, 1].plot(t, ref, 'k--', label='参考信号', linewidth=1.5)
        axes[0, 1].plot(t, y_no_dob_sine, 'r-', label='无DOB', linewidth=1.5)
        axes[0, 1].plot(t, y_with_dob_sine, 'b-', label='有DOB', linewidth=1.5)
        axes[0, 1].set_title('正弦扰动(5Hz)下的响应对比')
        axes[0, 1].set_xlabel('时间 (s)')
        axes[0, 1].set_ylabel('输出')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        # 不同Q滤波器截止频率对比
        colors = ['green', 'blue', 'red']
        for i, qc in enumerate(Q_cutoffs):
            self.make_dob_controller(Q_cutoff=qc)
            y = self.simulate_with_dob(t, ref, sinusoidal_dist)
            axes[1, 0].plot(t, y, color=colors[i],
                           label=f'Q截止频率={qc} rad/s', linewidth=1.5)

        axes[1, 0].plot(t, ref, 'k--', label='参考', linewidth=1)
        axes[1, 0].set_title('不同Q滤波器截止频率效果')
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('输出')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        # Q滤波器Bode图
        self.make_dob_controller(Q_cutoff=50)
        w = np.logspace(0, 4, 1000)
        w_q, mag_q, phase_q = signal.bode(
            signal.TransferFunction(self.Q_num, self.Q_den), w)

        ax_bode1 = axes[1, 1]
        ax_bode2 = ax_bode1.twinx()
        ax_bode1.semilogx(w_q, mag_q, 'b-', linewidth=2, label='|Q(jω)|')
        ax_bode2.semilogx(w_q, phase_q, 'r--', linewidth=2, label='∠Q(jω)')
        ax_bode1.set_xlabel('频率 (rad/s)')
        ax_bode1.set_ylabel('幅值 (dB)', color='b')
        ax_bode2.set_ylabel('相位 (°)', color='r')
        ax_bode1.set_title('Q滤波器Bode图')
        ax_bode1.grid(True, alpha=0.3)
        ax_bode1.legend(loc='upper left')
        ax_bode2.legend(loc='upper right')

        plt.suptitle('扰动观测器(DOB)性能分析', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('dob_simulation_result.png', dpi=150, bbox_inches='tight')
        plt.close('all')

        # ========== 图2: 参数鲁棒性 ==========
        fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))

        # 模型参数失配时的DOB效果
        param_variations = [0.5, 0.75, 1.0, 1.25, 1.5]
        for ratio in param_variations:
            self.J = 0.01 * ratio
            self.b = 0.1 * ratio
            self.make_dob_controller(Q_cutoff=50)
            y = self.simulate_with_dob(t, ref, sinusoidal_dist)
            axes2[0].plot(t, y, label=f'J={0.01*ratio:.3f}', linewidth=1.5)

        self.J = 0.01
        self.b = 0.1
        axes2[0].plot(t, ref, 'k--', label='参考', linewidth=1)
        axes2[0].set_title('模型参数变化时DOB效果')
        axes2[0].set_xlabel('时间 (s)')
        axes2[0].set_ylabel('输出')
        axes2[0].legend(fontsize=8)
        axes2[0].grid(True, alpha=0.3)

        # 扰动抑制比 vs 频率
        self.make_dob_controller(Q_cutoff=50)
        w = np.logspace(0, 3, 500)
        # 无DOB的灵敏度函数 S = 1/(1+CG)
        ol_num = np.polymul([2.0, 10.0], self.plant_num)
        ol_den = np.polymul([2.0, 10.0], self.plant_den)
        S_num = ol_den
        S_den = np.polyadd(ol_den, ol_num)

        sys_S = signal.TransferFunction(S_num, S_den)
        w_s, mag_s, _ = signal.bode(sys_S, w)

        # 有DOB的灵敏度函数 S_dob = (1-Q)/(1+CG)
        one_m_Q_num = np.polyadd(self.Q_den, [-x for x in self.Q_num])
        S_dob_num = np.polymul(ol_den, one_m_Q_num)
        S_dob_den = np.polymul(np.polyadd(ol_den, ol_num), self.Q_den)

        sys_S_dob = signal.TransferFunction(S_dob_num, S_dob_den)
        w_sd, mag_sd, _ = signal.bode(sys_S_dob, w)

        axes2[1].semilogx(w_s, mag_s, 'r-', label='无DOB (灵敏度)', linewidth=2)
        axes2[1].semilogx(w_sd, mag_sd, 'b-', label='有DOB (灵敏度)', linewidth=2)
        axes2[1].set_title('灵敏度函数对比 (扰动抑制)')
        axes2[1].set_xlabel('频率 (rad/s)')
        axes2[1].set_ylabel('幅值 (dB)')
        axes2[1].legend()
        axes2[1].grid(True, alpha=0.3)

        plt.suptitle('DOB鲁棒性与灵敏度分析', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('dob_robustness_result.png', dpi=150, bbox_inches='tight')
        plt.close('all')

        # 打印性能指标
        self._print_metrics(t, ref, disturbance, sinusoidal_dist)

    def _print_metrics(self, t, ref, step_dist, sine_dist):
        """打印性能指标"""
        self.make_dob_controller(Q_cutoff=50)
        self.J, self.b = 0.01, 0.1

        y_no = self.simulate_without_dob(t, ref, step_dist)
        y_yes = self.simulate_with_dob(t, ref, step_dist)

        # 阶跃扰动恢复时间
        idx_dist = np.where(t >= 0.5)[0]
        err_no = np.abs(y_no[idx_dist] - ref[idx_dist])
        err_yes = np.abs(y_yes[idx_dist] - ref[idx_dist])

        print("=" * 60)
        print("扰动观测器(DOB)性能指标")
        print("=" * 60)
        print(f"阶跃扰动下最大偏差:")
        print(f"  无DOB: {np.max(err_no):.4f}")
        print(f"  有DOB: {np.max(err_yes):.4f}")
        print(f"  改善比: {np.max(err_no)/np.max(err_yes):.1f}x")
        print(f"\n阶跃扰动下稳态误差:")
        # 扰动期间稳态
        ss_start = np.where(t[idx_dist] >= 0.8)[0]
        if len(ss_start) > 0:
            ss_idx = idx_dist[ss_start[0]:ss_start[0]+100]
            print(f"  无DOB: {np.mean(np.abs(y_no[ss_idx] - ref[ss_idx])):.4f}")
            print(f"  有DOB: {np.mean(np.abs(y_yes[ss_idx] - ref[ss_idx])):.4f}")
        print("=" * 60)


if __name__ == '__main__':
    sim = DOBSimulation(J=0.01, b=0.1, Kt=1.0)
    sim.run_comparison()
    print("仿真完成! 结果已保存。")
