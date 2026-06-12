"""
多变量控制仿真 - 解耦控制 + MIMO系统
Multi-Variable Control Simulation (Decoupling + MIMO)
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class MultiVariableSimulation:
    """多变量控制仿真 (2x2 MIMO系统)"""

    def __init__(self):
        # 2x2 MIMO传递函数矩阵
        # Y1 = G11*U1 + G12*U2
        # Y2 = G21*U1 + G22*U2
        # 典型: 双水箱耦合系统
        self.G = {
            (0, 0): {'num': [2.0], 'den': [1.0, 3.0, 2.0]},   # G11
            (0, 1): {'num': [0.5], 'den': [1.0, 2.0, 1.0]},   # G12 (耦合)
            (1, 0): {'num': [0.3], 'den': [1.0, 2.5, 1.5]},   # G21 (耦合)
            (1, 1): {'num': [1.5], 'den': [1.0, 2.0, 1.0]},   # G22
        }

    def simulate_mimo(self, t, u1, u2, G=None):
        """仿真2x2 MIMO系统"""
        if G is None:
            G = self.G

        y1 = np.zeros_like(t)
        y2 = np.zeros_like(t)

        for (i, j), gf in G.items():
            sys = signal.TransferFunction(gf['num'], gf['den'])
            if j == 0:
                _, y_part, _ = signal.lsim(sys, u1, t)
            else:
                _, y_part, _ = signal.lsim(sys, u2, t)

            if i == 0:
                y1 += y_part
            else:
                y2 += y_part

        return y1, y2

    def compute_static_decoupler(self):
        """
        静态解耦矩阵 D = G(0)^-1
        """
        # G(0): s=0时的增益矩阵
        G0 = np.zeros((2, 2))
        for (i, j), gf in self.G.items():
            G0[i, j] = np.polyval(gf['num'], 0) / np.polyval(gf['den'], 0)

        print(f"静态增益矩阵 G(0):\n{G0}")

        # 求逆
        D = np.linalg.inv(G0)
        print(f"静态解耦矩阵 D:\n{D}")
        return D, G0

    def compute_dynamic_decoupler(self, s_val=1.0):
        """
        动态解耦 (在特定频率点)
        D(s) = G(s)^-1
        """
        Gs = np.zeros((2, 2), dtype=complex)
        for (i, j), gf in self.G.items():
            Gs[i, j] = np.polyval(gf['num'], 1j * s_val) / np.polyval(gf['den'], 1j * s_val)

        Ds = np.linalg.inv(Gs)
        return Ds, Gs

    def apply_decoupler_and_simulate(self, t, ref1, ref2, D):
        """应用静态解耦器并仿真PID闭环"""
        dt = t[1] - t[0]
        n = len(t)

        # PID参数
        Kp = np.array([3.0, 3.0])
        Ki = np.array([2.0, 2.0])
        Kd = np.array([0.5, 0.5])

        y1 = np.zeros(n)
        y2 = np.zeros(n)
        e1_int = 0
        e2_int = 0

        # 离散化各传递函数
        G_disc = {}
        G_states = {}
        for key, gf in self.G.items():
            sys_d = signal.cont2discrete((gf['num'], gf['den']), dt, method='zoh')
            G_disc[key] = sys_d
            G_states[key] = np.zeros(sys_d[0].shape[0])

        u_decoupled = np.zeros(2)
        u_actual = np.zeros(2)

        for i in range(1, n):
            e1 = ref1[i] - y1[i-1]
            e2 = ref2[i] - y2[i-1]
            e1_int += e1 * dt
            e2_int += e2 * dt
            de1 = (e1 - (ref1[i-1] - y1[max(0, i-2)])) / dt if i > 1 else 0
            de2 = (e2 - (ref2[i-1] - y2[max(0, i-2)])) / dt if i > 1 else 0

            # PID
            u_pid = np.array([
                Kp[0]*e1 + Ki[0]*e1_int + Kd[0]*de1,
                Kp[1]*e2 + Ki[1]*e2_int + Kd[1]*de2
            ])

            # 解耦
            u_actual = D @ u_pid
            u_actual = np.clip(u_actual, -10, 10)

            # 系统响应
            for (r, c), sys_d in G_disc.items():
                A, B, C, D_mat, _ = sys_d
                G_states[(r, c)] = A @ G_states[(r, c)] + B.flatten() * u_actual[c]
                y_part = (C @ G_states[(r, c)] + D_mat.flatten() * u_actual[c])[0]
                if r == 0:
                    y1[i] += y_part
                else:
                    y2[i] += y_part

        return y1, y2

    def simulate_no_decoupling(self, t, ref1, ref2):
        """无解耦的独立PID控制"""
        dt = t[1] - t[0]
        n = len(t)

        Kp = np.array([3.0, 3.0])
        Ki = np.array([2.0, 2.0])
        Kd = np.array([0.5, 0.5])

        y1 = np.zeros(n)
        y2 = np.zeros(n)
        e1_int = 0
        e2_int = 0

        G_disc = {}
        G_states = {}
        for key, gf in self.G.items():
            sys_d = signal.cont2discrete((gf['num'], gf['den']), dt, method='zoh')
            G_disc[key] = sys_d
            G_states[key] = np.zeros(sys_d[0].shape[0])

        for i in range(1, n):
            e1 = ref1[i] - y1[i-1]
            e2 = ref2[i] - y2[i-1]
            e1_int += e1 * dt
            e2_int += e2 * dt
            de1 = (e1 - (ref1[i-1] - y1[max(0, i-2)])) / dt if i > 1 else 0
            de2 = (e2 - (ref2[i-1] - y2[max(0, i-2)])) / dt if i > 1 else 0

            u1 = Kp[0]*e1 + Ki[0]*e1_int + Kd[0]*de1
            u2 = Kp[1]*e2 + Ki[1]*e2_int + Kd[1]*de2
            u1 = np.clip(u1, -10, 10)
            u2 = np.clip(u2, -10, 10)

            for (r, c), sys_d in G_disc.items():
                A, B, C, D_mat, _ = sys_d
                u_c = u1 if c == 0 else u2
                G_states[(r, c)] = A @ G_states[(r, c)] + B.flatten() * u_c
                y_part = (C @ G_states[(r, c)] + D_mat.flatten() * u_c)[0]
                if r == 0:
                    y1[i] += y_part
                else:
                    y2[i] += y_part

        return y1, y2

    def run_comparison(self):
        """运行对比仿真"""
        t = np.linspace(0, 15, 3000)

        # 参考信号: 通道1先变化, 通道2后变化
        ref1 = np.ones_like(t)
        ref2 = np.zeros_like(t)
        ref2[t >= 5] = 1.0

        # 静态解耦
        D, G0 = self.compute_static_decoupler()

        # 动态解耦分析
        s_vals = np.logspace(-1, 1, 20)
        coupling_ratios = []
        for s in s_vals:
            _, Gs = self.compute_dynamic_decoupler(s)
            # 耦合度: 非对角元素/对角元素
            c1 = abs(Gs[0, 1]) / abs(Gs[0, 0])
            c2 = abs(Gs[1, 0]) / abs(Gs[1, 1])
            coupling_ratios.append((c1 + c2) / 2)

        # 仿真
        y1_no_dec, y2_no_dec = self.simulate_no_decoupling(t, ref1, ref2)
        y1_dec, y2_dec = self.apply_decoupler_and_simulate(t, ref1, ref2, D)

        # ========== 绘图 ==========
        fig, axes = plt.subplots(2, 3, figsize=(16, 10))

        # 通道1输出
        axes[0, 0].plot(t, ref1, 'k--', label='参考', linewidth=1.5)
        axes[0, 0].plot(t, y1_no_dec, 'r-', label='无解耦', linewidth=1.5)
        axes[0, 0].plot(t, y1_dec, 'b-', label='有解耦', linewidth=1.5)
        axes[0, 0].set_title('通道1输出响应')
        axes[0, 0].set_xlabel('时间 (s)')
        axes[0, 0].set_ylabel('y1')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)

        # 通道2输出
        axes[0, 1].plot(t, ref2, 'k--', label='参考', linewidth=1.5)
        axes[0, 1].plot(t, y2_no_dec, 'r-', label='无解耦', linewidth=1.5)
        axes[0, 1].plot(t, y2_dec, 'b-', label='有解耦', linewidth=1.5)
        axes[0, 1].set_title('通道2输出响应')
        axes[0, 1].set_xlabel('时间 (s)')
        axes[0, 1].set_ylabel('y2')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        # 交叉耦合影响
        # 当仅ref1变化时, y2应为0
        ref1_only = np.ones_like(t)
        ref2_zero = np.zeros_like(t)
        y1_only_no, y2_only_no = self.simulate_no_decoupling(t, ref1_only, ref2_zero)
        y1_only_dec, y2_only_dec = self.apply_decoupler_and_simulate(
            t, ref1_only, ref2_zero, D)

        axes[0, 2].plot(t, y2_only_no, 'r-', label='无解耦(y2泄漏)', linewidth=1.5)
        axes[0, 2].plot(t, y2_only_dec, 'b-', label='有解耦(y2泄漏)', linewidth=1.5)
        axes[0, 2].set_title('通道1激励时通道2的耦合泄漏')
        axes[0, 2].set_xlabel('时间 (s)')
        axes[0, 2].set_ylabel('y2 (应为0)')
        axes[0, 2].legend()
        axes[0, 2].grid(True, alpha=0.3)

        # RGA (相对增益矩阵)
        G0_mat = np.zeros((2, 2))
        for (i, j), gf in self.G.items():
            G0_mat[i, j] = np.polyval(gf['num'], 0) / np.polyval(gf['den'], 0)

        Lambda = G0_mat * np.linalg.inv(G0_mat).T
        axes[1, 0].imshow(Lambda, cmap='RdYlGn', vmin=-1, vmax=2)
        for i in range(2):
            for j in range(2):
                axes[1, 0].text(j, i, f'{Lambda[i,j]:.3f}',
                              ha='center', va='center', fontsize=14)
        axes[1, 0].set_title(f'相对增益矩阵(RGA)\n配对建议: {self._rga_pairing(Lambda)}')
        axes[1, 0].set_xticks([0, 1])
        axes[1, 0].set_yticks([0, 1])
        axes[1, 0].set_xticklabels(['U1', 'U2'])
        axes[1, 0].set_yticklabels(['Y1', 'Y2'])

        # 耦合度 vs 频率
        axes[1, 1].semilogx(s_vals, coupling_ratios, 'b-', linewidth=2)
        axes[1, 1].set_title('系统耦合度 vs 频率')
        axes[1, 1].set_xlabel('频率 (rad/s)')
        axes[1, 1].set_ylabel('平均耦合度')
        axes[1, 1].grid(True, alpha=0.3)

        # 奇异值分析
        sigma_max = []
        sigma_min = []
        for s in s_vals:
            _, Gs = self.compute_dynamic_decoupler(s)
            sv = np.linalg.svd(Gs, compute_uv=False)
            sigma_max.append(sv[0])
            sigma_min.append(sv[1])

        axes[1, 2].semilogx(s_vals, 20*np.log10(sigma_max), 'r-', label='σ_max', linewidth=2)
        axes[1, 2].semilogx(s_vals, 20*np.log10(sigma_min), 'b-', label='σ_min', linewidth=2)
        axes[1, 2].fill_between(s_vals,
                                20*np.log10(sigma_min),
                                20*np.log10(sigma_max),
                                alpha=0.2)
        axes[1, 2].set_title('奇异值分析 (条件数)')
        axes[1, 2].set_xlabel('频率 (rad/s)')
        axes[1, 2].set_ylabel('幅值 (dB)')
        axes[1, 2].legend()
        axes[1, 2].grid(True, alpha=0.3)

        plt.suptitle('多变量(MIMO)解耦控制仿真', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('multi_variable_result.png', dpi=150, bbox_inches='tight')
        plt.close('all')

        # 打印分析结果
        self._print_analysis(G0_mat, Lambda, D)

    def _rga_pairing(self, Lambda):
        """根据RGA推荐配对"""
        pairing = []
        used_cols = set()
        for i in range(2):
            best_j = -1
            best_val = -1
            for j in range(2):
                if j not in used_cols and Lambda[i, j] > best_val:
                    best_val = Lambda[i, j]
                    best_j = j
            pairing.append(f'Y{i+1}-U{best_j+1}')
            used_cols.add(best_j)
        return ', '.join(pairing)

    def _print_analysis(self, G0, Lambda, D):
        """打印分析结果"""
        print("=" * 60)
        print("MIMO系统分析")
        print("=" * 60)
        print(f"\n静态增益矩阵 G(0):\n{G0}")
        print(f"\n相对增益矩阵(RGA):\n{Lambda}")
        print(f"\n推荐配对: {self._rga_pairing(Lambda)}")
        print(f"\n静态解耦矩阵 D:\n{np.round(D, 4)}")
        print(f"\n条件数 κ(G) = {np.linalg.cond(G0):.2f}")
        print("=" * 60)


if __name__ == '__main__':
    sim = MultiVariableSimulation()
    sim.run_comparison()
    print("仿真完成!")
