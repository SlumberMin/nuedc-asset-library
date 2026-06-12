"""
鲁棒性分析仿真 - 参数变化对系统性能的影响
Robustness Analysis Simulation
"""

import numpy as np

# numpy兼容：np.trapz在1.x废弃，2.x移除，统一用np.trapezoid
if hasattr(np, 'trapezoid'):
    _trapz = np.trapezoid
else:
    _trapz = np.trapezoid

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class RobustnessAnalysis:
    """鲁棒性分析仿真"""

    def __init__(self):
        # 标称参数
        self.K_nom = 1.0      # 增益
        self.T_nom = 1.0      # 时间常数
        self.L_nom = 0.2      # 延迟
        self.zeta_nom = 0.1   # 阻尼比

        # PI控制器 (基于标称参数设计)
        self.Kp = 2.0
        self.Ki = 1.5

    def make_plant(self, K=None, T=None, L=None, zeta=None, order=2):
        """构建传递函数 (含Pade延迟近似)"""
        K = K or self.K_nom
        T = T or self.T_nom
        L = L or self.L_nom
        zeta = zeta or self.zeta_nom

        delay_num, delay_den = signal.pade(L, 2)

        if order == 1:
            plant_num = [K]
            plant_den = [T, 1]
        elif order == 2:
            plant_num = [K]
            plant_den = [T**2, 2*zeta*T, 1]
        else:
            plant_num = [K]
            plant_den = [T**3, 3*T**2, 3*T, 1]

        num = np.polymul(plant_num, delay_num)
        den = np.polymul(plant_den, delay_den)
        return num, den

    def simulate_closed_loop(self, t, ref, K=None, T=None, L=None, zeta=None,
                              Kp=None, Ki=None, order=2):
        """闭环仿真"""
        num, den = self.make_plant(K, T, L, zeta, order)
        Kp = Kp or self.Kp
        Ki = Ki or self.Ki

        dt = t[1] - t[0]
        sys_d = signal.cont2discrete((num, den), dt, method='zoh')
        A, B, C, D, _ = sys_d
        nx = A.shape[0]
        x = np.zeros(nx)

        n = len(t)
        y = np.zeros(n)
        e_int = 0

        for i in range(1, n):
            x = A @ x + B.flatten() * (ref[i-1] - y[i-1]) * 0  # 先算输出
            # 实际用PI控制器
            e = ref[i] - y[i-1]
            e_int += e * dt
            u = Kp * e + Ki * e_int

            x = np.zeros(nx)  # 重新计算
            # 离散仿真
            pass

        # 改用lsim方式更简洁
        # 开环: C(s)*G(s)
        C_num = [Kp, Ki]
        C_den = [1, 0]

        ol_num = np.polymul(C_num, num)
        ol_den = np.polymul(C_num, den)

        cl_num = ol_num
        cl_den = np.polyadd(ol_den, ol_num)

        sys_cl = signal.TransferFunction(cl_num, cl_den)
        _, y, _ = signal.lsim(sys_cl, ref, t)
        return y

    def get_step_metrics(self, t, y, ref_val=1.0):
        """计算阶跃响应性能指标"""
        n = len(t)
        y_final = y[-1]

        # 上升时间 (10%到90%)
        y10 = 0.1 * ref_val
        y90 = 0.9 * ref_val
        try:
            t10 = t[np.where(y >= y10)[0][0]]
            t90 = t[np.where(y >= y90)[0][0]]
            rise_time = t90 - t10
        except IndexError:
            rise_time = float('inf')

        # 超调量
        y_max = np.max(y)
        overshoot = max(0, (y_max - ref_val) / ref_val * 100)

        # 调节时间 (2%准则)
        settling_time = t[-1]
        for i in range(n-1, 0, -1):
            if np.abs(y[i] - ref_val) > 0.02 * ref_val:
                settling_time = t[min(i+1, n-1)]
                break

        # 稳态误差
        ss_error = np.abs(y_final - ref_val)

        # IAE
        iae = _trapz(np.abs(ref_val - y), t)

        return {
            'rise_time': rise_time,
            'overshoot': overshoot,
            'settling_time': settling_time,
            'ss_error': ss_error,
            'iae': iae,
            'y_final': y_final
        }

    def run_analysis(self):
        """运行完整鲁棒性分析"""
        t = np.linspace(0, 10, 2000)
        ref = np.ones_like(t)

        # ========== 1. 增益变化 ==========
        K_values = np.linspace(0.5, 2.0, 7)
        fig, axes = plt.subplots(3, 3, figsize=(16, 14))

        rise_times_K, overshoots_K, iae_K = [], [], []
        for Kv in K_values:
            y = self.simulate_closed_loop(t, ref, K=Kv)
            axes[0, 0].plot(t, y, label=f'K={Kv:.2f}', linewidth=1.5)
            m = self.get_step_metrics(t, y)
            rise_times_K.append(m['rise_time'])
            overshoots_K.append(m['overshoot'])
            iae_K.append(m['iae'])

        axes[0, 0].plot(t, ref, 'k--', linewidth=1)
        axes[0, 0].set_title('增益K变化时的响应')
        axes[0, 0].set_xlabel('时间 (s)')
        axes[0, 0].set_ylabel('输出')
        axes[0, 0].legend(fontsize=7)
        axes[0, 0].grid(True, alpha=0.3)

        axes[0, 1].plot(K_values, overshoots_K, 'ro-', linewidth=2, markersize=6)
        axes[0, 1].set_title('增益K vs 超调量')
        axes[0, 1].set_xlabel('增益 K')
        axes[0, 1].set_ylabel('超调量 (%)')
        axes[0, 1].grid(True, alpha=0.3)

        axes[0, 2].plot(K_values, iae_K, 'bs-', linewidth=2, markersize=6)
        axes[0, 2].set_title('增益K vs IAE (积分绝对误差)')
        axes[0, 2].set_xlabel('增益 K')
        axes[0, 2].set_ylabel('IAE')
        axes[0, 2].grid(True, alpha=0.3)

        # ========== 2. 时间常数变化 ==========
        T_values = np.linspace(0.5, 2.5, 7)
        rise_times_T, overshoots_T, iae_T = [], [], []
        for Tv in T_values:
            y = self.simulate_closed_loop(t, ref, T=Tv)
            axes[1, 0].plot(t, y, label=f'T={Tv:.2f}', linewidth=1.5)
            m = self.get_step_metrics(t, y)
            rise_times_T.append(m['rise_time'])
            overshoots_T.append(m['overshoot'])
            iae_T.append(m['iae'])

        axes[1, 0].plot(t, ref, 'k--', linewidth=1)
        axes[1, 0].set_title('时间常数T变化时的响应')
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('输出')
        axes[1, 0].legend(fontsize=7)
        axes[1, 0].grid(True, alpha=0.3)

        axes[1, 1].plot(T_values, rise_times_T, 'ro-', linewidth=2, markersize=6)
        axes[1, 1].set_title('时间常数T vs 上升时间')
        axes[1, 1].set_xlabel('时间常数 T')
        axes[1, 1].set_ylabel('上升时间 (s)')
        axes[1, 1].grid(True, alpha=0.3)

        axes[1, 2].plot(T_values, iae_T, 'bs-', linewidth=2, markersize=6)
        axes[1, 2].set_title('时间常数T vs IAE')
        axes[1, 2].set_xlabel('时间常数 T')
        axes[1, 2].set_ylabel('IAE')
        axes[1, 2].grid(True, alpha=0.3)

        # ========== 3. 延迟变化 ==========
        L_values = np.linspace(0.05, 0.5, 7)
        rise_times_L, overshoots_L, iae_L = [], [], []
        for Lv in L_values:
            y = self.simulate_closed_loop(t, ref, L=Lv)
            axes[2, 0].plot(t, y, label=f'L={Lv:.2f}', linewidth=1.5)
            m = self.get_step_metrics(t, y)
            rise_times_L.append(m['rise_time'])
            overshoots_L.append(m['overshoot'])
            iae_L.append(m['iae'])

        axes[2, 0].plot(t, ref, 'k--', linewidth=1)
        axes[2, 0].set_title('延迟L变化时的响应')
        axes[2, 0].set_xlabel('时间 (s)')
        axes[2, 0].set_ylabel('输出')
        axes[2, 0].legend(fontsize=7)
        axes[2, 0].grid(True, alpha=0.3)

        axes[2, 1].plot(L_values, overshoots_L, 'ro-', linewidth=2, markersize=6)
        axes[2, 1].set_title('延迟L vs 超调量')
        axes[2, 1].set_xlabel('延迟 L (s)')
        axes[2, 1].set_ylabel('超调量 (%)')
        axes[2, 1].grid(True, alpha=0.3)

        axes[2, 2].plot(L_values, iae_L, 'bs-', linewidth=2, markersize=6)
        axes[2, 2].set_title('延迟L vs IAE')
        axes[2, 2].set_xlabel('延迟 L (s)')
        axes[2, 2].set_ylabel('IAE')
        axes[2, 2].grid(True, alpha=0.3)

        plt.suptitle('系统参数鲁棒性分析 (标称: K=1, T=1, L=0.2)',
                    fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('robustness_analysis_1d.png', dpi=150, bbox_inches='tight')
        plt.close('all')

        # ========== 4. 二维参数地图 ==========
        fig2, axes2 = plt.subplots(1, 3, figsize=(16, 5))

        # K vs L -> 超调量热力图
        K_grid = np.linspace(0.5, 2.0, 15)
        L_grid = np.linspace(0.05, 0.5, 15)
        overshoot_map = np.zeros((len(K_grid), len(L_grid)))

        for i, Kv in enumerate(K_grid):
            for j, Lv in enumerate(L_grid):
                y = self.simulate_closed_loop(t, ref, K=Kv, L=Lv)
                m = self.get_step_metrics(t, y)
                overshoot_map[i, j] = m['overshoot']

        im0 = axes2[0].imshow(overshoot_map, origin='lower', aspect='auto',
                              extent=[L_grid[0], L_grid[-1], K_grid[0], K_grid[-1]],
                              cmap='hot')
        axes2[0].set_title('超调量 (%)')
        axes2[0].set_xlabel('延迟 L')
        axes2[0].set_ylabel('增益 K')
        plt.colorbar(im0, ax=axes2[0])

        # K vs T -> IAE热力图
        T_grid = np.linspace(0.5, 2.5, 15)
        iae_map = np.zeros((len(K_grid), len(T_grid)))

        for i, Kv in enumerate(K_grid):
            for j, Tv in enumerate(T_grid):
                y = self.simulate_closed_loop(t, ref, K=Kv, T=Tv)
                m = self.get_step_metrics(t, y)
                iae_map[i, j] = m['iae']

        im1 = axes2[1].imshow(iae_map, origin='lower', aspect='auto',
                              extent=[T_grid[0], T_grid[-1], K_grid[0], K_grid[-1]],
                              cmap='viridis')
        axes2[1].set_title('IAE')
        axes2[1].set_xlabel('时间常数 T')
        axes2[1].set_ylabel('增益 K')
        plt.colorbar(im1, ax=axes2[1])

        # 稳定性边界
        stability_map = np.zeros((len(K_grid), len(L_grid)))
        for i, Kv in enumerate(K_grid):
            for j, Lv in enumerate(L_grid):
                y = self.simulate_closed_loop(t, ref, K=Kv, L=Lv)
                # 判断是否稳定 (最终是否收敛)
                if np.abs(y[-1] - 1.0) < 0.5 and np.max(np.abs(y[-100:])) < 3:
                    stability_map[i, j] = 1  # 稳定
                else:
                    stability_map[i, j] = 0  # 不稳定

        axes2[2].imshow(stability_map, origin='lower', aspect='auto',
                       extent=[L_grid[0], L_grid[-1], K_grid[0], K_grid[-1]],
                       cmap='RdYlGn')
        axes2[2].set_title('稳定性区域 (绿=稳定, 红=不稳定)')
        axes2[2].set_xlabel('延迟 L')
        axes2[2].set_ylabel('增益 K')

        plt.suptitle('参数空间鲁棒性地图', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('robustness_analysis_2d.png', dpi=150, bbox_inches='tight')
        plt.close('all')

        # ========== 5. 不同控制器对比 ==========
        fig3, axes3 = plt.subplots(1, 2, figsize=(14, 5))

        controllers = {
            'PI (Kp=2, Ki=1.5)': (2.0, 1.5),
            'PI (Kp=1, Ki=0.5)': (1.0, 0.5),
            'PI (Kp=3, Ki=3)': (3.0, 3.0),
        }

        for name, (kp, ki) in controllers.items():
            iae_values = []
            for Lv in L_values:
                y = self.simulate_closed_loop(t, ref, L=Lv, Kp=kp, Ki=ki)
                m = self.get_step_metrics(t, y)
                iae_values.append(m['iae'])
            axes3[0].plot(L_values, iae_values, 'o-', linewidth=2, markersize=6, label=name)

        axes3[0].set_title('不同控制器对延迟变化的鲁棒性')
        axes3[0].set_xlabel('延迟 L (s)')
        axes3[0].set_ylabel('IAE')
        axes3[0].legend()
        axes3[0].grid(True, alpha=0.3)

        # 多阶系统对比
        for order in [1, 2, 3]:
            iae_values = []
            for Kv in K_values:
                y = self.simulate_closed_loop(t, ref, K=Kv, order=order)
                m = self.get_step_metrics(t, y)
                iae_values.append(m['iae'])
            axes3[1].plot(K_values, iae_values, 'o-', linewidth=2, markersize=6,
                         label=f'{order}阶系统')

        axes3[1].set_title('不同系统阶次对增益变化的鲁棒性')
        axes3[1].set_xlabel('增益 K')
        axes3[1].set_ylabel('IAE')
        axes3[1].legend()
        axes3[1].grid(True, alpha=0.3)

        plt.suptitle('控制器与系统阶次的鲁棒性对比', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('robustness_analysis_ctrl.png', dpi=150, bbox_inches='tight')
        plt.close('all')

        # 打印总结
        self._print_summary(K_values, L_values, overshoots_K, iae_L)

    def _print_summary(self, K_values, L_values, overshoots_K, iae_L):
        """打印分析总结"""
        print("=" * 60)
        print("鲁棒性分析总结")
        print("=" * 60)
        print(f"标称参数: K={self.K_nom}, T={self.T_nom}, L={self.L_nom}")
        print(f"控制器: Kp={self.Kp}, Ki={self.Ki}")
        print(f"\n增益范围 K ∈ [{K_values[0]:.1f}, {K_values[-1]:.1f}]:")
        print(f"  超调量范围: {min(overshoots_K):.1f}% ~ {max(overshoots_K):.1f}%")
        print(f"\n延迟范围 L ∈ [{L_values[0]:.2f}, {L_values[-1]:.2f}]:")
        print(f"  IAE范围: {min(iae_L):.3f} ~ {max(iae_L):.3f}")
        print(f"\n结论:")
        if max(overshoots_K) < 30:
            print("  ✓ 增益鲁棒性: 良好 (超调量<30%)")
        else:
            print("  ✗ 增益鲁棒性: 需改进")
        if max(iae_L) / min(iae_L) < 3:
            print("  ✓ 延迟鲁棒性: 良好")
        else:
            print("  ✗ 延迟鲁棒性: 需改进")
        print("=" * 60)


if __name__ == '__main__':
    analyzer = RobustnessAnalysis()
    analyzer.run_analysis()
    print("鲁棒性分析完成!")
