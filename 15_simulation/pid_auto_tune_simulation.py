"""
PID自动整定仿真V2 - 继电反馈法 + 阶跃响应法对比
PID Auto-Tuning Simulation V2 (Relay Feedback vs Step Response)
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import minimize_scalar
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class PIDAutoTuneV2:
    """PID自动整定仿真V2"""

    def __init__(self, K=1.0, T=1.0, L=0.1):
        """
        被控对象: G(s) = K * exp(-Ls) / (Ts + 1) (一阶加延迟)
        """
        self.K = K   # 增益
        self.T = T   # 时间常数
        self.L = L   # 延迟时间

    def plant_response(self, t, u):
        """一阶加延迟系统的阶跃响应"""
        # 用Pade近似处理延迟
        n_pade = 2
        # 延迟近似
        delay_num, delay_den = signal.pade(self.L, n_pade)
        # 一阶系统
        tf_num = [self.K]
        tf_den = [self.T, 1]

        sys_num = np.polymul(tf_num, delay_num)
        sys_den = np.polymul(tf_den, delay_den)

        sys = signal.TransferFunction(sys_num, sys_den)
        _, y, _ = signal.lsim(sys, u, t)
        return y

    def plant_tf_pade(self):
        """返回Pade近似的传递函数系数"""
        delay_num, delay_den = signal.pade(self.L, 2)
        tf_num = [self.K]
        tf_den = [self.T, 1]
        num = np.polymul(tf_num, delay_num)
        den = np.polymul(tf_den, delay_den)
        return num, den

    # ========== 方法1: 继电反馈法 ==========
    def relay_feedback_tune(self, t_sim=10.0):
        """
        继电反馈法自整定
        原理: 用继电特性使系统产生极限环振荡, 测量临界增益和周期
        """
        dt = 0.001
        t = np.arange(0, t_sim, dt)
        n = len(t)

        h = 1.0   # 继电幅值
        d = 0.2   # 继电迟滞宽度
        y = np.zeros(n)
        u = np.zeros(n)
        y[0] = 0

        num, den = self.plant_tf_pade()
        sys = signal.TransferFunction(num, den)

        # Pade近似用lsim, 逐步仿真继电反馈
        # 简化: 用离散状态空间
        sys_discrete = signal.cont2discrete((num, den), dt, method='zoh')
        A_d, B_d, C_d, D_d, _ = sys_discrete
        nx = A_d.shape[0]
        x = np.zeros(nx)

        for i in range(1, n):
            # 状态更新
            x = A_d @ x + B_d.flatten() * u[i-1]
            y[i] = (C_d @ x + D_d.flatten() * u[i-1])[0]

            # 继电特性 (带迟滞)
            if u[i-1] == h:
                if y[i] > -d:
                    u[i] = h
                else:
                    u[i] = -h
            else:
                if y[i] < d:
                    u[i] = -h
                else:
                    u[i] = h

        # 测量极限环参数
        # 找过零点
        crossings = []
        for i in range(n // 2, n - 1):
            if y[i] * y[i+1] < 0:
                crossings.append(i)

        if len(crossings) >= 3:
            Tu = (crossings[2] - crossings[0]) * dt  # 振荡周期
            # 测量输出振幅
            idx1, idx2 = crossings[0], crossings[1]
            a = np.max(np.abs(y[idx1:idx2]))

            # 临界增益 (继电描述函数)
            Ku = 4 * h / (np.pi * a)

            # Ziegler-Nichols整定
            Kp = 0.6 * Ku
            Ti = 0.5 * Tu
            Td = 0.125 * Tu
            Ki = Kp / Ti
            Kd = Kp * Td

            return {
                'method': '继电反馈法',
                'Ku': Ku, 'Tu': Tu, 'a': a,
                'Kp': Kp, 'Ki': Ki, 'Kd': Kd,
                't': t, 'y': y, 'u': u,
                'crossings': crossings
            }
        else:
            print("继电反馈未产生稳定振荡, 请调整参数")
            return None

    # ========== 方法2: 阶跃响应法 ==========
    def step_response_tune(self, t_sim=5.0):
        """
        阶跃响应法自整定 (Ziegler-Nichols开环法)
        原理: 从阶跃响应提取K, T, L, 然后用ZN公式计算PID参数
        """
        dt = 0.001
        t = np.arange(0, t_sim, dt)

        # 阶跃响应
        u = np.ones_like(t)
        y = self.plant_response(t, u)

        # 提取参数: 用S形曲线法
        # 找最大斜率点
        dy = np.gradient(y, dt)
        max_slope_idx = np.argmax(dy[10:]) + 10
        max_slope = dy[max_slope_idx]
        y_at_max_slope = y[max_slope_idx]

        # 切线: y = max_slope * (t - t0) + y0
        # 与y=0和y=K的交点
        if max_slope > 1e-6:
            # 切线与y=0的交点 -> L
            t_at_y0 = t[max_slope_idx] - y_at_max_slope / max_slope
            L_est = t_at_y0

            # 切线与y=K的交点 -> L+T
            t_at_yK = t[max_slope_idx] + (self.K - y_at_max_slope) / max_slope
            T_est = t_at_yK - L_est

            K_est = self.K  # 已知(实际中从稳态值获取)
        else:
            L_est, T_est, K_est = self.L, self.T, self.K

        # Ziegler-Nichols开环法
        a = K_est * L_est / T_est  # 无量纲参数

        if a > 0:
            Kp = 1.2 / a
            Ti = 2 * L_est
            Td = 0.5 * L_est
            Ki = Kp / Ti
            Kd = Kp * Td
        else:
            Kp, Ki, Kd = 1.0, 1.0, 0.0

        return {
            'method': '阶跃响应法',
            'K_est': K_est, 'T_est': T_est, 'L_est': L_est,
            'a': a, 'max_slope': max_slope,
            'Kp': Kp, 'Ki': Ki, 'Kd': Kd,
            't': t, 'y': y,
            'max_slope_idx': max_slope_idx,
            't_at_y0': t_at_y0 if max_slope > 1e-6 else 0,
            't_at_yK': t_at_yK if max_slope > 1e-6 else 0
        }

    # ========== 方法3: Cohen-Coon法 ==========
    def cohen_coon_tune(self):
        """Cohen-Coon整定法 (改进的ZN法)"""
        dt = 0.001
        t = np.arange(0, 5, dt)
        u = np.ones_like(t)
        y = self.plant_response(t, u)

        dy = np.gradient(y, dt)
        max_slope_idx = np.argmax(dy[10:]) + 10
        max_slope = dy[max_slope_idx]
        y_at_max_slope = y[max_slope_idx]

        t_at_y0 = t[max_slope_idx] - y_at_max_slope / max_slope
        L_est = max(t_at_y0, 0.01)
        t_at_yK = t[max_slope_idx] + (self.K - y_at_max_slope) / max_slope
        T_est = max(t_at_yK - L_est, 0.01)
        K_est = self.K

        r = L_est / T_est  # 延迟比

        # Cohen-Coon公式
        Kp = (1 / (K_est * r)) * (0.9 + r / 12)
        Ti = L_est * (30 + 3 * r) / (9 + 20 * r)
        Td = L_est * 4 / (11 + 2 * r)
        Ki = Kp / Ti
        Kd = Kp * Td

        return {
            'method': 'Cohen-Coon法',
            'Kp': Kp, 'Ki': Ki, 'Kd': Kd,
            'r': r, 'L_est': L_est, 'T_est': T_est
        }

    def simulate_pid(self, t, ref, Kp, Ki, Kd):
        """PID闭环仿真"""
        dt = t[1] - t[0]
        n = len(t)

        num, den = self.plant_tf_pade()
        sys_discrete = signal.cont2discrete((num, den), dt, method='zoh')
        A_d, B_d, C_d, D_d, _ = sys_discrete
        nx = A_d.shape[0]
        x = np.zeros(nx)

        y = np.zeros(n)
        e = np.zeros(n)
        u = np.zeros(n)
        integral = 0

        for i in range(1, n):
            x = A_d @ x + B_d.flatten() * u[i-1]
            y[i] = (C_d @ x + D_d.flatten() * u[i-1])[0]

            e[i] = ref[i] - y[i]
            integral += e[i] * dt
            derivative = (e[i] - e[i-1]) / dt

            u[i] = Kp * e[i] + Ki * integral + Kd * derivative
            u[i] = np.clip(u[i], -10, 10)  # 饱和限幅

        return y, u, e

    def run_comparison(self):
        """运行对比仿真"""
        # 获取各方法的整定参数
        relay_result = self.relay_feedback_tune()
        step_result = self.step_response_tune()
        cc_result = self.cohen_coon_tune()

        # 阶跃响应测试
        dt = 0.001
        t = np.arange(0, 8, dt)
        ref = np.ones_like(t)
        ref[t < 0.5] = 0

        methods = {}
        if relay_result:
            methods['继电反馈法'] = relay_result
        methods['阶跃响应法'] = step_result
        methods['Cohen-Coon法'] = cc_result

        # ========== 图1: 整定过程 ==========
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # 继电反馈过程
        if relay_result:
            t_relay = relay_result['t']
            axes[0, 0].plot(t_relay, relay_result['y'], 'b-', linewidth=1.5, label='输出')
            axes[0, 0].plot(t_relay, relay_result['u'] * 0.5, 'r-', alpha=0.5,
                           label='控制量(缩放)')
            for c in relay_result['crossings']:
                axes[0, 0].axvline(x=t_relay[c], color='g', linestyle='--', alpha=0.3)
            axes[0, 0].set_title(f'继电反馈法 (Ku={relay_result["Ku"]:.2f}, '
                                f'Tu={relay_result["Tu"]:.3f}s)')
            axes[0, 0].set_xlabel('时间 (s)')
            axes[0, 0].legend()
            axes[0, 0].grid(True, alpha=0.3)

        # 阶跃响应法
        axes[0, 1].plot(step_result['t'], step_result['y'], 'b-', linewidth=2, label='阶跃响应')
        # 画切线
        idx = step_result['max_slope_idx']
        slope = step_result['max_slope']
        t_line = np.array([step_result['t_at_y0'], step_result['t_at_yK']])
        y_line = slope * (t_line - step_result['t'][idx]) + step_result['y'][idx]
        axes[0, 1].plot(t_line, y_line, 'r--', linewidth=2, label='最大斜率切线')
        axes[0, 1].axhline(y=self.K, color='gray', linestyle=':', alpha=0.5)
        axes[0, 1].axvline(x=step_result['t_at_y0'], color='green',
                          linestyle=':', alpha=0.7, label=f'L={step_result["L_est"]:.3f}s')
        axes[0, 1].axvline(x=step_result['t_at_yK'], color='orange',
                          linestyle=':', alpha=0.7, label=f'L+T={step_result["t_at_yK"]:.3f}s')
        axes[0, 1].set_title('阶跃响应法参数辨识')
        axes[0, 1].set_xlabel('时间 (s)')
        axes[0, 1].legend(fontsize=8)
        axes[0, 1].grid(True, alpha=0.3)

        # ========== 图2: 闭环响应对比 ==========
        colors = {'继电反馈法': '#e74c3c', '阶跃响应法': '#3498db', 'Cohen-Coon法': '#2ecc71'}

        for name, result in methods.items():
            y, u, e = self.simulate_pid(t, ref,
                                         result['Kp'], result['Ki'], result['Kd'])
            axes[1, 0].plot(t, y, color=colors[name], linewidth=1.5, label=name)

        axes[1, 0].plot(t, ref, 'k--', label='参考', linewidth=1)
        axes[1, 0].set_title('闭环阶跃响应对比')
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('输出')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        # 控制量对比
        for name, result in methods.items():
            y, u, e = self.simulate_pid(t, ref,
                                         result['Kp'], result['Ki'], result['Kd'])
            axes[1, 1].plot(t, u, color=colors[name], linewidth=1.5, label=name)

        axes[1, 1].set_title('控制量对比')
        axes[1, 1].set_xlabel('时间 (s)')
        axes[1, 1].set_ylabel('控制量')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)

        plt.suptitle('PID自动整定方法对比', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('pid_auto_tune_v2_result.png', dpi=150, bbox_inches='tight')
        plt.close('all')

        # 打印参数
        self._print_params(methods)

    def _print_params(self, methods):
        """打印整定参数"""
        print("=" * 70)
        print(f"被控对象: G(s) = {self.K}*exp(-{self.L}s) / ({self.T}s + 1)")
        print("=" * 70)
        print(f"{'方法':<15} {'Kp':>8} {'Ki':>8} {'Kd':>8}")
        print("-" * 40)
        for name, result in methods.items():
            print(f"{name:<15} {result['Kp']:>8.4f} {result['Ki']:>8.4f} {result['Kd']:>8.4f}")
        print("=" * 70)


if __name__ == '__main__':
    sim = PIDAutoTuneV2(K=1.0, T=1.0, L=0.2)
    sim.run_comparison()
    print("仿真完成!")
