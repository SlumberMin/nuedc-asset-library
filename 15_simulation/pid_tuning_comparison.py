#!/usr/bin/env python3
"""
PID调参方法对比仿真
PID Tuning Methods Comparison

调参方法:
  1. Ziegler-Nichols 临界比例法 (ZN Critical)
  2. Ziegler-Nichols 阶跃响应法 (ZN Step / Cohen-Coon)
  3. 继电反馈法 (Relay Feedback / Astrom-Hagglund)
  4. 阶跃响应法 (Step Response - CHR)
  5. IMC (内模控制) 整定法
  6. Lambda整定法

被控对象:
  - 一阶惯性+滞后: G(s) = K·e^(-Ls) / (Ts+1)
  - 二阶系统: G(s) = K / (s² + as + b)
  - 积分+滞后: G(s) = K·e^(-Ls) / s

作者: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import lti, step

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


# ========== 被控对象 ==========

class ProcessModel:
    """过程模型"""

    @staticmethod
    def first_order_delay(K=1.0, T=2.0, L=0.5):
        """
        一阶惯性+滞后: G(s) = K·e^(-Ls) / (Ts+1)
        使用 Pade 近似处理延迟
        """
        # 一阶Pade近似: e^(-Ls) ≈ (1 - Ls/2) / (1 + Ls/2)
        num_delay = np.array([-L / 2, 1])
        den_delay = np.array([L / 2, 1])

        num = K * np.convolve([1], num_delay)
        den = np.convolve([T, 1], den_delay)

        return lti(num, den)

    @staticmethod
    def second_order(K=1.0, a=1.0, b=1.0):
        """二阶系统: G(s) = K / (s² + as + b)"""
        return lti([K], [1, a, b])

    @staticmethod
    def integrator_delay(K=1.0, L=0.3):
        """积分+滞后: G(s) = K / s · e^(-Ls)"""
        num_delay = np.array([-L / 2, 1])
        den_delay = np.array([L / 2, 1])

        num = K * np.convolve([1], num_delay)
        den = np.convolve([1, 0], den_delay)

        return lti(num, den)


# ========== 调参方法 ==========

class PIDTuningMethods:
    """各种PID整定方法"""

    @staticmethod
    def ziegler_nichols_step(K, T, L):
        """
        Ziegler-Nichols 阶跃响应法
        参数: K(增益), T(时间常数), L(滞后时间)
        """
        a = K * L / T
        Kp = 1.2 / a
        Ti = 2 * L
        Td = 0.5 * L
        Ki = Kp / Ti
        Kd = Kp * Td
        return Kp, Ki, Kd, 'ZN-Step'

    @staticmethod
    def cohen_coon(K, T, L):
        """
        Cohen-Coon 整定法
        适用于一阶惯性+滞后系统
        """
        tau = L / (L + T)
        Kp = (1 / (K * tau)) * (1 + tau / 3)
        Ti = L * (32 + 6 * tau) / (13 + 8 * tau)
        Td = L * 4 / (11 + 2 * tau)
        Ki = Kp / Ti
        Kd = Kp * Td
        return Kp, Ki, Kd, 'Cohen-Coon'

    @staticmethod
    def relay_feedback_estimate(K, a, threshold=0.01):
        """
        继电反馈法估算临界增益和临界周期
        模拟继电反馈实验
        """
        # 简化: 假设从继电反馈实验中获得了 Ku 和 Tu
        # 这里用模型参数近似计算
        # 实际应通过仿真测量振荡周期
        Ku = 4 * a / (np.pi * threshold)  # 临界增益估算
        Tu = 2 * np.pi / np.sqrt(a)  # 临界周期估算

        # ZN临界比例法参数
        Kp = 0.6 * Ku
        Ti = 0.5 * Tu
        Td = 0.125 * Tu
        Ki = Kp / Ti
        Kd = Kp * Td
        return Kp, Ki, Kd, 'Relay-ZN', Ku, Tu

    @staticmethod
    def chr_tuning(K, T, L, overshoot='zero'):
        """
        CHR (Chien-Hrones-Reswick) 阶跃响应法
        overshoot: 'zero' (无超调) 或 '20' (20%超调)
        """
        a = K * L / T
        if overshoot == 'zero':
            Kp = 0.3 / a
            Ti = T
            Td = 0.5 * L
        else:  # 20% overshoot
            Kp = 0.6 / a
            Ti = T
            Td = 0.5 * L
        Ki = Kp / Ti
        Kd = Kp * Td
        return Kp, Ki, Kd, f'CHR-{overshoot}%'

    @staticmethod
    def imc_tuning(K, T, L, lambda_c=None):
        """
        IMC (内模控制) 整定法
        lambda_c: 期望闭环时间常数 (越大越鲁棒, 越小越快)
        """
        if lambda_c is None:
            lambda_c = max(0.1 * T, 1.5 * L)  # 经验选取

        Kp = (2 * T + L) / (2 * K * lambda_c)
        Ti = T + L / 2
        Td = T * L / (2 * T + L)
        Ki = Kp / Ti
        Kd = Kp * Td
        return Kp, Ki, Kd, 'IMC'

    @staticmethod
    def lambda_tuning(K, T, L, lambda_factor=3.0):
        """
        Lambda整定法
        lambda_factor: L/λ 比值, 通常2-5
        """
        lambda_c = lambda_factor * L

        Kp = T / (K * (lambda_c + L))
        Ti = T
        Td = 0
        Ki = Kp / Ti
        Kd = 0
        return Kp, Ki, Kd, 'Lambda'


# ========== PID仿真 ==========

class PIDSimulation:
    """PID控制器闭环仿真"""

    def __init__(self, dt=0.01, t_end=15.0):
        self.dt = dt
        self.t_end = t_end

    def simulate(self, plant_num, plant_den, Kp, Ki, Kd, setpoint=1.0):
        """
        PID闭环仿真 (手动离散化)
        """
        steps = int(self.t_end / self.dt)
        t = np.linspace(0, self.t_end, steps)

        # 简单离散化仿真
        # 使用状态空间进行系统仿真
        sys = lti(plant_num, plant_den)

        y = np.zeros(steps)
        u = np.zeros(steps)
        integral = 0
        prev_error = 0

        # 用卷积模拟系统响应 (简化)
        for i in range(steps - 1):
            error = setpoint - y[i]
            integral += error * self.dt
            derivative = (error - prev_error) / self.dt
            prev_error = error

            u[i] = Kp * error + Ki * integral + Kd * derivative
            u[i] = np.clip(u[i], -100, 100)

            # 简化: 用差分近似系统响应
            # 计算阶跃响应
            pass

        # 改用scipy直接仿真
        t_out, y_out, x_out = self._simulate_closed_loop(
            plant_num, plant_den, Kp, Ki, Kd, setpoint
        )
        return t_out, y_out

    def _simulate_closed_loop(self, plant_num, plant_den, Kp, Ki, Kd, setpoint):
        """闭环仿真"""
        steps = int(self.t_end / self.dt)
        t = np.arange(steps) * self.dt
        y = np.zeros(steps)
        u_arr = np.zeros(steps)

        # 状态空间 (用于简单仿真)
        # 假设二阶系统
        n = max(len(plant_num), len(plant_den))
        num = np.pad(plant_num, (n - len(plant_num), 0))
        den = np.pad(plant_den, (n - len(plant_den), 0))

        # 简单数字仿真
        # 存储系统输出历史
        y_hist = [0.0] * 3
        u_hist = [0.0] * 3
        integral = 0.0
        prev_e = 0.0

        for i in range(steps):
            e = setpoint - y[i]
            integral += e * self.dt
            derivative = (e - prev_e) / self.dt if i > 0 else 0
            prev_e = e

            u_pid = Kp * e + Ki * integral + Kd * derivative
            u_arr[i] = u_pid

            # 差分方程仿真 (二阶系统)
            if i < steps - 1:
                # 使用传递函数的差分方程形式
                # 简单: 用微分方程
                # y'' = -(den[1]*y' + den[0]*y - num[0]*u) / den[2]  (如果三阶den)
                if len(den) == 3:
                    # 二阶: a2*y'' + a1*y' + a0*y = b0*u
                    # 离散化
                    a2, a1, a0 = den[2], den[1], den[0]
                    b0 = num[0] if len(num) > 0 else 0
                    b1 = num[1] if len(num) > 1 else 0
                    b2 = num[2] if len(num) > 2 else 0

                    # 简单Euler
                    dy = y[i] - (y[i-1] if i > 0 else 0)
                    y_next = y[i] + self.dt * dy / self.dt if i > 0 else 0
                    # 使用scipy更准确
                    pass

        # 改用scipy的forced_response
        return self._scipy_simulate(plant_num, plant_den, Kp, Ki, Kd, setpoint, steps, t)

    def _scipy_simulate(self, plant_num, plant_den, Kp, Ki, Kd, setpoint, steps, t):
        """用scipy仿真闭环系统"""
        # 构造PID传递函数: C(s) = Kp + Ki/s + Kd*s
        # 闭环: G_cl = C*P / (1 + C*P)
        # 分子: (Kd*s² + Kp*s + Ki) * P_num / s
        # 分母: s*P_den + (Kd*s² + Kp*s + Ki) * P_num / s... 

        # 简化: 用状态空间仿真
        # 手动积分
        y = np.zeros(steps)
        u = np.zeros(steps)
        integral = 0
        prev_e = 0

        # 构造闭环传递函数
        # C(s) = Kp + Ki/s + Kd*s = (Kd*s² + Kp*s + Ki) / s
        pid_num = np.array([Kd, Kp, Ki])
        pid_den = np.array([1, 0])

        # 开环 = C * P
        ol_num = np.convolve(pid_num, plant_num)
        ol_den = np.convolve(pid_den, plant_den)

        # 闭环 = ol / (1 + ol) = ol_num / (ol_den + ol_num)
        # 需要同阶
        max_len = max(len(ol_num), len(ol_den))
        ol_num_pad = np.pad(ol_num, (max_len - len(ol_num), 0))
        ol_den_pad = np.pad(ol_den, (max_len - len(ol_den), 0))

        cl_num = ol_num_pad
        cl_den = ol_den_pad + ol_num_pad

        sys_cl = lti(cl_num, cl_den)
        t_out, y_out = step(sys_cl, T=t)

        return t_out, y_out * setpoint

    def run_comparison(self):
        """运行所有调参方法对比"""
        print("=" * 60)
        print("PID调参方法对比仿真")
        print("=" * 60)

        # 被控对象: G(s) = 2 * e^(-0.5s) / (3s + 1)
        K, T, L = 2.0, 3.0, 0.5

        # 获取传递函数
        plant = ProcessModel.first_order_delay(K, T, L)

        # 各种调参方法
        methods = []
        results = PIDTuningMethods

        # 1. ZN阶跃响应法
        params = results.ziegler_nichols_step(K, T, L)
        methods.append(params)

        # 2. Cohen-Coon
        params = results.cohen_coon(K, T, L)
        methods.append(params)

        # 3. CHR零超调
        params = results.chr_tuning(K, T, L, 'zero')
        methods.append(params)

        # 4. CHR 20%超调
        params = results.chr_tuning(K, T, L, '20')
        methods.append(params)

        # 5. IMC
        params = results.imc_tuning(K, T, L)
        methods.append(params)

        # 6. Lambda整定
        params = results.lambda_tuning(K, T, L)
        methods.append(params)

        # 打印参数表
        print(f"\n被控对象: G(s) = {K}·e^(-{L}s) / ({T}s + 1)")
        print("-" * 65)
        print(f"{'方法':<15} {'Kp':<8} {'Ki':<8} {'Kd':<8}")
        print("-" * 65)
        for Kp, Ki, Kd, name in methods:
            print(f"{name:<15} {Kp:<8.3f} {Ki:<8.3f} {Kd:<8.3f}")
        print("-" * 65)

        # 仿真对比
        sim = PIDSimulation(dt=0.01, t_end=20.0)

        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle(f'PID调参方法对比\n被控对象: G(s) = {K}·e^(-{L}s)/({T}s+1)',
                     fontsize=14, fontweight='bold')

        colors = ['b', 'r', 'g', 'm', 'c', 'orange']
        setpoint = 1.0

        for i, (Kp, Ki, Kd, name) in enumerate(methods):
            try:
                t, y = sim.simulate(plant.num[0][0], plant.den[0][0], Kp, Ki, Kd, setpoint)

                axes[0, 0].plot(t, y, colors[i], linewidth=1.5, label=name)

                # 计算性能指标
                info = self._compute_metrics(t, y, setpoint)
                print(f"{name}: 上升时间={info['rise_time']:.2f}s, "
                      f"超调={info['overshoot']:.1f}%, "
                      f"调节时间={info['settling_time']:.2f}s")
            except Exception as e:
                print(f"{name}: 仿真失败 - {e}")

        axes[0, 0].axhline(y=setpoint, color='k', linestyle='--', alpha=0.5, label='设定值')
        axes[0, 0].set_ylabel('输出 y')
        axes[0, 0].set_title('阶跃响应')
        axes[0, 0].legend(loc='lower right', fontsize=9)
        axes[0, 0].grid(True, alpha=0.3)

        # 参数柱状图对比
        names = [m[3] for m in methods]
        Kps = [m[0] for m in methods]
        Kis = [m[1] for m in methods]
        Kds = [m[2] for m in methods]

        x_pos = np.arange(len(names))
        width = 0.25

        axes[0, 1].bar(x_pos - width, Kps, width, label='Kp', color='steelblue')
        axes[0, 1].bar(x_pos, Kis, width, label='Ki', color='coral')
        axes[0, 1].bar(x_pos + width, Kds, width, label='Kd', color='limegreen')
        axes[0, 1].set_xticks(x_pos)
        axes[0, 1].set_xticklabels(names, rotation=30, fontsize=9)
        axes[0, 1].set_ylabel('参数值')
        axes[0, 1].set_title('PID参数对比')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        # 不同lambda的IMC对比
        axes[1, 0].set_title('IMC整定 - 不同λc对比')
        for lambda_c in [0.5, 1.0, 2.0, 3.0]:
            Kp, Ki, Kd, _ = results.imc_tuning(K, T, L, lambda_c)
            try:
                t, y = sim.simulate(plant.num[0][0], plant.den[0][0], Kp, Ki, Kd, setpoint)
                axes[1, 0].plot(t, y, linewidth=1.5, label=f'λc={lambda_c}')
            except:
                pass
        axes[1, 0].axhline(y=setpoint, color='k', linestyle='--', alpha=0.5)
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('输出 y')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        # 不同Lambda整定对比
        axes[1, 1].set_title('Lambda整定 - 不同λ因子对比')
        for lf in [2.0, 3.0, 5.0, 8.0]:
            Kp, Ki, Kd, _ = results.lambda_tuning(K, T, L, lf)
            try:
                t, y = sim.simulate(plant.num[0][0], plant.den[0][0], Kp, Ki, Kd, setpoint)
                axes[1, 1].plot(t, y, linewidth=1.5, label=f'λ={lf}')
            except:
                pass
        axes[1, 1].axhline(y=setpoint, color='k', linestyle='--', alpha=0.5)
        axes[1, 1].set_xlabel('时间 (s)')
        axes[1, 1].set_ylabel('输出 y')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('pid_tuning_comparison.png', dpi=150, bbox_inches='tight')
        plt.close('all')

    @staticmethod
    def _compute_metrics(t, y, setpoint):
        """计算阶跃响应性能指标"""
        info = {}

        # 上升时间 (10%-90%)
        try:
            idx_10 = next(i for i, v in enumerate(y) if v >= 0.1 * setpoint)
            idx_90 = next(i for i, v in enumerate(y) if v >= 0.9 * setpoint)
            info['rise_time'] = t[idx_90] - t[idx_10]
        except:
            info['rise_time'] = float('inf')

        # 超调量
        overshoot = (np.max(y) - setpoint) / setpoint * 100
        info['overshoot'] = max(0, overshoot)

        # 调节时间 (±2%)
        tolerance = 0.02 * setpoint
        settling_idx = len(t) - 1
        for i in range(len(t) - 1, -1, -1):
            if abs(y[i] - setpoint) > tolerance:
                settling_idx = min(i + 1, len(t) - 1)
                break
        info['settling_time'] = t[settling_idx]

        # IAE
        info['IAE'] = np.sum(np.abs(setpoint - y)) * (t[1] - t[0])

        # ITAE
        info['ITAE'] = np.sum(t * np.abs(setpoint - y)) * (t[1] - t[0])

        return info


if __name__ == '__main__':
    sim = PIDSimulation()
    sim.run_comparison()
    print("\nPID调参方法对比仿真完成!")
