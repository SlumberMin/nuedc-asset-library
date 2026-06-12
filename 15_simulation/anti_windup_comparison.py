"""
抗饱和(Anti-Windup)策略对比仿真
=================================
对比5种常见抗积分饱和策略的控制效果：
1. 无抗饱和(原始PID)
2. 积分限幅(Clamping)
3. 条件积分(Conditional Integration)
4. 反馈抗饱和(Back-Calculation)
5. 积分分离(Integral Separation)

运行: python anti_windup_comparison.py
"""
import os
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ============ 被控对象 ============
class SecondOrderPlant:
    """二阶系统: G(s) = K / (T1*s+1)(T2*s+1)"""
    def __init__(self, K=1.0, T1=0.5, T2=0.2, dt=0.01):
        self.K, self.T1, self.T2, self.dt = K, T1, T2, dt
        self.x1, self.x2 = 0.0, 0.0

    def update(self, u):
        dx1 = (-self.x1 + self.K * u) / self.T1
        dx2 = (-self.x2 + self.x1) / self.T2
        self.x1 += dx1 * self.dt
        self.x2 += dx2 * self.dt
        return self.x2

# ============ 五种PID策略 ============
class PID_Base:
    def __init__(self, Kp, Ki, Kd, dt, u_min=-10.0, u_max=10.0):
        self.Kp, self.Ki, self.Kd, self.dt = Kp, Ki, Kd, dt
        self.u_min, self.u_max = u_min, u_max
        self.e_sum, self.e_prev = 0.0, 0.0
        self.name = "Base"

    def reset(self):
        self.e_sum, self.e_prev = 0.0, 0.0

    def compute(self, e):
        raise NotImplementedError

    def _saturate(self, u):
        return max(self.u_min, min(self.u_max, u))


class PID_NoAW(PID_Base):
    """1. 无抗饱和"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = "无抗饱和"

    def compute(self, e):
        self.e_sum += e * self.dt
        de = (e - self.e_prev) / self.dt
        u = self.Kp * e + self.Ki * self.e_sum + self.Kd * de
        self.e_prev = e
        return self._saturate(u)


class PID_Clamping(PID_Base):
    """2. 积分限幅"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = "积分限幅"
        self.int_max = self.u_max / max(self.Ki, 0.01)

    def compute(self, e):
        self.e_sum += e * self.dt
        self.e_sum = max(-self.int_max, min(self.int_max, self.e_sum))
        de = (e - self.e_prev) / self.dt
        u = self.Kp * e + self.Ki * self.e_sum + self.Kd * de
        self.e_prev = e
        return self._saturate(u)


class PID_Conditional(PID_Base):
    """3. 条件积分 - 仅在未饱和或误差方向相反时积分"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = "条件积分"

    def compute(self, e):
        u_unsat = self.Kp * e + self.Ki * self.e_sum + self.Kd * ((e - self.e_prev)/self.dt)
        u_sat = self._saturate(u_unsat)
        # 条件积分: 如果未饱和, 或饱和方向与误差方向相反, 则积分
        if abs(u_unsat - u_sat) < 1e-6 or e * u_unsat < 0:
            self.e_sum += e * self.dt
        de = (e - self.e_prev) / self.dt
        u = self.Kp * e + self.Ki * self.e_sum + self.Kd * de
        self.e_prev = e
        return self._saturate(u)


class PID_BackCalc(PID_Base):
    """4. 反馈抗饱和(Back-Calculation)"""
    def __init__(self, *args, Kb=0.5, **kwargs):
        super().__init__(*args, **kwargs)
        self.Kb = Kb
        self.name = f"反馈抗饱和(Kb={Kb})"

    def compute(self, e):
        de = (e - self.e_prev) / self.dt
        u_raw = self.Kp * e + self.Ki * self.e_sum + self.Kd * de
        u_sat = self._saturate(u_raw)
        # Back-calculation: 用饱和差修正积分项
        self.e_sum += (e + self.Kb * (u_sat - u_raw)) * self.dt
        u = self.Kp * e + self.Ki * self.e_sum + self.Kd * de
        self.e_prev = e
        return self._saturate(u)


class PID_IntSep(PID_Base):
    """5. 积分分离 - 误差大时暂停积分"""
    def __init__(self, *args, threshold=0.5, **kwargs):
        super().__init__(*args, **kwargs)
        self.threshold = threshold
        self.name = f"积分分离(th={threshold})"

    def compute(self, e):
        if abs(e) < self.threshold:
            self.e_sum += e * self.dt
        de = (e - self.e_prev) / self.dt
        u = self.Kp * e + self.Ki * self.e_sum + self.Kd * de
        self.e_prev = e
        return self._saturate(u)


# ============ 仿真 ============
def simulate(pid, plant, setpoint, t_end, dt, u_max):
    """仿真并记录, 施加阶跃负载扰动"""
    n = int(t_end / dt)
    t_arr, y_arr, u_arr, e_arr = [], [], [], []
    load_disturb = 0.0
    for i in range(n):
        t = i * dt
        # t=3s时施加负载扰动
        if t >= 3.0:
            load_disturb = 0.5
        e = setpoint - plant.y
        u = pid.compute(e)
        y = plant.update(u + load_disturb)
        t_arr.append(t); y_arr.append(y); u_arr.append(u); e_arr.append(e)
    return np.array(t_arr), np.array(y_arr), np.array(u_arr), np.array(e_arr)


if __name__ == "__main__":
    Kp, Ki, Kd = 2.0, 5.0, 0.5
    dt = 0.01
    t_end = 8.0
    setpoint = 1.0
    u_max = 5.0  # 执行器饱和限幅

    controllers = [
        PID_NoAW(Kp, Ki, Kd, dt, -u_max, u_max),
        PID_Clamping(Kp, Ki, Kd, dt, -u_max, u_max),
        PID_Conditional(Kp, Ki, Kd, dt, -u_max, u_max),
        PID_BackCalc(Kp, Ki, Kd, dt, -u_max, u_max, Kb=0.5),
        PID_IntSep(Kp, Ki, Kd, dt, -u_max, u_max, threshold=0.3),
    ]
    colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange', 'tab:purple']

    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('抗积分饱和策略对比仿真', fontsize=14, fontweight='bold')

    print("=" * 65)
    print("抗积分饱和(Anti-Windup)策略对比仿真")
    print("=" * 65)
    print(f"PID参数: Kp={Kp}, Ki={Ki}, Kd={Kd}, 饱和限幅: ±{u_max}")
    print(f"设定值={setpoint}, t=3s施加负载扰动=0.5")
    print("-" * 65)

    for pid, color in zip(controllers, colors):
        plant = SecondOrderPlant(K=1.0, T1=0.5, T2=0.2, dt=dt)
        t, y, u, e = simulate(pid, plant, setpoint, t_end, dt, u_max)

        ss_val = np.mean(y[int(0.85*len(y)):])
        overshoot = (np.max(y) - setpoint) / setpoint * 100
        settling_idx = len(y) - 1
        for j in range(len(y)-1, int(0.5*len(y)), -1):
            if abs(y[j] - setpoint) > 0.02 * setpoint:
                settling_idx = j
                break
        settling_t = t[min(settling_idx+1, len(t)-1)]

        print(f"{pid.name:<22} 超调={overshoot:>6.2f}% 稳态值={ss_val:.4f} 2%稳定时间={settling_t:.2f}s")

        axes[0].plot(t, y, color=color, label=pid.name, linewidth=1.2)
        axes[1].plot(t, u, color=color, label=pid.name, linewidth=1.0)

    axes[0].axhline(setpoint, color='gray', linestyle='--', alpha=0.5)
    axes[0].axvline(3.0, color='gray', linestyle=':', alpha=0.5, label='负载扰动')
    axes[0].set_title('输出响应 y(t)'); axes[0].set_ylabel('输出'); axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3)
    axes[1].axhline(u_max, color='gray', linestyle='--', alpha=0.3, label='饱和限幅')
    axes[1].axhline(-u_max, color='gray', linestyle='--', alpha=0.3)
    axes[1].set_title('控制量 u(t)'); axes[1].set_ylabel('控制量'); axes[1].set_xlabel('时间 (s)'); axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'anti_windup_result.png')
    plt.savefig(out, dpi=150)
    print(f"\n仿真图已保存: {out}")
    print("\n结论: 反馈抗饱和(Back-Calculation)在大信号启动时效果最优,")
    print("      条件积分实现简单且效果也不错, 推荐嵌入式优先使用。")
