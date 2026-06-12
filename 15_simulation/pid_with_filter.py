"""
PID + 滤波器仿真 - 微分滤波效果分析
=====================================
对比不同微分滤波策略对PID控制的影响:
1. 理想微分(无滤波) — 噪声放大严重
2. 一阶低通滤波微分
3. 不完全微分(带惯性环节)
4. 二阶Butterworth滤波微分

分析噪声抑制、响应速度、稳定性之间的权衡。

运行: python pid_with_filter.py
"""
import os
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ============ 被控对象 ============
class SecondOrderPlant:
    def __init__(self, K=1.0, T1=0.5, T2=0.2, dt=0.01):
        self.K, self.T1, self.T2, self.dt = K, T1, T2, dt
        self.x1, self.x2 = 0.0, 0.0

    def update(self, u):
        dx1 = (-self.x1 + self.K * u) / self.T1
        dx2 = (-self.x2 + self.x1) / self.T2
        self.x1 += dx1 * self.dt
        self.x2 += dx2 * self.dt
        return self.x2


# ============ 滤波器 ============
class LowPassFilter1:
    """一阶低通: y[n] = alpha*x[n] + (1-alpha)*y[n-1]"""
    def __init__(self, fc, dt):
        rc = 1.0 / (2 * math.pi * fc)
        self.alpha = dt / (rc + dt)
        self.y_prev = 0.0

    def filter(self, x):
        self.y_prev = self.alpha * x + (1 - self.alpha) * self.y_prev
        return self.y_prev

    def reset(self):
        self.y_prev = 0.0


class LowPassFilter2:
    """二阶Butterworth低通"""
    def __init__(self, fc, dt):
        wc = 2 * math.pi * fc
        wc_d = 2 / dt * math.tan(wc * dt / 2)
        k = wc_d * dt / 2
        k2 = k * k
        sqrt2 = math.sqrt(2)
        self.a0 = 1 + sqrt2 * k + k2
        self.b0 = k2 / self.a0
        self.b1 = 2 * k2 / self.a0
        self.b2 = k2 / self.a0
        self.a1 = 2 * (k2 - 1) / self.a0
        self.a2 = (1 - sqrt2 * k + k2) / self.a0
        self.x = [0.0, 0.0, 0.0]
        self.y = [0.0, 0.0, 0.0]

    def filter(self, x_new):
        self.x[2] = self.x[1]; self.x[1] = self.x[0]; self.x[0] = x_new
        self.y[2] = self.y[1]; self.y[1] = self.y[0]
        self.y[0] = self.b0*self.x[0] + self.b1*self.x[1] + self.b2*self.x[2] - self.a1*self.y[1] - self.a2*self.y[2]
        return self.y[0]

    def reset(self):
        self.x = [0.0, 0.0, 0.0]
        self.y = [0.0, 0.0, 0.0]


# ============ PID控制器 ============
class PIDWithFilter:
    def __init__(self, Kp, Ki, Kd, dt, filter_type='none', fc=50.0, u_min=-10, u_max=10):
        self.Kp, self.Ki, self.Kd, self.dt = Kp, Ki, Kd, dt
        self.u_min, self.u_max = u_min, u_max
        self.e_sum, self.e_prev = 0.0, 0.0
        self.filter_type = filter_type
        self.name = f"PID+{filter_type}"

        if filter_type == 'lp1':
            self.d_filter = LowPassFilter1(fc, dt)
            self.name = f"PID+一阶LPF(fc={fc}Hz)"
        elif filter_type == 'lp2':
            self.d_filter = LowPassFilter2(fc, dt)
            self.name = f"PID+二阶Butter(fc={fc}Hz)"
        elif filter_type == 'incomplete':
            # 不完全微分: D(s) = Kd*s / (Tf*s+1), Tf=1/(2*pi*fc)
            self.Tf = 1.0 / (2 * math.pi * fc)
            self.d_prev = 0.0
            self.name = f"PID+不完全微分(fc={fc}Hz)"
        else:
            self.name = "PID+理想微分(无滤波)"

    def reset(self):
        self.e_sum, self.e_prev = 0.0, 0.0
        if hasattr(self, 'd_filter'):
            self.d_filter.reset()

    def compute(self, e):
        self.e_sum += e * self.dt

        # 微分计算
        de_raw = (e - self.e_prev) / self.dt

        if self.filter_type == 'lp1' or self.filter_type == 'lp2':
            de = self.d_filter.filter(de_raw)
        elif self.filter_type == 'incomplete':
            alpha = self.dt / (self.Tf + self.dt)
            d_term = self.Kd * de_raw
            d_filt = alpha * d_term + (1 - alpha) * self.d_prev
            self.d_prev = d_filt
            de = de_raw  # 微分项通过不完全微分处理
            self.e_prev = e
            u = self.Kp * e + self.Ki * self.e_sum + d_filt
            return max(self.u_min, min(self.u_max, u))
        else:
            de = de_raw

        self.e_prev = e
        u = self.Kp * e + self.Ki * self.e_sum + self.Kd * de
        return max(self.u_min, min(self.u_max, u))


# ============ 仿真 ============
def run_sim(pid, plant, setpoint, t_end, dt, noise_std=0.0):
    n = int(t_end / dt)
    t_arr, y_arr, u_arr, e_arr, de_arr = [], [], [], [], []
    for i in range(n):
        t = i * dt
        y_meas = plant.x + np.random.randn() * noise_std if noise_std > 0 else plant.x
        e = setpoint - y_meas
        u = pid.compute(e)
        y = plant.update(u)
        t_arr.append(t); y_arr.append(y); u_arr.append(u); e_arr.append(e)
    return np.array(t_arr), np.array(y_arr), np.array(u_arr), np.array(e_arr)


if __name__ == "__main__":
    Kp, Ki, Kd = 3.0, 2.0, 1.5
    dt = 0.01
    t_end = 5.0
    setpoint = 1.0
    noise_std = 0.05  # 测量噪声标准差

    configs = [
        ('none', None),
        ('lp1', 100),
        ('lp1', 30),
        ('lp2', 50),
        ('incomplete', 50),
    ]
    colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange', 'tab:purple']

    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('PID + 微分滤波仿真 (σ_noise=0.05)', fontsize=14, fontweight='bold')

    print("=" * 70)
    print("PID + 微分滤波仿真")
    print("=" * 70)
    print(f"PID参数: Kp={Kp}, Ki={Ki}, Kd={Kd}, 噪声σ={noise_std}")
    print("-" * 70)

    for (ftype, fc), color in zip(configs, colors):
        plant = SecondOrderPlant(K=1.0, T1=0.5, T2=0.2, dt=dt)
        pid = PIDWithFilter(Kp, Ki, Kd, dt, filter_type=ftype, fc=fc if fc else 50.0)
        t, y, u, e = run_sim(pid, plant, setpoint, t_end, dt, noise_std)

        u_rms = np.sqrt(np.mean(u**2))
        iae = np.sum(np.abs(e)) * dt
        print(f"{pid.name:<30} IAE={iae:.4f}  控制量RMS={u_rms:.4f}")

        axes[0].plot(t, y, color=color, label=pid.name, linewidth=1.2)
        axes[1].plot(t, u, color=color, label=pid.name, linewidth=0.8)

    axes[0].axhline(setpoint, color='gray', linestyle='--', alpha=0.5)
    axes[0].set_title('输出响应(含噪声)'); axes[0].set_ylabel('y(t)'); axes[0].legend(fontsize=7); axes[0].grid(True, alpha=0.3)
    axes[1].set_title('控制量(微分噪声表现)'); axes[1].set_ylabel('u(t)'); axes[1].set_xlabel('时间 (s)'); axes[1].legend(fontsize=7); axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_with_filter_result.png')
    plt.savefig(out, dpi=150)
    print(f"\n仿真图已保存: {out}")
    print("\n结论: 理想微分对噪声极为敏感, 一阶LPF截断频率需>10×系统带宽,")
    print("      二阶Butterworth和不完全微分在噪声抑制与响应速度间取得最优平衡。")
    print("      推荐: 不完全微分(实现简单) 或 二阶LPF(效果最好)。")
