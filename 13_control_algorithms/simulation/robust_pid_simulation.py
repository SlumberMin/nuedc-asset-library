#!/usr/bin/env python3
"""
鲁棒PID仿真

研究PID控制器在参数不确定性、外部扰动、模型失配下的鲁棒性。
包含:
  1. 参数不确定性下的性能退化分析
  2. 外部阶跃扰动抑制
  3. 模型失配(标称 vs 实际被控对象)
  4. 鲁棒PID设计准则可视化
  5. 灵敏度/补灵敏度函数分析
"""

import numpy as np
# numpy 2.x 兼容
if not hasattr(np, 'trapz'):
    np.trapezoid = np.trapezoid


# numpy兼容：np.trapz在1.x废弃，2.x移除，统一用np.trapezoid
if hasattr(np, 'trapezoid'):
    _trapz = np.trapezoid
else:
    _trapz = np.trapezoid

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass
from scipy import signal

# ============================================================
# 被控对象模型
# ============================================================
@dataclass
class PlantParams:
    """二阶系统参数"""
    gain: float = 1.0         # 直流增益
    wn: float = 20.0          # 自然频率
    zeta: float = 0.3         # 阻尼比
    delay: float = 0.0        # 纯延迟 (s)

    def transfer_function(self):
        """返回scipy传递函数"""
        num = [self.gain * self.wn**2]
        den = [1, 2*self.zeta*self.wn, self.wn**2]
        return signal.TransferFunction(num, den)


class DiscreteSimulator:
    """离散时间仿真器(前向欧拉)"""
    def __init__(self, plant: PlantParams, dt: float):
        self.dt = dt
        self.wn = plant.wn
        self.zeta = plant.zeta
        self.gain = plant.gain
        self.x1 = 0.0
        self.x2 = 0.0
        self.disturbance = 0.0

    def set_disturbance(self, d: float):
        self.disturbance = d

    def step(self, u: float) -> float:
        dx1 = self.x2
        dx2 = (-self.wn**2 * self.x1
                - 2*self.zeta*self.wn*self.x2
                + self.wn**2 * self.gain * u
                + self.disturbance)
        self.x1 += dx1 * self.dt
        self.x2 += dx2 * self.dt
        return self.x1

    def reset(self):
        self.x1 = 0.0
        self.x2 = 0.0


# ============================================================
# PID 控制器
# ============================================================
@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float
    dt: float
    out_min: float = -10.0
    out_max: float = 10.0
    integral: float = 0.0
    prev_error: float = 0.0
    first: bool = True

    def update(self, sp: float, meas: float) -> float:
        e = sp - meas
        self.integral += self.ki * e * self.dt
        d = 0.0 if self.first else self.kd * (e - self.prev_error) / self.dt
        self.first = False
        out = self.kp * e + self.integral + d
        out = np.clip(out, self.out_min, self.out_max)
        self.prev_error = e
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True


# ============================================================
# Ziegler-Nichols 整定
# ============================================================
def ziegler_nichols(Ku: float, Tu: float, method: str = 'classic'):
    """
    Ziegler-Nichols 整定法则
    Ku: 临界增益
    Tu: 临界周期
    """
    if method == 'classic':
        kp = 0.6 * Ku
        Ti = 0.5 * Tu
        Td = 0.125 * Tu
    elif method == 'pessen':
        kp = 0.7 * Ku
        Ti = 0.4 * Tu
        Td = 0.15 * Tu
    elif method == 'some_overshoot':
        kp = 0.33 * Ku
        Ti = 0.5 * Tu
        Td = 0.33 * Tu
    elif method == 'no_overshoot':
        kp = 0.2 * Ku
        Ti = 0.5 * Tu
        Td = 0.33 * Tu
    else:
        raise ValueError(f"Unknown method: {method}")

    ki = kp / Ti
    kd = kp * Td
    return kp, ki, kd


# ============================================================
# 仿真实验
# ============================================================
def run_experiment(dt=0.001, duration=3.0, kp=2.0, ki=5.0, kd=0.1,
                   plant_params=None, disturbance_time=None,
                   disturbance_mag=0.0, noise_std=0.0):
    """运行一次仿真实验"""
    if plant_params is None:
        plant_params = PlantParams()

    sim = DiscreteSimulator(plant_params, dt)
    pid = PIDController(kp, ki, kd, dt, out_min=-10, out_max=10)

    steps = int(duration / dt)
    t = np.zeros(steps)
    y = np.zeros(steps)
    u = np.zeros(steps)
    sp = np.ones(steps)  # 阶跃设定值

    for i in range(steps):
        t[i] = i * dt

        # 注入扰动
        if disturbance_time and t[i] >= disturbance_time:
            sim.set_disturbance(disturbance_mag)

        meas = sim.x1
        if noise_std > 0:
            meas += np.random.normal(0, noise_std)

        u[i] = pid.update(sp[i], meas)
        y[i] = sim.step(u[i])

    return t, y, u, sp


# ============================================================
# 性能指标
# ============================================================
def calc_metrics(t, y, sp):
    """计算控制性能指标"""
    setpoint = sp[-1]
    error = sp - y

    # 上升时间 (10%~90%)
    idx_10 = np.argmax(y >= 0.1 * setpoint)
    idx_90 = np.argmax(y >= 0.9 * setpoint)
    rise_time = t[idx_90] - t[idx_10] if idx_90 > idx_10 else float('inf')

    # 超调量
    peak = np.max(y)
    overshoot = max(0, (peak - setpoint) / setpoint * 100)

    # 调整时间 (2%准则)
    settle_idx = len(t) - 1
    for i in range(len(t) - 1, -1, -1):
        if abs(y[i] - setpoint) > 0.02 * setpoint:
            settle_idx = min(i + 1, len(t) - 1)
            break
    settle_time = t[settle_idx]

    # IAE (误差绝对值积分)
    iae = _trapz(np.abs(error), t)

    # ITAE
    itae = _trapz(t * np.abs(error), t)

    # 稳态误差
    ss_error = np.mean(np.abs(error[-100:]))

    return {
        'rise_time': rise_time,
        'overshoot': overshoot,
        'settle_time': settle_time,
        'iae': iae,
        'itae': itae,
        'ss_error': ss_error
    }


# ============================================================
# 绘图
# ============================================================
def plot_all():
    """运行所有鲁棒性实验并绘图"""
    dt = 0.001
    kp, ki, kd = 2.0, 5.0, 0.1
    plt.rcParams['font.size'] = 10
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # ---- 实验1: 参数不确定性 (wn变化) ----
    ax = axes[0, 0]
    for wn_factor in [0.5, 0.75, 1.0, 1.25, 1.5, 2.0]:
        pp = PlantParams(wn=20.0 * wn_factor)
        t, y, _, sp = run_experiment(dt=dt, kp=kp, ki=ki, kd=kd, plant_params=pp)
        ax.plot(t, y, label=f'wn={20*wn_factor:.0f}')
    ax.plot(t, sp, 'k--', alpha=0.5)
    ax.set_title('模型不确定性: 自然频率变化')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.2, 1.8)

    # ---- 实验2: 阻尼比变化 ----
    ax = axes[0, 1]
    for zeta in [0.1, 0.2, 0.3, 0.5, 0.7, 1.0]:
        pp = PlantParams(zeta=zeta)
        t, y, _, sp = run_experiment(dt=dt, kp=kp, ki=ki, kd=kd, plant_params=pp)
        ax.plot(t, y, label=f'ζ={zeta}')
    ax.plot(t, sp, 'k--', alpha=0.5)
    ax.set_title('模型不确定性: 阻尼比变化')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.2, 1.8)

    # ---- 实验3: 阶跃扰动抑制 ----
    ax = axes[1, 0]
    for dist_mag in [0.0, 0.5, 1.0, 2.0, 5.0]:
        t, y, _, sp = run_experiment(dt=dt, kp=kp, ki=ki, kd=kd,
                                     disturbance_time=1.5,
                                     disturbance_mag=dist_mag)
        ax.plot(t, y, label=f'd={dist_mag}')
    ax.plot(t, sp, 'k--', alpha=0.5)
    ax.axvline(x=1.5, color='r', linestyle=':', alpha=0.5, label='扰动注入')
    ax.set_title('外部阶跃扰动抑制')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ---- 实验4: 不同整定方法对比 ----
    ax = axes[1, 1]
    # 假设通过实验测得 Ku=10, Tu=0.3
    Ku, Tu = 10.0, 0.3
    methods = {
        'Z-N Classic': 'classic',
        'Pessen': 'pessen',
        '少超调': 'some_overshoot',
        '无超调': 'no_overshoot',
    }
    for name, method in methods.items():
        kp_z, ki_z, kd_z = ziegler_nichols(Ku, Tu, method)
        t, y, _, sp = run_experiment(dt=dt, kp=kp_z, ki=ki_z, kd=kd_z)
        ax.plot(t, y, label=f'{name} (Kp={kp_z:.2f})')
    ax.plot(t, sp, 'k--', alpha=0.5)
    ax.set_title('Ziegler-Nichols 整定法则对比')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.2, 2.5)

    # ---- 实验5: 增益裕度分析 (Bode图) ----
    ax = axes[2, 0]
    plant = PlantParams()
    G = plant.transfer_function()

    # PID传递函数: C(s) = Kp + Ki/s + Kd*s
    C_num = [kd, kp, ki]
    C_den = [1, 0]
    C = signal.TransferFunction(C_num, C_den)

    # 开环传递函数 L = C * G
    L = signal.series(C, G)
    w, mag, phase = signal.bode(L, n=500)

    ax.semilogx(w/(2*np.pi), mag)
    ax.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    ax.set_title('开环Bode图 - 幅频特性')
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('增益 (dB)')
    ax.grid(True, alpha=0.3)

    ax2 = ax.twinx()
    ax2.semilogx(w/(2*np.pi), phase, 'g-')
    ax2.set_ylabel('相位 (°)', color='g')

    # ---- 实验6: 灵敏度函数 ----
    ax = axes[2, 1]

    # 灵敏度 S = 1/(1+L), 补灵敏度 T = L/(1+L)
    w = np.logspace(-1, 3, 1000)
    s = 1j * w
    # 被控对象频率响应
    G_jw = plant.gain * plant.wn**2 / (s**2 + 2*plant.zeta*plant.wn*s + plant.wn**2)
    # PID频率响应
    C_jw = kp + ki/s + kd*s
    L_jw = C_jw * G_jw

    S = 1 / (1 + L_jw)
    T = L_jw / (1 + L_jw)

    ax.semilogx(w/(2*np.pi), 20*np.log10(np.abs(S)), label='灵敏度 S(s)')
    ax.semilogx(w/(2*np.pi), 20*np.log10(np.abs(T)), label='补灵敏度 T(s)')
    ax.axhline(y=0, color='r', linestyle='--', alpha=0.3)
    ax.axhline(y=-6, color='orange', linestyle=':', alpha=0.5, label='-6dB')
    ax.set_title('灵敏度与补灵敏度函数')
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('幅值 (dB)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-40, 20)

    plt.tight_layout()
    plt.savefig('robust_pid_simulation.png', dpi=150, bbox_inches='tight')
    print("图表已保存: robust_pid_simulation.png")
    plt.close('all')


def print_robustness_report():
    """打印鲁棒性分析报告"""
    dt = 0.001
    kp, ki, kd = 2.0, 5.0, 0.1

    print("=" * 65)
    print("鲁棒PID性能分析报告")
    print("=" * 65)

    # 标称性能
    t, y, _, sp = run_experiment(dt=dt, kp=kp, ki=ki, kd=kd)
    m = calc_metrics(t, y, sp)
    print(f"\n标称性能 (wn=20, ζ=0.3):")
    print(f"  上升时间: {m['rise_time']*1000:.1f} ms")
    print(f"  超调量:   {m['overshoot']:.1f}%")
    print(f"  调整时间: {m['settle_time']*1000:.1f} ms")
    print(f"  IAE:      {m['iae']:.4f}")
    print(f"  稳态误差: {m['ss_error']:.6f}")

    # 参数灵敏度
    print(f"\n参数灵敏度分析:")
    print(f"  {'参数变化':<20} {'超调%':<10} {'调整时间':<12} {'IAE':<10}")
    print(f"  {'-'*52}")

    for factor in [0.5, 0.75, 1.0, 1.25, 1.5, 2.0]:
        pp = PlantParams(wn=20.0 * factor)
        t, y, _, sp = run_experiment(dt=dt, kp=kp, ki=ki, kd=kd, plant_params=pp)
        m = calc_metrics(t, y, sp)
        print(f"  wn×{factor:<16.2f} {m['overshoot']:<10.1f} "
              f"{m['settle_time']*1000:<12.1f} {m['iae']:<10.4f}")

    # 扰动抑制
    print(f"\n扰动抑制能力 (t=1.5s注入阶跃扰动):")
    print(f"  {'扰动幅度':<12} {'恢复时间':<12} {'最大偏差':<12}")
    print(f"  {'-'*36}")
    for d in [0.5, 1.0, 2.0, 5.0]:
        t, y, _, sp = run_experiment(dt=dt, kp=kp, ki=ki, kd=kd,
                                     disturbance_time=1.5, disturbance_mag=d)
        error = sp - y
        # 扰动后最大偏差
        idx_dist = int(1.5 / dt)
        max_dev = np.max(np.abs(error[idx_dist:]))
        print(f"  {d:<12.1f} {'--':<12} {max_dev:<12.4f}")


if __name__ == '__main__':
    print_robustness_report()
    plot_all()
