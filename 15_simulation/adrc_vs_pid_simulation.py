#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ADRC vs PID 对比仿真
====================
仿真内容：阶跃响应、扰动抑制、参数变化鲁棒性
被控对象：二阶系统 G(s) = K / (Ts^2 + bs + 1)
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

# ============ 仿真参数 ============
dt = 0.001          # 仿真步长
T_total = 5.0       # 仿真总时间
N = int(T_total / dt)
t = np.arange(N) * dt

# ============ 被控对象模型（二阶系统） ============
# 位置-速度状态: x = [位置, 速度]
# 标称参数
m_nominal = 1.0     # 质量
b_nominal = 0.5     # 阻尼
k_nominal = 1.0     # 刚度

def plant_step(x, u, m, b, k, dt):
    """二阶系统状态更新: m*x'' + b*x' + k*x = u"""
    pos, vel = x
    acc = (u - b * vel - k * pos) / m
    vel_new = vel + acc * dt
    pos_new = pos + vel_new * dt
    return np.array([pos_new, vel_new])

# ============ PID控制器 ============
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, ref, y, dt):
        error = ref - y
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

# ============ ADRC控制器（线性ADRC简化版） ============
class ADRCController:
    """线性自抗扰控制器 (LADRC)
    包含：跟踪微分器(TD)、扩张状态观测器(ESO)、状态误差反馈(SEF)
    """
    def __init__(self, wc, wo, b0):
        """
        wc: 控制器带宽
        wo: 观测器带宽
        b0: 对象增益估计
        """
        self.wc = wc
        self.wo = wo
        self.b0 = b0
        # ESO状态 [z1(位置估计), z2(速度估计), z3(总扰动估计)]
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0
        self.prev_y = 0.0

    def compute(self, ref, y, dt):
        """ADRC一步计算"""
        # 扩张状态观测器 (ESO) 更新
        e_eso = self.prev_y - self.z1
        self.z1 += (self.z2 + 3 * self.wo * e_eso) * dt
        self.z2 += (self.z3 + 3 * self.wo**2 * e_eso + self.b0 * self.u_prev if hasattr(self, 'u_prev') else 0) * dt
        self.z3 += (self.wo**3 * e_eso) * dt
        self.prev_y = y

        # 状态误差反馈 (SEF)
        # PD控制器 + 扰动补偿
        e1 = ref - self.z1
        e2 = 0 - self.z2  # 期望速度为0
        u0 = self.wc**2 * e1 + 2 * self.wc * e2
        u = (u0 - self.z3) / self.b0

        self.u_prev = u
        return u

    def reset(self):
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0
        if hasattr(self, 'u_prev'):
            del self.u_prev

# ============ 性能指标计算 ============
def calc_metrics(t, y, ref, t_start=0.0):
    """计算IAE/ISE/ITAE/超调量/调节时间"""
    idx_start = int(t_start / dt)
    error = ref - y
    abs_error = np.abs(error[idx_start:])
    t_sel = t[idx_start:]
    
    IAE = np.sum(abs_error) * dt
    ISE = np.sum(error[idx_start:]**2) * dt
    ITAE = np.sum(t_sel * abs_error) * dt
    
    # 超调量
    overshoot = 0.0
    if ref > 0:
        peak = np.max(y)
        overshoot = max(0, (peak - ref) / ref * 100)
    
    # 调节时间（2%准则）
    settling_time = t[-1]
    for i in range(len(y)-1, 0, -1):
        if np.abs(y[i] - ref) > 0.02 * ref:
            settling_time = t[min(i+1, len(t)-1)]
            break
    
    return {'IAE': IAE, 'ISE': ISE, 'ITAE': ITAE, 
            '超调量%': overshoot, '调节时间s': settling_time}

# ============ 仿真实验 ============
def run_simulation(controller, plant_params, ref_func, dist_func, dt, N):
    """运行一次仿真"""
    m, b, k = plant_params
    x = np.array([0.0, 0.0])
    y_log = np.zeros(N)
    u_log = np.zeros(N)
    
    for i in range(N):
        ref = ref_func(i * dt)
        y = x[0]
        u = controller.compute(ref, y, dt)
        d = dist_func(i * dt)
        u_total = u + d
        x = plant_step(x, u_total, m, b, k, dt)
        y_log[i] = x[0]
        u_log[i] = u_total
    
    return y_log, u_log

# ============ 实验1: 阶跃响应 ============
print("=" * 60)
print("实验1: 阶跃响应对比")
print("=" * 60)

pid = PIDController(Kp=10.0, Ki=5.0, Kd=3.0)
adrc = ADRCController(wc=10.0, wo=30.0, b0=1.0)
ref_func = lambda t: 1.0 if t >= 0.5 else 0.0
dist_func = lambda t: 0.0

y_pid1, u_pid1 = run_simulation(pid, (m_nominal, b_nominal, k_nominal), ref_func, dist_func, dt, N)
adrc.reset()
y_adrc1, u_adrc1 = run_simulation(adrc, (m_nominal, b_nominal, k_nominal), ref_func, dist_func, dt, N)

m_pid1 = calc_metrics(t, y_pid1, 1.0, 0.5)
m_adrc1 = calc_metrics(t, y_adrc1, 1.0, 0.5)
print(f"PID  - IAE:{m_pid1['IAE']:.3f} 超调:{m_pid1['超调量%']:.1f}% 调节时间:{m_pid1['调节时间s']:.3f}s")
print(f"ADRC - IAE:{m_adrc1['IAE']:.3f} 超调:{m_adrc1['超调量%']:.1f}% 调节时间:{m_adrc1['调节时间s']:.3f}s")

# ============ 实验2: 阶跃响应 + 阶跃扰动 ============
print("\n" + "=" * 60)
print("实验2: 阶跃扰动抑制 (t=2s施加0.5阶跃扰动)")
print("=" * 60)

pid.reset()
adrc2 = ADRCController(wc=10.0, wo=30.0, b0=1.0)
dist_func2 = lambda t: 0.5 if t >= 2.0 else 0.0

y_pid2, u_pid2 = run_simulation(pid, (m_nominal, b_nominal, k_nominal), ref_func, dist_func2, dt, N)
y_adrc2, u_adrc2 = run_simulation(adrc2, (m_nominal, b_nominal, k_nominal), ref_func, dist_func2, dt, N)

m_pid2 = calc_metrics(t, y_pid2, 1.0, 0.5)
m_adrc2 = calc_metrics(t, y_adrc2, 1.0, 0.5)
print(f"PID  - IAE:{m_pid2['IAE']:.3f}")
print(f"ADRC - IAE:{m_adrc2['IAE']:.3f}")

# ============ 实验3: 参数变化鲁棒性 ============
print("\n" + "=" * 60)
print("实验3: 参数变化鲁棒性 (质量变为2倍, 阻尼变为0.5倍)")
print("=" * 60)

m_var, b_var, k_var = 2.0, 0.25, 1.0
pid.reset()
adrc3 = ADRCController(wc=10.0, wo=30.0, b0=1.0)

y_pid3, u_pid3 = run_simulation(pid, (m_var, b_var, k_var), ref_func, dist_func, dt, N)
y_adrc3, u_adrc3 = run_simulation(adrc3, (m_var, b_var, k_var), ref_func, dist_func, dt, N)

m_pid3 = calc_metrics(t, y_pid3, 1.0, 0.5)
m_adrc3 = calc_metrics(t, y_adrc3, 1.0, 0.5)
print(f"PID  - IAE:{m_pid3['IAE']:.3f} 超调:{m_pid3['超调量%']:.1f}% 调节时间:{m_pid3['调节时间s']:.3f}s")
print(f"ADRC - IAE:{m_adrc3['IAE']:.3f} 超调:{m_adrc3['超调量%']:.1f}% 调节时间:{m_adrc3['调节时间s']:.3f}s")

# ============ 绘图 ============
fig, axes = plt.subplots(2, 3, figsize=(18, 10))
fig.suptitle('ADRC vs PID 对比仿真', fontsize=16, fontweight='bold')

# 实验1: 阶跃响应
ax = axes[0, 0]
ax.plot(t, y_pid1, 'b-', label='PID', linewidth=1.5)
ax.plot(t, y_adrc1, 'r--', label='ADRC', linewidth=1.5)
ax.axhline(y=1.0, color='k', linestyle=':', alpha=0.5, label='参考值')
ax.axvline(x=0.5, color='gray', linestyle=':', alpha=0.5)
ax.set_title('实验1: 阶跃响应')
ax.set_xlabel('时间 (s)')
ax.set_ylabel('输出')
ax.legend()
ax.grid(True, alpha=0.3)

# 实验1: 控制量
ax = axes[1, 0]
ax.plot(t, u_pid1, 'b-', label='PID', linewidth=1)
ax.plot(t, u_adrc1, 'r--', label='ADRC', linewidth=1)
ax.set_title('实验1: 控制量')
ax.set_xlabel('时间 (s)')
ax.set_ylabel('控制量 u')
ax.legend()
ax.grid(True, alpha=0.3)

# 实验2: 扰动抑制
ax = axes[0, 1]
ax.plot(t, y_pid2, 'b-', label='PID', linewidth=1.5)
ax.plot(t, y_adrc2, 'r--', label='ADRC', linewidth=1.5)
ax.axhline(y=1.0, color='k', linestyle=':', alpha=0.5, label='参考值')
ax.axvline(x=2.0, color='green', linestyle='--', alpha=0.5, label='扰动施加')
ax.set_title('实验2: 阶跃扰动抑制')
ax.set_xlabel('时间 (s)')
ax.set_ylabel('输出')
ax.legend()
ax.grid(True, alpha=0.3)

# 实验2: 误差
ax = axes[1, 1]
ax.plot(t, 1.0 - y_pid2, 'b-', label='PID误差', linewidth=1)
ax.plot(t, 1.0 - y_adrc2, 'r--', label='ADRC误差', linewidth=1)
ax.set_title('实验2: 跟踪误差')
ax.set_xlabel('时间 (s)')
ax.set_ylabel('误差')
ax.legend()
ax.grid(True, alpha=0.3)

# 实验3: 参数变化
ax = axes[0, 2]
ax.plot(t, y_pid3, 'b-', label='PID', linewidth=1.5)
ax.plot(t, y_adrc3, 'r--', label='ADRC', linewidth=1.5)
ax.axhline(y=1.0, color='k', linestyle=':', alpha=0.5, label='参考值')
ax.set_title('实验3: 参数变化鲁棒性')
ax.set_xlabel('时间 (s)')
ax.set_ylabel('输出')
ax.legend()
ax.grid(True, alpha=0.3)

# 性能指标柱状图
ax = axes[1, 2]
metrics_labels = ['IAE(正常)', 'IAE(扰动)', 'IAE(变参数)']
pid_vals = [m_pid1['IAE'], m_pid2['IAE'], m_pid3['IAE']]
adrc_vals = [m_adrc1['IAE'], m_adrc2['IAE'], m_adrc3['IAE']]
x_pos = np.arange(len(metrics_labels))
width = 0.35
bars1 = ax.bar(x_pos - width/2, pid_vals, width, label='PID', color='steelblue')
bars2 = ax.bar(x_pos + width/2, adrc_vals, width, label='ADRC', color='coral')
ax.set_title('IAE性能指标对比')
ax.set_xticks(x_pos)
ax.set_xticklabels(metrics_labels, fontsize=9)
ax.legend()
ax.grid(True, alpha=0.3, axis='y')
# 在柱上标注数值
for bar in bars1:
    ax.text(bar.get_x() + bar.get_width()/2., bar.get_height(), f'{bar.get_height():.2f}', ha='center', va='bottom', fontsize=8)
for bar in bars2:
    ax.text(bar.get_x() + bar.get_width()/2., bar.get_height(), f'{bar.get_height():.2f}', ha='center', va='bottom', fontsize=8)

plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'adrc_vs_pid_result.png'), dpi=150, bbox_inches='tight')
print("\n图表已保存: adrc_vs_pid_result.png")
plt.close('all')
print("ADRC vs PID 仿真完成!")

if __name__ == '__main__':
    run_simulation()
