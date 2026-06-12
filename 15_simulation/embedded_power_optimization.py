#!/usr/bin/env python3
"""嵌入式功耗优化仿真 — 睡眠模式 / 时钟门控 / 电压频率调节(DVFS)"""
import numpy as np, matplotlib.pyplot as plt, os
plt.rcParams['font.sans-serif'] = ['SimHei','Microsoft YaHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# MCU参数
V_nom = 3.3  # V
I_active = 20e-3   # 20mA active
I_sleep = 50e-6    # 50μA sleep
I_deep = 2e-6      # 2μA deep sleep
t_wake = 0.1e-3    # 唤醒延迟 100μs
P_periph = {'ADC': 2e-3, 'UART': 1e-3, 'SPI': 1.5e-3, 'Timer': 0.5e-3, 'GPIO': 0.2e-3}

dt = 0.001  # 1ms时间步
T_sim = 5.0  # 5秒仿真
t = np.arange(0, T_sim, dt)
N = len(t)

# ── 工作模式定义 ──
def generate_task_pattern():
    """任务调度模式：采集->计算->通信->空闲"""
    pattern = np.zeros(N, dtype=int)  # 0=空闲 1=采集 2=计算 3=通信
    cycle = int(1.0 / dt)  # 1秒周期
    for i in range(N):
        phase = (i % cycle) / cycle
        if phase < 0.1:
            pattern[i] = 1  # 采集 10%
        elif phase < 0.4:
            pattern[i] = 2  # 计算 30%
        elif phase < 0.5:
            pattern[i] = 3  # 通信 10%
        # else 空闲 50%
    return pattern

tasks = generate_task_pattern()

# ── 策略1: 无优化 (始终活跃) ──
P_naive = np.where(tasks > 0, I_active * V_nom, I_sleep * V_nom)

# ── 策略2: 睡眠模式 ──
P_sleep = np.zeros(N)
for i in range(N):
    if tasks[i] == 0:
        P_sleep[i] = I_deep * V_nom
    elif tasks[i] == 1:
        P_sleep[i] = (I_active + P_periph['ADC'] + P_periph['Timer']) 
    elif tasks[i] == 2:
        P_sleep[i] = I_active * V_nom
    else:
        P_sleep[i] = (I_active + P_periph['UART'] + P_periph['SPI'])

# ── 策略3: 时钟门控 ──
P_cg = np.zeros(N)
for i in range(N):
    if tasks[i] == 0:
        P_cg[i] = I_deep * V_nom  # 关闭时钟+深度睡眠
    elif tasks[i] == 1:
        P_cg[i] = (I_active + P_periph['ADC']) * V_nom  # 仅开ADC时钟
    elif tasks[i] == 2:
        P_cg[i] = I_active * V_nom * 0.7  # 降低非核心外设时钟
    else:
        P_cg[i] = (I_active + P_periph['UART']) * V_nom  # 仅开UART

# ── 策略4: DVFS (动态电压频率调节) ──
P_dvfs = np.zeros(N)
for i in range(N):
    if tasks[i] == 0:
        P_dvfs[i] = I_deep * V_nom
    elif tasks[i] == 1:
        # 低电压低频率采集
        V_low = 1.8; f_ratio = 0.5
        P_dvfs[i] = (I_active * f_ratio + P_periph['ADC']) * V_low
    elif tasks[i] == 2:
        # 高性能计算
        V_high = 3.3
        P_dvfs[i] = I_active * V_high * 1.2  # boost频率
    else:
        V_low = 2.5
        P_dvfs[i] = (I_active * 0.7 + P_periph['UART']) * V_low

# ── 策略5: 全优化组合 ──
P_opt = np.zeros(N)
for i in range(N):
    if tasks[i] == 0:
        P_opt[i] = I_deep * V_nom
    elif tasks[i] == 1:
        # DVFS + 时钟门控 + 仅必要外设
        P_opt[i] = (I_active * 0.4 + P_periph['ADC']) * 1.8
    elif tasks[i] == 2:
        P_opt[i] = I_active * 3.3 * 0.8  # 适度boost
    else:
        P_opt[i] = (I_active * 0.5 + P_periph['UART']) * 2.0

# 计算能耗
strategies = {'无优化': P_naive, '睡眠模式': P_sleep, '时钟门控': P_cg,
              'DVFS': P_dvfs, '全优化': P_opt}
energy = {k: np.sum(v) * dt for k, v in strategies.items()}
avg_power = {k: np.mean(v) * 1000 for k, v in strategies.items()}  # mW

# 绘图
fig, axes = plt.subplots(2, 3, figsize=(16, 10))
fig.suptitle('嵌入式功耗优化仿真', fontsize=14, fontweight='bold')

colors = ['#F44336', '#2196F3', '#4CAF50', '#FF9800', '#9C27B0']

# 各策略功耗时间线
for (name, pw), c in zip(strategies.items(), colors):
    axes[0,0].plot(t[:2000], pw[:2000]*1000, c, label=name, alpha=0.7)
axes[0,0].set_title('功耗时间线 (前2秒)'); axes[0,0].set_ylabel('Power (mW)')
axes[0,0].legend(fontsize=7); axes[0,0].set_xlabel('t (s)')

# 平均功耗对比
bars = axes[0,1].bar(avg_power.keys(), avg_power.values(), color=colors)
axes[0,1].set_title('平均功耗对比'); axes[0,1].set_ylabel('Power (mW)')
for bar, v in zip(bars, avg_power.values()):
    axes[0,1].text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.1, f'{v:.2f}', ha='center', fontsize=8)

# 5秒总能耗
bars2 = axes[0,2].bar(energy.keys(), [e*1000 for e in energy.values()], color=colors)
axes[0,2].set_title('5秒总能耗'); axes[0,2].set_ylabel('Energy (mJ)')
for bar, v in zip(bars2, energy.values()):
    axes[0,2].text(bar.get_x()+bar.get_width()/2, bar.get_height()*1000+0.05, f'{v*1000:.2f}', ha='center', fontsize=8)

# 节能百分比
baseline = energy['无优化']
savings = {k: (1 - v/baseline)*100 for k, v in energy.items()}
bars3 = axes[1,0].bar(savings.keys(), savings.values(), color=colors)
axes[1,0].set_title('节能比例 (%)'); axes[1,0].set_ylabel('Saving (%)')
for bar, v in zip(bars3, savings.values()):
    axes[1,0].text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.5, f'{v:.1f}%', ha='center', fontsize=8)

# 电池续航估算 (1000mAh电池)
battery_mAh = 1000
lifetime_h = {k: battery_mAh / (v / V_nom) for k, v in avg_power.items()}  # P(mW)/V=I(mA), mAh/mA=h
bars4 = axes[1,1].bar(lifetime_h.keys(), lifetime_h.values(), color=colors)
axes[1,1].set_title('电池续航估算 (1000mAh)'); axes[1,1].set_ylabel('时间 (h)')
for bar, v in zip(bars4, lifetime_h.values()):
    axes[1,1].text(bar.get_x()+bar.get_width()/2, bar.get_height()+1, f'{v:.0f}h', ha='center', fontsize=8)

# 任务占比饼图
task_labels = ['采集(10%)', '计算(30%)', '通信(10%)', '空闲(50%)']
task_counts = [np.sum(tasks==i)/N*100 for i in range(4)]
axes[1,2].pie(task_counts, labels=task_labels, autopct='%1.0f%%', startangle=90)
axes[1,2].set_title('任务时间占比')

plt.tight_layout()
out = os.path.join(os.path.dirname(__file__), 'embedded_power_optimization_result.png')
plt.savefig(out, dpi=150); print(f'已保存: {out}')
print('平均功耗 (mW):', {k: f'{v:.2f}' for k, v in avg_power.items()})
print('节能比例 (%):', {k: f'{v:.1f}' for k, v in savings.items()})
print('电池续航 (h):', {k: f'{v:.0f}' for k, v in lifetime_h.items()})
