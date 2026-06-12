"""
PID调参指南仿真
展示不同Kp/Ki/Kd参数对阶跃响应的影响
生成对比图表，帮助理解PID参数物理意义
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tests'))
from wrappers import PIDController


def simulate_pid(pid, setpoint, steps, dt, plant_gain=1.0, plant_tau=0.1):
    """一阶惯性环节 + PID仿真"""
    plant_state = 0.0
    outputs = []
    states = []
    for _ in range(steps):
        u = pid.calc(setpoint, plant_state)
        # 一阶惯性: tau * dy/dt + y = K*u
        plant_state += (plant_gain * u - plant_state) / plant_tau * dt
        outputs.append(u)
        states.append(plant_state)
    return states, outputs


def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False

    dt = 0.01
    steps = 2000
    setpoint = 10.0
    t = np.arange(steps) * dt

    # ═══════════════════════════════════════════════════════
    # 1. Kp对比: 不同比例增益
    # ═══════════════════════════════════════════════════════
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('PID调参指南仿真', fontsize=16, fontweight='bold')

    kp_values = [0.5, 1.0, 2.0, 5.0, 10.0]
    ax = axes[0, 0]
    for kp in kp_values:
        pid = PIDController(kp=kp, ki=0.0, kd=0.0, output_max=100.0)
        states, _ = simulate_pid(pid, setpoint, steps, dt)
        ax.plot(t, states, label=f'Kp={kp}')
    ax.axhline(y=setpoint, color='k', linestyle='--', alpha=0.5, label='设定值')
    ax.set_title('比例增益Kp对比 (Ki=0, Kd=0)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════
    # 2. Ki对比: 不同积分增益
    # ═══════════════════════════════════════════════════════
    ax = axes[0, 1]
    ki_values = [0.1, 0.5, 1.0, 2.0, 5.0]
    for ki in ki_values:
        pid = PIDController(kp=2.0, ki=ki, kd=0.0, output_max=100.0, integral_max=500.0)
        states, _ = simulate_pid(pid, setpoint, steps, dt)
        ax.plot(t, states, label=f'Ki={ki}')
    ax.axhline(y=setpoint, color='k', linestyle='--', alpha=0.5, label='设定值')
    ax.set_title('积分增益Ki对比 (Kp=2, Kd=0)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════
    # 3. Kd对比: 不同微分增益
    # ═══════════════════════════════════════════════════════
    ax = axes[1, 0]
    kd_values = [0.0, 0.1, 0.5, 1.0, 2.0]
    for kd in kd_values:
        pid = PIDController(kp=2.0, ki=1.0, kd=kd, output_max=100.0, integral_max=500.0)
        states, _ = simulate_pid(pid, setpoint, steps, dt)
        ax.plot(t, states, label=f'Kd={kd}')
    ax.axhline(y=setpoint, color='k', linestyle='--', alpha=0.5, label='设定值')
    ax.set_title('微分增益Kd对比 (Kp=2, Ki=1)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════
    # 4. 抗积分饱和对比
    # ═══════════════════════════════════════════════════════
    ax = axes[1, 1]
    limits = [10.0, 20.0, 50.0, 200.0]
    for lim in limits:
        pid = PIDController(kp=2.0, ki=5.0, kd=0.0, output_max=lim, integral_max=lim * 5)
        states, _ = simulate_pid(pid, setpoint, steps, dt)
        ax.plot(t, states, label=f'输出限幅={lim}')
    ax.axhline(y=setpoint, color='k', linestyle='--', alpha=0.5, label='设定值')
    ax.set_title('抗积分饱和 (不同输出限幅)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════
    # 5. 死区效果
    # ═══════════════════════════════════════════════════════
    ax = axes[2, 0]
    dz_values = [0.0, 0.1, 0.5, 1.0]
    for dz in dz_values:
        pid = PIDController(kp=2.0, ki=1.0, kd=0.5, output_max=100.0,
                            integral_max=500.0, dead_zone=dz)
        states, _ = simulate_pid(pid, setpoint, steps, dt)
        ax.plot(t, states, label=f'死区={dz}')
    ax.axhline(y=setpoint, color='k', linestyle='--', alpha=0.5, label='设定值')
    ax.set_title('死区效果对比')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════
    # 6. 推荐参数总结
    # ═══════════════════════════════════════════════════════
    ax = axes[2, 1]
    ax.axis('off')
    summary = """
    PID调参指南总结
    ═══════════════════════════════════════

    Kp (比例增益):
      ↑ 增大 → 响应加快, 但可能振荡/超调
      ↓ 减小 → 响应变慢, 稳态误差增大

    Ki (积分增益):
      ↑ 增大 → 消除稳态误差更快
               但可能导致积分饱和/超调
      ↓ 减小 → 稳态误差消除慢

    Kd (微分增益):
      ↑ 增大 → 抑制超调, 改善动态品质
               但对噪声敏感
      ↓ 减小 → 超调增大

    推荐调参步骤:
      1. 先调Kp: 从小到大, 直到振荡
      2. 再调Ki: 从小到大, 消除稳态误差
      3. 最后调Kd: 抑制超调和振荡
      4. 开启抗积分饱和

    """
    ax.text(0.05, 0.95, summary, transform=ax.transAxes,
            fontsize=10, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(out_dir, 'pid_tuning_guide.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[OK] 图表已保存: {out_path}")


if __name__ == '__main__':
    main()
