"""
PID调参方法仿真 (PID Tuning Methods Simulation)
=================================================
5种经典PID整定方法对比：
1. Ziegler-Nichols 临界比例法
2. Ziegler-Nichols 阶跃响应法
3. Cohen-Coon 法
4. IMC (内模控制) 法
5. Lambda 整定法

Author: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


def plant_response(u_seq, dt, K=1.0, tau=2.0, theta=0.5):
    """一阶加纯滞后系统响应: G(s) = K*e^(-theta*s)/(tau*s+1)"""
    N = len(u_seq)
    y = np.zeros(N)
    delay_steps = max(1, int(theta / dt))
    for i in range(1, N):
        u_delayed = u_seq[max(0, i - delay_steps)]
        y[i] = y[i-1] + dt / tau * (K * u_delayed - y[i-1])
    return y


def ziegler_nichols_step(K, tau, theta):
    """Z-N阶跃响应法（反应曲线法）"""
    a = K * theta / tau
    Kp = 1.2 / a
    Ti = 2.0 * theta
    Td = 0.5 * theta
    return Kp, Kp/Ti, Kp*Td


def ziegler_nichols_ultimate(Ku, Tu):
    """Z-N临界比例法"""
    Kp = 0.6 * Ku
    Ti = 0.5 * Tu
    Td = 0.125 * Tu
    return Kp, Kp/Ti, Kp*Td


def cohen_coon(K, tau, theta):
    """Cohen-Coon整定法"""
    r = theta / tau
    Kp = (1.0 + 0.35 * r / (1 - r)) / (K * r) if r < 1 else 1.0 / (K * r)
    Ti = theta * (3.3 + 3.3 * r) / (1 + 11.2 * r)
    Td = theta * 0.27 / (1 + 0.13 * r)
    return Kp, Kp/Ti, Kp*Td


def imc_tuning(K, tau, theta, lambda_c=None):
    """IMC内模控制整定法"""
    if lambda_c is None:
        lambda_c = max(0.8 * theta, 0.1 * tau)
    Kp = (2*tau + theta) / (2*K*lambda_c)
    Ti = tau + theta/2
    Td = tau * theta / (2*tau + theta)
    return Kp, Kp/Ti, Kp*Td


def lambda_tuning(K, tau, theta, lambda_c=None):
    """Lambda整定法"""
    if lambda_c is None:
        lambda_c = 2.0 * tau  # 保守选择
    Kp = tau / (K * (lambda_c + theta))
    Ti = tau
    Td = 0.0
    return Kp, Kp/Ti, 0.0


def simulate_pid(Kp, Ki, Kd, setpoint, dt, K=1.0, tau=2.0, theta=0.5, T=20.0):
    """仿真PID控制系统"""
    N = int(T / dt)
    y = np.zeros(N)
    u = np.zeros(N)
    e_int = 0.0
    e_prev = 0.0

    for i in range(N):
        error = setpoint[i] - y[i]
        e_int += error * dt
        de = (error - e_prev) / dt
        e_prev = error

        u[i] = Kp * error + Ki * e_int + Kd * de
        u[i] = np.clip(u[i], -50, 50)

        # 仿真被控对象
        if i > 0:
            delay_steps = max(1, int(theta / dt))
            u_delayed = u[max(0, i - delay_steps)]
            y[i] = y[i-1] + dt / tau * (K * u_delayed - y[i-1])

    return y, u


def evaluate_performance(y, ref, dt):
    """评估控制性能"""
    e = ref - y
    iae = np.sum(np.abs(e)) * dt
    ise = np.sum(e**2) * dt
    overshoot = (np.max(y) - ref[-1]) / max(ref[-1], 0.01) * 100 if ref[-1] > 0 else 0
    # 上升时间
    idx_10 = np.argmax(y >= 0.1 * ref[-1]) if ref[-1] > 0 else 0
    idx_90 = np.argmax(y >= 0.9 * ref[-1]) if ref[-1] > 0 else 0
    rise_time = (idx_90 - idx_10) * dt if idx_90 > idx_10 else float('inf')
    return {'IAE': iae, 'ISE': ise, 'overshoot': overshoot, 'rise_time': rise_time}


if __name__ == '__main__':
    dt = 0.01
    T = 25.0
    N = int(T / dt)
    K, tau, theta = 1.0, 2.0, 0.5

    ref = np.ones(N)  # 阶跃参考

    methods = {
        'Z-N 阶跃响应法': ziegler_nichols_step(K, tau, theta),
        'Z-N 临界比例法': ziegler_nichols_ultimate(2.8, 3.2),
        'Cohen-Coon': cohen_coon(K, tau, theta),
        'IMC 内模控制': imc_tuning(K, tau, theta),
        'Lambda 整定法': lambda_tuning(K, tau, theta),
    }

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    colors = ['b', 'r', 'g', 'm', 'c']
    t = np.linspace(0, T, N)
    results = {}

    for idx, (name, (Kp, Ki, Kd)) in enumerate(methods.items()):
        y, u = simulate_pid(Kp, Ki, Kd, ref, dt, K, tau, theta, T)
        perf = evaluate_performance(y, ref, dt)
        results[name] = perf

        axes[0, 0].plot(t, y, colors[idx], lw=1.3, label=f'{name}')
        axes[0, 1].plot(t, u, colors[idx], lw=1.0, label=f'{name}')

    axes[0, 0].plot(t, ref, 'k--', lw=1.5, label='参考')
    axes[0, 0].set_title('阶跃响应对比')
    axes[0, 0].set_ylabel('输出')
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].set_title('控制量对比')
    axes[0, 1].set_ylabel('控制量 u')
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True, alpha=0.3)

    # 性能指标柱状图
    names = list(results.keys())
    metrics = ['IAE', 'ISE', 'overshoot', 'rise_time']
    metric_labels = ['IAE', 'ISE', '超调量(%)', '上升时间(s)']

    for j, (metric, label) in enumerate(zip(metrics, metric_labels)):
        ax = axes[1, j // 2]
        vals = [results[n][metric] for n in names]
        bars = ax.bar(range(len(names)), vals, color=colors[:len(names)])
        ax.set_xticks(range(len(names)))
        ax.set_xticklabels([n.split(' ')[0] for n in names], fontsize=8, rotation=15)
        ax.set_title(label)
        ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig('pid_tuning_methods_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    # 打印结果表
    print(f"\n{'方法':<18} {'Kp':>8} {'Ki':>8} {'Kd':>8} {'IAE':>10} {'超调%':>8} {'上升时间':>10}")
    print('-' * 80)
    for name, (Kp, Ki, Kd) in methods.items():
        p = results[name]
        print(f"{name:<18} {Kp:>8.3f} {Ki:>8.3f} {Kd:>8.3f} {p['IAE']:>10.4f} {p['overshoot']:>8.1f} {p['rise_time']:>10.3f}")
