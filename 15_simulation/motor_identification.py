"""
电机参数辨识仿真
方法：阶跃响应法 + 最小二乘法
应用场景：直流电机的传递函数参数辨识 K, tau, theta（增益、时间常数、延迟）
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
# scipy可能不可用，使用numpy实现最小二乘


def main():
    try:
        from scipy.optimize import least_squares
        HAS_SCIPY = True
    except ImportError:
        HAS_SCIPY = False

    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False

    # ============================================================
    # 1. 电机真实模型（一阶惯性+延迟）: G(s) = K * e^(-theta*s) / (tau*s + 1)
    # ============================================================
    K_true = 2.5      # 真实增益
    tau_true = 0.3     # 真实时间常数(s)
    theta_true = 0.05  # 真实延迟(s)

    dt = 0.001         # 采样周期(s)
    t_end = 2.0        # 仿真时长(s)
    t = np.arange(0, t_end, dt)

    # 阶跃输入（幅值1V）
    u = np.ones_like(t)
    u[t < 0.1] = 0  # 0.1s时施加阶跃

    # 模拟真实电机响应（带噪声）
    y_true = np.zeros_like(t)
    delay_steps = int(theta_true / dt)
    for i in range(1, len(t)):
        if i > delay_steps:
            y_true[i] = y_true[i-1] + dt/tau_true * (K_true * u[i-delay_steps] - y_true[i-1])

    np.random.seed(42)
    noise = np.random.normal(0, 0.02, len(t))  # 测量噪声
    y_measured = y_true + noise

    # ============================================================
    # 2. 阶跃响应法辨识（两点法）
    # ============================================================
    def step_response_identify(t, y, u_step=1.0):
        """
        阶跃响应两点法辨识一阶惯性+延迟模型
        y(t) = K*(1 - exp(-(t-theta)/tau)) * u_step
        """
        y_final = np.mean(y[-200:])           # 稳态值
        K_est = y_final / u_step               # 增益估计

        # 找到 y = 0.283*K 和 y = 0.632*K 对应的时间
        y_283 = 0.283 * y_final
        y_632 = 0.632 * y_final

        idx_283 = np.argmax(y > y_283)
        idx_632 = np.argmax(y > y_632)
        t_283 = t[idx_283]
        t_632 = t[idx_632]

        tau_est = 1.5 * (t_632 - t_283)
        theta_est = t_632 - tau_est

        return K_est, tau_est, max(theta_est, 0)

    K_s, tau_s, theta_s = step_response_identify(t, y_measured)
    print(f"[阶跃响应法] K={K_s:.3f}, tau={tau_s:.3f}s, theta={theta_s:.3f}s")
    print(f"  真实值:      K={K_true:.3f}, tau={tau_true:.3f}s, theta={theta_true:.3f}s")

    # ============================================================
    # 3. 最小二乘法辨识（ARX模型）
    # ============================================================
    def simulate_first_order(params, t, u, dt):
        """根据参数模拟一阶响应"""
        K, tau, theta = params
        y = np.zeros_like(t)
        delay_steps = max(int(theta / dt), 0)
        for i in range(1, len(t)):
            if i > delay_steps:
                y[i] = y[i-1] + dt/tau * (K * u[i-delay_steps] - y[i-1])
        return y

    def residual(params, t, u, y_meas, dt):
        """残差函数"""
        y_sim = simulate_first_order(params, t, u, dt)
        return y_sim - y_meas

    # 初始猜测
    x0 = [2.0, 0.2, 0.03]
    bounds_lo = [0.1, 0.01, 0.0]
    bounds_hi = [10.0, 2.0, 0.5]

    if HAS_SCIPY:
        result = least_squares(residual, x0, bounds=(bounds_lo, bounds_hi), args=(t, u, y_measured, dt))
        K_ls, tau_ls, theta_ls = result.x
    else:
        # 简单网格搜索替代最小二乘
        best_sse = float('inf')
        best_params = x0
        for K_try in np.linspace(bounds_lo[0], bounds_hi[0], 20):
            for tau_try in np.linspace(bounds_lo[1], bounds_hi[1], 20):
                for theta_try in np.linspace(bounds_lo[2], bounds_hi[2], 10):
                    y_try = simulate_first_order([K_try, tau_try, theta_try], t, u, dt)
                    sse = np.sum((y_try - y_measured)**2)
                    if sse < best_sse:
                        best_sse = sse
                        best_params = [K_try, tau_try, theta_try]
        K_ls, tau_ls, theta_ls = best_params
    print(f"[最小二乘法] K={K_ls:.3f}, tau={tau_ls:.3f}s, theta={theta_ls:.3f}s")

    # 用辨识结果重建响应
    y_step = simulate_first_order([K_s, tau_s, theta_s], t, u, dt)
    y_ls = simulate_first_order([K_ls, tau_ls, theta_ls], t, u, dt)

    # ============================================================
    # 4. 绘图
    # ============================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 图1：阶跃响应辨识对比
    ax = axes[0, 0]
    ax.plot(t, y_measured, 'gray', alpha=0.5, label='测量值（含噪声）')
    ax.plot(t, y_true, 'b-', linewidth=2, label='真实响应')
    ax.plot(t, y_step, 'r--', linewidth=2, label=f'阶跃法: K={K_s:.2f}, τ={tau_s:.2f}s')
    ax.plot(t, y_ls, 'g-.', linewidth=2, label=f'最小二乘: K={K_ls:.2f}, τ={tau_ls:.2f}s')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('转速 (rpm)')
    ax.set_title('电机阶跃响应辨识对比')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 1.5])

    # 图2：辨识误差
    ax = axes[0, 1]
    err_step = y_step - y_true
    err_ls = y_ls - y_true
    ax.plot(t, err_step, 'r-', label=f'阶跃法 RMSE={np.sqrt(np.mean(err_step**2)):.4f}')
    ax.plot(t, err_ls, 'g-', label=f'最小二乘 RMSE={np.sqrt(np.mean(err_ls**2)):.4f}')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('误差')
    ax.set_title('辨识误差分析')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0.1, 1.5])

    # 图3：参数灵敏度分析
    ax = axes[1, 0]
    K_range = np.linspace(1.5, 3.5, 50)
    sse_K = []
    for K_try in K_range:
        y_try = simulate_first_order([K_try, tau_ls, theta_ls], t, u, dt)
        sse_K.append(np.sum((y_try - y_measured)**2))
    ax.plot(K_range, sse_K, 'b-', linewidth=2)
    ax.axvline(K_true, color='r', linestyle='--', label=f'K_true={K_true}')
    ax.axvline(K_ls, color='g', linestyle='-.', label=f'K_est={K_ls:.2f}')
    ax.set_xlabel('增益 K')
    ax.set_ylabel('误差平方和')
    ax.set_title('增益参数灵敏度分析')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 图4：Bode图对比
    ax = axes[1, 1]
    freq = np.logspace(-1, 2, 500)
    omega = 2 * np.pi * freq

    def bode_first_order(K, tau, theta, omega):
        """一阶+延迟系统的Bode图数据"""
        mag = K / np.sqrt(1 + (omega * tau)**2)
        phase = -np.arctan(omega * tau) - omega * theta
        return 20 * np.log10(mag), np.degrees(phase)

    mag_true, phase_true = bode_first_order(K_true, tau_true, theta_true, omega)
    mag_est, phase_est = bode_first_order(K_ls, tau_ls, theta_ls, omega)

    ax.semilogx(freq, mag_true, 'b-', linewidth=2, label='真实模型')
    ax.semilogx(freq, mag_est, 'r--', linewidth=2, label='辨识模型')
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('幅值 (dB)')
    ax.set_title('Bode图对比（幅频特性）')
    ax.legend()
    ax.grid(True, alpha=0.3, which='both')

    plt.suptitle('电机参数辨识仿真', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motor_identification.png'), dpi=150, bbox_inches='tight')
    print("图表已保存: motor_identification.png")
    plt.close('all')



if __name__ == '__main__':
    main()
