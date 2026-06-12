"""
阻抗控制仿真 - 力/位置混合控制 + 柔顺控制
"""
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ========== 阻抗控制器 ==========
class ImpedanceController:
    """二阶阻抗控制器: M*ẍ + B*ẋ + K*(x-xd) = F_ext"""
    def __init__(self, M, B, K, dt):
        self.M = M
        self.B = B
        self.K = K
        self.dt = dt

    def compute(self, x, xdot, xd, F_ext):
        """计算期望加速度"""
        xdd = (F_ext - self.B * xdot - self.K * (x - xd)) / self.M
        return xdd


# ========== 环境模型 ==========
class ContactEnvironment:
    """弹簧-阻尼接触环境"""
    def __init__(self, Ke, Be, x_surface):
        self.Ke = Ke  # 环境刚度
        self.Be = Be  # 环境阻尼
        self.x_surface = x_surface  # 接触面位置

    def force(self, x, xdot):
        if x >= self.x_surface:
            F = self.Ke * (x - self.x_surface) + self.Be * max(0, xdot)
        else:
            F = 0
        return F


# ========== 仿真: 不同阻抗参数 ==========
def impedance_parameter_study():
    """不同阻抗参数对柔顺控制的影响"""
    dt = 0.001
    T = 2.0
    N = int(T / dt)
    t = np.arange(N) * dt

    # 环境
    env = ContactEnvironment(Ke=5000, Be=50, x_surface=0.5)

    # 期望轨迹: 从0到0.6匀速运动(穿过接触面)
    xd = 0.6
    v_desired = 0.3  # m/s

    # 不同阻抗参数 [M, B, K]
    params = {
        '刚性': (1.0, 200, 1000),
        '中等': (1.0, 100, 200),
        '柔顺': (1.0, 50, 50),
        '过柔顺': (1.0, 20, 10),
    }

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('阻抗控制 - 不同阻抗参数对比', fontsize=14, fontweight='bold')

    for (name, (M, B, K)), ax_pos, ax_force in zip(params.items(), axes[0], axes[1]):
        x = 0.0
        xdot = v_desired
        x_hist, F_hist, xd_hist = [], [], []

        for i in range(N):
            F_ext = env.force(x, xdot)
            imp = ImpedanceController(M, B, K, dt)
            xdd = imp.compute(x, xdot, xd, F_ext)
            xdot += xdd * dt
            x += xdot * dt
            # 期望位置随时间移动
            xd_t = min(0.6, v_desired * t[i])
            x_hist.append(x)
            F_hist.append(F_ext)
            xd_hist.append(xd_t)

        ax_pos.plot(t, xd_hist, 'k--', linewidth=1.5, label='期望位置')
        ax_pos.plot(t, x_hist, linewidth=2, label=f'{name}(M={M},B={B},K={K})')
        ax_pos.axhline(0.5, color='red', linestyle=':', alpha=0.5, label='接触面')
        ax_pos.set_title(f'{name} - 位置响应', fontsize=11)
        ax_pos.set_xlabel('时间 (s)')
        ax_pos.set_ylabel('位置 (m)')
        ax_pos.legend(fontsize=8)
        ax_pos.grid(True, alpha=0.3)

        ax_force.plot(t, F_hist, linewidth=1.5, label=f'{name}')
        ax_force.set_title(f'{name} - 接触力', fontsize=11)
        ax_force.set_xlabel('时间 (s)')
        ax_force.set_ylabel('力 (N)')
        ax_force.legend()
        ax_force.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('impedance_parameter_study_result.png', dpi=150)
    plt.close()
    print("✅ 阻抗参数研究完成")


# ========== 力/位置混合控制 ==========
def force_position_hybrid():
    """力/位置混合控制"""
    dt = 0.001
    T = 3.0
    N = int(T / dt)
    t = np.arange(N) * dt

    # 环境
    env = ContactEnvironment(Ke=10000, Be=100, x_surface=0.3)

    # 期望: 力控制目标
    F_desired = 10.0  # N

    # 位置控制参数
    Kp_pos = 500
    Kd_pos = 50

    # 力控制参数
    Kp_force = 0.001
    Ki_force = 0.01

    results = {}

    # === 位置控制 ===
    x = 0.0; xdot = 0.0
    x_pos_hist, F_pos_hist = [], []
    for i in range(N):
        xd_t = min(0.5, 0.2 * t[i])
        F_ext = env.force(x, xdot)
        xdd = Kp_pos * (xd_t - x) - Kd_pos * xdot - F_ext
        xdot += xdd * dt
        x += xdot * dt
        x_pos_hist.append(x)
        F_pos_hist.append(F_ext)
    results['位置控制'] = (np.array(x_pos_hist), np.array(F_pos_hist))

    # === 力控制 ===
    x = 0.25; xdot = 0.0
    x_force_hist, F_force_hist = [], []
    e_fi = 0
    for i in range(N):
        F_ext = env.force(x, xdot)
        e_f = F_desired - F_ext
        e_fi += e_f * dt
        xdd = Kp_force * e_f + Ki_force * e_fi
        xdot += xdd * dt
        x += xdot * dt
        x_force_hist.append(x)
        F_force_hist.append(F_ext)
    results['力控制'] = (np.array(x_force_hist), np.array(F_force_hist))

    # === 混合控制 (位置X + 力Y) ===
    # 简化为1D: 先位置控制靠近，再切换力控制
    x = 0.0; xdot = 0.0
    x_hybrid_hist, F_hybrid_hist = [], []
    mode = 'position'
    e_fi = 0
    for i in range(N):
        F_ext = env.force(x, xdot)

        if mode == 'position' and F_ext > 0.5:
            mode = 'force'
            e_fi = 0

        if mode == 'position':
            xd_t = min(0.5, 0.3 * t[i])
            xdd = Kp_pos * (xd_t - x) - Kd_pos * xdot
        else:
            e_f = F_desired - F_ext
            e_fi += e_f * dt
            xdd = 0.002 * e_f + 0.02 * e_fi

        xdot += xdd * dt
        x += xdot * dt
        x_hybrid_hist.append(x)
        F_hybrid_hist.append(F_ext)
    results['混合控制'] = (np.array(x_hybrid_hist), np.array(F_hybrid_hist))

    # 绘图
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('力/位置混合控制对比', fontsize=14, fontweight='bold')

    colors = {'位置控制': 'blue', '力控制': 'red', '混合控制': 'green'}
    for name, (xh, fh) in results.items():
        axes[0, 0].plot(t, xh, linewidth=1.5, label=name, color=colors[name])
        axes[0, 1].plot(t, fh, linewidth=1.5, label=name, color=colors[name])

    axes[0, 0].axhline(0.3, color='orange', linestyle='--', alpha=0.5, label='接触面')
    axes[0, 0].set_title('位置响应', fontsize=12)
    axes[0, 0].set_xlabel('时间 (s)')
    axes[0, 0].set_ylabel('位置 (m)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].axhline(F_desired, color='orange', linestyle='--', alpha=0.5, label=f'目标力={F_desired}N')
    axes[0, 1].set_title('接触力', fontsize=12)
    axes[0, 1].set_xlabel('时间 (s)')
    axes[0, 1].set_ylabel('力 (N)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # 力-位移关系
    for name, (xh, fh) in results.items():
        mask = fh > 0
        axes[1, 0].plot(xh[mask], fh[mask], linewidth=1.5, label=name, color=colors[name])
    axes[1, 0].set_title('力-位移特性', fontsize=12)
    axes[1, 0].set_xlabel('位移 (m)')
    axes[1, 0].set_ylabel('力 (N)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 不同环境刚度下的柔顺性
    stiffnesses = [1000, 5000, 20000, 100000]
    for Ke in stiffnesses:
        env_tmp = ContactEnvironment(Ke=Ke, Be=50, x_surface=0.3)
        imp = ImpedanceController(1.0, 100, 200, dt)
        x = 0.0; xdot = 0.3
        xh, fh = [], []
        for i in range(N):
            F_ext = env_tmp.force(x, xdot)
            xdd = imp.compute(x, xdot, 0.5, F_ext)
            xdot += xdd * dt
            x += xdot * dt
            xh.append(x); fh.append(F_ext)
        mask = np.array(fh) > 0
        axes[1, 1].plot(np.array(xh)[mask], np.array(fh)[mask], linewidth=1.5, label=f'Ke={Ke}')

    axes[1, 1].set_title('不同环境刚度下的柔顺响应', fontsize=12)
    axes[1, 1].set_xlabel('位移 (m)')
    axes[1, 1].set_ylabel('力 (N)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('force_position_hybrid_result.png', dpi=150)
    plt.close()
    print("✅ 力/位置混合控制完成")


# ========== 柔顺控制稳定性分析 ==========
def compliance_stability():
    """不同柔顺度下的稳定性分析"""
    dt = 0.001
    T = 1.0
    N = int(T / dt)
    t = np.arange(N) * dt

    Ke_values = [500, 2000, 10000, 50000]
    Kc_values = [10, 50, 200, 1000]  # 柔顺度参数

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('柔顺控制稳定性分析', fontsize=14, fontweight='bold')

    # 不同环境刚度
    for Ke in Ke_values:
        env = ContactEnvironment(Ke=Ke, Be=20, x_surface=0.5)
        imp = ImpedanceController(1.0, 100, 200, dt)
        x = 0.4; xdot = 0.0
        xh = []
        for i in range(N):
            F_ext = env.force(x, xdot)
            xdd = imp.compute(x, xdot, 0.6, F_ext)
            xdot += xdd * dt
            x += xdot * dt
            xh.append(x)
        axes[0, 0].plot(t, xh, linewidth=1.5, label=f'Ke={Ke}')
    axes[0, 0].axhline(0.5, color='red', linestyle='--', alpha=0.5)
    axes[0, 0].set_title('不同环境刚度下的位置响应', fontsize=12)
    axes[0, 0].set_xlabel('时间 (s)')
    axes[0, 0].set_ylabel('位置 (m)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 不同目标柔顺度
    for Kc in Kc_values:
        env = ContactEnvironment(Ke=5000, Be=20, x_surface=0.5)
        imp = ImpedanceController(1.0, 2*np.sqrt(Kc), Kc, dt)
        x = 0.4; xdot = 0.0
        xh, fh = [], []
        for i in range(N):
            F_ext = env.force(x, xdot)
            xdd = imp.compute(x, xdot, 0.6, F_ext)
            xdot += xdd * dt
            x += xdot * dt
            xh.append(x); fh.append(F_ext)
        axes[0, 1].plot(t, xh, linewidth=1.5, label=f'Kc={Kc}')
    axes[0, 1].set_title('不同柔顺度下的位置响应', fontsize=12)
    axes[0, 1].set_xlabel('时间 (s)')
    axes[0, 1].set_ylabel('位置 (m)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # 阻尼比对振荡的影响
    zeta_values = [0.1, 0.5, 1.0, 2.0]
    for zeta in zeta_values:
        env = ContactEnvironment(Ke=5000, Be=20, x_surface=0.5)
        K = 200
        B = 2 * zeta * np.sqrt(K)
        imp = ImpedanceController(1.0, B, K, dt)
        x = 0.4; xdot = 0.0
        xh = []
        for i in range(N):
            F_ext = env.force(x, xdot)
            xdd = imp.compute(x, xdot, 0.6, F_ext)
            xdot += xdd * dt
            x += xdot * dt
            xh.append(x)
        axes[1, 0].plot(t, xh, linewidth=1.5, label=f'ζ={zeta}')
    axes[1, 0].set_title('不同阻尼比下的响应', fontsize=12)
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('位置 (m)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 稳定性边界图 (K vs Ke)
    Ke_range = np.logspace(2, 5, 50)
    K_range = np.logspace(0, 3, 50)
    stability = np.zeros((len(K_range), len(Ke_range)))

    for i, K in enumerate(K_range):
        for j, Ke in enumerate(Ke_range):
            # 简化稳定性判据: Ke < M*K*ωn² (近似)
            B = 2 * 0.7 * np.sqrt(K)  # 临界阻尼
            env = ContactEnvironment(Ke=Ke, Be=20, x_surface=0.5)
            imp = ImpedanceController(1.0, B, K, dt)
            x = 0.4; xdot = 0.0
            max_disp = 0
            for step in range(int(0.5/dt)):
                F_ext = env.force(x, xdot)
                xdd = imp.compute(x, xdot, 0.6, F_ext)
                xdot += xdd * dt
                x += xdot * dt
                max_disp = max(max_disp, abs(x - 0.6))
                if max_disp > 1.0:  # 不稳定
                    break
            stability[i, j] = 1 if max_disp < 0.1 else 0

    Ke_grid, K_grid = np.meshgrid(Ke_range, K_range)
    axes[1, 1].contourf(Ke_grid, K_grid, stability, levels=[-0.5, 0.5, 1.5],
                         colors=['salmon', 'lightgreen'], alpha=0.6)
    axes[1, 1].set_xscale('log')
    axes[1, 1].set_yscale('log')
    axes[1, 1].set_xlabel('环境刚度 Ke')
    axes[1, 1].set_ylabel('阻抗刚度 K')
    axes[1, 1].set_title('稳定性边界图', fontsize=12)

    plt.tight_layout()
    plt.savefig('compliance_stability_result.png', dpi=150)
    plt.close()
    print("✅ 柔顺控制稳定性分析完成")


if __name__ == '__main__':
    print("=" * 60)
    print("  阻抗控制仿真 - V3迭代")
    print("=" * 60)
    impedance_parameter_study()
    force_position_hybrid()
    compliance_stability()
    print("\n✅ 所有阻抗控制仿真完成!")
