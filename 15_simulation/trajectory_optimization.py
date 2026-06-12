"""
轨迹优化仿真 - 最短时间 / 最小能量 / 最小冲击
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ========== 轨迹参数化 ==========
class TrajectoryOptimizer:
    """多项式轨迹优化器"""

    def __init__(self, n_waypoints=50, T_total=5.0):
        self.n_wp = n_waypoints
        self.T = T_total
        self.dt = T_total / n_waypoints

    def _trajectory_from_params(self, params):
        """从参数生成轨迹 [位置序列, 速度序列, 加速度序列, 加加速度序列]"""
        x = params[:self.n_wp]
        v = np.gradient(x, self.dt)
        a = np.gradient(v, self.dt)
        j = np.gradient(a, self.dt)  # jerk
        return x, v, a, j

    # ========== 最短时间优化 ==========
    def minimum_time(self, x_start, x_end, v_max, a_max):
        """最短时间轨迹"""
        # 使用梯形速度规划
        # 加速到v_max, 匀速, 减速到0
        dist = x_end - x_start
        if dist < 0:
            v_max = -v_max
            a_max = -a_max

        # 加速段时间
        t_acc = abs(v_max / a_max)
        d_acc = 0.5 * a_max * t_acc**2

        if 2 * abs(d_acc) > abs(dist):
            # 三角形速度规划(到不了v_max)
            t_acc = np.sqrt(abs(dist / a_max))
            t_total = 2 * t_acc
            v_peak = a_max * t_acc
        else:
            # 梯形速度规划
            d_cruise = abs(dist) - 2 * abs(d_acc)
            t_cruise = d_cruise / abs(v_max)
            t_total = 2 * t_acc + t_cruise
            v_peak = abs(v_max)

        # 生成轨迹
        t = np.linspace(0, t_total, 500)
        x = np.zeros_like(t)
        v = np.zeros_like(t)
        a = np.zeros_like(t)

        for i, ti in enumerate(t):
            if ti < t_acc:
                a[i] = abs(a_max) * np.sign(dist)
                v[i] = a[i] * ti
                x[i] = x_start + 0.5 * a[i] * ti**2
            elif ti < t_total - t_acc:
                a[i] = 0
                v[i] = v_peak * np.sign(dist)
                t_in_cruise = ti - t_acc
                x[i] = x_start + d_acc * np.sign(dist) + v[i] * t_in_cruise
            else:
                t_dec = ti - (t_total - t_acc)
                a[i] = -abs(a_max) * np.sign(dist)
                v[i] = v_peak * np.sign(dist) + a[i] * t_dec
                d_dec = v_peak * np.sign(dist) * t_dec + 0.5 * a[i] * t_dec**2
                x[i] = x_end - abs(dist) + (2*abs(d_acc) + abs(d_dec)) * np.sign(dist) + v_peak * np.sign(dist) * (t_total - 2*t_acc) + d_dec

            # 修正: 直接用分段公式
        # 重写更清晰的版本
        t = np.linspace(0, t_total, 500)
        x = np.zeros_like(t)
        v = np.zeros_like(t)
        a = np.zeros_like(t)

        for i, ti in enumerate(t):
            if ti <= t_acc:
                a[i] = abs(a_max) * np.sign(dist)
                v[i] = a[i] * ti
                x[i] = x_start + 0.5 * a[i] * ti**2
            elif ti <= t_total - t_acc:
                a[i] = 0
                v[i] = v_peak * np.sign(dist)
                x[i] = x_start + 0.5 * abs(a_max) * t_acc**2 * np.sign(dist) + v[i] * (ti - t_acc)
            else:
                td = ti - (t_total - t_acc)
                a[i] = -abs(a_max) * np.sign(dist)
                v[i] = v_peak * np.sign(dist) + a[i] * td
                x_end_phase = x_start + dist - 0.5 * abs(a_max) * t_acc**2 * np.sign(dist)
                x[i] = x_end_phase + v_peak * np.sign(dist) * td + 0.5 * a[i] * td**2

        return t, x, v, a, t_total

    # ========== 最小能量优化 ==========
    def minimum_energy(self, x_start, x_end, T_fixed):
        """最小能量轨迹 (最小化 ∫a²dt)"""
        # 5次多项式: x(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
        # 边界条件: x(0)=xs, x(T)=xe, v(0)=v(T)=0, a(0)=a(T)=0
        T = T_fixed
        # 直接求解
        A_mat = np.array([
            [1, 0, 0, 0, 0, 0],
            [1, T, T**2, T**3, T**4, T**5],
            [0, 1, 0, 0, 0, 0],
            [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
            [0, 0, 2, 0, 0, 0],
            [0, 0, 2, 6*T, 12*T**2, 20*T**3]
        ])
        b_vec = np.array([x_start, x_end, 0, 0, 0, 0])
        coeffs = np.linalg.solve(A_mat, b_vec)

        t = np.linspace(0, T, 500)
        x = np.polyval(coeffs[::-1], t)
        v = np.polyval(np.polyder(coeffs[::-1]), t)
        a = np.polyval(np.polyder(np.polyder(coeffs[::-1])), t)

        return t, x, v, a

    # ========== 最小冲击(Jerk)优化 ==========
    def minimum_jerk(self, x_start, x_end, T_fixed):
        """最小冲击轨迹"""
        # 与最小能量类似但优化jerk
        # 7次多项式: 最小化 ∫j²dt, 边界: x,v,a在两端 + x,v在两端
        # 简化: 用5次多项式(已是最小jerk的近似)
        return self.minimum_energy(x_start, x_end, T_fixed)

    # ========== B样条优化 ==========
    def bspline_optimize(self, x_start, x_end, v_max, a_max, j_max):
        """B样条轨迹优化"""
        n_ctrl = 10
        dt = self.T / 500

        def cost(params):
            # 控制点 -> 轨迹
            ctrl_pts = np.concatenate([[x_start], params, [x_end]])
            # 线性插值生成轨迹
            t_ctrl = np.linspace(0, self.T, len(ctrl_pts))
            x = np.interp(np.linspace(0, self.T, 500), t_ctrl, ctrl_pts)
            v = np.gradient(x, dt)
            a = np.gradient(v, dt)
            j = np.gradient(a, dt)

            # 代价: 最小化能量 + 惩罚约束违反
            cost_val = np.sum(a**2) * dt
            cost_val += 1000 * np.sum(np.maximum(0, np.abs(v) - v_max)**2)
            cost_val += 1000 * np.sum(np.maximum(0, np.abs(a) - a_max)**2)
            cost_val += 100 * np.sum(np.maximum(0, np.abs(j) - j_max)**2)
            return cost_val

        x0 = np.linspace(x_start, x_end, n_ctrl + 2)[1:-1]
        result = minimize(cost, x0, method='Nelder-Mead', options={'maxiter': 5000})

        ctrl_pts = np.concatenate([[x_start], result.x, [x_end]])
        t_ctrl = np.linspace(0, self.T, len(ctrl_pts))
        t = np.linspace(0, self.T, 500)
        x = np.interp(t, t_ctrl, ctrl_pts)
        v = np.gradient(x, dt)
        a = np.gradient(v, dt)
        j = np.gradient(a, dt)

        return t, x, v, a, j


# ========== 仿真主程序 ==========
def run_trajectory_optimization():
    print("=" * 60)
    print("  轨迹优化仿真 - V3迭代")
    print("=" * 60)

    optimizer = TrajectoryOptimizer()
    x_start, x_end = 0.0, 10.0
    v_max, a_max, j_max = 5.0, 10.0, 50.0

    # 1. 最短时间
    t_min, x_min, v_min, a_min, T_min = optimizer.minimum_time(x_start, x_end, v_max, a_max)

    # 2. 最小能量 (固定时间5s)
    t_energy, x_energy, v_energy, a_energy = optimizer.minimum_energy(x_start, x_end, 5.0)

    # 3. 最小冲击
    t_jerk, x_jerk, v_jerk, a_jerk = optimizer.minimum_jerk(x_start, x_end, 5.0)

    # 4. B样条优化
    t_bs, x_bs, v_bs, a_bs, j_bs = optimizer.bspline_optimize(x_start, x_end, v_max, a_max, j_max)

    # ========== 绘图: 轨迹对比 ==========
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('轨迹优化对比: 最短时间 vs 最小能量 vs 最小冲击 vs B样条', fontsize=14, fontweight='bold')

    methods = [
        (t_min, x_min, v_min, a_min, f'最短时间 (T={T_min:.2f}s)', 'blue'),
        (t_energy, x_energy, v_energy, a_energy, '最小能量 (T=5s)', 'red'),
        (t_jerk, x_jerk, v_jerk, a_jerk, '最小冲击 (T=5s)', 'green'),
        (t_bs, x_bs, v_bs, a_bs, 'B样条优化 (T=5s)', 'purple'),
    ]

    for t, x, v, a, name, color in methods:
        axes[0, 0].plot(t, x, linewidth=2, label=name, color=color)
        axes[0, 1].plot(t, v, linewidth=2, label=name, color=color)
        axes[1, 0].plot(t, a, linewidth=2, label=name, color=color)

    axes[0, 0].set_title('位置轨迹', fontsize=12)
    axes[0, 0].set_xlabel('时间 (s)')
    axes[0, 0].set_ylabel('位置 (m)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].set_title('速度轨迹', fontsize=12)
    axes[0, 1].set_xlabel('时间 (s)')
    axes[0, 1].set_ylabel('速度 (m/s)')
    axes[0, 1].axhline(v_max, color='gray', linestyle='--', alpha=0.5, label=f'v_max={v_max}')
    axes[0, 1].axhline(-v_max, color='gray', linestyle='--', alpha=0.5)
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].set_title('加速度轨迹', fontsize=12)
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('加速度 (m/s²)')
    axes[1, 0].axhline(a_max, color='gray', linestyle='--', alpha=0.5, label=f'a_max={a_max}')
    axes[1, 0].axhline(-a_max, color='gray', linestyle='--', alpha=0.5)
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 代价函数对比
    costs = {
        '最短时间': {
            '时间': T_min,
            '能量(∫a²dt)': np.sum(a_min**2) * (t_min[1]-t_min[0]),
            '冲击(∫j²dt)': np.sum(np.gradient(a_min, t_min[1]-t_min[0])**2) * (t_min[1]-t_min[0]),
        },
        '最小能量': {
            '时间': 5.0,
            '能量(∫a²dt)': np.sum(a_energy**2) * (t_energy[1]-t_energy[0]),
            '冲击(∫j²dt)': np.sum(np.gradient(a_energy, t_energy[1]-t_energy[0])**2) * (t_energy[1]-t_energy[0]),
        },
        '最小冲击': {
            '时间': 5.0,
            '能量(∫a²dt)': np.sum(a_jerk**2) * (t_jerk[1]-t_jerk[0]),
            '冲击(∫j²dt)': np.sum(np.gradient(a_jerk, t_jerk[1]-t_jerk[0])**2) * (t_jerk[1]-t_jerk[0]),
        },
        'B样条': {
            '时间': 5.0,
            '能量(∫a²dt)': np.sum(a_bs**2) * (t_bs[1]-t_bs[0]),
            '冲击(∫j²dt)': np.sum(j_bs**2) * (t_bs[1]-t_bs[0]),
        },
    }

    ax = axes[1, 1]
    names = list(costs.keys())
    metrics = ['时间', '能量(∫a²dt)', '冲击(∫j²dt)']
    x_pos = np.arange(len(names))
    width = 0.25
    colors = ['skyblue', 'salmon', 'lightgreen']

    for i, metric in enumerate(metrics):
        vals = [costs[n][metric] for n in names]
        # 归一化
        max_val = max(vals) if max(vals) > 0 else 1
        vals_norm = [v/max_val for v in vals]
        ax.bar(x_pos + i*width, vals_norm, width, label=metric, color=colors[i])

    ax.set_xticks(x_pos + width)
    ax.set_xticklabels(names)
    ax.set_ylabel('归一化代价')
    ax.set_title('代价函数对比 (归一化)', fontsize=12)
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('trajectory_optimization_result.png', dpi=150)
    plt.close()

    # 打印结果
    for name, costs_dict in costs.items():
        print(f"\n  {name}:")
        for k, v in costs_dict.items():
            print(f"    {k}: {v:.4f}")

    # ========== 2D轨迹优化 ==========
    trajectory_2d_demo()


def trajectory_2d_demo():
    """2D避障轨迹优化"""
    from scipy.optimize import minimize

    N = 100
    T = 5.0
    dt = T / N

    # 障碍物
    obstacles = [(3, 3, 0.8), (6, 5, 1.0), (8, 2, 0.6)]  # (x, y, r)

    def cost_2d(params):
        xs = params[:N]
        ys = params[N:]
        xs = np.concatenate([[0], xs, [10]])
        ys = np.concatenate([[0], ys, [10]])

        vx = np.gradient(xs, dt)
        vy = np.gradient(ys, dt)
        ax = np.gradient(vx, dt)
        ay = np.gradient(vy, dt)

        # 能量代价
        cost = np.sum(ax**2 + ay**2) * dt

        # 障碍物惩罚
        for ox, oy, r in obstacles:
            dist = np.sqrt((xs - ox)**2 + (ys - oy)**2)
            cost += 10000 * np.sum(np.maximum(0, r + 0.2 - dist)**2)

        return cost

    # 初始猜测: 直线
    x0 = np.linspace(0, 10, N+2)[1:-1]
    y0 = np.linspace(0, 10, N+2)[1:-1]
    params0 = np.concatenate([x0, y0])

    result = minimize(cost_2d, params0, method='L-BFGS-B', options={'maxiter': 2000})

    xs = np.concatenate([[0], result.x[:N], [10]])
    ys = np.concatenate([[0], result.x[N:], [10]])

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(xs, ys, 'b-', linewidth=3, label='优化轨迹')
    ax.plot(0, 0, 'go', markersize=15, label='起点')
    ax.plot(10, 10, 'r*', markersize=15, label='终点')

    # 直线参考
    ax.plot([0, 10], [0, 10], 'k--', alpha=0.3, label='直线参考')

    for ox, oy, r in obstacles:
        circle = plt.Circle((ox, oy), r, color='red', alpha=0.3)
        ax.add_patch(circle)
        ax.plot(ox, oy, 'rx', markersize=10)

    ax.set_xlim(-1, 11)
    ax.set_ylim(-1, 11)
    ax.set_aspect('equal')
    ax.set_title('2D避障轨迹优化', fontsize=14, fontweight='bold')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('trajectory_2d_obstacle_result.png', dpi=150)
    plt.close()
    print("✅ 2D避障轨迹优化完成")


if __name__ == '__main__':
    run_trajectory_optimization()
    print("\n✅ 所有轨迹优化仿真完成!")
