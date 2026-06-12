#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模糊逻辑控制器仿真 - 规则库 + 隶属函数 + 解模糊
用于电赛智能控制与非线性系统控制
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


# ======================== 隶属函数 ========================
class MembershipFunction:
    """隶属函数库"""
    @staticmethod
    def trimf(x, a, b, c):
        """三角隶属函数"""
        return np.maximum(0, np.minimum((x - a) / max(b - a, 1e-9),
                                        (c - x) / max(c - b, 1e-9)))

    @staticmethod
    def trapmf(x, a, b, c, d):
        """梯形隶属函数"""
        return np.maximum(0, np.minimum(
            np.minimum((x - a) / max(b - a, 1e-9), 1),
            (d - x) / max(d - c, 1e-9)))

    @staticmethod
    def gaussmf(x, mean, sigma):
        """高斯隶属函数"""
        return np.exp(-0.5 * ((x - mean) / max(sigma, 1e-9))**2)

    @staticmethod
    def sigmf(x, a, c):
        """S型隶属函数"""
        return 1.0 / (1.0 + np.exp(-a * (x - c)))


# ======================== 模糊集合定义 ========================
class FuzzyVariable:
    """模糊变量（输入/输出）"""
    def __init__(self, name, universe, terms):
        self.name = name
        self.universe = universe  # 论域 [min, max]
        self.terms = terms        # {'NB': (a,b,c), 'ZO': (a,b,c), 'PB': (a,b,c)}

    def fuzzify(self, x):
        """模糊化: 计算各语言值的隶属度"""
        mf = MembershipFunction()
        degrees = {}
        x_arr = np.atleast_1d(x)
        for term, params in self.terms.items():
            if len(params) == 3:
                degrees[term] = mf.trimf(x_arr, *params).item()
            elif len(params) == 4:
                degrees[term] = mf.trapmf(x_arr, *params).item()
            elif len(params) == 2:  # gauss: (mean, sigma)
                degrees[term] = mf.gaussmf(x_arr, *params).item()
        return degrees

    def defuzzify_centroid(self, aggregated):
        """重心法解模糊"""
        x = np.linspace(self.universe[0], self.universe[1], 500)
        y = np.zeros_like(x)
        for term, (mu, params) in aggregated.items():
            mf = MembershipFunction()
            if len(params) == 3:
                y = np.maximum(y, np.minimum(mu, mf.trimf(x, *params)))
            elif len(params) == 2:
                y = np.maximum(y, np.minimum(mu, mf.gaussmf(x, *params)))
        area = np.sum(y)
        if area < 1e-9:
            return 0
        return np.sum(x * y) / area

    def defuzzify_bisector(self, aggregated):
        """面积平分法解模糊"""
        x = np.linspace(self.universe[0], self.universe[1], 500)
        y = np.zeros_like(x)
        for term, (mu, params) in aggregated.items():
            mf = MembershipFunction()
            if len(params) == 3:
                y = np.maximum(y, np.minimum(mu, mf.trimf(x, *params)))
            elif len(params) == 2:
                y = np.maximum(y, np.minimum(mu, mf.gaussmf(x, *params)))
        total = np.sum(y)
        if total < 1e-9:
            return 0
        cum = np.cumsum(y)
        idx = np.searchsorted(cum, total / 2)
        return x[min(idx, len(x)-1)]

    def defuzzify_mom(self, aggregated):
        """最大隶属度平均法"""
        x = np.linspace(self.universe[0], self.universe[1], 500)
        y = np.zeros_like(x)
        for term, (mu, params) in aggregated.items():
            mf = MembershipFunction()
            if len(params) == 3:
                y = np.maximum(y, np.minimum(mu, mf.trimf(x, *params)))
            elif len(params) == 2:
                y = np.maximum(y, np.minimum(mu, mf.gaussmf(x, *params)))
        max_val = np.max(y)
        if max_val < 1e-9:
            return 0
        return np.mean(x[y >= max_val * 0.99])


# ======================== 模糊规则库 ========================
class FuzzyRuleBase:
    """模糊规则库"""
    def __init__(self):
        # 规则表: (误差, 误差变化率) -> 输出
        # e: NB=负大, NM=负中, NS=负小, ZO=零, PS=正小, PM=正中, PB=正大
        # de: 同上
        # u: 同上
        self.rules = [
            # (e_term, de_term, u_term, weight)
            ('NB', 'NB', 'NB', 1.0),
            ('NB', 'NM', 'NB', 1.0),
            ('NB', 'NS', 'NM', 1.0),
            ('NB', 'ZO', 'NM', 1.0),
            ('NB', 'PS', 'NS', 1.0),
            ('NB', 'PM', 'ZO', 1.0),
            ('NB', 'PB', 'ZO', 1.0),

            ('NM', 'NB', 'NB', 1.0),
            ('NM', 'NM', 'NM', 1.0),
            ('NM', 'NS', 'NM', 1.0),
            ('NM', 'ZO', 'NS', 1.0),
            ('NM', 'PS', 'ZO', 1.0),
            ('NM', 'PM', 'PS', 1.0),
            ('NM', 'PB', 'PS', 1.0),

            ('NS', 'NB', 'NM', 1.0),
            ('NS', 'NM', 'NM', 1.0),
            ('NS', 'NS', 'NS', 1.0),
            ('NS', 'ZO', 'NS', 1.0),
            ('NS', 'PS', 'ZO', 1.0),
            ('NS', 'PM', 'PS', 1.0),
            ('NS', 'PB', 'PM', 1.0),

            ('ZO', 'NB', 'NM', 1.0),
            ('ZO', 'NM', 'NS', 1.0),
            ('ZO', 'NS', 'NS', 1.0),
            ('ZO', 'ZO', 'ZO', 1.0),
            ('ZO', 'PS', 'PS', 1.0),
            ('ZO', 'PM', 'PS', 1.0),
            ('ZO', 'PB', 'PM', 1.0),

            ('PS', 'NB', 'NM', 1.0),
            ('PS', 'NM', 'NS', 1.0),
            ('PS', 'NS', 'ZO', 1.0),
            ('PS', 'ZO', 'PS', 1.0),
            ('PS', 'PS', 'PS', 1.0),
            ('PS', 'PM', 'PM', 1.0),
            ('PS', 'PB', 'PM', 1.0),

            ('PM', 'NB', 'NS', 1.0),
            ('PM', 'NM', 'ZO', 1.0),
            ('PM', 'NS', 'PS', 1.0),
            ('PM', 'ZO', 'PM', 1.0),
            ('PM', 'PS', 'PM', 1.0),
            ('PM', 'PM', 'PB', 1.0),
            ('PM', 'PB', 'PB', 1.0),

            ('PB', 'NB', 'ZO', 1.0),
            ('PB', 'NM', 'ZO', 1.0),
            ('PB', 'NS', 'PS', 1.0),
            ('PB', 'ZO', 'PM', 1.0),
            ('PB', 'PS', 'PB', 1.0),
            ('PB', 'PM', 'PB', 1.0),
            ('PB', 'PB', 'PB', 1.0),
        ]

    def evaluate(self, e_degrees, de_degrees, output_var):
        """推理：Mamdani方法"""
        aggregated = {}
        for e_term, de_term, u_term, w in self.rules:
            mu_e = e_degrees.get(e_term, 0)
            mu_de = de_degrees.get(de_term, 0)
            # 取小运算 (AND)
            mu = min(mu_e, mu_de) * w
            if mu > 0:
                if u_term in aggregated:
                    aggregated[u_term] = (max(aggregated[u_term][0], mu),
                                          output_var.terms[u_term])
                else:
                    aggregated[u_term] = (mu, output_var.terms[u_term])
        return aggregated


# ======================== 模糊控制器 ========================
class FuzzyController:
    """完整的模糊控制器"""
    def __init__(self, defuzz_method='centroid'):
        # 误差论域 [-6, 6]
        self.e_var = FuzzyVariable('error', [-6, 6], {
            'NB': (-6, -6, -4), 'NM': (-6, -4, -2), 'NS': (-4, -2, 0),
            'ZO': (-2, 0, 2), 'PS': (0, 2, 4), 'PM': (2, 4, 6), 'PB': (4, 6, 6)
        })
        # 误差变化率论域 [-6, 6]
        self.de_var = FuzzyVariable('d_error', [-6, 6], {
            'NB': (-6, -6, -4), 'NM': (-6, -4, -2), 'NS': (-4, -2, 0),
            'ZO': (-2, 0, 2), 'PS': (0, 2, 4), 'PM': (2, 4, 6), 'PB': (4, 6, 6)
        })
        # 输出论域 [-6, 6] (缩放因子控制实际输出)
        self.u_var = FuzzyVariable('output', [-6, 6], {
            'NB': (-6, -6, -4), 'NM': (-6, -4, -2), 'NS': (-4, -2, 0),
            'ZO': (-2, 0, 2), 'PS': (0, 2, 4), 'PM': (2, 4, 6), 'PB': (4, 6, 6)
        })
        self.rule_base = FuzzyRuleBase()
        self.defuzz_method = defuzz_method
        self.Ke = 3.0     # 误差缩放因子
        self.Kde = 1.0    # 误差变化率缩放因子
        self.Ku = 2.0     # 输出缩放因子
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        # 缩放
        e_scaled = np.clip(error * self.Ke, -6, 6)
        de = (error - self.prev_error) / dt if dt > 0 else 0
        de_scaled = np.clip(de * self.Kde, -6, 6)
        self.prev_error = error

        # 模糊化
        e_deg = self.e_var.fuzzify(e_scaled)
        de_deg = self.de_var.fuzzify(de_scaled)

        # 规则推理
        aggregated = self.rule_base.evaluate(e_deg, de_deg, self.u_var)

        # 解模糊
        if self.defuzz_method == 'centroid':
            u_fuzzy = self.u_var.defuzzify_centroid(aggregated)
        elif self.defuzz_method == 'bisector':
            u_fuzzy = self.u_var.defuzzify_bisector(aggregated)
        else:
            u_fuzzy = self.u_var.defuzzify_mom(aggregated)

        # 缩放输出
        u = u_fuzzy * self.Ku
        return np.clip(u, -15, 15)

    def reset(self):
        self.prev_error = 0
        self.integral = 0


# ======================== 对比: PID ========================
class StandardPID:
    def __init__(self, Kp=5, Ki=1, Kd=2):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def reset(self):
        self.integral = 0
        self.prev_error = 0


# ======================== 被控对象 ========================
class NonlinearPlant:
    """非线性二阶系统"""
    def __init__(self):
        self.x = np.zeros(2)
        self.t = 0

    def step(self, u, dt):
        # 非线性项: 死区 + 饱和
        u_nl = u
        if abs(u) < 0.5:
            u_nl = 0
        u_nl = np.clip(u_nl, -10, 10)

        # 非线性动力学
        dx0 = self.x[1]
        dx1 = -2.0 * self.x[1] - 5.0 * self.x[0] - 0.1 * self.x[0]**3 + u_nl
        # 加入时变扰动
        dx1 += 0.5 * np.sin(3 * self.t)

        self.x[0] += dx0 * dt
        self.x[1] += dx1 * dt
        self.t += dt
        return self.x[0]

    def reset(self):
        self.x = np.zeros(2)
        self.t = 0


# ======================== 主程序 ========================
def main():
    T, dt = 20, 0.01
    N = int(T / dt)
    t_arr = np.linspace(0, T, N)

    # 参考信号
    ref = np.where(t_arr < 7, 1.0,
         np.where(t_arr < 14, -0.5 * np.ones_like(t_arr),
                  0.5 + 0.5 * np.sin(2 * np.pi * (t_arr - 14) / 6)))

    controllers = {
        '模糊控制(重心法)': FuzzyController('centroid'),
        '模糊控制(平分法)': FuzzyController('bisector'),
        '模糊控制(最大隶属度)': FuzzyController('mom'),
        'PID': StandardPID(Kp=5, Ki=1, Kd=2),
    }

    results = {}
    for name, ctrl in controllers.items():
        plant = NonlinearPlant()
        ctrl.reset()
        y_arr = np.zeros(N)
        u_arr = np.zeros(N)
        for i in range(N):
            error = ref[i] - plant.x[0]
            u = ctrl.compute(error, dt)
            plant.step(u, dt)
            y_arr[i] = plant.x[0]
            u_arr[i] = u
        results[name] = {'y': y_arr, 'u': u_arr}

    # 绘图
    colors = ['#2196F3', '#4CAF50', '#FF9800', '#E91E63']
    fig, axes = plt.subplots(3, 1, figsize=(14, 11))

    # 输出响应
    ax = axes[0]
    ax.plot(t_arr, ref, 'k--', lw=1.5, label='参考信号')
    for (name, data), c in zip(results.items(), colors):
        ax.plot(t_arr, data['y'], color=c, lw=1.0, label=name)
    ax.set_ylabel('输出 y(t)')
    ax.set_title('模糊控制器 vs PID - 非线性系统控制')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # 误差
    ax = axes[1]
    for (name, data), c in zip(results.items(), colors):
        ax.plot(t_arr, ref - data['y'], color=c, lw=0.8, label=name)
    ax.set_ylabel('跟踪误差')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # 控制量
    ax = axes[2]
    for (name, data), c in zip(results.items(), colors):
        ax.plot(t_arr, data['u'], color=c, lw=0.8, label=name)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制量 u(t)')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('fuzzy_control_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 隶属函数可视化
    fig2, axes2 = plt.subplots(1, 3, figsize=(15, 4))
    mf = MembershipFunction()
    x = np.linspace(-6, 6, 500)
    terms = {'NB': (-6, -6, -4), 'NM': (-6, -4, -2), 'NS': (-4, -2, 0),
             'ZO': (-2, 0, 2), 'PS': (0, 2, 4), 'PM': (2, 4, 6), 'PB': (4, 6, 6)}
    colors_mf = ['#1a237e', '#1565c0', '#42a5f5', '#66bb6a', '#ffa726', '#ef5350', '#b71c1c']

    for ax_idx, (title, var) in enumerate(zip(['误差 e', '误差变化率 de', '输出 u'], [None]*3)):
        ax = axes2[ax_idx]
        for (term, params), c in zip(terms.items(), colors_mf):
            ax.plot(x, mf.trimf(x, *params), color=c, lw=1.5, label=term)
        ax.set_xlabel('论域')
        ax.set_ylabel('隶属度')
        ax.set_title(f'{title} 隶属函数')
        ax.legend(fontsize=7, ncol=2)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('membership_functions.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 3D模糊控制曲面
    e_range = np.linspace(-6, 6, 50)
    de_range = np.linspace(-6, 6, 50)
    E, DE = np.meshgrid(e_range, de_range)
    U = np.zeros_like(E)
    fuzzy_ctrl = FuzzyController('centroid')
    for i in range(50):
        for j in range(50):
            e_deg = fuzzy_ctrl.e_var.fuzzify(E[i, j])
            de_deg = fuzzy_ctrl.de_var.fuzzify(DE[i, j])
            agg = fuzzy_ctrl.rule_base.evaluate(e_deg, de_deg, fuzzy_ctrl.u_var)
            U[i, j] = fuzzy_ctrl.u_var.defuzzify_centroid(agg)

    fig3 = plt.figure(figsize=(10, 7))
    ax3 = fig3.add_subplot(111, projection='3d')
    surf = ax3.plot_surface(E, DE, U, cmap='viridis', alpha=0.8)
    ax3.set_xlabel('误差 e')
    ax3.set_ylabel('误差变化率 de')
    ax3.set_zlabel('输出 u')
    ax3.set_title('模糊控制曲面')
    fig3.colorbar(surf, shrink=0.5)
    plt.tight_layout()
    plt.savefig('fuzzy_control_surface.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 性能指标
    print("\n" + "="*65)
    print("模糊控制器性能对比 (含非线性+扰动)")
    print("="*65)
    print(f"{'方法':<20} {'ISE':>10} {'IAE':>10} {'超调%':>10} {'控制能量':>10}")
    print("-"*65)
    for name, data in results.items():
        e = ref - data['y']
        ise = np.sum(e**2) * dt
        iae = np.sum(np.abs(e)) * dt
        overshoot = max(0, (np.max(data['y']) - 1.0) / 1.0 * 100)
        ctrl_energy = np.sum(data['u']**2) * dt
        print(f"{name:<20} {ise:>10.3f} {iae:>10.3f} {overshoot:>9.1f}% {ctrl_energy:>10.1f}")
    print("="*65)


if __name__ == '__main__':
    main()
