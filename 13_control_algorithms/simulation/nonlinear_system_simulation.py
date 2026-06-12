#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
非线性系统仿真 - 描述函数法分析工具

功能:
1. 常见非线性环节的描述函数计算(死区、饱和、继电、滞环)
2. 描述函数法判断非线性系统是否存在极限环(自激振荡)
3. 极限环频率和幅值的图解法求解(Nyquist图叠加)
4. 时域仿真验证

适用场景:
- 分析含饱和/死区/滞环的闭环系统稳定性
- 预测极限环振荡频率和幅值
- 电赛中电机控制、电源控制的非线性行为分析

Author: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal
import warnings

# ============================================================
# 1. 非线性环节描述函数
# ============================================================

def describing_function_relay(A, M):
    """
    理想继电器: N(A) = 4M / (πA)
    A: 输入正弦幅值
    M: 继电器输出幅值
    """
    if A == 0:
        return np.inf
    return 4 * M / (np.pi * A)


def describing_function_relay_hysteresis(A, M, delta):
    """
    带滞环的继电器: N(A) = (4M/πA) * [√(1-(δ/A)²) - j*(δ/A)]
    A: 输入正弦幅值
    M: 继电器输出幅值
    delta: 滞环宽度的一半
    """
    if A <= delta:
        return complex(0, 0)
    ratio = delta / A
    real_part = 4 * M / (np.pi * A) * np.sqrt(1 - ratio**2)
    imag_part = -4 * M * ratio / (np.pi * A)
    return complex(real_part, imag_part)


def describing_function_saturation(A, k, a):
    """
    饱和非线性: N(A) = (2k/π) * [arcsin(a/A) + (a/A)*√(1-(a/A)²)]  当 A >= a
                 N(A) = k  当 A < a
    A: 输入正弦幅值
    k: 线性区斜率
    a: 饱和值
    """
    if A < a:
        return k
    if A == 0:
        return k
    ratio = a / A
    NA = 2 * k / np.pi * (np.arcsin(ratio) + ratio * np.sqrt(1 - ratio**2))
    return NA


def describing_function_deadzone(A, k, d):
    """
    死区非线性: N(A) = (2k/π) * [π/2 - arcsin(d/A) - (d/A)*√(1-(d/A)²)]  当 A >= d
                 N(A) = 0  当 A < d
    A: 输入正弦幅值
    k: 线性区斜率
    d: 死区宽度的一半
    """
    if A < d:
        return 0.0
    if A == 0:
        return 0.0
    ratio = d / A
    NA = 2 * k / np.pi * (np.pi / 2 - np.arcsin(ratio) - ratio * np.sqrt(1 - ratio**2))
    return NA


def describing_function_deadzone_saturation(A, k, d, a):
    """
    死区+饱和组合非线性
    d: 死区宽度的一半
    a: 饱和值
    k: 线性区斜率
    """
    if A < d:
        return 0.0
    if A <= a:
        return describing_function_deadzone(A, k, d)
    ratio_d = d / A
    ratio_a = a / A
    NA = 2 * k / np.pi * (
        np.arcsin(ratio_a) - np.arcsin(ratio_d)
        + ratio_a * np.sqrt(1 - ratio_a**2)
        - ratio_d * np.sqrt(1 - ratio_d**2)
    )
    return NA


# ============================================================
# 2. 极限环分析
# ============================================================

class DescribingFunctionAnalyzer:
    """描述函数法分析器"""

    def __init__(self, linear_tf_num, linear_tf_den):
        """
        linear_tf_num: 线性部分传递函数分子系数
        linear_tf_den: 线性部分传递函数分母系数
        """
        self.G_num = linear_tf_num
        self.G_den = linear_tf_den
        self.G_sys = signal.TransferFunction(linear_tf_num, linear_tf_den)

    def linear_response(self, omega):
        """计算线性部分 G(jω) 的频率响应"""
        w, mag, phase = signal.bode(self.G_sys, omega)
        G_jw = mag * np.exp(1j * np.deg2rad(phase))
        return G_jw

    def find_limit_cycle(self, nf_func, A_range, omega_range, **nf_kwargs):
        """
        寻找极限环: 求解 -1/N(A) = G(jω) 的交点

        nf_func: 描述函数 (A, **kwargs) -> complex
        A_range: 幅值搜索范围
        omega_range: 频率搜索范围

        返回: 列表 [(A, omega, stable), ...]
              stable: True=稳定极限环, False=不稳定
        """
        results = []

        # 计算线性部分频率响应
        G_jw = self.linear_response(omega_range)

        # 计算负逆描述函数曲线
        for A in A_range:
            if A <= 0:
                continue
            NA = nf_func(A, **nf_kwargs)
            if isinstance(NA, complex):
                neg_inv_NA = -1.0 / NA
            else:
                if NA == 0:
                    continue
                neg_inv_NA = complex(-1.0 / NA, 0)

            # 寻找与G(jω)的交点
            for i, w in enumerate(omega_range):
                dist = abs(G_jw[i] - neg_inv_NA)
                if dist < 0.05:  # 精度阈值
                    # 判断稳定性: N(A)的轨迹从G的左侧穿过时为稳定
                    stable = self._check_stability(nf_func, A, w, **nf_kwargs)
                    results.append((A, w, stable))

        return results

    def _check_stability(self, nf_func, A0, omega0, **nf_kwargs):
        """通过幅值微扰判断极限环稳定性"""
        dA = A0 * 0.01
        NA_plus = nf_func(A0 + dA, **nf_kwargs)
        NA_minus = nf_func(A0 - dA, **nf_kwargs)

        if isinstance(NA_plus, (int, float)):
            NA_plus = complex(NA_plus, 0)
        if isinstance(NA_minus, (int, float)):
            NA_minus = complex(NA_minus, 0)

        inv_N_plus = -1.0 / NA_plus
        inv_N_minus = -1.0 / NA_minus

        G_jw0 = self.linear_response(np.array([omega0]))[0]

        # 简化判断: 幅值增大时-1/N(A)远离G(jω)则稳定
        dist_plus = abs(inv_N_plus - G_jw0)
        dist_minus = abs(inv_N_minus - G_jw0)

        return dist_plus > dist_minus

    def plot_nyquist_and_nf(self, nf_func, A_range, omega_range,
                            nf_kwargs=None, title="描述函数法分析"):
        """
        绘制Nyquist图 + 负逆描述函数曲线
        """
        if nf_kwargs is None:
            nf_kwargs = {}

        fig, ax = plt.subplots(1, 1, figsize=(10, 8))

        # Nyquist曲线
        G_jw = self.linear_response(omega_range)
        ax.plot(G_jw.real, G_jw.imag, 'b-', linewidth=1.5, label='G(jω)')
        ax.plot(G_jw.real, -G_jw.imag, 'b--', linewidth=0.8, alpha=0.5)

        # -1/N(A)曲线
        nf_real = []
        nf_imag = []
        for A in A_range:
            if A <= 0.01:
                continue
            NA = nf_func(A, **nf_kwargs)
            if isinstance(NA, (int, float)):
                NA = complex(NA, 0)
            if abs(NA) < 1e-10:
                continue
            inv = -1.0 / NA
            nf_real.append(inv.real)
            nf_imag.append(inv.imag)

        ax.plot(nf_real, nf_imag, 'r-', linewidth=2, label='-1/N(A)')
        # 标注幅值增大的方向
        if len(nf_real) > 1:
            ax.annotate('', xy=(nf_real[-1], nf_imag[-1]),
                       xytext=(nf_real[-2], nf_imag[-2]),
                       arrowprops=dict(arrowstyle='->', color='red', lw=2))

        ax.plot(-1, 0, 'ko', markersize=8, label='(-1, j0)')

        ax.set_xlabel('实部 Re')
        ax.set_ylabel('虚部 Im')
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        return fig, ax


# ============================================================
# 3. 非线性系统时域仿真
# ============================================================

class NonlinearSimulation:
    """非线性系统时域仿真器"""

    @staticmethod
    def simulate_relay_system(G_num, G_den, M, t_span, dt, x0=None):
        """
        仿真含理想继电器的闭环系统
        """
        G = signal.TransferFunction(G_num, G_den)
        n_states = max(len(G_num), len(G_den)) - 1
        if x0 is None:
            x0 = np.zeros(n_states)

        t = np.arange(0, t_span, dt)
        x = np.array(x0, dtype=float)
        y_out = []
        u_out = []

        # 转为状态空间
        sys_ss = signal.tf2ss(G_num, G_den)
        A, B, C, D = sys_ss

        for i in range(len(t)):
            y = C @ x + D * np.array([0.0])  # 无外部输入, 闭环
            y_val = y[0]
            y_out.append(y_val)

            # 继电器: u = M * sign(-y)
            if y_val > 0:
                u = -M
            elif y_val < 0:
                u = M
            else:
                u = 0
            u_out.append(u)

            # 状态更新 (RK4)
            def f(x, u):
                return A @ x + B.flatten() * u

            k1 = f(x, u)
            k2 = f(x + 0.5 * dt * k1, u)
            k3 = f(x + 0.5 * dt * k2, u)
            k4 = f(x + dt * k3, u)
            x = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        return t, np.array(y_out), np.array(u_out)

    @staticmethod
    def simulate_saturation_system(G_num, G_den, k_sat, a_sat,
                                   t_span, dt, ref=1.0, x0=None):
        """
        仿真含饱和非线性的闭环系统
        """
        G = signal.TransferFunction(G_num, G_den)
        sys_ss = signal.tf2ss(G_num, G_den)
        A, B, C, D = sys_ss
        n_states = A.shape[0]
        if x0 is None:
            x0 = np.zeros(n_states)

        t = np.arange(0, t_span, dt)
        x = np.array(x0, dtype=float)
        y_out = []
        u_out = []

        for i in range(len(t)):
            y = (C @ x)[0]
            y_out.append(y)

            e = ref - y
            # 饱和
            if e > a_sat:
                u = k_sat * a_sat
            elif e < -a_sat:
                u = -k_sat * a_sat
            else:
                u = k_sat * e
            u_out.append(u)

            def f(x, u):
                return A @ x + B.flatten() * u

            k1 = f(x, u)
            k2 = f(x + 0.5 * dt * k1, u)
            k3 = f(x + 0.5 * dt * k2, u)
            k4 = f(x + dt * k3, u)
            x = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        return t, np.array(y_out), np.array(u_out)

    @staticmethod
    def simulate_hysteresis_system(G_num, G_den, M, delta,
                                   t_span, dt, x0=None):
        """
        仿真含滞环继电器的闭环系统
        """
        sys_ss = signal.tf2ss(G_num, G_den)
        A, B, C, D = sys_ss
        n_states = A.shape[0]
        if x0 is None:
            x0 = np.zeros(n_states)

        t = np.arange(0, t_span, dt)
        x = np.array(x0, dtype=float)
        y_out = []
        u_out = []
        u_state = M  # 初始输出状态

        for i in range(len(t)):
            y = (C @ x)[0]
            y_out.append(y)

            # 滞环继电器
            if y > delta:
                u_state = -M
            elif y < -delta:
                u_state = M
            # else: 保持上一状态
            u = u_state
            u_out.append(u)

            def f(x, u):
                return A @ x + B.flatten() * u

            k1 = f(x, u)
            k2 = f(x + 0.5 * dt * k1, u)
            k3 = f(x + 0.5 * dt * k2, u)
            k4 = f(x + dt * k3, u)
            x = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        return t, np.array(y_out), np.array(u_out)


# ============================================================
# 4. 示例与演示
# ============================================================

def example_relay_limit_cycle():
    """
    示例: 含理想继电器的二阶系统极限环分析
    G(s) = 1 / (s(s+1)), 继电器 M=1
    """
    print("=" * 60)
    print("示例: 继电器 + G(s) = 1/(s(s+1)) 极限环分析")
    print("=" * 60)

    # 线性部分
    G_num = [1]
    G_den = [1, 1, 0]  # s(s+1)

    analyzer = DescribingFunctionAnalyzer(G_num, G_den)

    # 搜索
    A_range = np.linspace(0.1, 5.0, 200)
    omega_range = np.linspace(0.01, 10.0, 1000)

    results = analyzer.find_limit_cycle(
        describing_function_relay, A_range, omega_range, M=1.0
    )

    if results:
        for A, w, stable in results:
            print(f"  极限环: 幅值A={A:.3f}, 频率ω={w:.3f} rad/s "
                  f"(f={w/(2*np.pi):.3f} Hz), {'稳定' if stable else '不稳定'}")
    else:
        print("  未找到极限环")

    # 绘图
    fig, ax = analyzer.plot_nyquist_and_nf(
        describing_function_relay, A_range, omega_range,
        nf_kwargs={'M': 1.0},
        title="继电器系统描述函数法分析"
    )
    plt.tight_layout()
    plt.savefig("relay_limit_cycle_analysis.png", dpi=150)
    print("  图像已保存: relay_limit_cycle_analysis.png")

    # 时域仿真
    t, y, u = NonlinearSimulation.simulate_relay_system(
        G_num, G_den, M=1.0, t_span=30.0, dt=0.001,
        x0=[0.1, 0.0]
    )

    fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))
    ax1.plot(t, y, 'b-', linewidth=1)
    ax1.set_ylabel('输出 y(t)')
    ax1.set_title('含理想继电器闭环系统 - 时域响应')
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, u, 'r-', linewidth=1)
    ax2.set_ylabel('控制量 u(t)')
    ax2.set_xlabel('时间 (s)')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("relay_limit_cycle_time.png", dpi=150)
    print("  时域仿真图已保存: relay_limit_cycle_time.png")

    # 分析极限环参数
    if len(y) > 2000:
        y_ss = y[-2000:]
        amplitude = (np.max(y_ss) - np.min(y_ss)) / 2
        # 过零检测求周期
        crossings = np.where(np.diff(np.sign(y_ss)))[0]
        if len(crossings) > 2:
            period = np.mean(np.diff(crossings)) * 0.001
            freq = 1.0 / period if period > 0 else 0
            print(f"  时域验证: 振幅={amplitude:.3f}, 周期={period:.3f}s, 频率={freq:.3f}Hz")


def example_saturation_analysis():
    """示例: 饱和非线性分析"""
    print("\n" + "=" * 60)
    print("示例: 饱和非线性 + G(s) = 10/(s²+2s+1) 分析")
    print("=" * 60)

    G_num = [10]
    G_den = [1, 2, 1]
    k_sat = 1.0
    a_sat = 0.5

    A_range = np.linspace(0.1, 3.0, 100)

    print("  饱和描述函数 N(A):")
    for A in [0.3, 0.5, 1.0, 2.0, 3.0]:
        NA = describing_function_saturation(A, k_sat, a_sat)
        print(f"    A={A:.1f}: N(A)={NA:.4f}")


def example_deadzone_analysis():
    """示例: 死区非线性分析"""
    print("\n" + "=" * 60)
    print("示例: 死区非线性描述函数")
    print("=" * 60)

    k = 1.0
    d = 0.3

    print("  死区描述函数 N(A):")
    for A in [0.1, 0.3, 0.5, 1.0, 2.0, 5.0]:
        NA = describing_function_deadzone(A, k, d)
        print(f"    A={A:.1f}: N(A)={NA:.4f}")


if __name__ == "__main__":
    print("非线性系统仿真 - 描述函数法分析工具")
    print("=" * 60)

    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    example_relay_limit_cycle()
    example_saturation_analysis()
    example_deadzone_analysis()

    print("\n" + "=" * 60)
    print("分析完成!")
