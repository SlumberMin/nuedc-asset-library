#!/usr/bin/env python3
"""
非线性控制仿真 - 反馈线性化
==============================
基于微分几何方法的反馈线性化控制器设计与仿真。
适合电赛中存在非线性特性的被控对象，如:
  - 倒立摆
  - 机械臂
  - DC-DC变换器
  - 化学反应器

包含:
  1. SISO系统反馈线性化
  2. 相对阶计算与零动态分析
  3. 输入-输出线性化控制器
  4. 滑模变结构控制（非线性鲁棒方法）
  5. 反步法(Backstepping)控制

依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import Callable, Tuple, Optional
from dataclasses import dataclass
import os

# ============================================================
# 1. 非线性系统模型
# ============================================================

class NonlinearSystem:
    """通用仿射非线性系统: dx/dt = f(x) + g(x)*u, y = h(x)
    
    状态向量 x, 输入 u, 输出 y
    """
    def __init__(self, f: Callable, g: Callable, h: Callable,
                 x0: np.ndarray, name: str = "NonlinearSystem"):
        self.f = f  # f(x): 系统漂移
        self.g = g  # g(x): 输入向量场
        self.h = h  # h(x): 输出函数
        self.state = x0.copy().astype(float)
        self.name = name
        self.n = len(x0)  # 系统阶次
    
    def step(self, u: float, dt: float) -> float:
        """RK4积分一步"""
        x = self.state.copy()
        def dynamics(x_):
            return self.f(x_) + self.g(x_) * u
        
        k1 = dynamics(x)
        k2 = dynamics(x + 0.5*dt*k1)
        k3 = dynamics(x + 0.5*dt*k2)
        k4 = dynamics(x + dt*k3)
        self.state = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        return self.h(self.state)
    
    def output(self) -> float:
        return self.h(self.state)
    
    def reset(self, x0=None):
        if x0 is not None:
            self.state = x0.copy().astype(float)


# ============================================================
# 2. 反馈线性化控制器
# ============================================================

class FeedbackLinearizationController:
    """输入-输出反馈线性化控制器
    
    步骤:
    1. 计算输出的Lie导数，确定相对阶 r
    2. 构造线性化坐标变换 z = T(x)
    3. 设计线性控制器 v 使 z^(r) = v
    4. 反变换得实际控制输入 u
    
    对于相对阶为 n 的系统，可完全线性化。
    对于相对阶 < n 的系统，需分析零动态稳定性。
    """
    def __init__(self, system: NonlinearSystem, relative_degree: int = 2):
        self.sys = system
        self.r = relative_degree
        
        # 线性化后的PD控制器增益
        self.Kp = 10.0
        self.Kd = 5.0
    
    def lie_derivative(self, f: Callable, h: Callable, x: np.ndarray, 
                       order: int = 1) -> float:
        """计算Lie导数 L_f^k h(x)
        数值近似"""
        eps = 1e-6
        if order == 0:
            return h(x)
        
        # L_f h ≈ (h(x+eps*f(x)) - h(x)) / eps
        dh = np.zeros(len(x))
        for i in range(len(x)):
            x_plus = x.copy()
            x_plus[i] += eps
            x_minus = x.copy()
            x_minus[i] -= eps
            dh[i] = (h(x_plus) - h(x_minus)) / (2*eps)
        
        lf_h = np.dot(dh, f(x))
        
        if order == 1:
            return lf_h
        else:
            # 递归
            def new_h(x_):
                dh_ = np.zeros(len(x_))
                for i in range(len(x_)):
                    xp = x_.copy(); xp[i] += eps
                    xm = x_.copy(); xm[i] -= eps
                    dh_[i] = (h(xp) - h(xm)) / (2*eps)
                return np.dot(dh_, f(x_))
            return self.lie_derivative(f, new_h, x, order-1)
    
    def compute_control(self, x: np.ndarray, ref: float, 
                        ref_dot: float = 0.0) -> Tuple[float, dict]:
        """计算反馈线性化控制量
        
        u = (v - L_f^r h(x)) / (L_g L_f^{r-1} h(x))
        其中 v = ref^(r) - Kd*e_dot - Kp*e
        """
        f = self.sys.f
        g = self.sys.g
        h = self.sys.h
        
        # 计算 L_f^r h(x)
        Lf_r_h = self.lie_derivative(f, h, x, self.r)
        
        # 计算 L_g L_f^{r-1} h(x)
        def lf_rm1_h(x_):
            return self.lie_derivative(f, h, x_, self.r - 1)
        
        eps = 1e-6
        Lg_Lf_rm1_h = 0.0
        for i in range(len(x)):
            xp = x.copy(); xp[i] += eps
            xm = x.copy(); xm[i] -= eps
            Lg_Lf_rm1_h += g(x)[i] * (lf_rm1_h(xp) - lf_rm1_h(xm)) / (2*eps)
        
        # 误差信号
        e = h(x) - ref
        
        # 误差导数（数值）
        y_dot = self.lie_derivative(f, h, x, 1)
        e_dot = y_dot - ref_dot
        
        # 线性控制律: v = -Kp*e - Kd*e_dot
        v = -self.Kp * e - self.Kd * e_dot
        
        # 反馈线性化控制量
        if abs(Lg_Lf_rm1_h) > 1e-8:
            u = (v - Lf_r_h) / Lg_Lf_rm1_h
        else:
            u = 0.0
        
        u = np.clip(u, -100, 100)
        
        info = {'e': e, 'e_dot': e_dot, 'v': v, 
                'Lf_r_h': Lf_r_h, 'Lg_Lf_rm1_h': Lg_Lf_rm1_h}
        return u, info


# ============================================================
# 3. 滑模控制器
# ============================================================

class SlidingModeController:
    """滑模变结构控制器
    
    滑模面: s = c*e + e_dot
    控制律: u = u_eq + u_sw
      u_eq: 等效控制（维持在滑模面上）
      u_sw: 切换控制（到达滑模面）
      u_sw = -k * sat(s/phi)  (饱和函数替代符号函数，减小抖振)
    
    优点: 对匹配不确定性具有完全鲁棒性
    """
    def __init__(self, c: float = 5.0, k: float = 20.0, 
                 phi: float = 0.5, eta: float = 2.0):
        self.c = c        # 滑模面斜率
        self.k = k        # 切换增益
        self.phi = phi    # 边界层厚度
        self.eta = eta    # 到达速率
        self.prev_e = 0.0
        self.dt = 0.01
    
    def sat(self, x):
        """饱和函数"""
        if abs(x) <= 1:
            return x
        return np.sign(x)
    
    def compute_control(self, x: np.ndarray, ref: float,
                        system: NonlinearSystem) -> Tuple[float, dict]:
        """计算滑模控制量"""
        h = system.h
        f = system.f
        g = system.g
        
        e = h(x) - ref
        
        # e_dot (数值)
        eps = 1e-6
        dh = np.zeros(len(x))
        for i in range(len(x)):
            xp = x.copy(); xp[i] += eps
            xm = x.copy(); xm[i] -= eps
            dh[i] = (h(xp) - h(xm)) / (2*eps)
        y_dot = np.dot(dh, f(x))  # 不含u的部分
        e_dot = y_dot  # 近似
        
        # 滑模面
        s = self.c * e + e_dot
        
        # 等效控制 (ds/dt = 0 求解)
        # 需要 Lf^2 h 和 Lg Lf h
        def lf_h(x_):
            dh_ = np.zeros(len(x_))
            for i in range(len(x_)):
                xp = x_.copy(); xp[i] += eps
                xm = x_.copy(); xm[i] -= eps
                dh_[i] = (h(xp) - h(xm)) / (2*eps)
            return np.dot(dh_, f(x_))
        
        # 数值求LgLf_h
        Lg_Lf_h = 0.0
        for i in range(len(x)):
            xp = x.copy(); xp[i] += eps
            xm = x.copy(); xm[i] -= eps
            Lg_Lf_h += g(x)[i] * (lf_h(xp) - lf_h(xm)) / (2*eps)
        
        # 简化的等效控制
        if abs(Lg_Lf_h) > 1e-8:
            u_eq = -y_dot / Lg_Lf_h
        else:
            u_eq = 0.0
        
        # 切换控制
        u_sw = -self.k * self.sat(s / self.phi)
        
        # 到达律
        u = u_eq + u_sw - self.eta * np.sign(s)
        u = np.clip(u, -100, 100)
        
        info = {'e': e, 's': s, 'u_eq': u_eq, 'u_sw': u_sw}
        return u, info


# ============================================================
# 4. 反步法控制器 (Backstepping)
# ============================================================

class BacksteppingController:
    """反步法(Backstepping)控制器
    
    适用于严格反馈形式的非线性系统:
      dx1/dt = f1(x1) + g1(x1)*x2
      dx2/dt = f2(x1,x2) + g2(x1,x2)*u
    
    递推设计:
    Step 1: 设虚拟控制 x2d 使 x1 跟踪参考
    Step 2: 设实际控制 u 使 x2 跟踪 x2d
    
    李雅普诺夫函数逐步构造。
    """
    def __init__(self, k1: float = 5.0, k2: float = 8.0,
                 gamma1: float = 1.0, gamma2: float = 1.0):
        self.k1 = k1  # 第一层增益
        self.k2 = k2  # 第二层增益
        self.gamma1 = gamma1  # 自适应增益1
        self.gamma2 = gamma2  # 自适应增益2
        # 不确定参数估计
        self.theta1_hat = 0.0
        self.theta2_hat = 0.0
    
    def compute_control(self, x: np.ndarray, ref: float,
                        system: NonlinearSystem) -> Tuple[float, dict]:
        """反步法控制（针对二阶严格反馈系统）"""
        x1, x2 = x[0], x[1]
        
        # Step 1: 虚拟控制设计
        e1 = x1 - ref
        alpha = -self.k1 * e1  # 虚拟控制律（x2的期望值）
        
        # 自适应项（处理未知参数）
        alpha_adaptive = -self.theta1_hat * e1
        alpha += alpha_adaptive
        
        # Step 2: 实际控制设计
        e2 = x2 - alpha
        
        # 李雅普诺夫导数设计
        v = -self.k2 * e2 - e1 + self.theta2_hat * x2
        u = v
        u = np.clip(u, -100, 100)
        
        # 参数自适应更新
        self.theta1_hat += self.gamma1 * e1 * e1 * 0.01
        self.theta2_hat -= self.gamma2 * e2 * x2 * 0.01
        
        info = {'e1': e1, 'e2': e2, 'alpha': alpha, 
                'theta1': self.theta1_hat, 'theta2': self.theta2_hat}
        return u, info


# ============================================================
# 5. 典型非线性系统
# ============================================================

def create_van_der_pol_system(mu=1.0, x0=None):
    """Van der Pol振荡器: x'' - mu*(1-x^2)*x' + x = u
    重写为: dx1/dt = x2
            dx2/dt = mu*(1-x1^2)*x2 - x1 + u
    """
    if x0 is None:
        x0 = np.array([0.5, 0.0])
    
    f = lambda x: np.array([x[1], mu*(1-x[0]**2)*x[1] - x[0]])
    g = lambda x: np.array([0.0, 1.0])
    h = lambda x: x[0]
    
    return NonlinearSystem(f, g, h, x0, "Van der Pol")


def create_duffing_system(alpha=1.0, beta=-1.0, delta=0.2, x0=None):
    """Duffing振子: x'' + delta*x' + alpha*x + beta*x^3 = u
    """
    if x0 is None:
        x0 = np.array([1.0, 0.0])
    
    f = lambda x: np.array([x[1], -delta*x[1] - alpha*x[0] - beta*x[0]**3])
    g = lambda x: np.array([0.0, 1.0])
    h = lambda x: x[0]
    
    return NonlinearSystem(f, g, h, x0, "Duffing")


def create_pendulum_system(m=1.0, l=1.0, b=0.1, g_val=9.81, x0=None):
    """非线性摆: m*l^2*theta'' + b*theta' + m*g*l*sin(theta) = u
    """
    if x0 is None:
        x0 = np.array([0.5, 0.0])
    
    f = lambda x: np.array([x[1], -b/(m*l**2)*x[1] - g_val/l*np.sin(x[0])])
    g = lambda x: np.array([0.0, 1.0/(m*l**2)])
    h = lambda x: x[0]
    
    return NonlinearSystem(f, g, h, x0, "Pendulum")


# ============================================================
# 6. 仿真主程序
# ============================================================

def run_simulation(dt=0.005, T=10.0):
    """运行完整仿真"""
    t = np.arange(0, T, dt)
    N = len(t)
    results = {}
    
    # --- Van der Pol + 反馈线性化 ---
    print('  [1] Van der Pol + 反馈线性化...')
    sys1 = create_van_der_pol_system(mu=1.0, x0=np.array([2.0, 0.0]))
    fl_ctrl = FeedbackLinearizationController(sys1, relative_degree=2)
    
    y1 = np.zeros(N)
    u1 = np.zeros(N)
    e1 = np.zeros(N)
    
    for i in range(N):
        ref = 0.0  # 稳定到原点
        u, info = fl_ctrl.compute_control(sys1.state, ref)
        y = sys1.step(u, dt)
        y1[i] = y
        u1[i] = u
        e1[i] = info['e']
    
    results['vdp_fl'] = {'t': t, 'y': y1, 'u': u1, 'e': e1}
    
    # --- Van der Pol + 滑模控制 ---
    print('  [2] Van der Pol + 滑模控制...')
    sys2 = create_van_der_pol_system(mu=1.0, x0=np.array([2.0, 0.0]))
    sm_ctrl = SlidingModeController(c=5.0, k=15.0, phi=0.3)
    
    y2 = np.zeros(N)
    u2 = np.zeros(N)
    s2 = np.zeros(N)
    
    for i in range(N):
        ref = 0.0
        u, info = sm_ctrl.compute_control(sys2.state, ref, sys2)
        y = sys2.step(u, dt)
        y2[i] = y
        u2[i] = u
        s2[i] = info['s']
    
    results['vdp_sm'] = {'t': t, 'y': y2, 'u': u2, 's': s2}
    
    # --- Duffing + 反步法 ---
    print('  [3] Duffing振子 + 反步法...')
    sys3 = create_duffing_system(x0=np.array([1.5, 0.0]))
    bs_ctrl = BacksteppingController(k1=6.0, k2=10.0)
    
    y3 = np.zeros(N)
    u3 = np.zeros(N)
    e3 = np.zeros(N)
    
    for i in range(N):
        ref = 0.5 * np.sin(0.5 * t[i])  # 跟踪正弦
        u, info = bs_ctrl.compute_control(sys3.state, ref, sys3)
        y = sys3.step(u, dt)
        y3[i] = y
        u3[i] = u
        e3[i] = info['e1']
    
    results['duffing_bs'] = {'t': t, 'y': y3, 'u': u3, 'e': e3,
                              'ref': 0.5 * np.sin(0.5 * t)}
    
    # --- 非线性摆 + 反馈线性化 ---
    print('  [4] 非线性摆 + 反馈线性化...')
    sys4 = create_pendulum_system(x0=np.array([1.0, 0.0]))
    fl_ctrl2 = FeedbackLinearizationController(sys4, relative_degree=2)
    fl_ctrl2.Kp = 20.0
    fl_ctrl2.Kd = 8.0
    
    y4 = np.zeros(N)
    u4 = np.zeros(N)
    
    for i in range(N):
        ref = np.pi  # 摆到上方（不稳定平衡）
        u, info = fl_ctrl2.compute_control(sys4.state, ref)
        y = sys4.step(u, dt)
        y4[i] = y
        u4[i] = u
    
    results['pendulum_fl'] = {'t': t, 'y': y4, 'u': u4}
    
    return results


def plot_results(output_dir='.'):
    """绘制所有结果"""
    print('\n[非线性控制] 运行仿真...')
    res = run_simulation()
    
    fig = plt.figure(figsize=(16, 14))
    fig.suptitle('非线性控制仿真 - 反馈线性化/滑模/反步法', 
                 fontsize=14, fontweight='bold')
    
    # (1) Van der Pol + 反馈线性化
    ax1 = fig.add_subplot(3, 2, 1)
    ax1.plot(res['vdp_fl']['t'], res['vdp_fl']['y'], 'b-', linewidth=1.2)
    ax1.axhline(y=0, color='k', linestyle=':', alpha=0.5)
    ax1.set_ylabel('输出 x1')
    ax1.set_title('Van der Pol + 反馈线性化')
    ax1.grid(True, alpha=0.3)
    
    ax2 = fig.add_subplot(3, 2, 2)
    ax2.plot(res['vdp_sm']['t'], res['vdp_sm']['y'], 'r-', linewidth=1.2)
    ax2.axhline(y=0, color='k', linestyle=':', alpha=0.5)
    ax2.set_ylabel('输出 x1')
    ax2.set_title('Van der Pol + 滑模控制')
    ax2.grid(True, alpha=0.3)
    
    # (2) Duffing + 反步法
    ax3 = fig.add_subplot(3, 2, 3)
    ax3.plot(res['duffing_bs']['t'], res['duffing_bs']['ref'], 
             'k--', linewidth=1.5, label='参考')
    ax3.plot(res['duffing_bs']['t'], res['duffing_bs']['y'], 
             'g-', linewidth=1.2, label='输出')
    ax3.set_ylabel('输出')
    ax3.set_title('Duffing振子 + 反步法')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # (3) 非线性摆
    ax4 = fig.add_subplot(3, 2, 4)
    ax4.plot(res['pendulum_fl']['t'], res['pendulum_fl']['y'], 'm-', linewidth=1.2)
    ax4.axhline(y=np.pi, color='k', linestyle=':', alpha=0.5, label=r'$\pi$ (目标)')
    ax4.set_ylabel('角度 (rad)')
    ax4.set_title('非线性摆 + 反馈线性化 (稳定到上方)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # (4) 控制信号对比
    ax5 = fig.add_subplot(3, 2, 5)
    ax5.plot(res['vdp_fl']['t'], res['vdp_fl']['u'], 'b-', linewidth=1.0, 
             label='反馈线性化')
    ax5.plot(res['vdp_sm']['t'], res['vdp_sm']['u'], 'r-', linewidth=1.0, 
             label='滑模', alpha=0.7)
    ax5.set_ylabel('控制量 u')
    ax5.set_xlabel('时间 (s)')
    ax5.set_title('控制信号对比')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # (5) 滑模面
    ax6 = fig.add_subplot(3, 2, 6)
    ax6.plot(res['vdp_sm']['t'], res['vdp_sm']['s'], 'r-', linewidth=1.0)
    ax6.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax6.set_ylabel('滑模面 s')
    ax6.set_xlabel('时间 (s)')
    ax6.set_title('滑模面演化')
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    path = os.path.join(output_dir, 'nonlinear_control_simulation.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  已保存: {path}')
    
    return res


# ============================================================
# 主入口
# ============================================================

if __name__ == '__main__':
    print('=' * 60)
    print('非线性控制仿真 - 反馈线性化 / 滑模 / 反步法')
    print('=' * 60)
    
    output_dir = os.path.dirname(os.path.abspath(__file__))
    plot_results(output_dir)
    
    print('\n' + '=' * 60)
    print('仿真完成！')
    print('=' * 60)
