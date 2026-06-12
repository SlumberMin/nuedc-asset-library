#!/usr/bin/env python3
"""
最优控制仿真 - 动态规划 / LQR / MPC
=====================================
基于最优控制理论的控制器设计与仿真。
适合电赛中需要能量最优、时间最优等性能指标的控制系统。

包含:
  1. 离散动态规划(DP)求解有限时域最优控制
  2. 线性二次调节器(LQR)无限时域最优
  3. 模型预测控制(MPC)滚动优化
  4. 最优轨迹规划
  5. 性能指标对比

依赖: numpy, matplotlib, scipy
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import linalg
from typing import Tuple, Optional
from dataclasses import dataclass, field
import os

# ============================================================
# 1. 离散线性系统模型
# ============================================================

@dataclass
class DiscreteLinearSystem:
    """离散线性时不变系统: x(k+1) = A*x(k) + B*u(k), y(k) = C*x(k)
    
    状态: [位置, 速度] 或 [角度, 角速度] 等
    """
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray = None
    state: np.ndarray = None
    
    def __post_init__(self):
        self.n = self.A.shape[0]
        self.m = self.B.shape[1]
        if self.C is None:
            self.C = np.eye(self.n)
        if self.state is None:
            self.state = np.zeros(self.n)
    
    def step(self, u: np.ndarray) -> np.ndarray:
        """一步状态转移"""
        self.state = self.A @ self.state + self.B @ u
        return self.C @ self.state
    
    def reset(self, x0=None):
        self.state = x0 if x0 is not None else np.zeros(self.n)


def create_double_integrator(dt=0.1):
    """双积分器模型（典型的位置控制）
    x = [pos, vel], u = 加速度
    连续: x' = [0 1; 0 0]*x + [0; 1]*u
    离散化
    """
    A_c = np.array([[0, 1], [0, 0]])
    B_c = np.array([[0], [1]])
    
    # 简单前向欧拉离散化
    A = np.eye(2) + A_c * dt
    B = B_c * dt
    
    return DiscreteLinearSystem(A, B)


def create_motor_model(dt=0.01):
    """电机速度/位置模型
    x = [角位置, 角速度, 电流], u = 电压
    """
    J = 0.01   # 转动惯量
    B_f = 0.1  # 摩擦系数
    Kt = 1.0   # 转矩常数
    Ke = 0.1   # 反电势常数
    R = 1.0    # 电枢电阻
    L = 0.01   # 电感
    
    A_c = np.array([
        [0, 1, 0],
        [0, -B_f/J, Kt/J],
        [0, -Ke/L, -R/L]
    ])
    B_c = np.array([[0], [0], [1/L]])
    
    # Euler离散化
    A = np.eye(3) + A_c * dt
    B = B_c * dt
    C = np.array([[1, 0, 0]])  # 输出角位置
    
    return DiscreteLinearSystem(A, B, C)


# ============================================================
# 2. 动态规划 (Dynamic Programming)
# ============================================================

class DynamicProgramming:
    """离散动态规划求解有限时域LQR问题
    
    代价函数: J = x(N)^T*Qf*x(N) + sum_{k=0}^{N-1} [x(k)^T*Q*x(k) + u(k)^T*R*u(k)]
    
    通过Bellman方程逆推求解:
    V(k) = min_u { x^T*Q*x + u^T*R*u + V(k+1) }
    
    解析解: u(k) = -K(k)*x(k), 其中 K(k) 由Riccati递推得到
    """
    def __init__(self, system: DiscreteLinearSystem,
                 Q: np.ndarray, R: np.ndarray, Qf: np.ndarray = None,
                 N: int = 50):
        self.sys = system
        self.Q = Q
        self.R = R
        self.Qf = Qf if Qf is not None else Q.copy()
        self.N = N
        
        # 预计算反馈增益序列
        self.K_seq, self.P_seq = self._solve_riccati_backward()
    
    def _solve_riccati_backward(self):
        """逆向Riccati递推"""
        A, B = self.sys.A, self.sys.B
        n, m = self.sys.n, self.sys.m
        N = self.N
        
        P = self.Qf.copy()
        K_seq = np.zeros((N, m, n))
        P_seq = np.zeros((N+1, n, n))
        P_seq[N] = P.copy()
        
        for k in range(N-1, -1, -1):
            # 最优增益: K = (R + B^T*P*B)^{-1} * B^T*P*A
            BtPB = B.T @ P @ B
            BtPA = B.T @ P @ A
            K = np.linalg.solve(self.R + BtPB, BtPA)
            K_seq[k] = K
            
            # Riccati更新: P = Q + A^T*P*A - A^T*P*B*K
            P = self.Q + A.T @ P @ A - A.T @ P @ B @ K
            P_seq[k] = P.copy()
        
        return K_seq, P_seq
    
    def compute_control(self, x: np.ndarray, k: int) -> np.ndarray:
        """计算时刻k的最优控制"""
        K = self.K_seq[min(k, self.N-1)]
        return -K @ x
    
    def simulate(self, x0: np.ndarray) -> dict:
        """开环最优控制仿真"""
        n = self.sys.n
        N = self.N
        
        x_hist = np.zeros((N+1, n))
        u_hist = np.zeros((N, self.sys.m))
        J_hist = np.zeros(N+1)
        
        x = x0.copy()
        x_hist[0] = x
        J = 0.0
        
        for k in range(N):
            u = self.compute_control(x, k)
            stage_cost = x @ self.Q @ x + u.flatten() @ self.R @ u.flatten()
            J += stage_cost
            J_hist[k] = J
            
            x = self.sys.A @ x + self.sys.B @ u
            x_hist[k+1] = x
            u_hist[k] = u.flatten()
        
        # 终端代价
        terminal_cost = x @ self.Qf @ x
        J += terminal_cost
        J_hist[N] = J
        
        return {
            'x': x_hist, 'u': u_hist, 'J': J_hist,
            'total_cost': J, 'K_seq': self.K_seq,
            'P_seq': self.P_seq
        }


# ============================================================
# 3. 线性二次调节器 (LQR)
# ============================================================

class LQRController:
    """无限时域LQR控制器
    
    代价: J = integral [x^T*Q*x + u^T*R*u] dt → ∞
    最优控制: u = -K*x, K = R^{-1}*B^T*P
    其中P是代数Riccati方程的解: A^T*P + P*A - P*B*R^{-1}*B^T*P + Q = 0
    """
    def __init__(self, system: DiscreteLinearSystem,
                 Q: np.ndarray, R: np.ndarray):
        self.sys = system
        self.Q = Q
        self.R = R
        
        # 求解DARE (Discrete Algebraic Riccati Equation)
        self.P = self._solve_dare()
        self.K = self._compute_gain()
    
    def _solve_dare(self) -> np.ndarray:
        """求解离散代数Riccati方程"""
        A, B = self.sys.A, self.sys.B
        Q, R = self.Q, self.R
        
        # 使用scipy的dare求解器（如果可用）
        try:
            from scipy.linalg import solve_discrete_are
            P = solve_discrete_are(A, B, Q, R)
        except:
            # 迭代求解
            P = Q.copy()
            for _ in range(1000):
                P_new = Q + A.T @ P @ A - A.T @ P @ B @ \
                        np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
                if np.max(np.abs(P_new - P)) < 1e-10:
                    break
                P = P_new
        return P
    
    def _compute_gain(self) -> np.ndarray:
        """计算LQR增益 K = (R + B^T*P*B)^{-1} * B^T*P*A"""
        A, B = self.sys.A, self.sys.B
        return np.linalg.solve(
            self.R + B.T @ self.P @ B,
            B.T @ self.P @ A)
    
    def compute_control(self, x: np.ndarray) -> np.ndarray:
        return -self.K @ x
    
    def simulate_closed_loop(self, x0: np.ndarray, T: int = 200) -> dict:
        """闭环仿真"""
        n = self.sys.n
        x_hist = np.zeros((T+1, n))
        u_hist = np.zeros((T, self.sys.m))
        
        x = x0.copy()
        x_hist[0] = x
        
        for k in range(T):
            u = self.compute_control(x)
            x = self.sys.A @ x + self.sys.B @ u
            x_hist[k+1] = x
            u_hist[k] = u.flatten()
        
        return {'x': x_hist, 'u': u_hist, 'K': self.K, 'P': self.P}
    
    def get_closed_loop_poles(self) -> np.ndarray:
        """闭环极点"""
        Acl = self.sys.A - self.sys.B @ self.K
        return np.linalg.eigvals(Acl)


# ============================================================
# 4. 模型预测控制 (MPC)
# ============================================================

class MPCController:
    """模型预测控制器（简化实现）
    
    在每个时刻:
    1. 基于当前状态和模型，预测未来N步
    2. 求解有限时域最优控制问题
    3. 只执行第一步控制
    4. 下一时刻重复
    
    约束:
    - 状态约束: x_min <= x <= x_max
    - 控制约束: u_min <= u <= u_max
    """
    def __init__(self, system: DiscreteLinearSystem,
                 Q: np.ndarray, R: np.ndarray,
                 N: int = 20,
                 u_min: float = -10, u_max: float = 10,
                 x_min: np.ndarray = None, x_max: np.ndarray = None):
        self.sys = system
        self.Q = Q
        self.R = R
        self.N = N
        self.u_min = u_min
        self.u_max = u_max
        self.x_min = x_min
        self.x_max = x_max
    
    def compute_control(self, x0: np.ndarray, x_ref: np.ndarray = None) -> Tuple[np.ndarray, dict]:
        """求解MPC优化问题（无约束时解析解，有约束时简化QP）"""
        A, B = self.sys.A, self.sys.B
        n, m = self.sys.n, self.sys.m
        N = self.N
        
        if x_ref is None:
            x_ref = np.zeros(n)
        
        # 构建预测矩阵: X = Psi*x0 + Gamma*U
        Psi = np.zeros((n*N, n))
        Gamma = np.zeros((n*N, m*N))
        
        for i in range(N):
            A_power = np.linalg.matrix_power(A, i+1)
            Psi[i*n:(i+1)*n] = A_power
            for j in range(i+1):
                A_power_inner = np.linalg.matrix_power(A, i-j)
                Gamma[i*n:(i+1)*n, j*m:(j+1)*m] = A_power_inner @ B
        
        # 构建权重矩阵
        Q_bar = np.kron(np.eye(N), self.Q)
        R_bar = np.kron(np.eye(N), self.R)
        
        # QP: min 0.5*U^T*H*U + f^T*U
        H = Gamma.T @ Q_bar @ Gamma + R_bar
        # f = Gamma^T * Q_bar * (Psi*x0 - X_ref)
        x_ref_vec = np.tile(x_ref, N)
        f = Gamma.T @ Q_bar @ (Psi @ x0 - x_ref_vec)
        
        # 对称化
        H = (H + H.T) / 2
        
        # 无约束解析解
        try:
            U_opt = -np.linalg.solve(H, f)
        except np.linalg.LinAlgError:
            U_opt = -np.linalg.lstsq(H, f, rcond=None)[0]
        
        # 应用控制约束
        U_opt = np.clip(U_opt, self.u_min, self.u_max)
        
        # 只取第一步
        u_mpc = U_opt[:m]
        
        info = {'U_opt': U_opt, 'predicted_x': Psi @ x0 + Gamma @ U_opt}
        return u_mpc, info
    
    def simulate(self, x0: np.ndarray, T: int = 200, 
                 x_ref: np.ndarray = None) -> dict:
        """闭环MPC仿真"""
        n = self.sys.n
        x_hist = np.zeros((T+1, n))
        u_hist = np.zeros((T, self.sys.m))
        
        self.sys.reset(x0)
        x_hist[0] = x0.copy()
        
        for k in range(T):
            u, info = self.compute_control(self.sys.state, x_ref)
            self.sys.step(u)
            x_hist[k+1] = self.sys.state.copy()
            u_hist[k] = u.flatten()
        
        return {'x': x_hist, 'u': u_hist}


# ============================================================
# 5. 时间最优控制
# ============================================================

class TimeOptimalController:
    """Bang-Bang 时间最优控制
    
    代价: J = sum(1) = N (最小时间)
    约束: |u| <= u_max
    
    对于双积分器，最优控制为 bang-bang 形式:
    u = +u_max (先加速) → -u_max (后减速)
    
    切换曲线分析。
    """
    def __init__(self, system: DiscreteLinearSystem, u_max: float = 5.0):
        self.sys = system
        self.u_max = u_max
    
    def compute_control(self, x: np.ndarray, x_target: np.ndarray = None) -> float:
        """Bang-Bang控制（针对双积分器）"""
        if x_target is None:
            x_target = np.zeros(2)
        
        e = x[0] - x_target[0]   # 位置误差
        ed = x[1] - x_target[1]  # 速度误差
        
        # 切换曲线: s = e + |ed|*ed/(2*u_max) = 0
        s = e + ed * abs(ed) / (2 * self.u_max)
        
        if s > 0.1:
            return -self.u_max
        elif s < -0.1:
            return self.u_max
        else:
            # 切换曲线附近，减速
            return -self.u_max * np.sign(ed) if abs(ed) > 0.01 else 0.0
    
    def simulate(self, x0: np.ndarray, x_target: np.ndarray = None, 
                 T: int = 200) -> dict:
        if x_target is None:
            x_target = np.zeros(2)
        
        n = self.sys.n
        x_hist = np.zeros((T+1, n))
        u_hist = np.zeros(T)
        
        self.sys.reset(x0)
        x_hist[0] = x0.copy()
        
        for k in range(T):
            u = self.compute_control(self.sys.state, x_target)
            u = np.array([u])
            self.sys.step(u)
            x_hist[k+1] = self.sys.state.copy()
            u_hist[k] = u[0]
        
        return {'x': x_hist, 'u': u_hist}


# ============================================================
# 6. 仿真主程序
# ============================================================

def run_all_simulations():
    """运行所有最优控制仿真"""
    results = {}
    
    # --- 系统模型 ---
    dt = 0.1
    sys = create_double_integrator(dt)
    x0 = np.array([5.0, 0.0])  # 初始位置=5，速度=0
    
    Q = np.diag([10.0, 1.0])   # 位置权重 > 速度权重
    R = np.array([[0.1]])       # 控制代价
    
    # --- 动态规划 ---
    print('  [1] 动态规划...')
    dp = DynamicProgramming(sys, Q, R, N=50)
    res_dp = dp.simulate(x0)
    results['dp'] = res_dp
    
    # --- LQR ---
    print('  [2] LQR...')
    lqr = LQRController(sys, Q, R)
    res_lqr = lqr.simulate_closed_loop(x0, T=100)
    results['lqr'] = res_lqr
    print(f'      LQR增益 K = {lqr.K}')
    print(f'      闭环极点: {lqr.get_closed_loop_poles()}')
    
    # --- MPC ---
    print('  [3] MPC...')
    mpc = MPCController(sys, Q, R, N=15, u_min=-5, u_max=5)
    sys.reset(x0)
    res_mpc = mpc.simulate(x0, T=100)
    results['mpc'] = res_mpc
    
    # --- 时间最优 ---
    print('  [4] 时间最优控制...')
    sys.reset(x0)
    toc = TimeOptimalController(sys, u_max=5.0)
    res_toc = toc.simulate(x0, T=100)
    results['toc'] = res_toc
    
    # --- 性能对比 ---
    costs = {}
    for name, res in [('DP', res_dp), ('LQR', res_lqr), 
                       ('MPC', res_mpc), ('TOC', res_toc)]:
        x = res['x']
        u = res['u']
        J = sum(x[k] @ Q @ x[k] for k in range(min(len(u), len(x)-1))) + \
            sum(u[k].flatten() @ R @ u[k].flatten() for k in range(len(u)))
        costs[name] = J
    
    results['costs'] = costs
    return results


def plot_results(output_dir='.'):
    """绘制所有结果"""
    print('\n[最优控制] 运行仿真...')
    res = run_all_simulations()
    
    fig = plt.figure(figsize=(16, 14))
    fig.suptitle('最优控制仿真 - 动态规划 / LQR / MPC / 时间最优',
                 fontsize=14, fontweight='bold')
    
    dt = 0.1
    
    # (1) 位置响应对比
    ax1 = fig.add_subplot(3, 2, 1)
    for name, key, color, ls in [
        ('DP', 'dp', 'b', '-'), ('LQR', 'lqr', 'r', '--'),
        ('MPC', 'mpc', 'g', '-.'), ('时间最优', 'toc', 'm', ':')]:
        x = res[key]['x']
        t = np.arange(len(x)) * dt
        ax1.plot(t, x[:, 0], color=color, linestyle=ls, linewidth=1.5, label=name)
    ax1.axhline(y=0, color='k', linestyle=':', alpha=0.3)
    ax1.set_ylabel('位置')
    ax1.set_title('位置响应对比')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # (2) 速度响应
    ax2 = fig.add_subplot(3, 2, 2)
    for name, key, color, ls in [
        ('DP', 'dp', 'b', '-'), ('LQR', 'lqr', 'r', '--'),
        ('MPC', 'mpc', 'g', '-.'), ('时间最优', 'toc', 'm', ':')]:
        x = res[key]['x']
        t = np.arange(len(x)) * dt
        ax2.plot(t, x[:, 1], color=color, linestyle=ls, linewidth=1.5, label=name)
    ax2.set_ylabel('速度')
    ax2.set_title('速度响应对比')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # (3) 控制信号
    ax3 = fig.add_subplot(3, 2, 3)
    for name, key, color, ls in [
        ('DP', 'dp', 'b', '-'), ('LQR', 'lqr', 'r', '--'),
        ('MPC', 'mpc', 'g', '-.'), ('时间最优', 'toc', 'm', ':')]:
        u = res[key]['u']
        t = np.arange(len(u)) * dt
        ax3.plot(t, u.flatten()[:len(t)], color=color, linestyle=ls, 
                 linewidth=1.5, label=name)
    ax3.set_ylabel('控制量 u')
    ax3.set_xlabel('时间 (s)')
    ax3.set_title('控制信号对比')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # (4) DP的Riccati矩阵演化
    ax4 = fig.add_subplot(3, 2, 4)
    P_seq = res['dp']['P_seq']
    t_P = np.arange(len(P_seq)) * dt
    ax4.plot(t_P, P_seq[:, 0, 0], 'b-', label='P[0,0] (位置)')
    ax4.plot(t_P, P_seq[:, 1, 1], 'r-', label='P[1,1] (速度)')
    ax4.plot(t_P, P_seq[:, 0, 1], 'g--', label='P[0,1] (交叉)')
    ax4.set_ylabel('P矩阵元素')
    ax4.set_xlabel('时间 (s)')
    ax4.set_title('动态规划: Riccati矩阵演化')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # (5) 累积代价
    ax5 = fig.add_subplot(3, 2, 5)
    ax5.plot(res['dp']['J'], 'b-', linewidth=1.5, label='DP累积代价')
    ax5.set_ylabel('累积代价 J')
    ax5.set_xlabel('步数')
    ax5.set_title('累积代价函数')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # (6) 性能指标对比
    ax6 = fig.add_subplot(3, 2, 6)
    costs = res['costs']
    names = list(costs.keys())
    values = [costs[n] for n in names]
    colors = ['steelblue', 'coral', 'seagreen', 'mediumpurple']
    bars = ax6.bar(names, values, color=colors[:len(names)])
    ax6.set_ylabel('总代价 J')
    ax6.set_title('性能指标对比')
    ax6.grid(True, alpha=0.3, axis='y')
    
    # 标注数值
    for bar, val in zip(bars, values):
        ax6.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
                 f'{val:.1f}', ha='center', va='bottom', fontsize=10)
    
    plt.tight_layout()
    path = os.path.join(output_dir, 'optimal_control_simulation.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  已保存: {path}')
    
    # --- 电机最优控制 ---
    print('\n[电机最优控制]...')
    plot_motor_optimal(output_dir)
    
    return res


def plot_motor_optimal(output_dir='.'):
    """电机位置最优控制"""
    dt = 0.01
    sys = create_motor_model(dt)
    x0 = np.array([0.0, 0.0, 0.0])
    
    Q = np.diag([100.0, 1.0, 0.1])
    R = np.array([[0.01]])
    
    lqr = LQRController(sys, Q, R)
    T_sim = 200
    x_hist = np.zeros((T_sim+1, 3))
    u_hist = np.zeros(T_sim)
    
    x = np.array([1.0, 0.0, 0.0])  # 目标位置=1
    x_hist[0] = x
    
    for k in range(T_sim):
        u = lqr.compute_control(x)
        x = sys.A @ x + sys.B @ u
        x_hist[k+1] = x
        u_hist[k] = u[0]
    
    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    fig.suptitle('电机位置LQR最优控制', fontsize=14)
    
    t = np.arange(T_sim+1) * dt
    axes[0].plot(t, x_hist[:, 0], 'b-', linewidth=1.5, label='角位置')
    axes[0].axhline(y=0, color='k', linestyle=':', alpha=0.3)
    axes[0].set_ylabel('位置 (rad)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    t_u = np.arange(T_sim) * dt
    axes[1].plot(t_u, u_hist, 'r-', linewidth=1.0, label='控制电压')
    axes[1].set_ylabel('电压 (V)')
    axes[1].set_xlabel('时间 (s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    path = os.path.join(output_dir, 'optimal_motor_lqr.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  已保存: {path}')


# ============================================================
# 主入口
# ============================================================

if __name__ == '__main__':
    print('=' * 60)
    print('最优控制仿真 - 动态规划 / LQR / MPC / 时间最优')
    print('=' * 60)
    
    output_dir = os.path.dirname(os.path.abspath(__file__))
    res = plot_results(output_dir)
    
    print('\n' + '=' * 60)
    print('仿真完成！性能对比:')
    for name, cost in res['costs'].items():
        print(f'  {name}: J = {cost:.2f}')
    print('=' * 60)
