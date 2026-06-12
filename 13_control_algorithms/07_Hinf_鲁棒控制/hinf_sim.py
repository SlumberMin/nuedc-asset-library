"""
H∞鲁棒控制仿真验证 - 与LQR对比鲁棒性
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False
from scipy.linalg import solve_discrete_are

def simulate():
    dt = 0.01; T = 5.0; N = int(T/dt)
    t = np.arange(N)*dt
    
    # 二阶系统
    A_c = np.array([[0,1],[0,-2]])
    B_c = np.array([[0],[1]])
    nx = 2; I = np.eye(nx)
    A = I + A_c*dt; B = B_c*dt
    
    # H∞控制器: 修改的Riccati方程
    gamma = 2.0
    B1 = np.array([[0.1],[0.5]])  # 扰动输入
    C1 = np.array([[1, 0]])  # 性能输出
    Q_h = C1.T @ C1 + 0.01*I
    rho = 1.0 / gamma**2
    
    # 迭代求解修改的Riccati
    BB = B @ B.T - rho * B1 @ B1.T
    X = Q_h.copy()
    for _ in range(200):
        AXA = A.T @ X @ A
        Xn = AXA + Q_h - X @ BB @ X
        if np.max(np.abs(Xn - X)) < 1e-8: break
        X = Xn
    
    K_hinf = np.linalg.inv(B.T @ X @ B + 0.01) @ B.T @ X @ A
    
    # LQR对比
    Q_lqr = np.diag([1, 1]); R = np.array([[0.1]])
    P = solve_discrete_are(A, B, Q_lqr, R)
    K_lqr = np.linalg.inv(R + B.T@P@B) @ B.T@P@A
    
    print(f"H∞增益: {K_hinf.flatten()}")
    print(f"LQR增益: {K_lqr.flatten()}")
    
    # 仿真对比
    x_h = np.array([1.0, 0]); x_l = x_h.copy()
    y_h, y_l = np.zeros(N), np.zeros(N)
    
    for i in range(N):
        dist = 0.3 * np.sin(5*t[i]) + 0.1*np.random.randn()  # 持续扰动
        u_h = -K_hinf @ x_h; u_l = -K_lqr @ x_l
        x_h = A@x_h + B*u_h + B1*dist; y_h[i] = x_h[0]
        x_l = A@x_l + B*u_l + B1*dist; y_l[i] = x_l[0]
    
    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    axes[0].plot(t, y_h, 'b-', label='H∞鲁棒控制', linewidth=2)
    axes[0].plot(t, y_l, 'r--', label='LQR控制', alpha=0.8)
    axes[0].set_title('H∞ vs LQR 鲁棒性对比（持续扰动）'); axes[0].legend(); axes[0].grid(True)
    axes[0].set_ylabel('状态x1')
    
    # 不同gamma对比
    for gamma_val in [1.5, 2.0, 3.0, 5.0]:
        rho_v = 1.0/gamma_val**2
        BB_v = B@B.T - rho_v*B1@B1.T
        X_v = Q_h.copy()
        for _ in range(200):
            Xn_v = A.T@X_v@A + Q_h - X_v@BB_v@X_v
            if np.max(np.abs(Xn_v-X_v))<1e-8: break
            X_v = Xn_v
        K_v = np.linalg.inv(B.T@X_v@B+0.01)@B.T@X_v@A
        x_v = np.array([1.0,0]); y_v = np.zeros(N)
        for i in range(N):
            dist = 0.3*np.sin(5*t[i])
            u = -K_v@x_v; x_v = A@x_v + B*u + B1*dist; y_v[i] = x_v[0]
        axes[1].plot(t, y_v, label=f'γ={gamma_val}')
    
    axes[1].set_title('不同γ值的H∞控制效果'); axes[1].legend(); axes[1].grid(True)
    axes[1].set_xlabel('时间(s)'); axes[1].set_ylabel('状态x1')
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hinf_sim.png'), dpi=150)
    plt.close('all')
    print("H∞仿真完成")

if __name__ == '__main__':
    simulate()
