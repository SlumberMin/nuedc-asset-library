"""
LQR线性二次调节器仿真验证
===========================
以倒立摆线性化模型为例验证LQR控制效果
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
    t = np.arange(N) * dt
    
    # 倒立摆线性化模型 (M=1kg, m=0.1kg, l=0.5m, g=9.8)
    # 状态: [x, x_dot, theta, theta_dot]
    M, m, l, g = 1.0, 0.1, 0.5, 9.8
    A_c = np.array([[0,1,0,0],
                     [0,0,-m*g/M,0],
                     [0,0,0,1],
                     [0,0,(M+m)*g/(M*l),0]])
    B_c = np.array([[0],[1/M],[0],[-1/(M*l)]])
    
    # 离散化
    nx = 4; I = np.eye(nx)
    A = I + A_c * dt
    B = B_c * dt
    
    # LQR权重
    Q = np.diag([10, 1, 100, 10])
    R = np.array([[0.1]])
    
    # 求解Riccati方程
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
    
    print(f"最优反馈增益 K = {K.flatten()}")
    
    # 仿真
    x = np.array([0, 0, 0.2, 0])  # 初始偏角0.2rad
    x_hist = np.zeros((N, nx))
    u_hist = np.zeros(N)
    
    for i in range(N):
        u = -K @ x
        u_hist[i] = u[0]
        x_hist[i] = x
        x = A @ x + B @ u[0] + np.random.randn(nx) * 0.001  # 加噪声
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes[0,0].plot(t, x_hist[:,0]); axes[0,0].set_title('位移x(m)'); axes[0,0].grid(True)
    axes[0,1].plot(t, np.rad2deg(x_hist[:,2])); axes[0,1].set_title('角度θ(°)'); axes[0,1].grid(True)
    axes[1,0].plot(t, x_hist[:,1]); axes[1,0].set_title('速度(m/s)'); axes[1,0].grid(True)
    axes[1,1].plot(t, u_hist); axes[1,1].set_title('控制力(N)'); axes[1,1].grid(True)
    plt.suptitle('LQR倒立摆控制仿真', fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lqr_sim.png'), dpi=150)
    plt.close('all')
    print("LQR仿真完成")

if __name__ == '__main__':
    simulate()
