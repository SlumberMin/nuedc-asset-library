"""
模型预测控制 MPC 仿真验证
==========================
对二阶系统进行MPC控制，验证约束处理和跟踪能力
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

class MPC_Simulator:
    """MPC控制器（Python版，用于仿真验证）"""
    def __init__(self, A, B, C, Np=15, Nc=3, Q=1.0, R=0.1, u_min=-50, u_max=50):
        self.A, self.B, self.C = np.array(A), np.array(B).reshape(-1,1), np.array(C).reshape(1,-1)
        self.Np, self.Nc = Np, Nc
        self.Q, self.R = Q, R
        self.u_min, self.u_max = u_min, u_max
        self.nx = A.shape[0]
        self.x = np.zeros((self.nx, 1))
        self.u_last = 0
        
    def update(self, ref):
        du = np.zeros(self.Nc)
        lr = 0.01
        for _ in range(30):
            xp = self.x.copy()
            y_pred = []
            u_acc = self.u_last
            for k in range(self.Np):
                ci = min(k, self.Nc - 1)
                u_acc = np.clip(u_acc + du[ci], self.u_min, self.u_max)
                xp = self.A @ xp + self.B * u_acc
                y_pred.append(float(self.C @ xp))
            
            y_pred = np.array(y_pred)
            err = y_pred - ref
            grad = np.zeros(self.Nc)
            for j in range(self.Nc):
                grad[j] = -2 * self.Q * np.sum(err[j:])
                grad[j] += 2 * self.R * du[j]
            du -= lr * grad
        
        self.u_last = np.clip(self.u_last + du[0], self.u_min, self.u_max)
        self.x = self.A @ self.x + self.B * self.u_last
        return float(self.C @ self.x), self.u_last

def simulate():
    dt = 0.01
    T = 5.0
    N = int(T / dt)
    
    # 二阶系统离散化: x'' + 2*x' + 5*x = u
    # z变换离散化
    A = np.array([[1, dt], [-5*dt, 1-2*dt]])
    B = np.array([[0], [dt]])
    C = np.array([[1, 0]])
    
    mpc = MPC_Simulator(A, B, C, Np=15, Nc=3, Q=10, R=0.5, u_min=-50, u_max=50)
    
    t = np.arange(N) * dt
    ref = np.zeros(N)
    ref[int(0.5/dt):] = 1.0
    ref[int(2.5/dt):] = 0.5
    
    y_out, u_out = np.zeros(N), np.zeros(N)
    for i in range(N):
        y_out[i], u_out[i] = mpc.update(ref[i])
    
    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    axes[0].plot(t, ref, 'r--', label='参考', linewidth=2)
    axes[0].plot(t, y_out, 'b-', label='MPC输出')
    axes[0].set_title('MPC模型预测控制仿真'); axes[0].legend(); axes[0].grid(True)
    axes[1].plot(t, u_out, 'g-', label='控制量')
    axes[1].set_xlabel('时间(s)'); axes[1].legend(); axes[1].grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mpc_sim.png'), dpi=150)
    plt.close('all')
    print("MPC仿真完成")

if __name__ == '__main__':
    simulate()
