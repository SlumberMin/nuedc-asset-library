"""
迭代学习控制ILC仿真验证 - 展示多次迭代收敛过程
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

class ILC:
    def __init__(self, N, Lp=0.5, Li=0.0, Ld=0.1, Q=0.95, law='PD'):
        self.N = N; self.Lp, self.Li, self.Ld, self.Q = Lp, Li, Ld, Q
        self.law = law
        self.u_prev = np.zeros(N)
        self.e_prev = np.zeros(N)
        self.e_sum = np.zeros(N)
        self.iter = 0
    
    def learn(self, e_history, u_history):
        """一次迭代的学习更新"""
        u_new = np.zeros(self.N)
        for k in range(self.N):
            u_base = self.Q * self.u_prev[k]
            if self.iter == 0:
                u_new[k] = self.Lp * e_history[k]
            else:
                ep = self.e_prev[k]
                corr = self.Lp * ep
                if self.law in ['D', 'PD'] and k > 0:
                    corr += self.Ld * (ep - self.e_prev[k-1])
                if self.law == 'PID':
                    self.e_sum[k] += ep
                    corr += self.Li * self.e_sum[k]
                u_new[k] = u_base + corr
        self.u_prev = u_new.copy()
        self.e_prev = e_history.copy()
        self.iter += 1
        return u_new

def simulate():
    dt = 0.001; T = 2.0; N = int(T/dt)
    t = np.arange(N) * dt
    
    # 参考轨迹: 正弦波
    ref = np.sin(2 * np.pi * t / T)
    
    # 被控对象: 二阶系统
    def plant_sim(u_seq):
        x, dx = 0, 0
        y = np.zeros(N)
        for i in range(N):
            ddx = -2*dx + u_seq[i] + 0.2*np.sin(3*t[i])  # 含周期扰动
            dx += ddx * dt; x += dx * dt; y[i] = x
        return y
    
    ilc = ILC(N, Lp=1.5, Ld=0.3, Q=0.95, law='PD')
    
    n_iters = 15
    errors = []
    y_iters = {}
    
    u = np.zeros(N)
    for it in range(n_iters):
        y = plant_sim(u)
        e = ref - y
        errors.append(np.sqrt(np.mean(e**2)))
        if it in [0, 2, 4, 9, 14]:
            y_iters[it] = y.copy()
        u = ilc.learn(e, u)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 跟踪效果
    axes[0,0].plot(t, ref, 'k--', label='参考轨迹', linewidth=2)
    for it, y in y_iters.items():
        axes[0,0].plot(t, y, label=f'第{it+1}次迭代', alpha=0.8)
    axes[0,0].set_title('ILC轨迹跟踪收敛过程'); axes[0,0].legend(); axes[0,0].grid(True)
    
    # 误差收敛
    axes[0,1].semilogy(range(1, n_iters+1), errors, 'bo-', linewidth=2)
    axes[0,1].set_title('RMSE随迭代次数收敛'); axes[0,1].set_xlabel('迭代次数'); axes[0,1].grid(True)
    axes[0,1].set_ylabel('RMSE')
    
    # 不同学习律对比
    for law, label in [('P','P型'),('D','D型'),('PD','PD型'),('PID','PID型')]:
        ilc2 = ILC(N, Lp=1.5, Li=0.05, Ld=0.3, Q=0.95, law=law)
        u2 = np.zeros(N); errs = []
        for _ in range(15):
            y2 = plant_sim(u2); e2 = ref - y2
            errs.append(np.sqrt(np.mean(e2**2)))
            u2 = ilc2.learn(e2, u2)
        axes[1,0].semilogy(range(1,16), errs, label=label, linewidth=1.5)
    axes[1,0].set_title('不同学习律收敛对比'); axes[1,0].legend(); axes[1,0].grid(True)
    axes[1,0].set_xlabel('迭代次数'); axes[1,0].set_ylabel('RMSE')
    
    # 最终跟踪效果
    axes[1,1].plot(t, ref, 'k--', label='参考', linewidth=2)
    axes[1,1].plot(t, y_iters.get(14, y), 'b-', label='第15次迭代', linewidth=1.5)
    axes[1,1].fill_between(t, ref-0.05, ref+0.05, alpha=0.2, color='green', label='±5%误差带')
    axes[1,1].set_title('最终跟踪效果'); axes[1,1].legend(); axes[1,1].grid(True)
    axes[1,1].set_xlabel('时间(s)')
    
    plt.suptitle('迭代学习控制ILC仿真验证', fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ilc_sim.png'), dpi=150)
    plt.close('all')
    print(f"ILC仿真完成，误差从{errors[0]:.4f}收敛到{errors[-1]:.4f}")

if __name__ == '__main__':
    simulate()
