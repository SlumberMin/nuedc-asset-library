"""
滑模控制SMC仿真验证 - 4种趋近律对比
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

def sat(s, phi):
    if abs(s) > phi: return np.sign(s)
    return s / phi if phi > 0 else 0

class SMC:
    def __init__(self, law='exponential', c=5, eps=1.0, k=2.0, alpha=0.5, phi=0.05):
        self.law, self.c, self.eps, self.k = law, c, eps, k
        self.alpha, self.phi = alpha, phi
    
    def update(self, e, e_dot, u_eq=0):
        s = e_dot + self.c * e
        if self.law == 'constant':
            u_sw = self.eps * sat(s, self.phi)
        elif self.law == 'exponential':
            u_sw = self.eps * sat(s, self.phi) + self.k * s
        elif self.law == 'power':
            u_sw = self.k * abs(s)**self.alpha * sat(s, self.phi)
        else:
            u_sw = self.k * abs(s)**self.alpha * sat(s, self.phi) + self.eps * sat(s, self.phi) + 2*self.k*s
        return np.clip(u_eq + u_sw, -100, 100)

def simulate():
    dt = 0.001; T = 3.0; N = int(T/dt)
    t = np.arange(N)*dt
    ref = np.ones(N)  # 阶跃参考
    
    laws = ['constant', 'exponential', 'power', 'combined']
    labels = ['等速趋近律', '指数趋近律', '幂次趋近律', '组合趋近律']
    colors = ['r', 'b', 'g', 'm']
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    for law, label, color in zip(laws, labels, colors):
        # 被控对象: x'' = -2x' + u + d
        x, dx = 0, 0
        smc = SMC(law=law, c=10, eps=3, k=5, alpha=0.7, phi=0.1)
        y_out = np.zeros(N)
        
        for i in range(N):
            e = ref[i] - x
            e_dot = -dx
            u = smc.update(e, e_dot, u_eq=0)
            dist = 0.5 if 1.5 < t[i] < 2.0 else 0
            ddx = -2*dx + u + dist
            dx += ddx * dt
            x += dx * dt
            y_out[i] = x
        
        axes[0].plot(t, y_out, color=color, label=label, linewidth=1.2)
    
    axes[0].plot(t, ref, 'k--', label='参考', linewidth=2)
    axes[0].set_title('SMC四种趋近律对比'); axes[0].legend(); axes[0].grid(True)
    axes[0].set_ylabel('输出')
    
    # 显示指数趋近律的滑模面
    x, dx = 0, 0
    smc = SMC(law='exponential', c=10, eps=3, k=5, phi=0.1)
    s_hist = np.zeros(N)
    for i in range(N):
        e = ref[i] - x; e_dot = -dx
        s_hist[i] = e_dot + smc.c * e
        u = smc.update(e, e_dot)
        ddx = -2*dx + u; dx += ddx*dt; x += dx*dt
    
    axes[1].plot(t, s_hist, 'b-', label='滑模面s(t)')
    axes[1].axhline(0, color='k', linestyle='--')
    axes[1].set_title('滑模面变化（指数趋近律）'); axes[1].legend(); axes[1].grid(True)
    axes[1].set_xlabel('时间(s)'); axes[1].set_ylabel('s(t)')
    
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'smc_sim.png'), dpi=150)
    plt.close('all')
    print("SMC仿真完成")

if __name__ == '__main__':
    simulate()
