"""
神经网络PID仿真验证
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

def sigmoid(x): return 1.0 / (1.0 + np.exp(-np.clip(x, -500, 500)))

class NN_PID:
    def __init__(self, lr=0.3, Kp=2.0, Ki=0.5, Kd=1.0):
        self.lr = lr
        self.ni, self.nh, self.no = 3, 5, 3
        np.random.seed(42)
        self.w_ih = np.random.randn(self.nh, self.ni) * 0.3
        self.w_ho = np.random.randn(self.no, self.nh) * 0.3
        self.b_o = np.array([Kp, Ki, Kd])
        self.error = self.error_last = self.error_sum = self.u = 0
    
    def update(self, ref, y):
        self.error_last = self.error
        self.error = ref - y
        de = self.error - self.error_last
        self.error_sum = np.clip(self.error_sum + self.error, -100, 100)
        
        o_i = np.array([self.error*0.01, self.error_sum*0.001, de*0.1])
        o_h = sigmoid(self.w_ih @ o_i)
        o_o = sigmoid(self.w_ho @ o_h + self.b_o) * 10
        Kp, Ki, Kd = o_o
        
        self.u = np.clip(Kp*self.error + Ki*self.error_sum + Kd*de, -100, 100)
        
        # BP更新
        delta_o = np.array([self.error**2, self.error*self.error_sum, self.error*de])
        for i in range(self.no):
            sd = o_o[i]/10 * (1 - o_o[i]/10)
            self.w_ho[i] += self.lr * delta_o[i] * sd * o_h
        for j in range(self.nh):
            h_d = o_h[j] * (1 - o_h[j])
            s = sum(delta_o[i] * self.w_ho[i,j] for i in range(self.no))
            self.w_ih[j] += self.lr * s * h_d * o_i
        
        return self.u, Kp, Ki, Kd

def simulate():
    dt = 0.01; T = 5.0; N = int(T/dt)
    t = np.arange(N)*dt
    
    nn = NN_PID(lr=0.3, Kp=2, Ki=0.5, Kd=1)
    x = dx = 0
    ref = np.where(t < 2.5, 1.0, 2.0)
    
    y_out, u_out = np.zeros(N), np.zeros(N)
    kp_hist, ki_hist, kd_hist = np.zeros(N), np.zeros(N), np.zeros(N)
    
    for i in range(N):
        u, kp, ki, kd = nn.update(ref[i], x)
        kp_hist[i], ki_hist[i], kd_hist[i] = kp, ki, kd
        ddx = -2*dx + u; dx += ddx*dt; x += dx*dt
        y_out[i], u_out[i] = x, u
    
    fig, axes = plt.subplots(3, 1, figsize=(10, 9))
    axes[0].plot(t, ref, 'r--', label='参考', linewidth=2); axes[0].plot(t, y_out, 'b-', label='NN-PID输出')
    axes[0].set_title('神经网络PID仿真'); axes[0].legend(); axes[0].grid(True)
    axes[1].plot(t, u_out, 'g-'); axes[1].set_title('控制量'); axes[1].grid(True)
    axes[2].plot(t, kp_hist, label='Kp'); axes[2].plot(t, ki_hist, label='Ki'); axes[2].plot(t, kd_hist, label='Kd')
    axes[2].set_title('PID参数自适应变化'); axes[2].legend(); axes[2].grid(True); axes[2].set_xlabel('时间(s)')
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nn_pid_sim.png'), dpi=150)
    plt.close('all')
    print("NN-PID仿真完成")

if __name__ == '__main__':
    simulate()
