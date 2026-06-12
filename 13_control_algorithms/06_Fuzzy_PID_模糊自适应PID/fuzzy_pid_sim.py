"""
模糊自适应PID仿真验证
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

# ΔKp, ΔKi, ΔKd 模糊规则表
RULE_KP = [[3,3,2,2,1,0,0],[3,3,2,1,1,0,-1],[2,2,2,1,0,-1,-1],[2,2,1,0,-1,-2,-2],
           [1,1,0,-1,-1,-2,-2],[1,0,-1,-2,-2,-2,-3],[0,0,-2,-2,-2,-3,-3]]
RULE_KI = [[-3,-3,-2,-2,-1,0,0],[-3,-3,-2,-1,-1,0,0],[-2,-2,-1,-1,0,1,1],
           [-2,-2,-1,0,1,2,2],[-1,-1,0,1,1,2,2],[-1,0,1,2,2,3,3],[0,0,2,2,2,3,3]]
RULE_KD = [[3,1,0,0,0,2,3],[3,1,0,-1,0,1,2],[2,1,0,-1,0,1,2],[2,1,0,0,0,1,2],
           [2,2,1,0,0,1,2],[3,2,1,0,0,1,2],[3,3,2,1,0,2,3]]

def trimf(x, a, b, c):
    if x <= a or x >= c: return 0
    return (x-a)/(b-a) if x <= b else (c-x)/(c-b)

def fuzzy_infer(rule, e_f, ec_f):
    mf = [-3,-2,-1,0,1,2,3]
    num = den = 0
    for i in range(7):
        mu_e = trimf(e_f, mf[i]-1, mf[i], mf[i]+1)
        for j in range(7):
            mu_ec = trimf(ec_f, mf[j]-1, mf[j], mf[j]+1)
            w = min(mu_e, mu_ec)
            num += w * rule[i][j]; den += w
    return num/den if den > 0.001 else 0

class FuzzyPID:
    def __init__(self, ke=1, kec=1, Kp=5, Ki=0.5, Kd=2, kup=0.5, kui=0.1, kud=0.2):
        self.ke, self.kec = ke, kec
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.kup, self.kui, self.kud = kup, kui, kud
        self.e = self.e_last = self.e_sum = 0
    
    def update(self, ref, y):
        self.e_last = self.e; self.e = ref - y
        de = self.e - self.e_last; self.e_sum += self.e
        e_f = np.clip(self.ke * self.e, -3, 3)
        ec_f = np.clip(self.kec * de, -3, 3)
        Kp = max(0, self.Kp + self.kup * fuzzy_infer(RULE_KP, e_f, ec_f))
        Ki = max(0, self.Ki + self.kui * fuzzy_infer(RULE_KI, e_f, ec_f))
        Kd = max(0, self.Kd + self.kud * fuzzy_infer(RULE_KD, e_f, ec_f))
        u = Kp * self.e + Ki * self.e_sum + Kd * de
        return np.clip(u, -100, 100), Kp, Ki, Kd

def simulate():
    dt = 0.01; T = 5.0; N = int(T/dt)
    t = np.arange(N)*dt
    
    # 对比: 普通PID vs 模糊PID
    from collections import namedtuple
    PID = namedtuple('PID', 'kp ki kd')
    
    x1 = dx1 = x2 = dx2 = 0
    pid = PID(5, 0.5, 2)
    fp = FuzzyPID(ke=2, kec=1, Kp=5, Ki=0.5, Kd=2, kup=1, kui=0.2, kud=0.5)
    e1_sum = e1_last = e2_sum = e2_last = 0
    
    ref = np.where(t < 2.5, 1.0, 2.0)
    y_pid, y_fuzzy = np.zeros(N), np.zeros(N)
    kp_f, ki_f, kd_f = np.zeros(N), np.zeros(N), np.zeros(N)
    
    for i in range(N):
        # 普通PID
        e1 = ref[i] - x1; de1 = e1 - e1_last; e1_sum += e1; e1_last = e1
        u1 = pid.kp*e1 + pid.ki*e1_sum + pid.kd*de1
        ddx1 = -2*dx1 + u1; dx1 += ddx1*dt; x1 += dx1*dt; y_pid[i] = x1
        
        # 模糊PID
        u2, kp_f[i], ki_f[i], kd_f[i] = fp.update(ref[i], x2)
        ddx2 = -2*dx2 + u2; dx2 += ddx2*dt; x2 += dx2*dt; y_fuzzy[i] = x2
    
    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    axes[0].plot(t, ref, 'k--', label='参考', linewidth=2)
    axes[0].plot(t, y_pid, 'r-', label='普通PID', alpha=0.7)
    axes[0].plot(t, y_fuzzy, 'b-', label='模糊自适应PID', linewidth=2)
    axes[0].set_title('模糊自适应PID vs 普通PID'); axes[0].legend(); axes[0].grid(True)
    axes[1].plot(t, kp_f, label='Kp'); axes[1].plot(t, ki_f, label='Ki'); axes[1].plot(t, kd_f, label='Kd')
    axes[1].set_title('模糊PID参数自适应调整'); axes[1].legend(); axes[1].grid(True); axes[1].set_xlabel('时间(s)')
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fuzzy_pid_sim.png'), dpi=150)
    plt.close('all')
    print("模糊PID仿真完成")

if __name__ == '__main__':
    simulate()
