"""
自抗扰控制 ADRC 仿真验证
========================
对二阶被控对象进行ADRC控制仿真，验证跟踪性能和抗扰能力
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

# ============ ADRC 控制器 ============
class ADRC:
    def __init__(self, r0=100, h0=0.01, b0=1.0, omega_c=10, omega_o=50, delta=0.05, dt=0.001):
        self.r0, self.h0 = r0, h0
        self.beta01 = 3 * omega_o
        self.beta02 = 3 * omega_o**2
        self.beta03 = omega_o**3
        self.b0 = b0
        self.delta = delta
        self.dt = dt
        self.kp = omega_c**2
        self.kd = 2 * omega_c
        self.v1 = self.v2 = 0
        self.z1 = self.z2 = self.z3 = 0
        self.u = 0
        self.u_max = 100
    
    @staticmethod
    def fal(e, alpha, delta):
        if abs(e) > delta:
            return (abs(e)**alpha) * np.sign(e)
        return e / (delta**(1 - alpha))
    
    def fhan(self, x1, x2, r, h):
        d = r * h; d0 = d * h
        y = x1 + h * x2
        a0 = np.sqrt(d**2 + 8*r*abs(y))
        a = x2 + (a0 - d)/2 * np.sign(y) if abs(y) > d0 else x2 + y/h
        return -r * np.sign(a) if abs(a) > d else -r * a / d
    
    def update(self, ref, y):
        # TD
        fh = self.r0 * self.h0**2
        self.v1 += self.h0 * self.v2
        self.v2 += fh * self.fhan(self.v1 - ref, self.v2, self.r0, self.h0)
        
        # ESO
        e = self.z1 - y
        self.z1 += self.dt * (self.z2 - self.beta01 * e)
        self.z2 += self.dt * (self.z3 - self.beta02 * self.fal(e, 0.5, self.delta) + self.b0 * self.u)
        self.z3 += self.dt * (-self.beta03 * self.fal(e, 0.25, self.delta))
        
        # NLSEF + 扰动补偿
        e1 = self.v1 - self.z1
        e2 = self.v2 - self.z2
        u0 = self.kp * self.fal(e1, 0.5, self.delta) + self.kd * self.fal(e2, 0.25, self.delta)
        self.u = np.clip((u0 - self.z3) / self.b0, -self.u_max, self.u_max)
        return self.u

# ============ 被控对象: 二阶系统 G(s) = K / (s^2 + a*s) ============
class Plant:
    """x'' = -a*x' + K*u + d(t)"""
    def __init__(self, K=1.0, a=2.0, dt=0.001):
        self.K, self.a, self.dt = K, a, dt
        self.x = self.dx = 0
    
    def update(self, u, disturbance=0):
        self.dx += self.dt * (-self.a * self.dx + self.K * u + disturbance)
        self.x += self.dt * self.dx
        return self.x

# ============ 仿真 ============
def simulate():
    dt = 0.001
    T = 3.0
    N = int(T / dt)
    t = np.linspace(0, T, N)
    
    plant = Plant(K=1.0, a=2.0, dt=dt)
    adrc = ADRC(r0=100, h0=0.01, b0=1.0, omega_c=15, omega_o=60, delta=0.05, dt=dt)
    adrc.u_max = 50
    
    ref = np.zeros(N)
    ref[int(0.5/dt):] = 1.0  # 阶跃参考
    ref[int(1.5/dt):] = 2.0
    
    y_out = np.zeros(N)
    u_out = np.zeros(N)
    z3_out = np.zeros(N)
    
    for i in range(N):
        dist = 0.5 if 2.0 < t[i] < 2.5 else 0  # 2~2.5s加扰动
        y = plant.x
        u = adrc.update(ref[i], y)
        y_out[i] = plant.update(u, dist)
        u_out[i] = u
        z3_out[i] = adrc.z3
    
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    
    axes[0].plot(t, ref, 'r--', label='参考信号', linewidth=2)
    axes[0].plot(t, y_out, 'b-', label='ADRC输出', linewidth=1.5)
    axes[0].set_ylabel('输出')
    axes[0].set_title('ADRC自抗扰控制仿真')
    axes[0].legend(); axes[0].grid(True)
    
    axes[1].plot(t, u_out, 'g-', label='控制量')
    axes[1].set_ylabel('控制量u')
    axes[1].legend(); axes[1].grid(True)
    
    axes[2].plot(t, z3_out, 'm-', label='ESO扰动估计z3')
    axes[2].set_xlabel('时间(s)'); axes[2].set_ylabel('扰动估计')
    axes[2].legend(); axes[2].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'adrc_sim.png'), dpi=150)
    plt.close('all')
    print("ADRC仿真完成，结果已保存")

if __name__ == '__main__':
    simulate()
