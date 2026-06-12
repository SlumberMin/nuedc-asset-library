"""
auto_tune.py - PID参数自动整定

实现三种经典整定方法:
1. Ziegler-Nichols 临界比例度法
2. Ziegler-Nichols 衰减曲线法
3. Relay反馈法(继电整定)

依赖: pip install numpy matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ============================================================
# 被控对象
# ============================================================

class Plant:
    def __init__(self, K=1.0, T=1.0, L=0.2, zeta=0.3, omega_n=5.0, order=1):
        self.K = K
        self.T = T
        self.L = L
        self.zeta = zeta
        self.omega_n = omega_n
        self.order = order
        self.reset()
    
    def reset(self):
        self.x = [0.0] * 5
        self.delay_buf = [0.0] * max(int(self.L / 0.01), 1)
    
    def update(self, u, dt=0.01):
        if self.order == 1:
            # 一阶+延迟
            self.x[0] += dt / self.T * (self.K * u - self.x[0])
            self.delay_buf.append(self.x[0])
            return self.delay_buf.pop(0)
        elif self.order == 2:
            # 二阶
            acc = self.omega_n**2 * (self.K * u - self.x[0]) - 2 * self.zeta * self.omega_n * self.x[1]
            self.x[1] += acc * dt
            self.x[0] += self.x[1] * dt
            return self.x[0]
        else:
            self.x[0] += dt / self.T * (self.K * u - self.x[0])
            return self.x[0]


# ============================================================
# PID控制器
# ============================================================

class PID:
    def __init__(self, kp=0, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._ei = 0
        self._ed = 0
        self._el = 0
    
    def reset(self):
        self._ei = self._ed = self._el = 0
    
    def update(self, target, meas):
        e = target - meas
        self._ei += e
        d = e - self._el
        self._el = e
        out = self.kp * e + self.ki * self._ei + self.kd * d
        return max(-1000, min(1000, out))


# ============================================================
# 方法1: Ziegler-Nichols 临界比例度法
# ============================================================

def zn_ultimate(plant: Plant, dt=0.01, duration=50.0):
    """
    逐步增大Kp直到系统等幅振荡, 记录临界增益Ku和振荡周期Tu
    返回: (Ku, Tu, Kp, Ki, Kd)
    """
    print("[ZN临界比例度法] 搜索临界增益...")
    
    for kp_try in np.arange(1, 200, 0.5):
        pid = PID(kp=kp_try, ki=0, kd=0)
        plant.reset()
        
        steps = int(duration / dt)
        y = np.zeros(steps)
        target = 1.0
        
        for i in range(steps):
            y[i] = plant.update(pid.update(target, y[i-1] if i > 0 else 0), dt)
        
        # 检查是否等幅振荡: 看最后20%的波形
        tail = y[int(steps * 0.8):]
        peaks = []
        for i in range(1, len(tail) - 1):
            if tail[i] > tail[i-1] and tail[i] > tail[i+1]:
                peaks.append(i)
        
        if len(peaks) >= 3:
            # 检查峰值是否近似相等(等幅)
            peak_vals = [tail[p] for p in peaks]
            if max(peak_vals) - min(peak_vals) < 0.1 * np.mean(peak_vals):
                # 振荡周期
                Tu = (peaks[-1] - peaks[0]) / (len(peaks) - 1) * dt
                Ku = kp_try
                
                if Tu > 0.1:  # 合理周期
                    print(f"  临界增益 Ku = {Ku:.1f}")
                    print(f"  振荡周期 Tu = {Tu:.3f} s")
                    
                    # ZN整定公式
                    kp = 0.6 * Ku
                    ki = 2 * kp / Tu
                    kd = kp * Tu / 8
                    
                    print(f"  ZN整定: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")
                    return Ku, Tu, kp, ki, kd
    
    print("  未找到临界振荡, 使用默认值")
    return 10, 1.0, 6.0, 12.0, 0.75


# ============================================================
# 方法2: Relay反馈法(继电整定)
# ============================================================

def relay_tune(plant: Plant, relay_amplitude=50.0, dt=0.01, duration=20.0):
    """
    继电反馈法: 用继电器替代PID, 测量极限增益和频率
    返回: (Ku, Tu, Kp, Ki, Kd)
    """
    print("\n[Relay反馈法] 继电整定...")
    
    plant.reset()
    steps = int(duration / dt)
    y = np.zeros(steps)
    target = 1.0
    
    # 继电器输出
    for i in range(1, steps):
        e = target - y[i-1]
        u = relay_amplitude if e > 0 else -relay_amplitude
        y[i] = plant.update(u, dt)
    
    # 找振荡周期
    tail = y[int(steps * 0.6):]
    zero_cross = []
    for i in range(1, len(tail)):
        if tail[i-1] * tail[i] < 0:
            zero_cross.append(i)
    
    if len(zero_cross) >= 4:
        Tu = (zero_cross[-1] - zero_cross[0]) / (len(zero_cross) - 1) * 2 * dt
        
        # 振幅
        a = (np.max(tail) - np.min(tail)) / 2
        
        # Ku = 4*d / (π*a), d=继电器幅值
        Ku = 4 * relay_amplitude / (np.pi * a)
        
        print(f"  振幅 a = {a:.3f}")
        print(f"  临界增益 Ku = {Ku:.1f}")
        print(f"  振荡周期 Tu = {Tu:.3f} s")
        
        # ZN整定
        kp = 0.6 * Ku
        ki = 2 * kp / Tu
        kd = kp * Tu / 8
        
        print(f"  整定: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")
        return Ku, Tu, kp, ki, kd
    
    print("  振荡数据不足, 使用默认值")
    return 10, 1.0, 6.0, 12.0, 0.75


# ============================================================
# 方法3: 经验公式法(基于阶跃响应)
# ============================================================

def step_response_tune(plant: Plant, dt=0.01, duration=10.0):
    """
    通过开环阶跃响应获取K(增益)、L(延迟)、T(时间常数)
    使用ZN阶跃响应法整定
    """
    print("\n[阶跃响应法] 开环测试...")
    
    plant.reset()
    steps = int(duration / dt)
    y = np.zeros(steps)
    
    # 开环阶跃响应
    for i in range(1, steps):
        y[i] = plant.update(1.0, dt)  # 阶跃输入=1
    
    # 找K(稳态增益)
    K = y[-1]
    
    # 找L(延迟)和T(时间常数) - 切线法
    max_slope = 0
    max_slope_idx = 0
    for i in range(1, steps):
        slope = (y[i] - y[i-1]) / dt
        if slope > max_slope:
            max_slope = slope
            max_slope_idx = i
    
    # 切线与y=0和y=K的交点
    if max_slope > 0.001:
        L = -y[max_slope_idx] / max_slope + max_slope_idx * dt
        T = K / max_slope
    else:
        L = 0.1
        T = 1.0
    
    L = max(0.01, L)
    T = max(0.1, T)
    
    print(f"  增益 K = {K:.3f}")
    print(f"  延迟 L = {L:.3f} s")
    print(f"  时间常数 T = {T:.3f} s")
    
    # ZN阶跃响应法
    a = K * L / T
    kp = 1.2 / a
    ki = kp / (2 * L)
    kd = kp * 0.5 * L
    
    print(f"  ZN阶跃响应整定: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")
    
    return K, L, T, kp, ki, kd


# ============================================================
# 仿真验证
# ============================================================

def verify_tuning(plant: Plant, kp: float, ki: float, kd: float, 
                  dt=0.01, duration=5.0, label=""):
    """验证整定结果"""
    pid = PID(kp=kp, ki=ki, kd=kd)
    plant.reset()
    
    steps = int(duration / dt)
    t = np.arange(steps) * dt
    y = np.zeros(steps)
    target = 1.0
    
    for i in range(1, steps):
        y[i] = plant.update(pid.update(target, y[i-1]), dt)
    
    # 计算性能
    overshoot = (np.max(y) - target) / target * 100 if target != 0 else 0
    steady_err = abs(target - np.mean(y[-100:]))
    
    return t, y, overshoot, steady_err


# ============================================================
# 主程序
# ============================================================

if __name__ == '__main__':
    print("=" * 60)
    print("  PID参数自动整定程序")
    print("=" * 60)
    
    # 被控对象: 二阶振荡+延迟
    plant = Plant(K=1.0, T=1.0, L=0.2, zeta=0.3, omega_n=5.0, order=2)
    
    # 方法1: 临界比例度法
    Ku1, Tu1, kp1, ki1, kd1 = zn_ultimate(plant, duration=30.0)
    
    # 方法2: Relay反馈法
    Ku2, Tu2, kp2, ki2, kd2 = relay_tune(plant)
    
    # 方法3: 阶跃响应法
    plant_step = Plant(K=1.0, T=1.0, L=0.2, order=1)
    K3, L3, T3, kp3, ki3, kd3 = step_response_tune(plant_step)
    
    # 对比验证
    print("\n" + "=" * 60)
    print("  整定结果对比验证")
    print("=" * 60)
    
    methods = [
        (f"ZN临界法 (Kp={kp1:.1f}, Ki={ki1:.1f}, Kd={kd1:.2f})", kp1, ki1, kd1),
        (f"Relay法 (Kp={kp2:.1f}, Ki={ki2:.1f}, Kd={kd2:.2f})", kp2, ki2, kd2),
        (f"阶跃响应法 (Kp={kp3:.1f}, Ki={ki3:.1f}, Kd={kd3:.2f})", kp3, ki3, kd3),
    ]
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    for ax, (name, kp, ki, kd) in zip(axes, methods):
        plant_v = Plant(K=1.0, T=1.0, L=0.2, zeta=0.3, omega_n=5.0, order=2)
        t, y, overshoot, steady_err = verify_tuning(plant_v, kp, ki, kd)
        
        ax.plot(t, np.ones_like(t), 'r--', label='Target')
        ax.plot(t, y, 'b-', label='Output')
        ax.set_title(name)
        ax.set_xlabel('Time (s)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.text(0.02, 0.95, f"Overshoot: {overshoot:.1f}%\nSteady Err: {steady_err:.4f}",
                transform=ax.transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        print(f"\n{name}")
        print(f"  超调量: {overshoot:.1f}%")
        print(f"  稳态误差: {steady_err:.4f}")
    
    plt.suptitle('Auto-Tuning Methods Comparison', fontsize=14)
    plt.tight_layout()
    plt.savefig('auto_tune_comparison.png', dpi=150)
    plt.close('all')
