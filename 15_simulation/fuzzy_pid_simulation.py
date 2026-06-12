"""
模糊PID控制仿真 - 对比普通PID
适用于电赛温度控制、电机调速等非线性场景

依赖: pip install numpy matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ============ 模糊控制器 ============
class FuzzyPID:
    """
    模糊PID控制器
    输入: 误差e, 误差变化率ec
    输出: ΔKp, ΔKi, ΔKd
    """
    def __init__(self, kp0, ki0, kd0, dt, e_range=(-3, 3), ec_range=(-3, 3)):
        self.kp = kp0
        self.ki = ki0
        self.kd = kd0
        self.kp0, self.ki0, self.kd0 = kp0, ki0, kd0
        self.dt = dt
        self.integral = 0
        self.prev_error = 0
        self.e_range = e_range
        self.ec_range = ec_range

        # 隶属函数: NB, NM, NS, ZO, PS, PM, PB
        self.labels = ['NB', 'NM', 'NS', 'ZO', 'PS', 'PM', 'PB']
        # 各隶属函数的中心点
        self.centers = np.linspace(-3, 3, 7)

    def _membership(self, x, center, sigma=1.0):
        """高斯隶属函数"""
        return np.exp(-((x - center) ** 2) / (2 * sigma ** 2))

    def _fuzzify(self, x, x_range):
        """模糊化"""
        x_norm = np.clip((x - x_range[0]) / (x_range[1] - x_range[0]) * 6 - 3, -3, 3)
        degrees = np.array([self._membership(x_norm, c) for c in self.centers])
        return degrees / (degrees.sum() + 1e-10)

    def _defuzzify(self, degrees, out_range=(-0.5, 0.5)):
        """加权平均去模糊化"""
        centers = np.linspace(out_range[0], out_range[1], 7)
        return np.dot(degrees, centers) / (degrees.sum() + 1e-10)

    def _rule_base(self, e_deg, ec_deg):
        """模糊规则表 (7x7)
        行: e  (NB->PB), 列: ec (NB->PB)
        输出: ΔKp 的调整量
        """
        # Kp规则: 误差大时增大Kp, 误差小时减小Kp
        kp_rules = np.array([
            [ 1,  1,  0.5,  0.5,  0,   0,   0],
            [ 1,  0.5, 0.5,  0,   0,  -0.5, 0],
            [ 0.5, 0.5, 0,   0,   0,  -0.5,-0.5],
            [ 0.5,  0,   0,   0,   0,   0,  -0.5],
            [ 0,    0,   0,   0,   0,  -0.5,-0.5],
            [ 0,   -0.5, 0,  -0.5,-0.5, -1,  -1],
            [ 0,   -0.5,-0.5,-0.5, -1,  -1,  -1],
        ])
        # Ki规则: 误差小时增大Ki消除稳态误差
        ki_rules = np.array([
            [-1,  -1,  -0.5, -0.5,  0,   0,   0],
            [-1,  -0.5,-0.5,  0,    0,   0.5, 0],
            [-0.5,-0.5, 0,    0,    0.5, 0.5, 0],
            [-0.5, 0,   0,    0,    0,   0,   0.5],
            [ 0,   0,   0,    0,    0.5, 0.5, 0.5],
            [ 0,   0,   0.5,  0.5,  0.5, 1,   1],
            [ 0,   0.5, 0.5,  0.5,  1,   1,   1],
        ])
        # Kd规则
        kd_rules = np.array([
            [ 0,   0,   0.5,  0.5,  1,   1,   1],
            [ 0,   0,   0,    0.5,  0.5, 0.5, 1],
            [-0.5, 0,   0,    0,    0.5, 0.5, 0.5],
            [-0.5,-0.5, 0,    0,    0,   0.5, 0.5],
            [-0.5,-0.5,-0.5,  0,    0,   0,   0.5],
            [-1,  -0.5,-0.5, -0.5,  0,   0,   0],
            [-1,  -1,  -0.5, -0.5, -0.5, 0,   0],
        ])

        # 使用中心平均去模糊
        d_kp = np.dot(e_deg, np.dot(kp_rules, ec_deg)) / (np.sum(e_deg) * np.sum(ec_deg) + 1e-10)
        d_ki = np.dot(e_deg, np.dot(ki_rules, ec_deg)) / (np.sum(e_deg) * np.sum(ec_deg) + 1e-10)
        d_kd = np.dot(e_deg, np.dot(kd_rules, ec_deg)) / (np.sum(e_deg) * np.sum(ec_deg) + 1e-10)

        return d_kp, d_ki, d_kd

    def update(self, ref, y):
        error = ref - y
        d_error = (error - self.prev_error) / self.dt
        self.prev_error = error

        # 模糊推理
        e_deg = self._fuzzify(error, self.e_range)
        ec_deg = self._fuzzify(d_error, self.ec_range)
        d_kp, d_ki, d_kd = self._rule_base(e_deg, ec_deg)

        # 在线调整PID参数
        kp = self.kp0 + d_kp * self.kp0 * 0.5
        ki = self.ki0 + d_ki * self.ki0 * 0.5
        kd = self.kd0 + d_kd * self.kd0 * 0.5
        kp = max(kp, 0)
        ki = max(ki, 0)
        kd = max(kd, 0)

        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -10, 10)
        derivative = d_error

        return kp * error + ki * self.integral + kd * derivative

# ============ 普通PID ============
class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0
        self.prev_error = 0

    def update(self, ref, y):
        error = ref - y
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -10, 10)
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# ============ 被控对象: 温度系统(非线性) ============
def temp_plant(u, T, T_env=25.0, dt=0.1):
    """一阶温度系统(含非线性散热)"""
    # 加热功率与散热(非线性: 散热∝(T-T_env)^1.2)
    heating = 0.8 * u
    cooling = 0.05 * (T - T_env) ** 1.2 * np.sign(T - T_env)
    T += (heating - cooling) * dt
    return max(T, T_env)

# ============ 仿真 ============
if __name__ == '__main__':
    dt = 0.1
    steps = 500
    t = np.arange(steps) * dt

    # 温度设定值
    ref = np.ones(steps) * 60.0
    ref[200:350] = 80.0
    ref[350:] = 45.0
    T_env = 25.0

    results = {}
    for name, ctrl in [('PID', PID(2.0, 0.1, 0.5, dt)),
                        ('FuzzyPID', FuzzyPID(2.0, 0.1, 0.5, dt,
                                              e_range=(-30, 30), ec_range=(-10, 10)))]:
        T = T_env
        temps = []
        for i in range(steps):
            u = np.clip(ctrl.update(ref[i], T), 0, 100)
            T = temp_plant(u, T, T_env, dt)
            temps.append(T)
        results[name] = np.array(temps)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    axes[0].plot(t, ref, 'k--', label='设定值', linewidth=1.5)
    axes[0].plot(t, results['PID'], 'r-', label='PID', alpha=0.8)
    axes[0].plot(t, results['FuzzyPID'], 'b-', label='模糊PID', alpha=0.8)
    axes[0].set_ylabel('温度 (°C)')
    axes[0].set_title('模糊PID vs 普通PID 温度控制')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 误差对比
    for name in ['PID', 'FuzzyPID']:
        err = np.abs(ref - results[name])
        axes[1].plot(t, err, label=f'{name} 误差')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('绝对误差 (°C)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('fuzzy_pid_vs_pid.png', dpi=150)
    plt.close('all')

    for name in ['PID', 'FuzzyPID']:
        err = np.abs(ref - results[name])
        print(f'{name}: 平均误差={np.mean(err):.2f}°C, 最大误差={np.max(err):.2f}°C, '
              f'RMSE={np.sqrt(np.mean(err**2)):.2f}°C')
