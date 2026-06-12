"""
PID算法仿真 - 模拟不同系统特性，对比不同PID变种
用法: python pid_simulation.py
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


class PIDController:
    """标准PID控制器"""
    def __init__(self, kp, ki, kd, output_min=-100, output_max=100):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_min, self.output_max = output_min, output_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.output_min, min(self.output_max, output))


class IncrementalPID:
    """增量式PID"""
    def __init__(self, kp, ki, kd, output_min=-100, output_max=100):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_min, self.output_max = output_min, output_max
        self.prev_error = 0.0
        self.prev_prev_error = 0.0
        self.output = 0.0

    def compute(self, error, dt):
        delta = (self.kp * (error - self.prev_error) + self.ki * error * dt +
                 self.kd * (error - 2 * self.prev_error + self.prev_prev_error) / dt) if dt > 0 else 0
        self.prev_prev_error = self.prev_error
        self.prev_error = error
        self.output += delta
        self.output = max(self.output_min, min(self.output_max, self.output))
        return self.output


class IntegralSeparationPID:
    """积分分离PID"""
    def __init__(self, kp, ki, kd, threshold=10, output_min=-100, output_max=100):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.threshold = threshold
        self.output_min, self.output_max = output_min, output_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if abs(error) < self.threshold:
            self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.output_min, min(self.output_max, output))


class AntiWindupPID:
    """抗积分饱和PID"""
    def __init__(self, kp, ki, kd, output_min=-100, output_max=100):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_min, self.output_max = output_min, output_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        output_unsat = (self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error) / dt) if dt > 0 else 0
        output = max(self.output_min, min(self.output_max, output_unsat))
        # 回推抗饱和
        if output != output_unsat:
            self.integral -= error * dt
        self.prev_error = error
        return output


class Plant:
    """一阶惯性系统 (带可选时延和噪声)"""
    def __init__(self, gain=1.0, tau=0.5, delay=0.0, noise_std=0.0):
        self.gain, self.tau, self.delay, self.noise_std = gain, tau, delay, noise_std
        self.state = 0.0
        self.delay_buf = []

    def update(self, u, dt):
        delay_steps = int(self.delay / dt)
        self.delay_buf.append(u)
        u_delayed = self.delay_buf[-delay_steps - 1] if len(self.delay_buf) > delay_steps else 0
        self.state += dt / self.tau * (self.gain * u_delayed - self.state)
        noise = np.random.normal(0, self.noise_std) if self.noise_std > 0 else 0
        return self.state + noise


def simulate(controller, plant, setpoint, duration=5.0, dt=0.01):
    t_vals, y_vals, u_vals = [], [], []
    y = 0.0
    for i in range(int(duration / dt)):
        t = i * dt
        error = setpoint - y
        u = controller.compute(error, dt)
        y = plant.update(u, dt)
        t_vals.append(t)
        y_vals.append(y)
        u_vals.append(u)
    return np.array(t_vals), np.array(y_vals), np.array(u_vals)


def main():
    dt, duration, setpoint = 0.01, 10.0, 1.0
    scenarios = [
        ("标准系统 (τ=0.5)", 1.0, 0.5, 0, 0),
        ("慢速系统 (τ=2.0)", 1.0, 2.0, 0, 0),
        ("带时延 (delay=0.3)", 1.0, 0.5, 0.3, 0),
        ("带噪声 (σ=0.05)", 1.0, 0.5, 0, 0.05),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()
    pid_params = (2.0, 0.5, 0.3)  # Kp, Ki, Kd

    for idx, (name, gain, tau, delay, noise) in enumerate(scenarios):
        ax = axes[idx]
        controllers = {
            "位置式PID": PIDController(*pid_params),
            "增量式PID": IncrementalPID(*pid_params),
            "积分分离PID": IntegralSeparationPID(*pid_params, threshold=0.3),
            "抗饱和PID": AntiWindupPID(*pid_params),
        }
        for cname, ctrl in controllers.items():
            plant = Plant(gain, tau, delay, noise)
            t, y, _ = simulate(ctrl, plant, setpoint, duration, dt)
            ax.plot(t, y, label=cname, linewidth=1.2)
        ax.axhline(setpoint, color='k', linestyle='--', label='设定值')
        ax.set_title(name, fontsize=12)
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('输出')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.suptitle('PID算法变种对比仿真', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('pid_simulation_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("仿真结果已保存: pid_simulation_result.png")


if __name__ == '__main__':
    main()
