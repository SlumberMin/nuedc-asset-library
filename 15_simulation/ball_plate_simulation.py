"""
滚球控制仿真 - 双轴PID控制小球位置
用法: python ball_plate_simulation.py
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


class BallOnPlate:
    """滚球-平板系统动力学模型
    状态: [x, y, vx, vy]
    """
    def __init__(self, radius=0.15, g=9.81, mu=0.01):
        self.radius = radius    # 板半径 (m)
        self.g = g
        self.mu = mu            # 摩擦系数
        self.state = np.array([0.0, 0.0, 0.0, 0.0])

    def update(self, tilt_x, tilt_y, dt):
        """tilt_x, tilt_y: 平板倾斜角 (rad)"""
        x, y, vx, vy = self.state
        # 加速度 = g * sin(θ) ≈ g * θ (小角度)
        ax = self.g * tilt_x - self.mu * vx
        ay = self.g * tilt_y - self.mu * vy
        vx += ax * dt
        vy += ay * dt
        x += vx * dt
        y += vy * dt
        # 边界限制 (球掉出板外)
        if x**2 + y**2 > self.radius**2:
            x, y = 0, 0
            vx, vy = 0, 0
        self.state = np.array([x, y, vx, vy])
        return self.state.copy()

    def reset(self, x=0, y=0):
        self.state = np.array([x, y, 0.0, 0.0])


class AxisPID:
    """单轴PID控制器 (带微分滤波)"""
    def __init__(self, kp, ki, kd, out_min=-0.3, out_max=0.3):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.d_filter = 0.0
        self.alpha = 0.1  # 微分低通滤波系数

    def compute(self, error, dt):
        self.integral += error * dt
        # 积分限幅
        self.integral = max(-0.5, min(0.5, self.integral))
        raw_d = (error - self.prev_error) / dt if dt > 0 else 0
        self.d_filter = self.alpha * raw_d + (1 - self.alpha) * self.d_filter
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * self.d_filter
        return max(self.out_min, min(self.out_max, out))


def simulate_trajectory(plate_radius=0.15, target_points=None):
    """仿真滚球轨迹"""
    if target_points is None:
        # 阶跃 -> 圆 -> 方形轨迹
        target_points = [
            (0.05, 0.05, 3.0),
            (0.08, 0.0, 3.0),
            (0.0, 0.0, 2.0),
        ]

    ball = BallOnPlate(radius=plate_radius)
    pid_x = AxisPID(kp=2.0, ki=0.5, kd=0.8)
    pid_y = AxisPID(kp=2.0, ki=0.5, kd=0.8)

    dt = 0.005
    log = {'t': [], 'x': [], 'y': [], 'tx': [], 'ty': [], 'vx': [], 'vy': []}
    t = 0

    for tx, ty, duration in target_points:
        for _ in range(int(duration / dt)):
            err_x = tx - ball.state[0]
            err_y = ty - ball.state[1]
            tilt_x = pid_x.compute(err_x, dt)
            tilt_y = pid_y.compute(err_y, dt)
            ball.update(tilt_x, tilt_y, dt)
            log['t'].append(t)
            log['x'].append(ball.state[0])
            log['y'].append(ball.state[1])
            log['tx'].append(tx)
            log['ty'].append(ty)
            log['vx'].append(ball.state[2])
            log['vy'].append(ball.state[3])
            t += dt

    # 圆形轨迹
    ball.reset()
    circle_time = 8.0
    for i in range(int(circle_time / dt)):
        t_sim = i * dt
        tx = 0.06 * np.cos(2 * np.pi * t_sim / 4)
        ty = 0.06 * np.sin(2 * np.pi * t_sim / 4)
        err_x = tx - ball.state[0]
        err_y = ty - ball.state[1]
        tilt_x = pid_x.compute(err_x, dt)
        tilt_y = pid_y.compute(err_y, dt)
        ball.update(tilt_x, tilt_y, dt)
        log['t'].append(t + t_sim)
        log['x'].append(ball.state[0])
        log['y'].append(ball.state[1])
        log['tx'].append(tx)
        log['ty'].append(ty)
        log['vx'].append(ball.state[2])
        log['vy'].append(ball.state[3])

    return {k: np.array(v) for k, v in log.items()}


def main():
    log = simulate_trajectory()

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # XY轨迹
    axes[0, 0].plot(log['tx'], log['ty'], 'r--', linewidth=1, label='目标')
    axes[0, 0].plot(log['x'], log['y'], 'b-', linewidth=0.8, label='实际')
    axes[0, 0].set_title('XY平面轨迹')
    axes[0, 0].set_xlabel('X (m)')
    axes[0, 0].set_ylabel('Y (m)')
    axes[0, 0].legend()
    axes[0, 0].set_aspect('equal')
    axes[0, 0].grid(True, alpha=0.3)

    # X轴响应
    n_step = int(11 / 0.005)
    t_seg = log['t'][:n_step]
    axes[0, 1].plot(t_seg, log['tx'][:n_step], 'r--', label='目标X')
    axes[0, 1].plot(t_seg, log['x'][:n_step], 'b-', label='实际X')
    axes[0, 1].set_title('X轴阶跃响应')
    axes[0, 1].set_xlabel('时间 (s)')
    axes[0, 1].set_ylabel('X (m)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # 速度
    axes[1, 0].plot(log['t'], log['vx'], label='Vx')
    axes[1, 0].plot(log['t'], log['vy'], label='Vy')
    axes[1, 0].set_title('球速')
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('速度 (m/s)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 跟踪误差
    err = np.sqrt((log['x'] - log['tx'])**2 + (log['y'] - log['ty'])**2)
    axes[1, 1].plot(log['t'], err * 1000)
    axes[1, 1].set_title('跟踪误差')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].set_ylabel('误差 (mm)')
    axes[1, 1].grid(True, alpha=0.3)

    plt.suptitle('滚球控制系统仿真 (双轴PID)', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('ball_plate_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("仿真结果已保存: ball_plate_result.png")


if __name__ == '__main__':
    main()
