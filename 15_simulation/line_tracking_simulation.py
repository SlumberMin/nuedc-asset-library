"""
循迹算法仿真 - 模拟赛道 + 传感器阵列 + PID控制
用法: python line_tracking_simulation.py
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


class Track:
    """赛道生成器 - 支持直线/弯道/S弯/十字"""
    def __init__(self):
        self.points = self._generate_track()

    def _generate_track(self):
        pts = []
        # 直线段
        for x in np.linspace(0, 2, 50):
            pts.append((x, 0))
        # 右弯
        for angle in np.linspace(0, np.pi/2, 40):
            pts.append((2 + np.sin(angle), -1 + np.cos(angle)))
        # 直线
        for x in np.linspace(2, 5, 80):
            pts.append((x, 0))
        # S弯
        for x in np.linspace(5, 8, 80):
            y = 0.8 * np.sin((x - 5) * np.pi / 3)
            pts.append((x, y))
        # 左弯
        for angle in np.linspace(0, np.pi/2, 40):
            pts.append((8 - np.sin(angle), 1 - np.cos(angle)))
        # 直线回
        for x in np.linspace(8, 10, 50):
            pts.append((x, 0))
        return np.array(pts)

    def get_nearest(self, x, y):
        """返回最近赛道点的横向偏差"""
        dists = np.sqrt((self.points[:, 0] - x)**2 + (self.points[:, 1] - y)**2)
        idx = np.argmin(dists)
        return self.points[idx], dists[idx], idx


class SensorArray:
    """红外传感器阵列模拟"""
    def __init__(self, n_sensors=7, spacing=3.0):
        self.n = n_sensors
        self.spacing = spacing  # mm

    def read(self, track, car_x, car_y, car_theta):
        """返回各传感器读数 (0=在线上, 1=在线外)"""
        sensor_positions = np.linspace(-(self.n-1)/2 * self.spacing,
                                        (self.n-1)/2 * self.spacing, self.n)
        readings = []
        for offset in sensor_positions:
            sx = car_x + offset * np.cos(np.pi/2 - car_theta)
            sy = car_y + offset * np.sin(np.pi/2 - car_theta)
            _, dist, _ = track.get_nearest(sx, sy)
            readings.append(1.0 if dist > 5.0 else 0.0)
        return np.array(readings)

    def get_error(self, readings):
        """加权平均计算偏差 (左正右负)"""
        weights = np.linspace(-(self.n-1)/2, (self.n-1)/2, self.n)
        if readings.sum() == self.n:
            return 0.0  # 全部离线 - 丢失
        return np.dot(weights, 1 - readings) / max((1 - readings).sum(), 1)


class CarModel:
    """简化的两轮差速车模型"""
    def __init__(self, x=0, y=0, theta=0, wheelbase=20, max_speed=50):
        self.x, self.y, self.theta = x, y, theta
        self.wheelbase = wheelbase  # mm
        self.max_speed = max_speed  # mm/s
        self.base_speed = 30
        self.track_x, self.track_y = [x], [y]

    def update(self, steering, dt):
        """steering: -1(左) 到 1(右)"""
        speed = self.base_speed
        omega = steering * speed / self.wheelbase * 2
        self.theta += omega * dt
        self.x += speed * np.cos(self.theta) * dt
        self.y += speed * np.sin(self.theta) * dt
        self.track_x.append(self.x)
        self.track_y.append(self.y)

    def get_position(self):
        return self.x, self.y, self.theta


class PID:
    def __init__(self, kp, ki, kd, out_min=-1, out_max=1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        deriv = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        return max(self.out_min, min(self.out_max, out))


def main():
    track = Track()
    sensors = SensorArray(n_sensors=7, spacing=3.0)
    car = CarModel(x=0, y=0, theta=0)
    pid = PID(kp=0.08, ki=0.001, kd=0.05)

    dt = 0.05
    total_time = 60.0
    errors = []

    for step in range(int(total_time / dt)):
        cx, cy, ctheta = car.get_position()
        readings = sensors.read(track, cx, cy, ctheta)
        error = sensors.get_error(readings)
        errors.append(error)
        steering = pid.compute(error, dt)
        car.update(steering, dt)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # 赛道与轨迹
    axes[0].plot(track.points[:, 0], track.points[:, 1], 'k-', linewidth=3, label='赛道中心线')
    axes[0].plot(car.track_x, car.track_y, 'r-', linewidth=1, label='小车轨迹')
    axes[0].set_title('赛道与行驶轨迹')
    axes[0].set_xlabel('X (mm)')
    axes[0].set_ylabel('Y (mm)')
    axes[0].legend()
    axes[0].set_aspect('equal')
    axes[0].grid(True, alpha=0.3)

    # 偏差曲线
    t = np.arange(len(errors)) * dt
    axes[1].plot(t, errors)
    axes[1].set_title('循迹偏差')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('偏差 (格)')
    axes[1].grid(True, alpha=0.3)

    plt.suptitle('循迹算法仿真', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('line_tracking_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("仿真结果已保存: line_tracking_result.png")


if __name__ == '__main__':
    main()
