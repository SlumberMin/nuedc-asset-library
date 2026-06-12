"""
舵机运动仿真 - 轨迹规划/多舵机协调/动作组
nuedc-asset-library V3
"""
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
from enum import Enum

class MotionProfile(Enum):
    TRAPEZOIDAL = "trapezoidal"
    S_CURVE = "s_curve"
    LINEAR = "linear"
    CUBIC_POLYNOMIAL = "cubic"
    MINIMUM_JERK = "minimum_jerk"


@dataclass
class ServoParams:
    """舵机物理参数"""
    min_angle: float = 0.0      # 度
    max_angle: float = 180.0    # 度
    max_speed: float = 600.0    # 度/秒 (60度/0.1秒典型值)
    max_accel: float = 3000.0   # 度/秒²
    max_decel: float = 3000.0   # 度/秒²
    pwm_min_us: float = 500.0   # 最小脉宽
    pwm_max_us: float = 2500.0  # 最大脉宽
    pwm_period_us: float = 20000.0  # 20ms周期
    position_resolution: float = 0.3  # 度
    deadband: float = 1.0       # 死区(度)
    torque_kg: float = 15.0     # 扭矩(kg·cm)

    def angle_to_pwm(self, angle: float) -> float:
        ratio = (angle - self.min_angle) / (self.max_angle - self.min_angle)
        return self.pwm_min_us + ratio * (self.pwm_max_us - self.pwm_min_us)

    def pwm_to_angle(self, pwm_us: float) -> float:
        ratio = (pwm_us - self.pwm_min_us) / (self.pwm_max_us - self.pwm_min_us)
        return self.min_angle + ratio * (self.max_angle - self.min_angle)


@dataclass
class MotionState:
    """运动状态"""
    angle: float = 90.0
    velocity: float = 0.0      # 度/秒
    acceleration: float = 0.0  # 度/秒²
    time: float = 0.0
    at_target: bool = True


@dataclass
class Waypoint:
    """路径点"""
    angle: float
    duration: float  # 秒
    speed: Optional[float] = None  # 可选：指定到达速度


class TrajectoryPlanner:
    """轨迹规划器"""

    def __init__(self, params: ServoParams):
        self.params = params

    def trapezoidal_profile(self, start: float, end: float, dt: float) -> List[float]:
        """梯形速度规划"""
        distance = end - start
        direction = 1 if distance > 0 else -1
        distance = abs(distance)

        v_max = self.params.max_speed
        a_max = self.params.max_accel

        # 计算各阶段时间
        t_accel = v_max / a_max
        d_accel = 0.5 * a_max * t_accel ** 2

        if 2 * d_accel > distance:
            # 三角形轮廓（到不了最大速度）
            t_accel = np.sqrt(distance / a_max)
            t_cruise = 0
            t_decel = t_accel
        else:
            d_decel = d_accel
            d_cruise = distance - d_accel - d_decel
            t_cruise = d_cruise / v_max
            t_decel = t_accel

        total_time = t_accel + t_cruise + t_decel
        n_steps = int(total_time / dt) + 1
        positions = []
        pos = start

        for i in range(n_steps):
            t = i * dt
            if t <= t_accel:
                pos = start + direction * 0.5 * a_max * t ** 2
            elif t <= t_accel + t_cruise:
                t_c = t - t_accel
                pos = start + direction * (d_accel + v_max * t_c)
            elif t <= total_time:
                t_d = t - t_accel - t_cruise
                pos = start + direction * (d_accel + v_max * t_cruise + v_max * t_d - 0.5 * a_max * t_d ** 2)
            else:
                pos = end
            positions.append(pos)

        return positions

    def s_curve_profile(self, start: float, end: float, dt: float) -> List[float]:
        """S曲线规划（7段式）"""
        distance = abs(end - start)
        direction = 1 if end > start else -1

        v_max = self.params.max_speed
        a_max = self.params.max_accel
        j_max = a_max * 10  # 加加速度

        # 简化S曲线：使用平滑插值
        t_total = distance / v_max * 2  # 估算总时间
        n_steps = int(t_total / dt) + 1
        if n_steps < 2:
            n_steps = 2

        positions = []
        for i in range(n_steps):
            t = i / (n_steps - 1)
            # 5次多项式平滑
            s = 10 * t**3 - 15 * t**4 + 6 * t**5
            pos = start + direction * distance * s
            positions.append(pos)

        return positions

    def cubic_polynomial(self, start: float, end: float, dt: float,
                         v_start: float = 0, v_end: float = 0) -> List[float]:
        """三次多项式插值"""
        distance = abs(end - start)
        direction = 1 if end > start else -1
        t_total = distance / self.params.max_speed * 1.5
        n_steps = max(int(t_total / dt) + 1, 2)

        # 三次多项式系数: s(t) = a0 + a1*t + a2*t² + a3*t³
        T = t_total
        a0 = 0
        a1 = v_start * direction
        a2 = (3 * distance - (2 * v_start + v_end) * T * direction) / (T ** 2) if T > 0 else 0
        a3 = (-2 * distance + (v_start + v_end) * T * direction) / (T ** 3) if T > 0 else 0

        positions = []
        for i in range(n_steps):
            t_norm = i / (n_steps - 1) if n_steps > 1 else 1
            s = a0 + a1 * (t_norm * T) + a2 * (t_norm * T)**2 + a3 * (t_norm * T)**3
            positions.append(start + s)
        return positions

    def minimum_jerk(self, start: float, end: float, dt: float) -> List[float]:
        """最小加加速度轨迹"""
        distance = abs(end - start)
        direction = 1 if end > start else -1
        t_total = distance / self.params.max_speed * 1.5
        n_steps = max(int(t_total / dt) + 1, 2)

        positions = []
        for i in range(n_steps):
            t = i / (n_steps - 1)
            # 最小jerk: s(t) = s0 + (s1-s0)[10(t/T)^3 - 15(t/T)^4 + 6(t/T)^5]
            s = 10 * t**3 - 15 * t**4 + 6 * t**5
            positions.append(start + direction * distance * s)
        return positions

    def plan(self, start: float, end: float, dt: float, profile: MotionProfile) -> List[float]:
        if profile == MotionProfile.TRAPEZOIDAL:
            return self.trapezoidal_profile(start, end, dt)
        elif profile == MotionProfile.S_CURVE:
            return self.s_curve_profile(start, end, dt)
        elif profile == MotionProfile.CUBIC_POLYNOMIAL:
            return self.cubic_polynomial(start, end, dt)
        elif profile == MotionProfile.MINIMUM_JERK:
            return self.minimum_jerk(start, end, dt)
        else:
            n = max(int(abs(end - start) / self.params.max_speed / dt) + 1, 2)
            return list(np.linspace(start, end, n))


class Servo:
    """单个舵机模型"""

    def __init__(self, servo_id: int, params: ServoParams = None):
        self.id = servo_id
        self.params = params or ServoParams()
        self.planner = TrajectoryPlanner(self.params)
        self.state = MotionState(angle=90.0)
        self.trajectory: List[float] = []
        self.trajectory_index = 0
        self.move_log: List[dict] = []

    def set_target(self, target_angle: float, profile: MotionProfile = MotionProfile.S_CURVE, dt: float = 0.02):
        """设置目标角度"""
        target_angle = np.clip(target_angle, self.params.min_angle, self.params.max_angle)
        self.trajectory = self.planner.plan(self.state.angle, target_angle, dt, profile)
        self.trajectory_index = 0
        self.state.at_target = False

    def update(self, dt: float) -> MotionState:
        """更新一步"""
        if self.trajectory_index < len(self.trajectory):
            prev_angle = self.state.angle
            self.state.angle = self.trajectory[self.trajectory_index]
            self.state.velocity = (self.state.angle - prev_angle) / dt if dt > 0 else 0
            self.state.time += dt
            self.trajectory_index += 1
        else:
            self.state.velocity = 0
            self.state.acceleration = 0
            self.state.at_target = True
        return self.state

    @property
    def pwm_us(self) -> float:
        return self.params.angle_to_pwm(self.state.angle)


class MultiServoCoordinator:
    """多舵机协调器"""

    def __init__(self, num_servos: int = 4):
        self.servos = [Servo(i) for i in range(num_servos)]

    def synchronized_move(self, targets: List[float], profile: MotionProfile = MotionProfile.S_CURVE, dt: float = 0.02):
        """同步运动 - 所有舵机同时到达"""
        trajectories = []
        max_steps = 0
        for servo, target in zip(self.servos, targets):
            traj = servo.planner.plan(servo.state.angle, target, dt, profile)
            trajectories.append(traj)
            max_steps = max(max_steps, len(traj))

        # 拉伸所有轨迹到相同长度
        for i, traj in enumerate(trajectories):
            if len(traj) < max_steps:
                stretched = np.interp(
                    np.linspace(0, len(traj) - 1, max_steps),
                    np.arange(len(traj)),
                    traj
                ).tolist()
                self.servos[i].trajectory = stretched
            else:
                self.servos[i].trajectory = traj
            self.servos[i].trajectory_index = 0
            self.servos[i].state.at_target = False

    def update_all(self, dt: float) -> List[MotionState]:
        return [s.update(dt) for s in self.servos]

    @property
    def all_at_target(self) -> bool:
        return all(s.state.at_target for s in self.servos)


class ActionGroup:
    """动作组 - 预设动作序列"""

    def __init__(self, name: str, coordinator: MultiServoCoordinator):
        self.name = name
        self.coordinator = coordinator
        self.keyframes: List[Dict] = []
        self.dt = 0.02

    def add_keyframe(self, targets: List[float], hold_time: float = 0.5,
                     profile: MotionProfile = MotionProfile.S_CURVE):
        self.keyframes.append({
            "targets": targets,
            "hold_time": hold_time,
            "profile": profile,
        })

    def execute(self) -> dict:
        """执行动作组"""
        total_time = 0
        all_positions = []

        for kf in self.keyframes:
            self.coordinator.synchronized_move(kf["targets"], kf["profile"], self.dt)

            # 运行到目标
            positions = []
            while not self.coordinator.all_at_target:
                states = self.coordinator.update_all(self.dt)
                total_time += self.dt
                positions.append([s.angle for s in states])
            total_time += kf["hold_time"]
            all_positions.extend(positions)

        return {
            "keyframes": len(self.keyframes),
            "total_time_s": total_time,
            "position_log": np.array(all_positions) if all_positions else np.array([]),
        }


# ── 演示 ──
def demo():
    print("=" * 60)
    print("舵机运动仿真 - Demo")
    print("=" * 60)

    # 1. 轨迹规划对比
    params = ServoParams()
    planner = TrajectoryPlanner(params)
    print("\n[轨迹规划对比] 0° -> 180°")
    for profile in MotionProfile:
        traj = planner.plan(0, 180, 0.02, profile)
        print(f"  {profile.value:20s}: {len(traj)}步, 终点={traj[-1]:.1f}°")

    # 2. 多舵机协调
    print("\n[多舵机协调] 4舵机同步运动")
    coord = MultiServoCoordinator(4)
    # 初始位置
    for s in coord.servos:
        s.state.angle = 45.0

    coord.synchronized_move([135, 90, 45, 170], MotionProfile.S_CURVE)
    steps = 0
    while not coord.all_at_target:
        states = coord.update_all(0.02)
        steps += 1
    print(f"  同步运动完成: {steps}步, {steps * 0.02:.2f}秒")
    for i, s in enumerate(coord.servos):
        print(f"  舵机{i}: {s.state.angle:.1f}°")

    # 3. 动作组
    print("\n[动作组] 机械臂抓取动作")
    coord2 = MultiServoCoordinator(6)
    for s in coord2.servos:
        s.state.angle = 90.0

    group = ActionGroup("grab", coord2)
    group.add_keyframe([45, 120, 90, 90, 30, 90], 0.3)   # 移动到物体上方
    group.add_keyframe([45, 120, 90, 90, 30, 150], 0.2)   # 抓取
    group.add_keyframe([90, 90, 90, 90, 150, 150], 0.5)   # 抬起
    result = group.execute()
    print(f"  关键帧: {result['keyframes']}")
    print(f"  总时间: {result['total_time_s']:.2f}秒")

    # 4. PWM输出
    print("\n[PWM输出] 角度映射")
    for angle in [0, 45, 90, 135, 180]:
        pwm = params.angle_to_pwm(angle)
        print(f"  {angle:3d}° -> {pwm:.0f}us")

    print("\n✅ 舵机运动仿真完成")


if __name__ == "__main__":
    demo()
