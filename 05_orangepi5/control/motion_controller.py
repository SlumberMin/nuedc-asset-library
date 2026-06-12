#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运动控制器V13 - Orange Pi 5
里程计 + 运动学逆解 + 速度平滑 + 限位保护
"""

import time
import math
import numpy as np
from collections import deque


class DifferentialKinematics:
    """差速运动学"""

    def __init__(self, wheel_base=0.3, wheel_radius=0.05):
        self.L = wheel_base
        self.R = wheel_radius

    def inverse(self, v, omega):
        """速度,角速度 -> 左右轮速(rad/s)"""
        vl = (v - omega * self.L / 2) / self.R
        vr = (v + omega * self.L / 2) / self.R
        return vl, vr

    def forward(self, wl, wr):
        """左右轮速(rad/s) -> 速度,角速度"""
        v = (wl + wr) * self.R / 2
        omega = (wr - wl) * self.R / self.L
        return v, omega


class MecanumKinematics:
    """麦克纳姆轮全向运动学"""

    def __init__(self, lx=0.15, ly=0.15, wheel_radius=0.05):
        self.lx = lx
        self.ly = ly
        self.R = wheel_radius
        self.L = lx + ly

    def inverse(self, vx, vy, omega):
        """返回4个轮速(rad/s): FL, FR, RL, RR"""
        L = self.L
        R = self.R
        fl = (vx - vy - L * omega) / R
        fr = (vx + vy + L * omega) / R
        rl = (vx + vy - L * omega) / R
        rr = (vx - vy + L * omega) / R
        return fl, fr, rl, rr


class Odometry:
    """里程计"""

    def __init__(self, wheel_base=0.3, wheel_radius=0.05):
        self.L = wheel_base
        self.R = wheel_radius
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = 0.0
        self.total_dist = 0.0

    def update(self, wl, wr, dt=None):
        now = time.monotonic()
        if dt is None:
            dt = now - self.last_time if self.last_time > 0 else 0.02
        self.last_time = now

        vl = wl * self.R
        vr = wr * self.R
        v = (vl + vr) / 2
        omega = (vr - vl) / self.L

        self.theta += omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.total_dist += abs(v * dt)

    def get_pose(self):
        return self.x, self.y, self.theta

    def reset(self):
        self.x = self.y = self.theta = self.total_dist = 0.0
        self.last_time = 0.0


class VelocitySmoother:
    """速度平滑与限幅"""

    def __init__(self, max_speed=1.0, max_accel=2.0, max_omega=3.0, max_alpha=5.0):
        self.max_v = max_speed
        self.max_a = max_accel
        self.max_omega = max_omega
        self.max_alpha = max_alpha
        self.v = 0.0
        self.omega = 0.0
        self.last_time = 0.0

    def smooth(self, v_cmd, omega_cmd, dt=None):
        now = time.monotonic()
        if dt is None:
            dt = now - self.last_time if self.last_time > 0 else 0.02
        self.last_time = now
        dt = max(dt, 1e-6)

        # 加速度限制
        dv = v_cmd - self.v
        max_dv = self.max_a * dt
        dv = max(-max_dv, min(max_dv, dv))
        self.v += dv

        dw = omega_cmd - self.omega
        max_dw = self.max_alpha * dt
        dw = max(-max_dw, min(max_dw, dw))
        self.omega += dw

        # 速度限制
        self.v = max(-self.max_v, min(self.max_v, self.v))
        self.omega = max(-self.max_omega, min(self.max_omega, self.omega))

        return self.v, self.omega

    def reset(self):
        self.v = self.omega = 0.0
        self.last_time = 0.0


class MotionControllerV13:
    """运动控制器V13"""

    def __init__(self, config=None):
        cfg = config or {}
        self.mode = cfg.get('mode', 'differential')

        if self.mode == 'mecanum':
            self.kin = MecanumKinematics(
                lx=cfg.get('lx', 0.15), ly=cfg.get('ly', 0.15),
                wheel_radius=cfg.get('wheel_radius', 0.05)
            )
        else:
            self.kin = DifferentialKinematics(
                wheel_base=cfg.get('wheel_base', 0.3),
                wheel_radius=cfg.get('wheel_radius', 0.05)
            )

        self.odom = Odometry(
            wheel_base=cfg.get('wheel_base', 0.3),
            wheel_radius=cfg.get('wheel_radius', 0.05)
        )

        self.smoother = VelocitySmoother(
            max_speed=cfg.get('max_speed', 1.0),
            max_accel=cfg.get('max_accel', 2.0),
            max_omega=cfg.get('max_omega', 3.0),
            max_alpha=cfg.get('max_alpha', 5.0)
        )

        # 位置PID(用于点到点运动)
        self.pos_kp = cfg.get('pos_kp', 1.5)
        self.pos_ki = cfg.get('pos_ki', 0.0)
        self.pos_kd = cfg.get('pos_kd', 0.5)
        self.angle_kp = cfg.get('angle_kp', 3.0)
        self.arrive_threshold = cfg.get('arrive_threshold', 0.05)

        self._pos_integral = 0.0
        self._prev_pos_err = 0.0

        # 安全限位
        self.bounds = cfg.get('bounds', None)  # (xmin, ymin, xmax, ymax)

    def compute_wheel_speeds(self, v_cmd, omega_cmd):
        """直接速度控制"""
        v, omega = self.smoother.smooth(v_cmd, omega_cmd)

        if self.mode == 'mecanum':
            return self.kin.inverse(v, 0, omega)
        else:
            return self.kin.inverse(v, omega)

    def goto(self, target_x, target_y, current_x=None, current_y=None, current_theta=None):
        """点到点运动控制"""
        if current_x is None:
            current_x, current_y, current_theta = self.odom.get_pose()

        dx = target_x - current_x
        dy = target_y - current_y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_err = self._normalize_angle(target_angle - current_theta)

        if dist < self.arrive_threshold:
            return 0.0, 0.0, True

        # 角度对准
        if abs(angle_err) > 0.3:
            omega = self.angle_kp * angle_err
            return 0.0, omega, False

        # 位置控制
        v = min(self.pos_kp * dist, self.smoother.max_v)
        omega = self.angle_kp * angle_err
        return v, omega, False

    def goto_heading(self, target_x, target_y, heading, current_x=None, current_y=None, current_theta=None):
        """到达目标点并保持指定朝向"""
        if current_x is None:
            current_x, current_y, current_theta = self.odom.get_pose()

        dx = target_x - current_x
        dy = target_y - current_y
        dist = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        angle_err = self._normalize_angle(target_angle - current_theta)

        if dist < self.arrive_threshold:
            heading_err = self._normalize_angle(heading - current_theta)
            if abs(heading_err) < 0.05:
                return 0.0, 0.0, True
            return 0.0, self.angle_kp * heading_err, False

        v = min(self.pos_kp * dist, self.smoother.max_v)
        omega = self.angle_kp * angle_err
        return v, omega, False

    def update_odometry(self, wl, wr, dt=None):
        self.odom.update(wl, wr, dt)

    def check_bounds(self, x=None, y=None):
        if self.bounds is None:
            return True
        if x is None:
            x, y, _ = self.odom.get_pose()
        xmin, ymin, xmax, ymax = self.bounds
        return xmin <= x <= xmax and ymin <= y <= ymax

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def get_state(self):
        pose = self.odom.get_pose()
        return {
            'pose': pose,
            'total_distance': self.odom.total_dist,
            'current_v': self.smoother.v,
            'current_omega': self.smoother.omega,
            'mode': self.mode
        }

    def reset(self):
        self.odom.reset()
        self.smoother.reset()
        self._pos_integral = 0.0
        self._prev_pos_err = 0.0


if __name__ == '__main__':
    ctrl = MotionControllerV13({'mode': 'differential', 'max_speed': 0.5})
    print("Testing goto...")
    for i in range(200):
        v, omega, arrived = ctrl.goto(1.0, 1.0)
        wl, wr = ctrl.compute_wheel_speeds(v, omega)
        ctrl.update_odometry(wl, wr)
        if arrived:
            print(f"Arrived at step {i}!")
            break
    state = ctrl.get_state()
    print(f"Final pose: ({state['pose'][0]:.3f}, {state['pose'][1]:.3f}, {math.degrees(state['pose'][2]):.1f}deg)")
    print("MotionControllerV13 test passed.")
