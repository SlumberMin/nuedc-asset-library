#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人运动学仿真 - 正/逆运动学+轨迹规划+避障
===============================================
功能:
  1. DH参数建模 (2-6自由度机械臂)
  2. 正运动学 (齐次变换矩阵)
  3. 逆运动学 (解析法 + 数值法/雅可比迭代)
  4. 轨迹规划 (多项式插值/梯形速度/样条)
  5. 工作空间分析
  6. 简化避障 (势场法)
  7. 差速/全向移动机器人运动学

依赖: numpy (必需), matplotlib (可选)
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
import warnings


# ============================================================
# 1. DH参数与变换矩阵
# ============================================================

@dataclass
class DHParam:
    """DH参数"""
    a: float      # 连杆长度
    alpha: float  # 连杆扭角 (rad)
    d: float      # 连杆偏距
    theta: float  # 关节角 (rad)
    joint_type: str = "revolute"  # revolute / prismatic


def dh_matrix(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """计算单个DH变换矩阵 (标准DH)"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ],
    ])


def dh_matrix_modified(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """计算单个MDH变换矩阵 (改进DH)"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct,    -st,     0,   a   ],
        [st*ca,  ct*ca, -sa, -sa*d],
        [st*sa,  ct*sa,  ca,  ca*d],
        [0,      0,      0,   1   ],
    ])


# ============================================================
# 2. 机械臂建模
# ============================================================

class RobotArm:
    """多自由度机械臂"""

    def __init__(self, name: str = "robot"):
        self.name = name
        self.joints: List[DHParam] = []
        self.n_joints = 0
        self.joint_limits: List[Tuple[float, float]] = []
        self._predefined_robots()

    def _predefined_robots(self):
        """预定义常用机器人构型"""
        pass  # 通过工厂方法创建

    @classmethod
    def create_2dof(cls) -> 'RobotArm':
        """2自由度平面机械臂"""
        arm = cls("2-DOF Planar")
        arm.joints = [
            DHParam(a=0.4, alpha=0, d=0, theta=0),
            DHParam(a=0.3, alpha=0, d=0, theta=0),
        ]
        arm.n_joints = 2
        arm.joint_limits = [(-np.pi, np.pi), (-np.pi, np.pi)]
        return arm

    @classmethod
    def create_3dof(cls) -> 'RobotArm':
        """3自由度机械臂 (RRR)"""
        arm = cls("3-DOF RRR")
        arm.joints = [
            DHParam(a=0, alpha=np.pi/2, d=0.3, theta=0),
            DHParam(a=0.4, alpha=0, d=0, theta=0),
            DHParam(a=0.3, alpha=0, d=0, theta=0),
        ]
        arm.n_joints = 3
        arm.joint_limits = [(-np.pi, np.pi), (-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2)]
        return arm

    @classmethod
    def create_6dof(cls) -> 'RobotArm':
        """6自由度机械臂 (类PUMA)"""
        arm = cls("6-DOF PUMA-like")
        arm.joints = [
            DHParam(a=0,      alpha=-np.pi/2, d=0.4,  theta=0),
            DHParam(a=0.4,    alpha=0,         d=0,    theta=0),
            DHParam(a=0.05,   alpha=-np.pi/2, d=0,    theta=0),
            DHParam(a=0,      alpha=np.pi/2,   d=0.4,  theta=0),
            DHParam(a=0,      alpha=-np.pi/2, d=0,    theta=0),
            DHParam(a=0,      alpha=0,         d=0.1,  theta=0),
        ]
        arm.n_joints = 6
        arm.joint_limits = [(-np.pi, np.pi)] * 6
        return arm

    def forward_kinematics(self, joint_angles: np.ndarray) -> np.ndarray:
        """正运动学: 关节角 -> 末端位姿 (4x4齐次变换矩阵)"""
        assert len(joint_angles) == self.n_joints

        T = np.eye(4)
        for i, (joint, q) in enumerate(zip(self.joints, joint_angles)):
            theta = q if joint.joint_type == "revolute" else joint.theta
            d = joint.d if joint.joint_type == "revolute" else q
            T = T @ dh_matrix(joint.a, joint.alpha, d, theta)

        return T

    def forward_kinematics_chain(self, joint_angles: np.ndarray) -> List[np.ndarray]:
        """返回每个关节的变换矩阵链"""
        assert len(joint_angles) == self.n_joints

        chain = [np.eye(4)]
        T = np.eye(4)
        for i, (joint, q) in enumerate(zip(self.joints, joint_angles)):
            theta = q if joint.joint_type == "revolute" else joint.theta
            d = joint.d if joint.joint_type == "revolute" else q
            T = T @ dh_matrix(joint.a, joint.alpha, d, theta)
            chain.append(T.copy())

        return chain

    def get_end_effector_position(self, joint_angles: np.ndarray) -> np.ndarray:
        """获取末端执行器位置"""
        T = self.forward_kinematics(joint_angles)
        return T[:3, 3]

    def get_end_effector_pose(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """获取末端执行器位姿 (位置, 旋转矩阵)"""
        T = self.forward_kinematics(joint_angles)
        return T[:3, 3], T[:3, :3]


# ============================================================
# 3. 逆运动学
# ============================================================

class InverseKinematics:
    """逆运动学求解器"""

    def __init__(self, robot: RobotArm):
        self.robot = robot

    def jacobian_numerical(self, q: np.ndarray, delta: float = 1e-6) -> np.ndarray:
        """数值雅可比矩阵 (6xn)"""
        n = self.robot.n_joints
        J = np.zeros((6, n))

        T0 = self.robot.forward_kinematics(q)
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]

        for i in range(n):
            q_plus = q.copy()
            q_plus[i] += delta

            T_plus = self.robot.forward_kinematics(q_plus)
            p_plus = T_plus[:3, 3]
            R_plus = T_plus[:3, :3]

            # 位置部分
            J[:3, i] = (p_plus - p0) / delta

            # 姿态部分 (使用旋转矩阵的对数映射近似)
            dR = R_plus @ R0.T
            J[3:, i] = np.array([
                dR[2, 1] - dR[1, 2],
                dR[0, 2] - dR[2, 0],
                dR[1, 0] - dR[0, 1],
            ]) / (2 * delta)

        return J

    def solve_numerical(self, target_pos: np.ndarray,
                        target_rot: Optional[np.ndarray] = None,
                        q0: Optional[np.ndarray] = None,
                        max_iter: int = 100,
                        tol: float = 1e-4,
                        damping: float = 0.1) -> Tuple[np.ndarray, bool]:
        """数值法逆运动学 (阻尼最小二乘法)"""
        n = self.robot.n_joints
        if q0 is None:
            q = np.zeros(n)
        else:
            q = q0.copy()

        for iteration in range(max_iter):
            T = self.robot.forward_kinematics(q)
            pos = T[:3, 3]
            rot = T[:3, :3]

            # 位置误差
            pos_error = target_pos - pos

            if target_rot is not None:
                # 姿态误差
                rot_error = 0.5 * (np.cross(rot[:, 0], target_rot[:, 0]) +
                                   np.cross(rot[:, 1], target_rot[:, 1]) +
                                   np.cross(rot[:, 2], target_rot[:, 2]))
                error = np.concatenate([pos_error, rot_error])
            else:
                error = pos_error

            if np.linalg.norm(error) < tol:
                return q, True

            # 雅可比矩阵
            if target_rot is not None:
                J = self.jacobian_numerical(q)
            else:
                J_full = self.jacobian_numerical(q)
                J = J_full[:3, :]

            # 阻尼最小二乘 (DLS)
            JtJ = J.T @ J
            dq = J.T @ np.linalg.solve(JtJ + damping**2 * np.eye(n), error)

            q += dq

            # 关节限位
            for i in range(n):
                low, high = self.robot.joint_limits[i]
                q[i] = np.clip(q[i], low, high)

        return q, False

    def solve_2dof_analytical(self, target_x: float, target_y: float) -> List[np.ndarray]:
        """2自由度平面臂解析逆运动学"""
        assert self.robot.n_joints == 2

        L1 = self.robot.joints[0].a
        L2 = self.robot.joints[1].a

        r2 = target_x**2 + target_y**2
        cos_q2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)

        if abs(cos_q2) > 1:
            return []  # 不可达

        solutions = []
        for sign in [1, -1]:
            q2 = sign * np.arccos(np.clip(cos_q2, -1, 1))
            q1 = np.arctan2(target_y, target_x) - np.arctan2(
                L2 * np.sin(q2), L1 + L2 * np.cos(q2))
            solutions.append(np.array([q1, q2]))

        return solutions


# ============================================================
# 4. 工作空间分析
# ============================================================

class WorkspaceAnalyzer:
    """工作空间分析"""

    def __init__(self, robot: RobotArm):
        self.robot = robot

    def sample_workspace(self, n_samples: int = 10000) -> np.ndarray:
        """蒙特卡洛采样工作空间"""
        points = []
        n = self.robot.n_joints

        for _ in range(n_samples):
            q = np.array([
                np.random.uniform(*self.robot.joint_limits[i])
                for i in range(n)
            ])
            pos = self.robot.get_end_effector_position(q)
            points.append(pos)

        return np.array(points)

    def compute_reachability(self, target_pos: np.ndarray,
                             n_samples: int = 1000) -> Dict:
        """评估目标位置的可达性"""
        ik = InverseKinematics(self.robot)
        n = self.robot.n_joints
        success_count = 0
        solutions = []

        for _ in range(n_samples):
            q0 = np.array([
                np.random.uniform(*self.robot.joint_limits[i])
                for i in range(n)
            ])
            q_sol, success = ik.solve_numerical(target_pos, q0=q0)
            if success:
                success_count += 1
                solutions.append(q_sol)

        return {
            "reachable": success_count > 0,
            "success_rate": success_count / n_samples,
            "n_solutions": len(solutions),
            "solutions": solutions[:10],  # 保留最多10个
        }


# ============================================================
# 5. 轨迹规划
# ============================================================

class TrajectoryPlanner:
    """轨迹规划器"""

    @staticmethod
    def cubic_polynomial(q0: float, qf: float, t0: float, tf: float,
                         dq0: float = 0, dqf: float = 0) -> callable:
        """三次多项式轨迹"""
        A = np.array([
            [1, t0, t0**2, t0**3],
            [0, 1, 2*t0, 3*t0**2],
            [1, tf, tf**2, tf**3],
            [0, 1, 2*tf, 3*tf**2],
        ])
        b = np.array([q0, dq0, qf, dqf])
        coeffs = np.linalg.solve(A, b)

        def trajectory(t):
            t_arr = np.atleast_1d(t)
            result = sum(coeffs[i] * t_arr**i for i in range(4))
            return result if np.ndim(t) > 0 else float(result)

        return trajectory

    @staticmethod
    def quintic_polynomial(q0: float, qf: float, t0: float, tf: float) -> callable:
        """五次多项式轨迹 (保证加速度连续)"""
        A = np.array([
            [1, t0, t0**2, t0**3, t0**4, t0**5],
            [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
            [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
            [1, tf, tf**2, tf**3, tf**4, tf**5],
            [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
            [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3],
        ])
        b = np.array([q0, 0, 0, qf, 0, 0])
        coeffs = np.linalg.solve(A, b)

        def trajectory(t):
            t_arr = np.atleast_1d(t)
            result = sum(coeffs[i] * t_arr**i for i in range(6))
            return result if np.ndim(t) > 0 else float(result)

        return trajectory

    @staticmethod
    def trapezoidal_profile(q0: float, qf: float, v_max: float, a_max: float) -> callable:
        """梯形速度规划"""
        dq = qf - q0
        sign = np.sign(dq) if dq != 0 else 1
        dq = abs(dq)

        # 加速段时间
        t_acc = v_max / a_max
        # 匀速段距离
        d_acc = 0.5 * a_max * t_acc**2

        if 2 * d_acc > dq:
            # 无匀速段 (三角形)
            t_acc = np.sqrt(dq / a_max)
            t_total = 2 * t_acc
            v_peak = a_max * t_acc
            d_cruise = 0
            t_cruise = 0
        else:
            d_cruise = dq - 2 * d_acc
            t_cruise = d_cruise / v_max
            t_total = 2 * t_acc + t_cruise
            v_peak = v_max

        def trajectory(t):
            t_arr = np.atleast_1d(t)
            result = np.zeros_like(t_arr, dtype=float)

            for i, ti in enumerate(t_arr):
                if ti < 0:
                    result[i] = q0
                elif ti < t_acc:
                    result[i] = q0 + sign * 0.5 * a_max * ti**2
                elif ti < t_acc + t_cruise:
                    dt = ti - t_acc
                    result[i] = q0 + sign * (d_acc + v_peak * dt)
                elif ti <= t_total:
                    dt = ti - t_acc - t_cruise
                    result[i] = qf - sign * 0.5 * a_max * (t_total - ti)**2
                else:
                    result[i] = qf

            return result if np.ndim(t) > 0 else float(result)

        return trajectory

    @staticmethod
    def multi_joint_trajectory(waypoints: List[np.ndarray],
                               segment_duration: float = 2.0,
                               method: str = "quintic") -> callable:
        """多关节轨迹规划"""
        n_segments = len(waypoints) - 1
        n_joints = len(waypoints[0])

        trajectories = []
        for seg in range(n_segments):
            seg_trajs = []
            for j in range(n_joints):
                t0 = seg * segment_duration
                tf = (seg + 1) * segment_duration
                if method == "quintic":
                    traj = TrajectoryPlanner.quintic_polynomial(
                        waypoints[seg][j], waypoints[seg+1][j], t0, tf)
                else:
                    traj = TrajectoryPlanner.cubic_polynomial(
                        waypoints[seg][j], waypoints[seg+1][j], t0, tf)
                seg_trajs.append(traj)
            trajectories.append(seg_trajs)

        total_duration = n_segments * segment_duration

        def multi_traj(t):
            t_arr = np.atleast_1d(t)
            result = np.zeros((len(t_arr), n_joints))

            for i, ti in enumerate(t_arr):
                seg = min(int(ti / segment_duration), n_segments - 1)
                for j in range(n_joints):
                    result[i, j] = trajectories[seg][j](ti)

            return result

        return multi_traj, total_duration


# ============================================================
# 6. 避障 (人工势场法)
# ============================================================

class ObstacleAvoidance:
    """人工势场法避障"""

    def __init__(self, k_att: float = 1.0, k_rep: float = 0.5,
                 d0: float = 0.2, step_size: float = 0.01):
        self.k_att = k_att   # 引力系数
        self.k_rep = k_rep   # 斥力系数
        self.d0 = d0          # 斥力影响范围
        self.step_size = step_size

    def attractive_force(self, current: np.ndarray, goal: np.ndarray) -> np.ndarray:
        """引力"""
        diff = goal - current
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            return np.zeros_like(current)
        return self.k_att * diff

    def repulsive_force(self, current: np.ndarray, obstacles: List[Tuple[np.ndarray, float]]) -> np.ndarray:
        """斥力"""
        force = np.zeros_like(current)
        for center, radius in obstacles:
            diff = current - center
            dist = np.linalg.norm(diff) - radius
            if dist < 1e-6:
                dist = 1e-6
            if dist < self.d0:
                magnitude = self.k_rep * (1/dist - 1/self.d0) / (dist**2)
                force += magnitude * (diff / np.linalg.norm(diff))
        return force

    def plan_path(self, start: np.ndarray, goal: np.ndarray,
                  obstacles: List[Tuple[np.ndarray, float]],
                  max_iter: int = 1000) -> Tuple[List[np.ndarray], bool]:
        """势场法路径规划"""
        path = [start.copy()]
        current = start.copy()

        for i in range(max_iter):
            if np.linalg.norm(current - goal) < 0.02:
                return path, True

            f_att = self.attractive_force(current, goal)
            f_rep = self.repulsive_force(current, obstacles)
            f_total = f_att + f_rep

            # 归一化步进
            if np.linalg.norm(f_total) > 1e-6:
                direction = f_total / np.linalg.norm(f_total)
            else:
                direction = (goal - current) / (np.linalg.norm(goal - current) + 1e-6)

            current = current + self.step_size * direction
            path.append(current.copy())

        return path, False


# ============================================================
# 7. 移动机器人运动学
# ============================================================

class DifferentialDriveRobot:
    """差速驱动移动机器人"""

    def __init__(self, wheel_radius: float = 0.05, wheel_base: float = 0.3,
                 x: float = 0, y: float = 0, theta: float = 0):
        self.R = wheel_radius
        self.L = wheel_base
        self.x = x
        self.y = y
        self.theta = theta
        self.trajectory = [(x, y, theta)]

    def update(self, v_left: float, v_right: float, dt: float) -> Tuple[float, float, float]:
        """运动更新"""
        # 差速运动学
        v = (v_right + v_left) / 2
        omega = (v_right - v_left) / self.L

        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt

        # 规范化角度
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.trajectory.append((self.x, self.y, self.theta))
        return self.x, self.y, self.theta

    def inverse_kinematics(self, v: float, omega: float) -> Tuple[float, float]:
        """逆运动学: 线速度+角速度 -> 左右轮速"""
        v_left = v - omega * self.L / 2
        v_right = v + omega * self.L / 2
        return v_left, v_right

    def go_to_point(self, target_x: float, target_y: float,
                    v_max: float = 0.5, k_omega: float = 2.0,
                    dt: float = 0.01, max_time: float = 60.0) -> List[Tuple]:
        """点到点控制"""
        path = []
        t = 0

        while t < max_time:
            dx = target_x - self.x
            dy = target_y - self.y
            dist = np.sqrt(dx**2 + dy**2)

            if dist < 0.05:
                break

            target_angle = np.arctan2(dy, dx)
            angle_error = target_angle - self.theta
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

            # 速度控制
            v = min(v_max, dist)
            omega = k_omega * angle_error

            vl, vr = self.inverse_kinematics(v, omega)
            pos = self.update(vl, vr, dt)
            path.append(pos)
            t += dt

        return path


class OmnidirectionalRobot:
    """全向移动机器人 (麦克纳姆轮)"""

    def __init__(self, radius: float = 0.15, x: float = 0, y: float = 0, theta: float = 0):
        self.R = radius
        self.x = x
        self.y = y
        self.theta = theta
        self.trajectory = [(x, y, theta)]

    def inverse_kinematics(self, vx: float, vy: float, omega: float) -> np.ndarray:
        """逆运动学: 世界坐标速度 -> 四轮转速"""
        # 麦克纳姆轮运动学矩阵
        L = self.R * 2  # 轮距
        H = np.array([
            [1, -1, -(L)],
            [1,  1, -(L)],
            [1,  1,  (L)],
            [1, -1,  (L)],
        ])

        v_body = np.array([
            vx * np.cos(self.theta) + vy * np.sin(self.theta),
            -vx * np.sin(self.theta) + vy * np.cos(self.theta),
            omega
        ])

        return H @ v_body / self.R

    def update(self, wheel_speeds: np.ndarray, dt: float) -> Tuple[float, float, float]:
        """运动更新"""
        L = self.R * 2
        H_inv = np.array([
            [1, 1, 1, 1],
            [-1, 1, 1, -1],
            [-1/L, -1/L, 1/L, 1/L],
        ]) / 4

        v_body = H_inv @ wheel_speeds * self.R

        # 转换到世界坐标
        vx = v_body[0] * np.cos(self.theta) - v_body[1] * np.sin(self.theta)
        vy = v_body[0] * np.sin(self.theta) + v_body[1] * np.cos(self.theta)
        omega = v_body[2]

        self.x += vx * dt
        self.y += vy * dt
        self.theta += omega * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.trajectory.append((self.x, self.y, self.theta))
        return self.x, self.y, self.theta


# ============================================================
# 8. 综合仿真管线
# ============================================================

class RoboticsSimulation:
    """机器人运动学综合仿真"""

    def __init__(self):
        self.results = {}

    def demo_2dof_arm(self) -> Dict:
        """2自由度机械臂演示"""
        print("[1] 2-DOF平面臂仿真")

        arm = RobotArm.create_2dof()
        ik = InverseKinematics(arm)

        # 正运动学
        q_test = np.array([np.pi/4, np.pi/6])
        T = arm.forward_kinematics(q_test)
        pos = T[:3, 3]
        print(f"  正运动学 q={np.degrees(q_test)}° -> 位置: {pos[:2]}")

        # 逆运动学
        target = np.array([0.5, 0.3, 0])
        solutions = ik.solve_2dof_analytical(target[0], target[1])
        print(f"  解析IK 目标{target[:2]}:")
        for i, sol in enumerate(solutions):
            print(f"    解{i+1}: {np.degrees(sol)}°")

        # 轨迹规划
        planner = TrajectoryPlanner()
        wp = [np.array([0, 0]), np.array([np.pi/3, np.pi/4]), np.array([np.pi/6, np.pi/3])]
        traj_fn, duration = planner.multi_joint_trajectory(wp, segment_duration=2.0)

        t = np.linspace(0, duration, 100)
        q_traj = traj_fn(t)

        # 工作空间
        ws = WorkspaceAnalyzer(arm)
        ws_points = ws.sample_workspace(5000)

        # 避障
        oa = ObstacleAvoidance()
        start = np.array([0.5, 0.3, 0])
        goal = np.array([-0.3, 0.5, 0])
        obstacles = [(np.array([0.2, 0.4, 0]), 0.1)]
        path, success = oa.plan_path(start, goal, obstacles)

        result = {
            "fk_position": pos,
            "ik_solutions": solutions,
            "trajectory": q_traj,
            "workspace": ws_points,
            "obstacle_path": path,
            "obstacle_avoidance_success": success,
        }

        print(f"  轨迹规划: {len(wp)}个路点, {duration}s")
        print(f"  工作空间: {len(ws_points)}个采样点")
        print(f"  避障规划: {'成功' if success else '失败'}, {len(path)}步")

        return result

    def demo_3dof_arm(self) -> Dict:
        """3自由度机械臂演示"""
        print("\n[2] 3-DOF机械臂仿真")

        arm = RobotArm.create_3dof()
        ik = InverseKinematics(arm)
        planner = TrajectoryPlanner()

        # 正运动学测试
        q = np.array([0.3, 0.5, -0.2])
        T = arm.forward_kinematics(q)
        target = T[:3, 3]
        print(f"  FK: q={np.degrees(q).round(1)}° -> pos={target.round(3)}")

        # 数值IK
        q0 = np.array([0.1, 0.1, 0.1])
        q_sol, converged = ik.solve_numerical(target, q0=q0)
        print(f"  数值IK: {'收敛' if converged else '未收敛'}, q={np.degrees(q_sol).round(1)}°")

        # 多点轨迹
        waypoints = [
            np.array([0, 0, 0]),
            np.array([0.5, 0.3, -0.2]),
            np.array([0.3, 0.6, 0.1]),
            np.array([0, 0, 0]),
        ]
        traj_fn, duration = planner.multi_joint_trajectory(waypoints, method="quintic")
        t = np.linspace(0, duration, 200)
        q_traj = traj_fn(t)

        # 计算末端轨迹
        end_effector_traj = []
        for qi in q_traj:
            pos = arm.get_end_effector_position(qi)
            end_effector_traj.append(pos)

        result = {
            "fk_position": target,
            "ik_solution": q_sol,
            "ik_converged": converged,
            "joint_trajectory": q_traj,
            "end_effector_trajectory": np.array(end_effector_traj),
        }

        print(f"  轨迹: {len(waypoints)}路点, {duration}s")
        return result

    def demo_6dof_arm(self) -> Dict:
        """6自由度机械臂演示"""
        print("\n[3] 6-DOF机械臂仿真")

        arm = RobotArm.create_6dof()
        ik = InverseKinematics(arm)

        # 正运动学
        q = np.array([0, np.pi/4, -np.pi/4, 0, np.pi/6, 0])
        T = arm.forward_kinematics(q)
        pos = T[:3, 3]
        rot = T[:3, :3]
        print(f"  FK: 位置={pos.round(3)}")

        # 位置+姿态IK
        q_sol, converged = ik.solve_numerical(pos, target_rot=rot,
                                               q0=np.ones(6)*0.1, max_iter=200)
        pos_check = arm.get_end_effector_position(q_sol)
        error = np.linalg.norm(pos - pos_check)
        print(f"  IK: {'收敛' if converged else '未收敛'}, 位置误差={error:.6f}")

        # 雅可比矩阵
        J = ik.jacobian_numerical(q)
        print(f"  雅可比矩阵条件数: {np.linalg.cond(J):.1f}")

        result = {
            "fk_position": pos,
            "fk_rotation": rot,
            "ik_solution": q_sol,
            "ik_converged": converged,
            "position_error": error,
            "jacobian": J,
            "condition_number": np.linalg.cond(J),
        }

        return result

    def demo_differential_drive(self) -> Dict:
        """差速机器人演示"""
        print("\n[4] 差速机器人仿真")

        robot = DifferentialDriveRobot()

        # 点到点导航
        path = robot.go_to_point(2.0, 1.5, v_max=0.5)
        print(f"  导航: (0,0)->(2,1.5), {len(path)}步")
        print(f"  最终位姿: ({robot.x:.3f}, {robot.y:.3f}, {np.degrees(robot.theta):.1f}°)")

        # 圆弧运动
        robot2 = DifferentialDriveRobot()
        circle_path = []
        for t in np.arange(0, 10, 0.01):
            vl, vr = robot2.inverse_kinematics(v=0.3, omega=0.5)
            pos = robot2.update(vl, vr, 0.01)
            circle_path.append(pos)

        result = {
            "point_to_point": {"path": path, "final_pose": (robot.x, robot.y, robot.theta)},
            "circle_motion": {"path": circle_path, "n_steps": len(circle_path)},
            "trajectory": robot.trajectory,
        }

        return result

    def demo_omnidirectional(self) -> Dict:
        """全向机器人演示"""
        print("\n[5] 全向移动机器人仿真")

        robot = OmnidirectionalRobot()

        # 横向移动
        path = []
        for t in np.arange(0, 5, 0.01):
            ws = robot.inverse_kinematics(vx=0.3, vy=0.2, omega=0.1)
            pos = robot.update(ws, 0.01)
            path.append(pos)

        print(f"  全向移动: {len(path)}步")
        print(f"  最终位姿: ({robot.x:.3f}, {robot.y:.3f}, {np.degrees(robot.theta):.1f}°)")

        return {"path": path, "trajectory": robot.trajectory}

    def run_all(self) -> Dict:
        """运行所有仿真"""
        print("=" * 60)
        print("  机器人运动学仿真")
        print("=" * 60)

        results = {
            "2dof": self.demo_2dof_arm(),
            "3dof": self.demo_3dof_arm(),
            "6dof": self.demo_6dof_arm(),
            "differential": self.demo_differential_drive(),
            "omnidirectional": self.demo_omnidirectional(),
        }

        print("\n" + "=" * 60)
        print("  仿真完成!")
        print("=" * 60)

        return results


def plot_results(results: Dict, save_path: Optional[str] = None):
    """绘制仿真结果"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib未安装, 跳过绘图")
        return

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("机器人运动学仿真结果", fontsize=14)

    # 2DOF工作空间
    ax1 = fig.add_subplot(2, 3, 1)
    if "2dof" in results:
        ws = results["2dof"]["workspace"]
        ax1.scatter(ws[:, 0], ws[:, 1], s=1, alpha=0.3)
        ax1.set_title("2-DOF工作空间")
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_aspect("equal")

    # 3DOF末端轨迹
    ax2 = fig.add_subplot(2, 3, 2)
    if "3dof" in results:
        ee = results["3dof"]["end_effector_trajectory"]
        ax2.plot(ee[:, 0], ee[:, 1], 'b-', linewidth=1)
        ax2.plot(ee[0, 0], ee[0, 1], 'go', markersize=8, label="起点")
        ax2.plot(ee[-1, 0], ee[-1, 1], 'ro', markersize=8, label="终点")
        ax2.set_title("3-DOF末端轨迹")
        ax2.legend()
        ax2.set_aspect("equal")

    # 6DOF关节轨迹
    ax3 = fig.add_subplot(2, 3, 3)
    if "6dof" in results:
        J = results["6dof"]["jacobian"]
        im = ax3.imshow(np.abs(J), cmap="hot", aspect="auto")
        ax3.set_title(f"雅可比矩阵 (cond={results['6dof']['condition_number']:.0f})")
        plt.colorbar(im, ax=ax3)

    # 差速机器人轨迹
    ax4 = fig.add_subplot(2, 3, 4)
    if "differential" in results:
        traj = results["differential"]["trajectory"]
        xs = [p[0] for p in traj]
        ys = [p[1] for p in traj]
        ax4.plot(xs, ys, 'b-')
        ax4.plot(xs[0], ys[0], 'go', label="起点")
        ax4.plot(xs[-1], ys[-1], 'ro', label="终点")
        ax4.set_title("差速机器人轨迹")
        ax4.legend()
        ax4.set_aspect("equal")

    # 避障规划
    ax5 = fig.add_subplot(2, 3, 5)
    if "2dof" in results and "obstacle_path" in results["2dof"]:
        path = results["2dof"]["obstacle_path"]
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax5.plot(xs, ys, 'b-', linewidth=2)
        # 绘制障碍物
        from matplotlib.patches import Circle
        ax5.add_patch(Circle((0.2, 0.4), 0.1, color='red', alpha=0.5))
        ax5.set_title("避障路径规划")
        ax5.set_aspect("equal")

    # 全向机器人轨迹
    ax6 = fig.add_subplot(2, 3, 6)
    if "omnidirectional" in results:
        traj = results["omnidirectional"]["trajectory"]
        xs = [p[0] for p in traj]
        ys = [p[1] for p in traj]
        ax6.plot(xs, ys, 'g-')
        ax6.set_title("全向机器人轨迹")
        ax6.set_aspect("equal")

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[OK] 图像已保存: {save_path}")
    plt.show()


# ============================================================
# 主程序
# ============================================================

def main():
    sim = RoboticsSimulation()
    results = sim.run_all()

    try:
        plot_results(results)
    except Exception as e:
        print(f"[INFO] 绘图跳过: {e}")

    return results


if __name__ == "__main__":
    main()
