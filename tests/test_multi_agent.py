#!/usr/bin/env python3
"""
多智能体仿真单元测试
覆盖: Vehicle模型、运动学更新、距离/角度计算、跟随控制器、编队控制器
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from _15_simulation.multi_agent_simulation import Vehicle, follow_controller, formation_controller


class TestVehicleInit(unittest.TestCase):
    def test_basic(self):
        v = Vehicle(10, 20, 0.5)
        self.assertEqual(v.x, 10.0)
        self.assertEqual(v.y, 20.0)
        self.assertEqual(v.theta, 0.5)
        self.assertEqual(v.v, 0.0)
        self.assertEqual(v.omega, 0.0)

    def test_color(self):
        v = Vehicle(0, 0, color="#ff0000")
        self.assertEqual(v.color, "#ff0000")

    def test_name(self):
        v = Vehicle(0, 0, name="test_car")
        self.assertEqual(v.name, "test_car")


class TestVehicleSetControl(unittest.TestCase):
    def test_clamp_velocity(self):
        v = Vehicle(0, 0)
        v.set_control(200, 0)
        self.assertEqual(v.target_v, v.max_v)

    def test_clamp_negative(self):
        v = Vehicle(0, 0)
        v.set_control(-200, 0)
        self.assertEqual(v.target_v, -v.max_v)

    def test_clamp_omega(self):
        v = Vehicle(0, 0)
        v.set_control(0, 5.0)
        self.assertEqual(v.target_omega, v.max_omega)


class TestVehicleUpdate(unittest.TestCase):
    def test_stationary(self):
        v = Vehicle(100, 100, 0)
        v.set_control(0, 0)
        v.update(0.1)
        # With smoothing, should stay near start
        self.assertAlmostEqual(v.x, 100.0, delta=1.0)
        self.assertAlmostEqual(v.y, 100.0, delta=1.0)

    def test_forward_motion(self):
        v = Vehicle(0, 0, 0)
        v.v = 50.0
        v.omega = 0.0
        v.target_v = 50.0
        v.target_omega = 0.0
        v.update(1.0)
        self.assertGreater(v.x, 0)

    def test_trail_recorded(self):
        v = Vehicle(0, 0, 0)
        v.target_v = 10
        v.target_omega = 0
        for _ in range(5):
            v.update(0.1)
        self.assertGreater(len(v.trail), 0)


class TestVehicleDistanceTo(unittest.TestCase):
    def test_same_position(self):
        v1 = Vehicle(0, 0)
        v2 = Vehicle(0, 0)
        self.assertAlmostEqual(v1.distance_to(v2), 0.0)

    def test_known_distance(self):
        v1 = Vehicle(0, 0)
        v2 = Vehicle(3, 4)
        self.assertAlmostEqual(v1.distance_to(v2), 5.0, places=3)

    def test_symmetry(self):
        v1 = Vehicle(0, 0)
        v2 = Vehicle(10, 20)
        self.assertAlmostEqual(v1.distance_to(v2), v2.distance_to(v1))


class TestVehicleAngleTo(unittest.TestCase):
    def test_right(self):
        v1 = Vehicle(0, 0)
        v2 = Vehicle(10, 0)
        angle = v1.angle_to(v2)
        self.assertAlmostEqual(angle, 0.0, places=3)

    def test_up(self):
        v1 = Vehicle(0, 0)
        v2 = Vehicle(0, 10)
        angle = v1.angle_to(v2)
        self.assertAlmostEqual(angle, math.pi / 2, places=3)


class TestFollowController(unittest.TestCase):
    def test_already_at_distance(self):
        leader = Vehicle(100, 0)
        follower = Vehicle(30, 0)
        follower.theta = 0
        v, omega = follow_controller(follower, leader, desired_dist=70)
        self.assertAlmostEqual(v, 0.0, delta=5)

    def test_too_far(self):
        leader = Vehicle(200, 0)
        follower = Vehicle(0, 0)
        follower.theta = 0
        v, omega = follow_controller(follower, leader, desired_dist=60)
        self.assertGreater(v, 0)

    def test_too_close(self):
        leader = Vehicle(30, 0)
        follower = Vehicle(0, 0)
        follower.theta = 0
        v, omega = follow_controller(follower, leader, desired_dist=60)
        self.assertLess(v, 0)


class TestFormationController(unittest.TestCase):
    def test_leader_returns_forward(self):
        vehicles = [Vehicle(0, 0), Vehicle(-70, -50)]
        offsets = [(0, 0), (-70, -50)]
        v, omega = formation_controller(vehicles[0], vehicles, offsets, leader_idx=0)
        self.assertEqual(v, 40)
        self.assertEqual(omega, 0)

    def test_follower_has_control(self):
        vehicles = [Vehicle(0, 0), Vehicle(-100, -100)]
        offsets = [(0, 0), (-70, -50)]
        v, omega = formation_controller(vehicles[1], vehicles, offsets, leader_idx=0)
        self.assertIsInstance(v, (int, float))
        self.assertIsInstance(omega, (int, float))


if __name__ == '__main__':
    unittest.main()
