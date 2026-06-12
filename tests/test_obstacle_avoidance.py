#!/usr/bin/env python3
"""
避障算法仿真单元测试
覆盖: Obstacle、Environment、APFPlanner(引力/斥力/规划)、VFHPlanner(直方图/候选方向/规划)
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from _15_simulation.obstacle_avoidance_simulation import Obstacle, Environment, APFPlanner, VFHPlanner


class TestObstacle(unittest.TestCase):
    def test_init(self):
        o = Obstacle(100, 200, 25)
        self.assertEqual(o.x, 100)
        self.assertEqual(o.y, 200)
        self.assertEqual(o.radius, 25)


class TestEnvironmentInit(unittest.TestCase):
    def test_default(self):
        env = Environment()
        self.assertEqual(env.width, 900)
        self.assertEqual(env.height, 600)
        self.assertEqual(len(env.obstacles), 0)

    def test_custom(self):
        env = Environment(width=500, height=400)
        self.assertEqual(env.width, 500)
        self.assertEqual(env.height, 400)


class TestEnvironmentAddObstacle(unittest.TestCase):
    def test_add(self):
        env = Environment()
        env.add_obstacle(100, 200, 30)
        self.assertEqual(len(env.obstacles), 1)
        self.assertEqual(env.obstacles[0].x, 100)


class TestEnvironmentGenerateRandom(unittest.TestCase):
    def test_generates(self):
        env = Environment()
        env.generate_random(10)
        self.assertGreater(len(env.obstacles), 0)
        self.assertLessEqual(len(env.obstacles), 10)


class TestEnvironmentIsCollision(unittest.TestCase):
    def test_inside_obstacle(self):
        env = Environment()
        env.add_obstacle(100, 100, 30)
        self.assertTrue(env.is_collision(100, 100))

    def test_far_away(self):
        env = Environment()
        env.add_obstacle(100, 100, 10)
        self.assertFalse(env.is_collision(500, 500))

    def test_safe_radius(self):
        """safe_r应扩大碰撞范围"""
        env = Environment()
        env.add_obstacle(100, 100, 10)
        # Just outside obstacle but within safe_r
        self.assertTrue(env.is_collision(100, 115, safe_r=10))


class TestEnvironmentNearestObstacleDist(unittest.TestCase):
    def test_no_obstacles(self):
        env = Environment()
        d = env.nearest_obstacle_dist(100, 100)
        self.assertEqual(d, float('inf'))

    def test_known_distance(self):
        env = Environment()
        env.add_obstacle(100, 100, 20)
        d = env.nearest_obstacle_dist(150, 100)
        self.assertAlmostEqual(d, 30.0, places=1)

    def test_inside_obstacle(self):
        env = Environment()
        env.add_obstacle(100, 100, 30)
        d = env.nearest_obstacle_dist(100, 100)
        self.assertEqual(d, 0)  # clamped to 0


class TestAPFPlannerInit(unittest.TestCase):
    def test_defaults(self):
        env = Environment()
        apf = APFPlanner(env)
        self.assertEqual(apf.k_att, 1.0)
        self.assertEqual(apf.k_rep, 500.0)
        self.assertEqual(apf.d0, 80.0)
        self.assertEqual(apf.step, 5.0)


class TestAPFAttractive(unittest.TestCase):
    def test_toward_goal(self):
        env = Environment()
        apf = APFPlanner(env)
        fx, fy = apf.attractive((0, 0), (100, 0))
        self.assertGreater(fx, 0)
        self.assertAlmostEqual(fy, 0, places=1)

    def test_at_goal(self):
        env = Environment()
        apf = APFPlanner(env)
        fx, fy = apf.attractive((100, 100), (100, 100))
        self.assertEqual(fx, 0)
        self.assertEqual(fy, 0)

    def test_direction(self):
        """引力应指向目标"""
        env = Environment()
        apf = APFPlanner(env)
        fx, fy = apf.attractive((0, 0), (0, 100))
        self.assertAlmostEqual(fx, 0, places=1)
        self.assertGreater(fy, 0)


class TestAPFRepulsive(unittest.TestCase):
    def test_no_obstacles(self):
        env = Environment()
        apf = APFPlanner(env)
        fx, fy = apf.repulsive((100, 100))
        self.assertEqual(fx, 0)
        self.assertEqual(fy, 0)

    def test_near_obstacle(self):
        """靠近障碍物应有斥力"""
        env = Environment()
        env.add_obstacle(100, 100, 20)
        apf = APFPlanner(env, d0=100)
        fx, fy = apf.repulsive((50, 100))
        # Should push away from obstacle (negative x direction)
        self.assertLess(fx, 0)

    def test_far_from_obstacle(self):
        """远离障碍物应无斥力"""
        env = Environment()
        env.add_obstacle(100, 100, 20)
        apf = APFPlanner(env, d0=50)
        fx, fy = apf.repulsive((500, 500))
        self.assertAlmostEqual(fx, 0, places=5)
        self.assertAlmostEqual(fy, 0, places=5)


class TestAPFPlan(unittest.TestCase):
    def test_reaches_goal_no_obstacles(self):
        env = Environment(width=500, height=500)
        env.start = (50, 250)
        env.goal = (450, 250)
        apf = APFPlanner(env)
        path = apf.plan(env.start, env.goal)
        self.assertGreater(len(path), 1)
        # End should be near goal
        end = path[-1]
        dist = math.sqrt((end[0] - env.goal[0])**2 + (end[1] - env.goal[1])**2)
        self.assertLess(dist, 30)

    def test_path_is_list(self):
        env = Environment(width=500, height=500)
        apf = APFPlanner(env)
        path = apf.plan((50, 250), (450, 250))
        self.assertIsInstance(path, list)
        self.assertGreater(len(path), 0)

    def test_path_starts_at_start(self):
        env = Environment(width=500, height=500)
        apf = APFPlanner(env)
        path = apf.plan((50, 250), (450, 250))
        self.assertAlmostEqual(path[0][0], 50, delta=1)
        self.assertAlmostEqual(path[0][1], 250, delta=1)


class TestVFHPlannerInit(unittest.TestCase):
    def test_defaults(self):
        env = Environment()
        vfh = VFHPlanner(env)
        self.assertEqual(vfh.num_sectors, 72)
        self.assertEqual(vfh.step, 6.0)


class TestVFHBuildHistogram(unittest.TestCase):
    def test_no_obstacles(self):
        env = Environment()
        vfh = VFHPlanner(env)
        hist = vfh.build_histogram((100, 100))
        self.assertEqual(len(hist), 72)
        self.assertTrue(all(h == 0.0 for h in hist))

    def test_near_obstacle(self):
        env = Environment()
        env.add_obstacle(150, 100, 20)
        vfh = VFHPlanner(env, safe_dist=200)
        hist = vfh.build_histogram((100, 100))
        # Should have non-zero values
        self.assertTrue(any(h > 0 for h in hist))


class TestVFHFindCandidates(unittest.TestCase):
    def test_all_open(self):
        env = Environment()
        vfh = VFHPlanner(env)
        hist = [0.0] * 72  # All clear
        candidates = vfh.find_candidates(hist)
        # Should find open directions
        self.assertIsInstance(candidates, list)

    def test_all_blocked(self):
        env = Environment()
        vfh = VFHPlanner(env)
        hist = [1.0] * 72  # All blocked
        candidates = vfh.find_candidates(hist)
        self.assertEqual(len(candidates), 0)


class TestVFHAngleDiff(unittest.TestCase):
    def test_same_angle(self):
        env = Environment()
        vfh = VFHPlanner(env)
        self.assertAlmostEqual(vfh._angle_diff(1.0, 1.0), 0.0, places=5)

    def test_wrap_positive(self):
        env = Environment()
        vfh = VFHPlanner(env)
        d = vfh._angle_diff(0.1, 6.0)
        self.assertGreater(d, 0)
        self.assertLess(d, math.pi)

    def test_wrap_negative(self):
        env = Environment()
        vfh = VFHPlanner(env)
        d = vfh._angle_diff(6.0, 0.1)
        self.assertLess(d, 0)
        self.assertGreater(d, -math.pi)


class TestVFHPlan(unittest.TestCase):
    def test_reaches_goal(self):
        env = Environment(width=500, height=500)
        env.start = (50, 250)
        env.goal = (450, 250)
        vfh = VFHPlanner(env)
        path = vfh.plan(env.start, env.goal)
        self.assertGreater(len(path), 1)
        end = path[-1]
        dist = math.sqrt((end[0] - env.goal[0])**2 + (end[1] - env.goal[1])**2)
        self.assertLess(dist, 20)

    def test_path_is_list(self):
        env = Environment(width=500, height=500)
        vfh = VFHPlanner(env)
        path = vfh.plan((50, 250), (450, 250))
        self.assertIsInstance(path, list)
        self.assertGreater(len(path), 0)


if __name__ == '__main__':
    unittest.main()
