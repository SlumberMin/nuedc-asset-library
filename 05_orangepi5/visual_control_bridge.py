"""
视觉-控制融合桥接
将视觉识别结果转换为控制指令，实现视觉跟踪/寻迹/避障等功能
适用于 Orange Pi 5 (RK3588S) 上视觉系统与控制系统的协同
"""
import time
import math
import logging
from typing import Optional, Tuple, List
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class VisualTarget:
    """视觉目标"""
    class_id: int = -1          # 类别 ID
    class_name: str = ""        # 类别名称
    confidence: float = 0.0     # 置信度
    cx: float = 0.0             # 中心 x (像素)
    cy: float = 0.0             # 中心 y (像素)
    width: float = 0.0          # 宽度 (像素)
    height: float = 0.0         # 高度 (像素)
    depth: float = -1.0         # 深度 (米, -1 表示未知)
    angle: float = 0.0          # 朝向角度 (度)
    timestamp: float = 0.0      # 时间戳


@dataclass
class ControlCommand:
    """控制指令"""
    linear_x: float = 0.0       # 前进速度 (-100 ~ 100)
    linear_y: float = 0.0       # 横移速度 (-100 ~ 100)
    angular_z: float = 0.0      # 转向速度 (-100 ~ 100)
    servo_pan: float = 90.0     # 云台水平角度
    servo_tilt: float = 90.0    # 云台俯仰角度
    action: str = ""            # 动作指令 (如 "stop", "grasp")
    timestamp: float = 0.0


class VisualControlBridge:
    """
    视觉-控制融合桥接

    Parameters
    ----------
    image_width : int
        图像宽度 (像素)
    image_height : int
        图像高度 (像素)
    fov_h : float
        水平视场角 (度)
    fov_v : float
        垂直视场角 (度)
    """

    def __init__(
        self,
        image_width: int = 640,
        image_height: int = 480,
        fov_h: float = 60.0,
        fov_v: float = 45.0,
    ):
        self.image_width = image_width
        self.image_height = image_height
        self.fov_h = fov_h
        self.fov_v = fov_v

        self._target: Optional[VisualTarget] = None
        self._prev_targets: List[VisualTarget] = []

    def update_target(self, target: VisualTarget):
        """更新视觉目标"""
        if self._prev_targets:
            self._prev_targets.append(self._target)
            if len(self._prev_targets) > 10:
                self._prev_targets.pop(0)
        self._target = target

    @property
    def target(self) -> Optional[VisualTarget]:
        return self._target

    def _pixel_to_angle(self, px: float, py: float) -> Tuple[float, float]:
        """像素坐标 → 角度偏移"""
        # 图像中心为 (image_width/2, image_height/2)
        dx = px - self.image_width / 2
        dy = py - self.image_height / 2

        angle_h = dx / (self.image_width / 2) * (self.fov_h / 2)
        angle_v = dy / (self.image_height / 2) * (self.fov_v / 2)

        return angle_h, angle_v

    def track_target(self, target: VisualTarget = None) -> ControlCommand:
        """
        视觉跟踪模式

        通过 PID 控制使目标保持在画面中心。

        Parameters
        ----------
        target : VisualTarget
            跟踪目标 (None 则使用最近更新的目标)

        Returns
        -------
        ControlCommand
        """
        if target is None:
            target = self._target
        if target is None:
            return ControlCommand(action="no_target")

        cmd = ControlCommand(timestamp=time.monotonic())

        # 计算角度偏差
        angle_h, angle_v = self._pixel_to_angle(target.cx, target.cy)

        # 转向控制: 用角度偏差做 PD 控制
        cmd.angular_z = self._clamp(-angle_h * 0.8, -80, 80)

        # 前进控制: 根据目标大小(距离)调整
        target_area = target.width * target.height
        image_area = self.image_width * self.image_height
        area_ratio = target_area / image_area

        if area_ratio < 0.05:
            cmd.linear_x = 50  # 太远，前进
        elif area_ratio > 0.4:
            cmd.linear_x = -30  # 太近，后退
        else:
            cmd.linear_x = 20  # 适中距离，缓慢前进

        # 云台控制
        cmd.servo_pan = 90 - angle_h
        cmd.servo_tilt = 90 - angle_v

        return cmd

    def follow_line(self, target: VisualTarget = None) -> ControlCommand:
        """
        视觉循线模式

        适用于智能小车循线，目标为检测到的线条。

        Parameters
        ----------
        target : VisualTarget

        Returns
        -------
        ControlCommand
        """
        if target is None:
            target = self._target
        if target is None:
            return ControlCommand(action="line_lost")

        cmd = ControlCommand(timestamp=time.monotonic())

        # 横向偏差 → 转向
        dx = target.cx - self.image_width / 2
        # 归一化到 -1 ~ 1
        norm_dx = dx / (self.image_width / 2)

        cmd.angular_z = self._clamp(-norm_dx * 60, -80, 80)

        # 纵向偏差 → 前进速度
        dy = target.cy - self.image_height / 2
        norm_dy = dy / (self.image_height / 2)

        # 目标在下方 → 线在近处，减速
        if norm_dy > 0.5:
            cmd.linear_x = 20
        else:
            cmd.linear_x = 50

        # 角度偏差 → 转向修正
        if abs(target.angle) > 5:
            cmd.angular_z += self._clamp(-target.angle * 0.5, -30, 30)

        return cmd

    def avoid_obstacle(self, targets: List[VisualTarget] = None) -> ControlCommand:
        """
        视觉避障模式

        Parameters
        ----------
        targets : list of VisualTarget
            检测到的障碍物列表

        Returns
        -------
        ControlCommand
        """
        if targets is None:
            targets = [self._target] if self._target else []

        cmd = ControlCommand(timestamp=time.monotonic())

        if not targets:
            cmd.linear_x = 50  # 无障碍，前进
            return cmd

        # 找最近的障碍物 (面积最大的)
        nearest = max(targets, key=lambda t: t.width * t.height)
        angle_h, _ = self._pixel_to_angle(nearest.cx, nearest.cy)

        # 障碍物在前方 → 转向避开
        if abs(angle_h) < 30:
            area_ratio = (nearest.width * nearest.height) / (self.image_width * self.image_height)
            if area_ratio > 0.3:
                cmd.linear_x = -30  # 后退
                cmd.angular_z = 60 if angle_h >= 0 else -60  # 向左/右转
            else:
                cmd.linear_x = 20
                cmd.angular_z = 40 if angle_h >= 0 else -40
        else:
            cmd.linear_x = 40  # 障碍物在侧面，继续前进

        return cmd

    def grasp_target(self, target: VisualTarget = None) -> ControlCommand:
        """
        抓取模式

        计算到达目标所需的运动控制指令。

        Parameters
        ----------
        target : VisualTarget

        Returns
        -------
        ControlCommand
        """
        if target is None:
            target = self._target
        if target is None:
            return ControlCommand(action="no_target")

        cmd = ControlCommand(timestamp=time.monotonic())
        angle_h, angle_v = self._pixel_to_angle(target.cx, target.cy)

        # 位置对齐
        cmd.angular_z = self._clamp(-angle_h * 0.5, -50, 50)
        cmd.linear_y = self._clamp(-angle_v * 0.3, -30, 30)  # 横移

        # 距离判断
        if target.depth > 0:
            if target.depth > 0.3:
                cmd.linear_x = 30
                cmd.action = "approach"
            else:
                cmd.linear_x = 0
                cmd.action = "grasp"
        else:
            # 无深度信息，用目标大小判断
            area_ratio = (target.width * target.height) / (self.image_width * self.image_height)
            if area_ratio < 0.2:
                cmd.linear_x = 30
                cmd.action = "approach"
            else:
                cmd.linear_x = 0
                cmd.action = "grasp"

        # 云台对准
        cmd.servo_pan = 90 - angle_h
        cmd.servo_tilt = 90 - angle_v

        return cmd

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    def __repr__(self):
        return (f"VisualControlBridge({self.image_width}x{self.image_height}, "
                f"fov={self.fov_h}°x{self.fov_v}°)")
