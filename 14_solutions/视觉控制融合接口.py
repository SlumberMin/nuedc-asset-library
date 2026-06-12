"""
视觉控制融合接口 (Vision-Control Fusion Interface)
==================================================
功能：视觉结果 → 控制指令的转换接口
平台：Orange Pi 5 (RK3588S)
用途：电赛系统架构 - 视觉与控制进程间的数据融合

架构：
  视觉进程 → SharedMemory → 本模块 → 控制指令 → 控制进程
"""

import os
import time
import struct
import mmap
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable, Tuple
from enum import IntEnum
from collections import deque


# ============================================================
# 数据结构定义
# ============================================================

class TargetType(IntEnum):
    """目标类型"""
    NONE = 0
    CROSS = 1       # 十字标志
    CIRCLE = 2      # 圆形目标
    ARROW = 3       # 箭头
    NUMBER = 4      # 数字
    QR_CODE = 5     # 二维码
    CUSTOM = 99     # 自定义


class ControlMode(IntEnum):
    """控制模式"""
    STOP = 0
    POSITION = 1    # 位置控制
    SPEED = 2       # 速度控制
    TRACK = 3       # 跟踪模式
    PATH = 4        # 路径跟踪


@dataclass
class VisionResult:
    """视觉检测结果"""
    timestamp_ns: int = 0
    frame_id: int = 0
    target_x: float = 0.0       # mm
    target_y: float = 0.0       # mm
    target_z: float = 0.0       # mm
    confidence: float = 0.0     # 0~1
    target_type: TargetType = TargetType.NONE
    valid: bool = False
    # 扩展字段
    angle: float = 0.0          # 目标角度 (度)
    area: float = 0.0           # 目标面积


@dataclass
class ControlCommand:
    """控制指令"""
    timestamp_ns: int = 0
    mode: ControlMode = ControlMode.STOP
    target_x: float = 0.0       # mm
    target_y: float = 0.0       # mm
    target_angle: float = 0.0   # 度
    speed_x: float = 0.0        # mm/s
    speed_y: float = 0.0        # mm/s
    speed_z: float = 0.0        # 度/s
    sequence_id: int = 0


@dataclass
class SystemStatus:
    """系统状态"""
    timestamp_ns: int = 0
    phase: int = 0              # 比赛阶段
    motor_pos: Tuple[int, ...] = (0, 0, 0, 0)
    motor_vel: Tuple[int, ...] = (0, 0, 0, 0)
    battery_voltage: float = 0.0
    error_code: int = 0
    system_ready: bool = False
    control_loop_us: int = 0
    vision_fps: float = 0.0


# ============================================================
# 共享内存IPC层
# ============================================================

class SharedMemoryChannel:
    """POSIX共享内存通道"""

    SHM_SIZE = 4096

    def __init__(self, name: str, create: bool = True):
        flags = os.O_CREAT if create else 0
        try:
            import posix_ipc
            self.shm = posix_ipc.SharedMemory(
                f'/{name}', flags, size=self.SHM_SIZE)
            self.mm = mmap.mmap(self.shm.fd, self.SHM_SIZE)
            self._use_posix = True
        except ImportError:
            # Fallback: 使用文件映射
            self._shm_path = f'/dev/shm/{name}'
            if create:
                with open(self._shm_path, 'wb') as f:
                    f.write(b'\x00' * self.SHM_SIZE)
            self._fd = os.open(self._shm_path, os.O_RDWR)
            self.mm = mmap.mmap(self._fd, self.SHM_SIZE)
            self._use_posix = False

    def write(self, data: bytes, offset: int = 0):
        self.mm.seek(offset)
        self.mm.write(data)

    def read(self, size: int, offset: int = 0) -> bytes:
        self.mm.seek(offset)
        return self.mm.read(size)

    def close(self):
        self.mm.close()
        if hasattr(self, 'shm'):
            pass  # posix_ipc handles cleanup separately


class SemaphoreWrapper:
    """信号量封装 (兼容posix_ipc / threading)"""

    def __init__(self, name: str, initial: int = 0):
        try:
            import posix_ipc
            self._sem = posix_ipc.Semaphore(
                f'/{name}', os.O_CREAT, initial_value=initial)
            self._use_posix = True
        except ImportError:
            # Fallback: 使用threading
            self._sem = threading.Semaphore(initial)
            self._use_posix = False

    def acquire(self, timeout: float = 1.0):
        if self._use_posix:
            self._sem.acquire(timeout)
        else:
            self._sem.acquire(timeout=timeout)

    def release(self):
        self._sem.release()

    def unlink(self):
        if self._use_posix:
            self._sem.unlink()


# ============================================================
# 坐标变换
# ============================================================

class CoordinateTransformer:
    """
    像素坐标 → 物理坐标的变换器

    支持：
    - 单应性矩阵变换 (Homography)
    - 仿射变换
    - 简单线性缩放
    """

    def __init__(self):
        self.homography_matrix = None
        self.scale_x = 1.0    # mm/pixel
        self.scale_y = 1.0    # mm/pixel
        self.offset_x = 0.0   # 偏移 mm
        self.offset_y = 0.0   # 偏移 mm
        self.calibrated = False

    def calibrate_simple(self, camera_res: Tuple[int, int],
                         fov_mm: Tuple[float, float],
                         offset: Tuple[float, float] = (0, 0)):
        """
        简单标定：已知FOV物理尺寸

        Args:
            camera_res: (width, height) 像素分辨率
            fov_mm: (width_mm, height_mm) 视场物理尺寸
            offset: (x, y) 中心偏移 mm
        """
        self.scale_x = fov_mm[0] / camera_res[0]
        self.scale_y = fov_mm[1] / camera_res[1]
        self.offset_x = offset[0]
        self.offset_y = offset[1]
        self.calibrated = True

    def pixel_to_physical(self, px: float, py: float,
                          img_center: Tuple[float, float] = None
                          ) -> Tuple[float, float]:
        """
        像素坐标 → 物理坐标 (相对画面中心)

        Args:
            px, py: 像素坐标
            img_center: 图像中心像素坐标

        Returns:
            (x_mm, y_mm): 物理坐标 mm
        """
        if not self.calibrated:
            raise RuntimeError("坐标变换器未标定，请先调用 calibrate_simple()")

        if img_center is None:
            img_center = (320, 240)  # 默认640x480

        x_mm = (px - img_center[0]) * self.scale_x + self.offset_x
        y_mm = (py - img_center[1]) * self.scale_y + self.offset_y

        return (x_mm, y_mm)

    def set_homography(self, H):
        """设置单应性矩阵 (4点标定)"""
        import numpy as np
        self.homography_matrix = np.array(H, dtype=np.float64)
        self.calibrated = True

    def pixel_to_physical_homography(self, px: float, py: float
                                     ) -> Tuple[float, float]:
        """使用单应性矩阵变换"""
        import numpy as np
        if self.homography_matrix is None:
            raise RuntimeError("单应性矩阵未设置")

        pt = np.array([px, py, 1.0])
        result = self.homography_matrix @ pt
        result /= result[2]
        return (float(result[0]), float(result[1]))


# ============================================================
# 目标滤波与预测
# ============================================================

class TargetFilter:
    """
    目标坐标滤波器
    - 低通滤波 (平滑噪声)
    - 卡尔曼滤波 (运动预测)
    - 野值剔除 (误检过滤)
    """

    def __init__(self, alpha: float = 0.3, max_jump_mm: float = 50.0):
        """
        Args:
            alpha: 低通滤波系数 (0~1, 越小越平滑)
            max_jump_mm: 最大允许跳变 (mm), 超过则认为是野值
        """
        self.alpha = alpha
        self.max_jump = max_jump_mm
        self._prev_x = None
        self._prev_y = None
        self._history = deque(maxlen=10)
        self._vel_x = 0.0
        self._vel_y = 0.0
        self._last_time = None

    def update(self, x: float, y: float, timestamp_ns: int = None
               ) -> Tuple[float, float, bool]:
        """
        更新并滤波目标坐标

        Returns:
            (filtered_x, filtered_y, is_valid)
        """
        # 野值检测
        if self._prev_x is not None:
            dx = x - self._prev_x
            dy = y - self._prev_y
            dist = (dx**2 + dy**2) ** 0.5
            if dist > self.max_jump:
                # 跳变过大，可能是误检，使用预测值
                if timestamp_ns and self._last_time:
                    dt = (timestamp_ns - self._last_time) * 1e-9
                    pred_x = self._prev_x + self._vel_x * dt
                    pred_y = self._prev_y + self._vel_y * dt
                    return (pred_x, pred_y, False)

        # 低通滤波
        if self._prev_x is None:
            fx, fy = x, y
        else:
            fx = self.alpha * x + (1 - self.alpha) * self._prev_x
            fy = self.alpha * y + (1 - self.alpha) * self._prev_y

        # 更新速度估计
        if timestamp_ns and self._last_time:
            dt = (timestamp_ns - self._last_time) * 1e-9
            if dt > 0:
                self._vel_x = (fx - self._prev_x) / dt if self._prev_x else 0
                self._vel_y = (fy - self._prev_y) / dt if self._prev_y else 0

        self._prev_x = fx
        self._prev_y = fy
        self._last_time = timestamp_ns
        self._history.append((fx, fy))

        return (fx, fy, True)

    def predict(self, dt: float) -> Tuple[float, float]:
        """预测dt秒后的位置"""
        if self._prev_x is None:
            return (0, 0)
        return (
            self._prev_x + self._vel_x * dt,
            self._prev_y + self._vel_y * dt
        )

    def reset(self):
        self._prev_x = None
        self._prev_y = None
        self._vel_x = 0.0
        self._vel_y = 0.0
        self._last_time = None


# ============================================================
# 融合接口主类
# ============================================================

class VisionControlFusion:
    """
    视觉-控制融合接口

    功能：
    1. 接收视觉检测结果
    2. 坐标变换 (像素→物理)
    3. 目标滤波与预测
    4. 生成控制指令
    5. 通过共享内存发送给控制进程

    使用方式：
        fusion = VisionControlFusion()
        fusion.set_camera_params((640, 480), (200, 150))
        fusion.start()

        # 视觉进程回调
        fusion.on_vision_result(pixel_x, pixel_y, confidence, target_type)
    """

    def __init__(self, use_shm: bool = True):
        """
        Args:
            use_shm: 是否使用共享内存 (False则纯Python模式)
        """
        self.use_shm = use_shm
        self.transformer = CoordinateTransformer()
        self.filter = TargetFilter(alpha=0.4, max_jump_mm=100)
        self._callbacks = []
        self._running = False
        self._lock = threading.Lock()

        # 共享内存通道
        if use_shm:
            self._shm_vision = SharedMemoryChannel('vision_result', create=True)
            self._shm_status = SharedMemoryChannel('system_status', create=True)
            self._sem_vision = SemaphoreWrapper('sem_vision_ready', initial=0)

        # 统计
        self._frame_count = 0
        self._last_fps_time = time.monotonic()
        self._current_fps = 0.0

    def set_camera_params(self, resolution: Tuple[int, int],
                          fov_mm: Tuple[float, float],
                          offset: Tuple[float, float] = (0, 0)):
        """设置相机参数并标定坐标变换"""
        self.transformer.calibrate_simple(resolution, fov_mm, offset)
        self._resolution = resolution
        self._center = (resolution[0] / 2, resolution[1] / 2)

    def set_homography(self, H):
        """使用单应性矩阵标定"""
        self.transformer.set_homography(H)

    def on_vision_result(self, pixel_x: float, pixel_y: float,
                         confidence: float,
                         target_type: TargetType = TargetType.NONE,
                         angle: float = 0.0,
                         ) -> Optional[ControlCommand]:
        """
        视觉结果回调 → 生成控制指令

        Args:
            pixel_x, pixel_y: 目标像素坐标
            confidence: 检测置信度 (0~1)
            target_type: 目标类型
            angle: 目标角度

        Returns:
            ControlCommand 或 None (如果目标无效)
        """
        with self._lock:
            now = time.monotonic_ns()

            # 置信度门限
            if confidence < 0.5:
                return None

            # 坐标变换
            if self.transformer.homography_matrix is not None:
                phys_x, phys_y = self.transformer.pixel_to_physical_homography(
                    pixel_x, pixel_y)
            else:
                phys_x, phys_y = self.transformer.pixel_to_physical(
                    pixel_x, pixel_y, self._center)

            # 目标滤波
            fx, fy, is_valid = self.filter.update(phys_x, phys_y, now)

            # 构建视觉结果
            result = VisionResult(
                timestamp_ns=now,
                frame_id=self._frame_count,
                target_x=fx,
                target_y=fy,
                confidence=confidence,
                target_type=target_type,
                valid=is_valid,
                angle=angle,
            )

            # 写入共享内存
            if self.use_shm:
                self._write_vision_shm(result)

            # 生成控制指令
            cmd = self._generate_command(result)

            # 回调通知
            for cb in self._callbacks:
                try:
                    cb(cmd)
                except Exception as e:
                    print(f"回调异常: {e}")

            # FPS统计
            self._frame_count += 1
            if now - self._last_fps_time > 1e9:
                self._current_fps = self._frame_count / (
                    (now - self._last_fps_time) * 1e-9)
                self._frame_count = 0
                self._last_fps_time = now

            return cmd

    def _generate_command(self, result: VisionResult) -> ControlCommand:
        """根据视觉结果生成控制指令"""
        cmd = ControlCommand(timestamp_ns=result.timestamp_ns)

        if not result.valid or result.confidence < 0.3:
            cmd.mode = ControlMode.STOP
            return cmd

        # 位置控制模式：目标坐标即指令
        cmd.mode = ControlMode.POSITION
        cmd.target_x = result.target_x
        cmd.target_y = result.target_y
        cmd.target_angle = result.angle

        # 可选：前馈补偿 (预测目标移动)
        pred_x, pred_y = self.filter.predict(0.05)  # 50ms预测
        cmd.speed_x = (pred_x - result.target_x) * 20  # 比例前馈
        cmd.speed_y = (pred_y - result.target_y) * 20

        return cmd

    def _write_vision_shm(self, result: VisionResult):
        """写入视觉结果到共享内存"""
        # 打包为C结构体格式 (32字节)
        data = struct.pack(
            '=QIiii fBB2x',
            result.timestamp_ns,
            result.frame_id,
            int(result.target_x * 10),   # 0.1mm精度
            int(result.target_y * 10),
            int(result.target_z * 10),
            result.confidence,
            int(result.target_type),
            1 if result.valid else 0,
        )
        self._shm_vision.write(data)
        self._sem_vision.release()

    def register_callback(self, callback: Callable[[ControlCommand], None]):
        """注册控制指令回调"""
        self._callbacks.append(callback)

    def get_fps(self) -> float:
        """获取当前处理帧率"""
        return self._current_fps

    def get_last_command(self) -> Optional[ControlCommand]:
        """获取最后一条控制指令"""
        return self._last_cmd if hasattr(self, '_last_cmd') else None

    def stop(self):
        """停止并清理"""
        self._running = False
        if self.use_shm:
            self._shm_vision.close()
            self._shm_status.close()


# ============================================================
# 高级融合策略
# ============================================================

class FusionStrategy:
    """融合策略基类"""

    def compute(self, result: VisionResult) -> ControlCommand:
        raise NotImplementedError


class PositionTrackingStrategy(FusionStrategy):
    """位置跟踪策略 - 目标位置即为控制目标"""

    def __init__(self, dead_zone_mm: float = 2.0):
        self.dead_zone = dead_zone_mm

    def compute(self, result: VisionResult) -> ControlCommand:
        cmd = ControlCommand(timestamp_ns=result.timestamp_ns)
        dist = (result.target_x**2 + result.target_y**2) ** 0.5

        if dist < self.dead_zone:
            cmd.mode = ControlMode.STOP
        else:
            cmd.mode = ControlMode.POSITION
            cmd.target_x = result.target_x
            cmd.target_y = result.target_y

        return cmd


class SpeedControlStrategy(FusionStrategy):
    """速度控制策略 - 根据偏差计算速度指令"""

    def __init__(self, kp: float = 2.0, max_speed: float = 500.0):
        self.kp = kp
        self.max_speed = max_speed

    def compute(self, result: VisionResult) -> ControlCommand:
        cmd = ControlCommand(timestamp_ns=result.timestamp_ns)
        cmd.mode = ControlMode.SPEED

        cmd.speed_x = max(-self.max_speed,
                          min(self.max_speed, result.target_x * self.kp))
        cmd.speed_y = max(-self.max_speed,
                          min(self.max_speed, result.target_y * self.kp))

        return cmd


class PredictiveTrackingStrategy(FusionStrategy):
    """预测跟踪策略 - 基于运动估计前馈补偿"""

    def __init__(self, filter_obj: TargetFilter, horizon_s: float = 0.05):
        self.filter = filter_obj
        self.horizon = horizon_s

    def compute(self, result: VisionResult) -> ControlCommand:
        cmd = ControlCommand(timestamp_ns=result.timestamp_ns)
        cmd.mode = ControlMode.TRACK

        # 当前位置
        cmd.target_x = result.target_x
        cmd.target_y = result.target_y

        # 前馈速度
        pred_x, pred_y = self.filter.predict(self.horizon)
        cmd.speed_x = (pred_x - result.target_x) / self.horizon
        cmd.speed_y = (pred_y - result.target_y) / self.horizon

        return cmd


# ============================================================
# 使用示例
# ============================================================

def demo():
    """演示：视觉→控制融合接口使用流程"""

    # 1. 初始化融合接口
    fusion = VisionControlFusion(use_shm=False)

    # 2. 设置相机参数
    # 640x480分辨率，视场200mm x 150mm
    fusion.set_camera_params(
        resolution=(640, 480),
        fov_mm=(200, 150),
        offset=(0, 0)
    )

    # 3. 注册回调
    def on_command(cmd: ControlCommand):
        print(f"控制指令: mode={cmd.mode.name}, "
              f"x={cmd.target_x:.1f}mm, y={cmd.target_y:.1f}mm, "
              f"vx={cmd.speed_x:.1f}mm/s, vy={cmd.speed_y:.1f}mm/s")

    fusion.register_callback(on_command)

    # 4. 模拟视觉检测结果输入
    test_cases = [
        (320, 240, 0.95),  # 画面中心 → 应该零偏差
        (400, 300, 0.90),  # 右下方
        (200, 150, 0.85),  # 左上方
        (320, 240, 0.30),  # 低置信度 → 应该过滤
    ]

    print("=" * 50)
    print("视觉控制融合接口演示")
    print("=" * 50)

    for i, (px, py, conf) in enumerate(test_cases):
        print(f"\n[帧 {i+1}] 像素=({px},{py}), 置信度={conf}")
        cmd = fusion.on_vision_result(
            pixel_x=px,
            pixel_y=py,
            confidence=conf,
            target_type=TargetType.CIRCLE,
        )
        if cmd is None:
            print("  → 目标无效，未生成指令")

    print(f"\n处理帧率: {fusion.get_fps():.1f} fps")
    fusion.stop()


if __name__ == '__main__':
    demo()
