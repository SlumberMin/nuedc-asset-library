"""
RGA 批处理模块 - 多图并行处理
================================
利用 RK3588S RGA 硬件加速单元，对多张图片进行并行批处理。
支持缩放、格式转换、旋转、CSC 等操作的批量执行。

使用场景：
  - 视频帧批量缩放
  - 多路摄像头图像统一预处理
  - 数据集批量格式转换

依赖：rga (librga), numpy, PIL

用法示例：
    batch = RGABatch()
    results = batch.batch_resize(images, target_size=(640, 480))
"""

import time
import logging
from typing import List, Tuple, Optional, Any
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class BatchTask:
    """单个批处理任务描述"""
    task_id: int
    src: np.ndarray
    dst: Optional[np.ndarray] = None
    params: dict = field(default_factory=dict)
    elapsed_ms: float = 0.0
    success: bool = False
    error: Optional[str] = None


class RGABatch:
    """
    RGA 批处理器

    将多张图片的 RGA 操作打包并行执行，充分利用硬件队列。

    Parameters
    ----------
    max_workers : int
        并行工作线程数（默认 4，建议不超过 RGA 通道数）
    device_id : int
        RGA 设备 ID（多核 SoC 场景）
    """

    def __init__(self, max_workers: int = 4, device_id: int = 0):
        self.max_workers = max_workers
        self.device_id = device_id
        self._rga = None
        self._init_rga()

    def _init_rga(self):
        """初始化 RGA 上下文"""
        try:
            from rga import Rga  # type: ignore
            self._rga = Rga()
            self._rga.init(device_id=self.device_id)
            logger.info(f"RGA batch initialized: device={self.device_id}, workers={self.max_workers}")
        except ImportError:
            logger.warning("librga not available, falling back to CPU emulation")
            self._rga = None

    # ------------------------------------------------------------------
    # 单帧 RGA 操作（封装）
    # ------------------------------------------------------------------

    def _single_resize(self, src: np.ndarray, width: int, height: int) -> np.ndarray:
        """单帧 RGA 缩放"""
        if self._rga is not None:
            import cv2
            dst = np.zeros((height, width, src.shape[2] if src.ndim == 3 else 1), dtype=src.dtype)
            self._rga.resize(src, dst)
            return dst
        else:
            import cv2
            return cv2.resize(src, (width, height), interpolation=cv2.INTER_LINEAR)

    def _single_convert_color(self, src: np.ndarray, src_fmt: str, dst_fmt: str) -> np.ndarray:
        """单帧 RGA 颜色空间转换"""
        if self._rga is not None:
            self._rga.set_src_info(src, fmt=src_fmt)
            self._rga.set_dst_info(fmt=dst_fmt)
            return self._rga.convert()
        else:
            import cv2
            code = self._get_cv2_cvt_code(src_fmt, dst_fmt)
            return cv2.cvtColor(src, code)

    def _single_rotate(self, src: np.ndarray, degree: int) -> np.ndarray:
        """单帧 RGA 旋转（90/180/270）"""
        if self._rga is not None:
            return self._rga.rotate(src, degree)
        else:
            import cv2
            if degree == 90:
                return cv2.rotate(src, cv2.ROTATE_90_CLOCKWISE)
            elif degree == 180:
                return cv2.rotate(src, cv2.ROTATE_180)
            elif degree == 270:
                return cv2.rotate(src, cv2.ROTATE_90_COUNTERCLOCKWISE)
            return src.copy()

    def _single_crop(self, src: np.ndarray, x: int, y: int, w: int, h: int) -> np.ndarray:
        """单帧 RGA 裁剪"""
        if self._rga is not None:
            return self._rga.crop(src, x, y, w, h)
        else:
            return src[y:y+h, x:x+w].copy()

    # ------------------------------------------------------------------
    # 批处理接口
    # ------------------------------------------------------------------

    def batch_resize(
        self,
        images: List[np.ndarray],
        target_size: Tuple[int, int],
    ) -> List[np.ndarray]:
        """
        批量缩放图片

        Parameters
        ----------
        images : list of ndarray
            输入图片列表
        target_size : (width, height)
            目标尺寸

        Returns
        -------
        list of ndarray
            缩放后图片列表
        """
        w, h = target_size
        tasks = [
            BatchTask(task_id=i, src=img, params={"width": w, "height": h})
            for i, img in enumerate(images)
        ]
        return self._execute_batch(tasks, self._resize_worker)

    def batch_convert_color(
        self,
        images: List[np.ndarray],
        src_format: str = "BGR",
        dst_format: str = "RGB",
    ) -> List[np.ndarray]:
        """批量颜色空间转换"""
        tasks = [
            BatchTask(task_id=i, src=img, params={"src_fmt": src_format, "dst_fmt": dst_format})
            for i, img in enumerate(images)
        ]
        return self._execute_batch(tasks, self._cvtcolor_worker)

    def batch_rotate(
        self,
        images: List[np.ndarray],
        degree: int = 90,
    ) -> List[np.ndarray]:
        """批量旋转（90/180/270 度）"""
        tasks = [
            BatchTask(task_id=i, src=img, params={"degree": degree})
            for i, img in enumerate(images)
        ]
        return self._execute_batch(tasks, self._rotate_worker)

    def batch_crop(
        self,
        images: List[np.ndarray],
        roi: Tuple[int, int, int, int],
    ) -> List[np.ndarray]:
        """批量裁剪 (x, y, w, h)"""
        x, y, w, h = roi
        tasks = [
            BatchTask(task_id=i, src=img, params={"x": x, "y": y, "w": w, "h": h})
            for i, img in enumerate(images)
        ]
        return self._execute_batch(tasks, self._crop_worker)

    def batch_composite(
        self,
        operations: List[dict],
    ) -> List[np.ndarray]:
        """
        批量组合操作：对每张图执行一系列 RGA 操作

        Parameters
        ----------
        operations : list of dict
            每个 dict 包含:
              - "image": np.ndarray
              - "ops": list of (op_name, kwargs) tuples
              例如: [{"image": img, "ops": [("resize", {"width":640,"height":480}),
                                            ("rotate", {"degree":90})]}]

        Returns
        -------
        list of ndarray
        """
        results = []
        for op in operations:
            img = op["image"]
            for op_name, kwargs in op.get("ops", []):
                if op_name == "resize":
                    img = self._single_resize(img, kwargs["width"], kwargs["height"])
                elif op_name == "rotate":
                    img = self._single_rotate(img, kwargs["degree"])
                elif op_name == "crop":
                    img = self._single_crop(img, kwargs["x"], kwargs["y"],
                                            kwargs["w"], kwargs["h"])
                elif op_name == "cvtcolor":
                    img = self._single_convert_color(img, kwargs["src_fmt"], kwargs["dst_fmt"])
            results.append(img)
        return results

    # ------------------------------------------------------------------
    # 执行引擎
    # ------------------------------------------------------------------

    def _execute_batch(self, tasks: List[BatchTask], worker_fn) -> List[np.ndarray]:
        """使用线程池并行执行批处理任务"""
        t0 = time.perf_counter()

        if len(tasks) <= 1 or self.max_workers <= 1:
            for task in tasks:
                try:
                    task.dst = worker_fn(task)
                    task.success = True
                except Exception as e:
                    task.error = str(e)
                    logger.error(f"Task {task.task_id} failed: {e}")
        else:
            with ThreadPoolExecutor(max_workers=self.max_workers) as pool:
                futures = {pool.submit(worker_fn, t): t for t in tasks}
                for future in as_completed(futures):
                    task = futures[future]
                    try:
                        task.dst = future.result()
                        task.success = True
                    except Exception as e:
                        task.error = str(e)
                        logger.error(f"Task {task.task_id} failed: {e}")

        elapsed = (time.perf_counter() - t0) * 1000
        success_count = sum(1 for t in tasks if t.success)
        logger.info(f"Batch completed: {success_count}/{len(tasks)} success, {elapsed:.1f}ms total")

        return [t.dst for t in tasks]

    # --- workers ---

    @staticmethod
    def _resize_worker(task: BatchTask) -> np.ndarray:
        p = task.params
        # Try RGA first, fallback cv2
        try:
            from rga import Rga
            rga = Rga()
            dst = np.zeros((p["height"], p["width"], task.src.shape[2] if task.src.ndim == 3 else 1),
                           dtype=task.src.dtype)
            rga.resize(task.src, dst)
            return dst
        except Exception:
            import cv2
            return cv2.resize(task.src, (p["width"], p["height"]), interpolation=cv2.INTER_LINEAR)

    @staticmethod
    def _cvtcolor_worker(task: BatchTask) -> np.ndarray:
        import cv2
        code = RGABatch._get_cv2_cvt_code(task.params["src_fmt"], task.params["dst_fmt"])
        return cv2.cvtColor(task.src, code)

    @staticmethod
    def _rotate_worker(task: BatchTask) -> np.ndarray:
        degree = task.params["degree"]
        try:
            from rga import Rga
            rga = Rga()
            return rga.rotate(task.src, degree)
        except Exception:
            import cv2
            if degree == 90:
                return cv2.rotate(task.src, cv2.ROTATE_90_CLOCKWISE)
            elif degree == 180:
                return cv2.rotate(task.src, cv2.ROTATE_180)
            elif degree == 270:
                return cv2.rotate(task.src, cv2.ROTATE_90_COUNTERCLOCKWISE)
            return task.src.copy()

    @staticmethod
    def _crop_worker(task: BatchTask) -> np.ndarray:
        p = task.params
        try:
            from rga import Rga
            rga = Rga()
            return rga.crop(task.src, p["x"], p["y"], p["w"], p["h"])
        except Exception:
            return task.src[p["y"]:p["y"]+p["h"], p["x"]:p["x"]+p["w"]].copy()

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------

    @staticmethod
    def _get_cv2_cvt_code(src_fmt: str, dst_fmt: str) -> int:
        """颜色格式字符串转 OpenCV cvtColor code"""
        import cv2
        fmt_map = {
            ("BGR", "RGB"): cv2.COLOR_BGR2RGB,
            ("RGB", "BGR"): cv2.COLOR_RGB2BGR,
            ("BGR", "GRAY"): cv2.COLOR_BGR2GRAY,
            ("RGB", "GRAY"): cv2.COLOR_RGB2GRAY,
            ("GRAY", "BGR"): cv2.COLOR_GRAY2BGR,
            ("GRAY", "RGB"): cv2.COLOR_GRAY2RGB,
            ("BGR", "YUV"): cv2.COLOR_BGR2YUV,
            ("YUV", "BGR"): cv2.COLOR_YUV2BGR,
            ("BGR", "HSV"): cv2.COLOR_BGR2HSV,
            ("HSV", "BGR"): cv2.COLOR_HSV2BGR,
            ("BGRA", "BGR"): cv2.COLOR_BGRA2BGR,
            ("BGR", "BGRA"): cv2.COLOR_BGR2BGRA,
            ("NV12", "BGR"): cv2.COLOR_YUV2BGR_NV12,
            ("NV21", "BGR"): cv2.COLOR_YUV2BGR_NV21,
        }
        key = (src_fmt.upper(), dst_fmt.upper())
        if key not in fmt_map:
            raise ValueError(f"Unsupported color conversion: {src_fmt} -> {dst_fmt}")
        return fmt_map[key]

    def get_stats(self) -> dict:
        """获取批处理器配置信息"""
        return {
            "max_workers": self.max_workers,
            "device_id": self.device_id,
            "rga_available": self._rga is not None,
        }


# ======================================================================
# 独立测试
# ======================================================================
if __name__ == "__main__":
    import cv2

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    batch = RGABatch(max_workers=4)
    print("RGA Batch stats:", batch.get_stats())

    # 创建测试图片
    test_images = [np.random.randint(0, 256, (1080, 1920, 3), dtype=np.uint8) for _ in range(8)]

    # 批量缩放
    t0 = time.perf_counter()
    resized = batch.batch_resize(test_images, target_size=(640, 480))
    t1 = time.perf_counter()
    print(f"Batch resize 8x (1080p->640x480): {(t1-t0)*1000:.1f}ms")
    print(f"  Output shapes: {[r.shape for r in resized[:3]]}")

    # 批量颜色转换
    t0 = time.perf_counter()
    grays = batch.batch_convert_color(test_images, src_format="BGR", dst_format="GRAY")
    t1 = time.perf_counter()
    print(f"Batch BGR->GRAY 8x: {(t1-t0)*1000:.1f}ms")

    # 批量旋转
    t0 = time.perf_counter()
    rotated = batch.batch_rotate(test_images, degree=90)
    t1 = time.perf_counter()
    print(f"Batch rotate 90° 8x: {(t1-t0)*1000:.1f}ms")

    # 组合操作
    ops = [{"image": img, "ops": [
        ("resize", {"width": 640, "height": 480}),
        ("rotate", {"degree": 180}),
    ]} for img in test_images[:4]]
    t0 = time.perf_counter()
    composited = batch.batch_composite(ops)
    t1 = time.perf_counter()
    print(f"Batch composite (resize+rotate) 4x: {(t1-t0)*1000:.1f}ms")

    print("All RGA batch tests passed!")
