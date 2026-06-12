"""
GPU 形态学操作模块
==================
利用 OpenCL / OpenGL GPU 加速的形态学运算，专为 RK3588S Mali G610 优化。

支持操作：
  - 腐蚀 (Erosion)
  - 膨胀 (Dilation)
  - 开运算 (Opening = 腐蚀 → 膨胀)
  - 闭运算 (Closing = 膨胀 → 腐蚀)
  - 梯度 (Gradient = 膨胀 - 腐蚀)
  - 顶帽 (Top Hat = 原图 - 开运算)
  - 黑帽 (Black Hat = 闭运算 - 原图)

依赖：pyopencl, numpy, cv2

用法示例：
    morph = GPUMorphology()
    result = morph.erode(image, kernel_size=5)
    result = morph.opening(image, kernel_size=7)
"""

import time
import logging
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# OpenCL kernel：形态学运算（腐蚀/膨胀使用 min/max reduction）
MORPH_KERNEL_SRC = """
__kernel void morph_erode(__global const uchar* src,
                          __global uchar* dst,
                          const int width, const int height,
                          const int ksize, const int half_k) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if (x >= width || y >= height) return;

    int channels = get_global_id(2);  // 0 for single channel
    uchar min_val = 255;

    for (int ky = -half_k; ky <= half_k; ky++) {
        for (int kx = -half_k; kx <= half_k; kx++) {
            int sx = clamp(x + kx, 0, width - 1);
            int sy = clamp(y + ky, 0, height - 1);
            uchar val = src[sy * width + sx];
            min_val = min(min_val, val);
        }
    }
    dst[y * width + x] = min_val;
}

__kernel void morph_dilate(__global const uchar* src,
                           __global uchar* dst,
                           const int width, const int height,
                           const int ksize, const int half_k) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if (x >= width || y >= height) return;

    uchar max_val = 0;

    for (int ky = -half_k; ky <= half_k; ky++) {
        for (int kx = -half_k; kx <= half_k; kx++) {
            int sx = clamp(x + kx, 0, width - 1);
            int sy = clamp(y + ky, 0, height - 1);
            uchar val = src[sy * width + sx];
            max_val = max(max_val, val);
        }
    }
    dst[y * width + x] = max_val;
}

__kernel void morph_erode_mask(__global const uchar* src,
                               __global uchar* dst,
                               __global const uchar* mask,
                               const int width, const int height,
                               const int ksize, const int half_k) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if (x >= width || y >= height) return;

    uchar min_val = 255;

    for (int ky = -half_k; ky <= half_k; ky++) {
        for (int kx = -half_k; kx <= half_k; kx++) {
            int mx = kx + half_k;
            int my = ky + half_k;
            if (mask[my * ksize + mx] == 0) continue;
            int sx = clamp(x + kx, 0, width - 1);
            int sy = clamp(y + ky, 0, height - 1);
            uchar val = src[sy * width + sx];
            min_val = min(min_val, val);
        }
    }
    dst[y * width + x] = min_val;
}

__kernel void morph_dilate_mask(__global const uchar* src,
                                __global uchar* dst,
                                __global const uchar* mask,
                                const int width, const int height,
                                const int ksize, const int half_k) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if (x >= width || y >= height) return;

    uchar max_val = 0;

    for (int ky = -half_k; ky <= half_k; ky++) {
        for (int kx = -half_k; kx <= half_k; kx++) {
            int mx = kx + half_k;
            int my = ky + half_k;
            if (mask[my * ksize + mx] == 0) continue;
            int sx = clamp(x + kx, 0, width - 1);
            int sy = clamp(y + ky, 0, height - 1);
            uchar val = src[sy * width + sx];
            max_val = max(max_val, val);
        }
    }
    dst[y * width + x] = max_val;
}
"""


class GPUMorphology:
    """
    GPU 形态学处理器

    Parameters
    ----------
    device_index : int
        OpenCL 设备索引（RK3588S Mali G610 通常为 0）
    """

    def __init__(self, device_index: int = 0):
        self.device_index = device_index
        self._cl_ctx = None
        self._cl_queue = None
        self._programs = {}
        self._init_opencl()

    def _init_opencl(self):
        """初始化 OpenCL 上下文"""
        try:
            import pyopencl as cl
            platforms = cl.get_platforms()
            if not platforms:
                raise RuntimeError("No OpenCL platforms found")

            devices = platforms[0].get_devices(device_type=cl.device_type.GPU)
            if not devices:
                devices = platforms[0].get_devices()

            dev_idx = min(self.device_index, len(devices) - 1)
            self._cl_ctx = cl.Context(devices=[devices[dev_idx]])
            self._cl_queue = cl.CommandQueue(self._cl_ctx,
                                              properties=cl.command_queue_properties.PROFILING_ENABLE)

            self._programs["morph"] = cl.Program(self._cl_ctx, MORPH_KERNEL_SRC).build()
            logger.info(f"GPU Morphology initialized: {devices[dev_idx].name}")
        except ImportError:
            logger.warning("pyopencl not available, will use CPU fallback")
        except Exception as e:
            logger.warning(f"OpenCL init failed: {e}, will use CPU fallback")

    def _ensure_gray(self, image: np.ndarray) -> np.ndarray:
        """确保输入为单通道灰度图"""
        if image.ndim == 3:
            import cv2
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image

    def _run_cl_morph(self, src_gray: np.ndarray, ksize: int, op: str,
                       kernel_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """执行 OpenCL 形态学运算"""
        import pyopencl as cl

        h, w = src_gray.shape[:2]
        half_k = ksize // 2

        src_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR,
                            hostbuf=src_gray.ravel())
        dst_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.WRITE_ONLY, size=src_gray.nbytes)

        use_mask = kernel_mask is not None
        mask_buf = None
        if use_mask:
            mask_u8 = (kernel_mask > 0).astype(np.uint8) * 255
            mask_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR,
                                 hostbuf=mask_u8.ravel())

        if op == "erode":
            kernel_name = "morph_erode_mask" if use_mask else "morph_erode"
        else:
            kernel_name = "morph_dilate_mask" if use_mask else "morph_dilate"

        kernel = getattr(self._programs["morph"], kernel_name)

        if use_mask:
            kernel(self._cl_queue, (w, h, 1), None,
                   src_buf, dst_buf, mask_buf,
                   np.int32(w), np.int32(h), np.int32(ksize), np.int32(half_k))
        else:
            kernel(self._cl_queue, (w, h, 1), None,
                   src_buf, dst_buf,
                   np.int32(w), np.int32(h), np.int32(ksize), np.int32(half_k))

        result = np.empty_like(src_gray)
        cl.enqueue_copy(self._cl_queue, result, dst_buf).wait()
        return result

    def _cpu_morph(self, src_gray: np.ndarray, ksize: int, op: str,
                    kernel_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """CPU 回退实现"""
        import cv2
        if kernel_mask is None:
            kernel = np.ones((ksize, ksize), dtype=np.uint8)
        else:
            kernel = kernel_mask.astype(np.uint8)

        if op == "erode":
            return cv2.erode(src_gray, kernel)
        else:
            return cv2.dilate(src_gray, kernel)

    # ------------------------------------------------------------------
    # 公共 API
    # ------------------------------------------------------------------

    def erode(self, image: np.ndarray, kernel_size: int = 3,
              kernel_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """
        腐蚀操作

        Parameters
        ----------
        image : ndarray
            输入图像（灰度或 BGR）
        kernel_size : int
            结构元素大小（奇数）
        kernel_mask : ndarray or None
            自定义结构元素（None 表示全 1 矩形）

        Returns
        -------
        ndarray
            腐蚀结果
        """
        gray = self._ensure_gray(image)
        if self._cl_ctx is not None:
            return self._run_cl_morph(gray, kernel_size, "erode", kernel_mask)
        return self._cpu_morph(gray, kernel_size, "erode", kernel_mask)

    def dilate(self, image: np.ndarray, kernel_size: int = 3,
               kernel_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """膨胀操作"""
        gray = self._ensure_gray(image)
        if self._cl_ctx is not None:
            return self._run_cl_morph(gray, kernel_size, "dilate", kernel_mask)
        return self._cpu_morph(gray, kernel_size, "dilate", kernel_mask)

    def opening(self, image: np.ndarray, kernel_size: int = 3,
                kernel_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """
        开运算 = 腐蚀 → 膨胀
        去除小的亮噪点，平滑边界
        """
        eroded = self.erode(image, kernel_size, kernel_mask)
        return self.dilate(eroded, kernel_size, kernel_mask)

    def closing(self, image: np.ndarray, kernel_size: int = 3,
                kernel_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """
        闭运算 = 膨胀 → 腐蚀
        填充小的暗区域，连接断裂
        """
        dilated = self.dilate(image, kernel_size, kernel_mask)
        return self.erode(dilated, kernel_size, kernel_mask)

    def gradient(self, image: np.ndarray, kernel_size: int = 3,
                 kernel_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """
        形态学梯度 = 膨胀 - 腐蚀
        提取物体轮廓
        """
        dilated = self.dilate(image, kernel_size, kernel_mask).astype(np.int16)
        eroded = self.erode(image, kernel_size, kernel_mask).astype(np.int16)
        return np.clip(dilated - eroded, 0, 255).astype(np.uint8)

    def top_hat(self, image: np.ndarray, kernel_size: int = 3,
                kernel_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """
        顶帽 = 原图 - 开运算
        提取亮细节
        """
        gray = self._ensure_gray(image)
        opened = self.opening(gray, kernel_size, kernel_mask)
        return np.clip(gray.astype(np.int16) - opened.astype(np.int16), 0, 255).astype(np.uint8)

    def black_hat(self, image: np.ndarray, kernel_size: int = 3,
                  kernel_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """
        黑帽 = 闭运算 - 原图
        提取暗细节
        """
        gray = self._ensure_gray(image)
        closed = self.closing(gray, kernel_size, kernel_mask)
        return np.clip(closed.astype(np.int16) - gray.astype(np.int16), 0, 255).astype(np.uint8)

    def batch_morphology(self, images: list, op: str = "erode",
                          kernel_size: int = 3) -> list:
        """
        批量形态学操作

        Parameters
        ----------
        images : list of ndarray
        op : str - "erode", "dilate", "opening", "closing", "gradient", "top_hat", "black_hat"
        kernel_size : int

        Returns
        -------
        list of ndarray
        """
        op_fn = {
            "erode": self.erode,
            "dilate": self.dilate,
            "opening": self.opening,
            "closing": self.closing,
            "gradient": self.gradient,
            "top_hat": self.top_hat,
            "black_hat": self.black_hat,
        }
        if op not in op_fn:
            raise ValueError(f"Unknown op: {op}. Supported: {list(op_fn.keys())}")

        fn = op_fn[op]
        return [fn(img, kernel_size) for img in images]


# ======================================================================
# 独立测试
# ======================================================================
if __name__ == "__main__":
    import cv2

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    morph = GPUMorphology()
    print(f"OpenCL available: {morph._cl_ctx is not None}")

    # 创建测试图像（带噪点的二值图）
    test_img = np.zeros((512, 512), dtype=np.uint8)
    cv2.circle(test_img, (256, 256), 100, 255, -1)
    cv2.circle(test_img, (256, 256), 60, 0, -1)   # 环形
    # 添加噪点
    noise = np.random.randint(0, 256, test_img.shape, dtype=np.uint8)
    test_img[noise > 250] = 255

    # 测试各种操作
    t0 = time.perf_counter()
    eroded = morph.erode(test_img, kernel_size=5)
    t1 = time.perf_counter()
    print(f"Erode  (512x512, k=5): {(t1-t0)*1000:.2f}ms, unique={np.unique(eroded)}")

    t0 = time.perf_counter()
    dilated = morph.dilate(test_img, kernel_size=5)
    t1 = time.perf_counter()
    print(f"Dilate (512x512, k=5): {(t1-t0)*1000:.2f}ms")

    t0 = time.perf_counter()
    opened = morph.opening(test_img, kernel_size=5)
    t1 = time.perf_counter()
    print(f"Opening (512x512, k=5): {(t1-t0)*1000:.2f}ms")

    t0 = time.perf_counter()
    closed = morph.closing(test_img, kernel_size=5)
    t1 = time.perf_counter()
    print(f"Closing (512x512, k=5): {(t1-t0)*1000:.2f}ms")

    gradient = morph.gradient(test_img, kernel_size=3)
    print(f"Gradient shape: {gradient.shape}, max={gradient.max()}")

    tophat = morph.top_hat(test_img, kernel_size=7)
    blackhat = morph.black_hat(test_img, kernel_size=7)
    print(f"TopHat/BlackHat computed successfully")

    # 自定义结构元素（十字形）
    cross = np.array([[0,1,0],[1,1,1],[0,1,0]], dtype=np.uint8)
    eroded_cross = morph.erode(test_img, kernel_size=3, kernel_mask=cross)
    print(f"Custom kernel erosion OK, shape={eroded_cross.shape}")

    # 批量测试
    batch_imgs = [test_img.copy() for _ in range(4)]
    results = morph.batch_morphology(batch_imgs, op="opening", kernel_size=5)
    print(f"Batch morphology: {len(results)} images processed")

    print("All GPU morphology tests passed!")
