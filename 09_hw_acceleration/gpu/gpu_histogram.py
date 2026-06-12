"""
GPU 直方图计算与均衡化模块
============================
利用 OpenCL GPU 加速直方图计算和直方图均衡化，
针对 RK3588S Mali G610 优化。

功能：
  - GPU 加速直方图统计（256 bins）
  - 直方图均衡化（全局 / CLAHE 局部自适应）
  - 直方图匹配（规定化）
  - 多通道独立直方图

依赖：pyopencl, numpy, cv2

用法示例：
    hg = GPUHistogram()
    hist = hg.compute(image)
    equalized = hg.equalize(image)
    clahe_result = hg.clahe(image, clip_limit=2.0, grid_size=(8,8))
"""

import time
import logging
from typing import Tuple, Optional

import numpy as np

logger = logging.getLogger(__name__)

# OpenCL kernel：直方图统计 + 均衡化
HISTOGRAM_KERNEL_SRC = """
// 直方图统计（单通道 uchar 图像）
// 使用原子操作累加 bin
__kernel void hist_compute(__global const uchar* src,
                           __global uint* hist,
                           const int width, const int height) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if (x >= width || y >= height) return;

    uchar val = src[y * width + x];
    atomic_inc(&hist[val]);
}

// 直方图均衡化映射
// LUT: src_val -> dst_val (precomputed from CDF)
__kernel void hist_apply_lut(__global const uchar* src,
                              __global uchar* dst,
                              __global const uchar* lut,
                              const int width, const int height) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if (x >= width || y >= height) return;

    dst[y * width + x] = lut[src[y * width + x]];
}

// CLAHE: 分块直方图统计
__kernel void clahe_tile_hist(__global const uchar* src,
                               __global uint* tile_hists,
                               const int width, const int height,
                               const int tile_x, const int tile_y,
                               const int tile_w, const int tile_h,
                               const int tiles_x, const int tile_idx) {
    int lx = get_global_id(0);
    int ly = get_global_id(1);
    if (lx >= tile_w || ly >= tile_h) return;

    int gx = tile_x + lx;
    int gy = tile_y + ly;
    if (gx >= width || gy >= height) return;

    uchar val = src[gy * width + gx];
    // 每个 tile 有自己的 256-bin 直方图
    __global uint* my_hist = &tile_hists[tile_idx * 256];
    atomic_inc(&my_hist[val]);
}

// CLAHE: 应用 LUT 双线性插值
__kernel void clahe_apply(__global const uchar* src,
                           __global uchar* dst,
                           __global const uchar* tile_luts,
                           const int width, const int height,
                           const int tiles_x, const int tiles_y,
                           const int tile_w, const int tile_h) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if (x >= width || y >= height) return;

    // 计算当前像素属于哪个 tile 区域，以及在 tile 内的归一化坐标
    float fx = (float)x / tile_w - 0.5f;
    float fy = (float)y / tile_h - 0.5f;

    int tx0 = clamp((int)floor(fx), 0, tiles_x - 1);
    int ty0 = clamp((int)floor(fy), 0, tiles_y - 1);
    int tx1 = min(tx0 + 1, tiles_x - 1);
    int ty1 = min(ty0 + 1, tiles_y - 1);

    float ax = fx - tx0;
    float ay = fy - ty0;
    ax = clamp(ax, 0.0f, 1.0f);
    ay = clamp(ay, 0.0f, 1.0f);

    uchar val = src[y * width + x];

    // 双线性插值 4 个 tile 的 LUT
    float v00 = tile_luts[(ty0 * tiles_x + tx0) * 256 + val];
    float v10 = tile_luts[(ty0 * tiles_x + tx1) * 256 + val];
    float v01 = tile_luts[(ty1 * tiles_x + tx0) * 256 + val];
    float v11 = tile_luts[(ty1 * tiles_x + tx1) * 256 + val];

    float result = v00 * (1-ax) * (1-ay) + v10 * ax * (1-ay)
                 + v01 * (1-ax) * ay      + v11 * ax * ay;

    dst[y * width + x] = clamp((uchar)round(result), (uchar)0, (uchar)255);
}
"""


class GPUHistogram:
    """
    GPU 直方图处理器

    Parameters
    ----------
    device_index : int
        OpenCL 设备索引
    """

    def __init__(self, device_index: int = 0):
        self.device_index = device_index
        self._cl_ctx = None
        self._cl_queue = None
        self._programs = {}
        self._init_opencl()

    def _init_opencl(self):
        """初始化 OpenCL"""
        try:
            import pyopencl as cl
            platforms = cl.get_platforms()
            if not platforms:
                raise RuntimeError("No OpenCL platforms")

            devices = platforms[0].get_devices(device_type=cl.device_type.GPU)
            if not devices:
                devices = platforms[0].get_devices()

            dev_idx = min(self.device_index, len(devices) - 1)
            self._cl_ctx = cl.Context(devices=[devices[dev_idx]])
            self._cl_queue = cl.CommandQueue(
                self._cl_ctx,
                properties=cl.command_queue_properties.PROFILING_ENABLE
            )
            self._programs["hist"] = cl.Program(self._cl_ctx, HISTOGRAM_KERNEL_SRC).build()
            logger.info(f"GPU Histogram initialized: {devices[dev_idx].name}")
        except ImportError:
            logger.warning("pyopencl not available, using CPU fallback")
        except Exception as e:
            logger.warning(f"OpenCL init failed: {e}")

    def _ensure_gray(self, image: np.ndarray) -> np.ndarray:
        if image.ndim == 3:
            import cv2
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image

    # ------------------------------------------------------------------
    # 直方图计算
    # ------------------------------------------------------------------

    def compute(self, image: np.ndarray) -> np.ndarray:
        """
        计算灰度直方图（256 bins）

        Parameters
        ----------
        image : ndarray
            输入图像（灰度或 BGR）

        Returns
        -------
        ndarray (256,) uint32
            各 bin 的像素计数
        """
        gray = self._ensure_gray(image).astype(np.uint8).copy()
        h, w = gray.shape

        if self._cl_ctx is not None:
            return self._compute_gpu(gray, w, h)
        return self._compute_cpu(gray)

    def _compute_gpu(self, gray: np.ndarray, w: int, h: int) -> np.ndarray:
        import pyopencl as cl

        src_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR,
                            hostbuf=gray.ravel())
        hist_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.READ_WRITE, size=256 * 4)
        # 清零
        cl.enqueue_fill_buffer(self._cl_queue, hist_buf, np.uint32(0), 0, 256 * 4).wait()

        kernel = self._programs["hist"].hist_compute
        kernel(self._cl_queue, (w, h), None, src_buf, hist_buf,
               np.int32(w), np.int32(h))

        hist = np.zeros(256, dtype=np.uint32)
        cl.enqueue_copy(self._cl_queue, hist, hist_buf).wait()
        return hist

    @staticmethod
    def _compute_cpu(gray: np.ndarray) -> np.ndarray:
        hist = np.zeros(256, dtype=np.uint32)
        for val in range(256):
            hist[val] = np.sum(gray == val)
        return hist

    # ------------------------------------------------------------------
    # 均衡化
    # ------------------------------------------------------------------

    def equalize(self, image: np.ndarray) -> np.ndarray:
        """
        全局直方图均衡化

        Parameters
        ----------
        image : ndarray
            输入图像

        Returns
        -------
        ndarray
            均衡化后的图像
        """
        gray = self._ensure_gray(image)
        h, w = gray.shape

        # 计算直方图
        if self._cl_ctx is not None:
            hist = self._compute_gpu(gray, w, h)
        else:
            hist, _ = np.histogram(gray.ravel(), bins=256, range=(0, 256))

        # CPU 端计算 CDF → LUT
        lut = self._build_lut_from_hist(hist, h * w)

        # 应用 LUT
        if self._cl_ctx is not None:
            return self._apply_lut_gpu(gray, lut)
        return self._apply_lut_cpu(gray, lut)

    @staticmethod
    def _build_lut_from_hist(hist: np.ndarray, total_pixels: int) -> np.ndarray:
        """从直方图构建均衡化查找表"""
        cdf = hist.astype(np.float64).cumsum()
        # 找到第一个非零 CDF 值
        cdf_min = cdf[cdf > 0].min() if np.any(cdf > 0) else 0
        if total_pixels == cdf_min:
            return np.arange(256, dtype=np.uint8)

        lut = np.round((cdf - cdf_min) / (total_pixels - cdf_min) * 255)
        return np.clip(lut, 0, 255).astype(np.uint8)

    def _apply_lut_gpu(self, gray: np.ndarray, lut: np.ndarray) -> np.ndarray:
        import pyopencl as cl

        h, w = gray.shape
        gray_c = gray.astype(np.uint8).copy()

        src_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR,
                            hostbuf=gray_c.ravel())
        dst_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.WRITE_ONLY, size=gray_c.nbytes)
        lut_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR,
                            hostbuf=lut.ravel())

        kernel = self._programs["hist"].hist_apply_lut
        kernel(self._cl_queue, (w, h), None, src_buf, dst_buf, lut_buf,
               np.int32(w), np.int32(h))

        result = np.empty_like(gray_c)
        cl.enqueue_copy(self._cl_queue, result, dst_buf).wait()
        return result

    @staticmethod
    def _apply_lut_cpu(gray: np.ndarray, lut: np.ndarray) -> np.ndarray:
        return lut[gray].astype(np.uint8)

    # ------------------------------------------------------------------
    # CLAHE（对比度受限的自适应直方图均衡化）
    # ------------------------------------------------------------------

    def clahe(self, image: np.ndarray, clip_limit: float = 2.0,
              grid_size: Tuple[int, int] = (8, 8)) -> np.ndarray:
        """
        CLAHE - 对比度受限的自适应直方图均衡化

        Parameters
        ----------
        image : ndarray
            输入图像
        clip_limit : float
            直方图裁剪限制（越大对比度增强越强）
        grid_size : (tiles_x, tiles_y)
            分块网格大小

        Returns
        -------
        ndarray
            CLAHE 处理结果
        """
        gray = self._ensure_gray(image).astype(np.uint8).copy()
        h, w = gray.shape
        tiles_x, tiles_y = grid_size

        tile_w = w // tiles_x
        tile_h = h // tiles_y

        if self._cl_ctx is not None:
            return self._clahe_gpu(gray, w, h, tiles_x, tiles_y, tile_w, tile_h, clip_limit)
        return self._clahe_cpu(gray, tiles_x, tiles_y, tile_w, tile_h, clip_limit)

    def _clahe_gpu(self, gray, w, h, tiles_x, tiles_y, tile_w, tile_h, clip_limit):
        """GPU CLAHE 实现"""
        import pyopencl as cl

        n_tiles = tiles_x * tiles_y
        total_pixels_per_tile = tile_w * tile_h

        # 1. 为每个 tile 统计直方图
        tile_hists = np.zeros(n_tiles * 256, dtype=np.uint32)

        src_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR,
                            hostbuf=gray.ravel())
        tile_hists_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.READ_WRITE, size=tile_hists.nbytes)
        cl.enqueue_fill_buffer(self._cl_queue, tile_hists_buf, np.uint32(0), 0, tile_hists.nbytes).wait()

        for ty in range(tiles_y):
            for tx in range(tiles_x):
                idx = ty * tiles_x + tx
                tile_x_off = tx * tile_w
                tile_y_off = ty * tile_h

                kernel = self._programs["hist"].clahe_tile_hist
                kernel(self._cl_queue, (tile_w, tile_h), None,
                       src_buf, tile_hists_buf,
                       np.int32(w), np.int32(h),
                       np.int32(tile_x_off), np.int32(tile_y_off),
                       np.int32(tile_w), np.int32(tile_h),
                       np.int32(tiles_x), np.int32(idx))

        cl.enqueue_copy(self._cl_queue, tile_hists, tile_hists_buf).wait()

        # 2. CPU 端计算每个 tile 的 LUT（含裁剪）
        tile_luts = np.zeros((n_tiles, 256), dtype=np.uint8)
        for i in range(n_tiles):
            hist = tile_hists[i * 256:(i + 1) * 256].astype(np.float64)
            tile_luts[i] = self._clip_and_equalize(hist, clip_limit, total_pixels_per_tile)

        # 3. GPU 双线性插值应用
        dst = np.empty_like(gray)
        dst_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.WRITE_ONLY, size=dst.nbytes)
        lut_buf = cl.Buffer(self._cl_ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR,
                            hostbuf=tile_luts.ravel())

        kernel = self._programs["hist"].clahe_apply
        kernel(self._cl_queue, (w, h), None,
               src_buf, dst_buf, lut_buf,
               np.int32(w), np.int32(h),
               np.int32(tiles_x), np.int32(tiles_y),
               np.int32(tile_w), np.int32(tile_h))

        cl.enqueue_copy(self._cl_queue, dst, dst_buf).wait()
        return dst

    def _clahe_cpu(self, gray, tiles_x, tiles_y, tile_w, tile_h, clip_limit):
        """CPU 回退 CLAHE"""
        h, w = gray.shape
        total_pixels = tile_w * tile_h

        # 计算每个 tile 的 LUT
        tile_luts = np.zeros((tiles_y, tiles_x, 256), dtype=np.uint8)
        for ty in range(tiles_y):
            for tx in range(tiles_x):
                x0, y0 = tx * tile_w, ty * tile_h
                tile = gray[y0:y0+tile_h, x0:x0+tile_w]
                hist, _ = np.histogram(tile.ravel(), bins=256, range=(0, 256))
                tile_luts[ty, tx] = self._clip_and_equalize(hist.astype(np.float64),
                                                              clip_limit, total_pixels)

        # 双线性插值
        result = np.zeros_like(gray)
        for y in range(h):
            for x in range(w):
                fx = x / tile_w - 0.5
                fy = y / tile_h - 0.5
                tx0 = max(0, min(int(np.floor(fx)), tiles_x - 1))
                ty0 = max(0, min(int(np.floor(fy)), tiles_y - 1))
                tx1 = min(tx0 + 1, tiles_x - 1)
                ty1 = min(ty0 + 1, tiles_y - 1)
                ax = max(0.0, min(fx - tx0, 1.0))
                ay = max(0.0, min(fy - ty0, 1.0))

                val = gray[y, x]
                v = (tile_luts[ty0, tx0, val] * (1-ax) * (1-ay) +
                     tile_luts[ty0, tx1, val] * ax * (1-ay) +
                     tile_luts[ty1, tx0, val] * (1-ax) * ay +
                     tile_luts[ty1, tx1, val] * ax * ay)
                result[y, x] = int(round(v))

        return result.astype(np.uint8)

    @staticmethod
    def _clip_and_equalize(hist: np.ndarray, clip_limit: float,
                            total_pixels: int) -> np.ndarray:
        """裁剪直方图并计算均衡化 LUT"""
        # 裁剪
        clipped_sum = 0
        threshold = clip_limit * total_pixels / 256
        for i in range(256):
            excess = hist[i] - threshold
            if excess > 0:
                clipped_sum += excess
                hist[i] = threshold

        # 重新分配被裁剪的像素
        redistrib = clipped_sum / 256
        hist += redistrib

        # CDF → LUT
        cdf = hist.cumsum()
        cdf_min = cdf[cdf > 0].min() if np.any(cdf > 0) else 0
        if total_pixels == cdf_min:
            return np.arange(256, dtype=np.uint8)
        lut = np.round((cdf - cdf_min) / (total_pixels - cdf_min) * 255)
        return np.clip(lut, 0, 255).astype(np.uint8)

    # ------------------------------------------------------------------
    # 多通道直方图
    # ------------------------------------------------------------------

    def compute_multichannel(self, image: np.ndarray) -> dict:
        """
        多通道直方图统计

        Parameters
        ----------
        image : ndarray (H, W, C)

        Returns
        -------
        dict: {"channel_0": ndarray(256), "channel_1": ..., ...}
        """
        if image.ndim == 2:
            return {"gray": self.compute(image)}

        result = {}
        for c in range(image.shape[2]):
            result[f"channel_{c}"] = self.compute(image[:, :, c])
        return result

    def equalize_multichannel(self, image: np.ndarray) -> np.ndarray:
        """
        多通道独立均衡化（在 YCrCb 空间的 Y 通道上做均衡化）

        Parameters
        ----------
        image : ndarray (H, W, 3) BGR

        Returns
        -------
        ndarray
        """
        import cv2
        ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
        ycrcb[:, :, 0] = self.equalize(ycrcb[:, :, 0])
        return cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)

    # ------------------------------------------------------------------
    # 直方图匹配（规定化）
    # ------------------------------------------------------------------

    def match_histogram(self, source: np.ndarray, reference: np.ndarray) -> np.ndarray:
        """
        直方图匹配：将 source 的直方图变换为 reference 的分布

        Parameters
        ----------
        source : ndarray
            源图像
        reference : ndarray
            参考图像

        Returns
        -------
        ndarray
            匹配后的图像
        """
        src_gray = self._ensure_gray(source)
        ref_gray = self._ensure_gray(reference)

        h_src, w_src = src_gray.shape
        h_ref, w_ref = ref_gray.shape

        # 计算两张图的直方图
        hist_src = self.compute(src_gray).astype(np.float64)
        hist_ref = self.compute(ref_gray).astype(np.float64)

        # CDF
        cdf_src = hist_src.cumsum() / hist_src.sum()
        cdf_ref = hist_ref.cumsum() / hist_ref.sum()

        # 构建映射：对于 source 的每个灰度级，找到 CDF 最接近的 reference 灰度级
        lut = np.zeros(256, dtype=np.uint8)
        for src_val in range(256):
            # 找到 cdf_ref 中最接近 cdf_src[src_val] 的索引
            diffs = np.abs(cdf_ref - cdf_src[src_val])
            lut[src_val] = np.argmin(diffs)

        return lut[src_gray].astype(np.uint8)

    # ------------------------------------------------------------------
    # 可视化
    # ------------------------------------------------------------------

    @staticmethod
    def draw_histogram(hist: np.ndarray, width: int = 512, height: int = 256,
                        color: Tuple[int, int, int] = (255, 255, 255)) -> np.ndarray:
        """
        绘制直方图可视化图

        Parameters
        ----------
        hist : ndarray (256,)
        width, height : 输出图尺寸
        color : BGR 颜色

        Returns
        -------
        ndarray (H, W, 3)
        """
        import cv2
        canvas = np.zeros((height, width, 3), dtype=np.uint8)

        if hist.max() == 0:
            return canvas

        bin_w = width / 256
        norm_hist = hist / hist.max() * (height - 10)

        for i in range(256):
            x1 = int(i * bin_w)
            x2 = int((i + 1) * bin_w)
            y = int(norm_hist[i])
            cv2.rectangle(canvas, (x1, height - y), (x2, height), color, -1)

        return canvas


# ======================================================================
# 独立测试
# ======================================================================
if __name__ == "__main__":
    import cv2

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    hg = GPUHistogram()
    print(f"OpenCL available: {hg._cl_ctx is not None}")

    # 创建低对比度测试图像
    test_img = np.random.randint(80, 180, (480, 640), dtype=np.uint8)
    # 添加一些结构
    cv2.circle(test_img, (320, 240), 80, 200, -1)
    cv2.rectangle(test_img, (100, 100), (200, 200), 60, -1)

    # 直方图计算
    t0 = time.perf_counter()
    hist = hg.compute(test_img)
    t1 = time.perf_counter()
    print(f"Histogram compute: {(t1-t0)*1000:.2f}ms")
    print(f"  Total pixels counted: {hist.sum()}, expected: {480*640}")

    # 全局均衡化
    t0 = time.perf_counter()
    eq = hg.equalize(test_img)
    t1 = time.perf_counter()
    print(f"Equalize: {(t1-t0)*1000:.2f}ms, range=[{eq.min()}, {eq.max()}]")

    # CLAHE
    t0 = time.perf_counter()
    clahe = hg.clahe(test_img, clip_limit=2.0, grid_size=(8, 8))
    t1 = time.perf_counter()
    print(f"CLAHE: {(t1-t0)*1000:.2f}ms, range=[{clahe.min()}, {clahe.max()}]")

    # 多通道
    color_img = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
    multi_hist = hg.compute_multichannel(color_img)
    print(f"Multi-channel histograms: {list(multi_hist.keys())}")

    # 直方图匹配
    ref_img = np.random.randint(0, 256, (480, 640), dtype=np.uint8)
    matched = hg.match_histogram(test_img, ref_img)
    print(f"Histogram match: shape={matched.shape}, range=[{matched.min()}, {matched.max()}]")

    # 可视化
    canvas = hg.draw_histogram(hist)
    print(f"Histogram visualization: {canvas.shape}")

    # 多通道均衡化
    eq_color = hg.equalize_multichannel(color_img)
    print(f"Multi-channel equalized: {eq_color.shape}")

    print("All GPU histogram tests passed!")
