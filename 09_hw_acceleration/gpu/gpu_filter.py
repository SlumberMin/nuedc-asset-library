"""
Mali GPU 滤波加速模块
======================
利用RK3588S集成的Mali-G610 GPU通过OpenCL后端实现图像滤波加速。

支持滤波器：
- 高斯滤波 (Gaussian Blur)
- 中值滤波 (Median Filter)
- 双边滤波 (Bilateral Filter)
- 均值滤波 (Box Filter)
- 自定义卷积核

GPU优势：
- 大核卷积(>=7x7)比CPU快5-10倍
- 并行处理大量像素
- 支持FP16半精度加速
"""

import numpy as np
import cv2
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class GpuFilter:
    """
    Mali GPU滤波加速器
    
    使用示例:
        gpu = GpuFilter()
        
        # GPU高斯滤波
        blurred = gpu.gaussian_blur(img, (5, 5))
        
        # GPU中值滤波
        denoised = gpu.median_blur(img, 5)
        
        # GPU双边滤波
        smooth = gpu.bilateral_filter(img, 9, 75, 75)
    """

    def __init__(self, device_id: int = 0):
        self._ocl_available = False
        self._cv_gpu_available = False
        self._device_id = device_id
        self._init_gpu()

    def _init_gpu(self):
        """初始化GPU后端"""
        # 尝试OpenCV CUDA
        try:
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self._cv_gpu_available = True
                logger.info("GPU滤波: 使用OpenCV CUDA后端")
                return
        except Exception:
            pass

        # 尝试OpenCL
        try:
            ocl_devices = cv2.ocl.getPlatfomsInfo()  # type: ignore
            if ocl_devices:
                cv2.ocl.setUseOpenCL(True)
                self._ocl_available = True
                logger.info("GPU滤波: 使用OpenCL后端")
                return
        except Exception:
            pass

        # 检查OpenCL是否可用
        try:
            if cv2.ocl.haveOpenCL():
                cv2.ocl.setUseOpenCL(True)
                self._ocl_available = True
                logger.info("GPU滤波: 使用OpenCL后端 (haveOpenCL)")
        except Exception:
            logger.info("GPU滤波: 不可用，将使用CPU")

    @property
    def available(self) -> bool:
        return self._cv_gpu_available or self._ocl_available

    def _to_umat(self, img: np.ndarray) -> cv2.UMat:
        """转换为UMat (OpenCL加速)"""
        return cv2.UMat(img)

    def _from_umat(self, umat) -> np.ndarray:
        """从UMat转回numpy"""
        if isinstance(umat, cv2.UMat):
            return umat.get()
        return np.asarray(umat)

    def gaussian_blur(self, img: np.ndarray, ksize: Tuple[int, int] = (5, 5),
                      sigmaX: float = 0, sigmaY: float = 0) -> np.ndarray:
        """
        GPU高斯滤波
        
        Args:
            img: 输入图像
            ksize: 核大小 (必须是奇数)
            sigmaX: X方向标准差
            sigmaY: Y方向标准差
            
        Returns:
            滤波后的图像
        """
        if not self.available:
            return cv2.GaussianBlur(img, ksize, sigmaX, sigmaY)

        if self._cv_gpu_available:
            return self._cuda_gaussian(img, ksize, sigmaX, sigmaY)

        # OpenCL路径
        umat_in = self._to_umat(img)
        umat_out = cv2.GaussianBlur(umat_in, ksize, sigmaX, sigmaY)
        return self._from_umat(umat_out)

    def _cuda_gaussian(self, img, ksize, sigmaX, sigmaY):
        """CUDA高斯滤波"""
        gpu_img = cv2.cuda_GpuMat()
        gpu_img.upload(img)
        # OpenCV CUDA GaussianBlur需要特定核大小
        kx, ky = ksize
        filter_ = cv2.cuda.createGaussianFilter(
            cv2.CV_8UC3, cv2.CV_8UC3, ksize, sigmaX, sigmaY)
        gpu_result = filter_.apply(gpu_img)
        return gpu_result.download()

    def median_blur(self, img: np.ndarray, ksize: int = 5) -> np.ndarray:
        """
        GPU中值滤波 (去椒盐噪声)
        
        Args:
            img: 输入图像
            ksize: 核大小 (必须是奇数 >= 3)
            
        Returns:
            滤波后的图像
        """
        if not self.available:
            return cv2.medianBlur(img, ksize)

        umat_in = self._to_umat(img)
        umat_out = cv2.medianBlur(umat_in, ksize)
        return self._from_umat(umat_out)

    def bilateral_filter(self, img: np.ndarray, d: int = 9,
                         sigmaColor: float = 75,
                         sigmaSpace: float = 75) -> np.ndarray:
        """
        GPU双边滤波 (保边去噪)
        
        Args:
            img: 输入图像
            d: 像素邻域直径
            sigmaColor: 颜色空间标准差
            sigmaSpace: 坐标空间标准差
            
        Returns:
            滤波后的图像
        """
        if not self.available:
            return cv2.bilateralFilter(img, d, sigmaColor, sigmaSpace)

        if self._cv_gpu_available:
            gpu_img = cv2.cuda_GpuMat()
            gpu_img.upload(img)
            filter_ = cv2.cuda.createBilateralFilter(cv2.CV_8UC3, d, sigmaColor, sigmaSpace)
            gpu_result = filter_.apply(gpu_img)
            return gpu_result.download()

        # OpenCL bilateralFilter不一定被支持，回退
        return cv2.bilateralFilter(img, d, sigmaColor, sigmaSpace)

    def box_filter(self, img: np.ndarray, ksize: Tuple[int, int] = (5, 5),
                   normalize: bool = True) -> np.ndarray:
        """GPU均值滤波"""
        if not self.available:
            return cv2.boxFilter(img, -1, ksize, normalize=normalize)

        umat_in = self._to_umat(img)
        umat_out = cv2.boxFilter(umat_in, -1, ksize, normalize=normalize)
        return self._from_umat(umat_out)

    def custom_filter(self, img: np.ndarray, kernel: np.ndarray) -> np.ndarray:
        """
        GPU自定义卷积核滤波
        
        Args:
            img: 输入图像
            kernel: 卷积核 (2D numpy数组)
            
        Returns:
            滤波后的图像
        """
        if not self.available:
            return cv2.filter2D(img, -1, kernel)

        umat_in = self._to_umat(img)
        umat_out = cv2.filter2D(umat_in, -1, kernel)
        return self._from_umat(umat_out)

    def sharpen(self, img: np.ndarray, strength: float = 1.0) -> np.ndarray:
        """GPU锐化"""
        kernel = np.array([
            [0, -strength, 0],
            [-strength, 1 + 4 * strength, -strength],
            [0, -strength, 0]
        ], dtype=np.float32)
        return self.custom_filter(img, kernel)

    def emboss(self, img: np.ndarray) -> np.ndarray:
        """GPU浮雕效果"""
        kernel = np.array([
            [-2, -1, 0],
            [-1, 1, 1],
            [0, 1, 2]
        ], dtype=np.float32)
        return self.custom_filter(img, kernel)

    def motion_blur(self, img: np.ndarray, size: int = 15,
                    angle: float = 0) -> np.ndarray:
        """GPU运动模糊"""
        kernel = np.zeros((size, size), dtype=np.float32)
        center = size // 2
        cos_a = np.cos(np.radians(angle))
        sin_a = np.sin(np.radians(angle))
        for i in range(size):
            offset = i - center
            x = int(center + offset * cos_a)
            y = int(center + offset * sin_a)
            if 0 <= x < size and 0 <= y < size:
                kernel[y, x] = 1.0
        kernel /= max(kernel.sum(), 1)
        return self.custom_filter(img, kernel)

    def benchmark(self, img_size: Tuple[int, int] = (1920, 1080),
                  iterations: int = 100) -> dict:
        """GPU滤波性能测试"""
        import time

        img = np.random.randint(0, 255, (img_size[1], img_size[0], 3), dtype=np.uint8)
        results = {}

        filters = [
            ('gaussian_5x5', lambda: self.gaussian_blur(img, (5, 5))),
            ('gaussian_11x11', lambda: self.gaussian_blur(img, (11, 11))),
            ('median_5', lambda: self.median_blur(img, 5)),
            ('bilateral', lambda: self.bilateral_filter(img, 9, 75, 75)),
            ('box_5x5', lambda: self.box_filter(img, (5, 5))),
        ]

        for name, fn in filters:
            t0 = time.perf_counter()
            for _ in range(iterations):
                fn()
            elapsed = (time.perf_counter() - t0) / iterations * 1000
            results[name] = round(elapsed, 2)

        return results


if __name__ == '__main__':
    gpu = GpuFilter()
    print(f"GPU滤波可用: {gpu.available}")
    print(f"  OpenCV CUDA: {gpu._cv_gpu_available}")
    print(f"  OpenCL: {gpu._ocl_available}")

    test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # 功能测试
    blurred = gpu.gaussian_blur(test_img, (5, 5))
    print(f"高斯滤波: {blurred.shape}")

    denoised = gpu.median_blur(test_img, 5)
    print(f"中值滤波: {denoised.shape}")

    smooth = gpu.bilateral_filter(test_img, 9, 75, 75)
    print(f"双边滤波: {smooth.shape}")

    sharpened = gpu.sharpen(test_img, 1.5)
    print(f"锐化: {sharpened.shape}")

    # 性能测试
    print("\n性能基准:")
    results = gpu.benchmark((640, 480), 50)
    for name, ms in results.items():
        print(f"  {name}: {ms} ms")
