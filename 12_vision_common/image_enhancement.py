"""
图像增强模块 - 去雾/去噪/超分/HDR色调映射/自适应直方图均衡
功能：多种图像质量提升算法
依赖：opencv-python, numpy
"""

import cv2
import numpy as np


# ==================== 去雾算法 ====================

class Defogger:
    """图像去雾算法集合"""

    @staticmethod
    def dark_channel_prior(image, patch_size=15, omega=0.95, t_min=0.1):
        """
        暗通道先验去雾（He et al., 2009）
        经典去雾算法，效果优秀
        
        原理：无雾图像的暗通道值接近0，有雾时暗通道值增大
        
        参数:
            image: 有雾的BGR图像
            patch_size: 暗通道计算的patch大小
            omega: 去雾强度 (0~1，越大去雾越强)
            t_min: 透射率下限
            
        返回:
            result: 去雾后的图像
        """
        img = image.astype(np.float64) / 255.0
        h, w, _ = img.shape

        # 1. 计算暗通道：每个像素在RGB三通道的最小值，再取局部最小
        dark_channel = np.min(img, axis=2)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (patch_size, patch_size))
        dark_channel = cv2.erode(dark_channel, kernel)

        # 2. 估计大气光（取暗通道中最亮的0.1%像素）
        num_pixels = max(int(h * w * 0.001), 1)
        flat_dark = dark_channel.ravel()
        indices = np.argsort(flat_dark)[-num_pixels:]
        atmospheric = np.zeros(3)
        for ch in range(3):
            flat_ch = img[:, :, ch].ravel()
            atmospheric[ch] = np.max(flat_ch[indices])

        # 3. 估计透射率
        transmission = 1 - omega * dark_channel / np.max(atmospheric)
        transmission = np.maximum(transmission, t_min)

        # 4. 引导滤波细化透射率
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float64) / 255.0
        transmission = cv2.ximgproc.guidedFilter(
            gray.astype(np.float32),
            transmission.astype(np.float32),
            radius=50, eps=1e-3
        ) if hasattr(cv2, 'ximgproc') else cv2.GaussianBlur(
            transmission.astype(np.float32), (patch_size * 2 + 1, patch_size * 2 + 1), 0)

        # 5. 恢复无雾图像
        transmission = np.expand_dims(transmission, axis=2)
        result = (img - atmospheric) / np.maximum(transmission, t_min) + atmospheric
        result = np.clip(result * 255, 0, 255).astype(np.uint8)

        return result

    @staticmethod
    def simple_defog(image):
        """
        简化去雾：基于直方图拉伸
        适合实时场景
        """
        # 分通道拉伸
        result = np.zeros_like(image)
        for ch in range(3):
            channel = image[:, :, ch]
            low = np.percentile(channel, 1)
            high = np.percentile(channel, 99)
            result[:, :, ch] = np.clip(
                (channel.astype(np.float32) - low) / (high - low + 1e-6) * 255,
                0, 255).astype(np.uint8)
        return result

    @staticmethod
    def white_balance_defog(image):
        """
        白平衡去雾：基于灰度世界假设
        """
        b, g, r = cv2.split(image.astype(np.float32))
        avg_b, avg_g, avg_r = np.mean(b), np.mean(g), np.mean(r)
        avg_gray = (avg_b + avg_g + avg_r) / 3

        b = np.clip(b * avg_gray / (avg_b + 1e-6), 0, 255)
        g = np.clip(g * avg_gray / (avg_g + 1e-6), 0, 255)
        r = np.clip(r * avg_gray / (avg_r + 1e-6), 0, 255)

        return cv2.merge([b, g, r]).astype(np.uint8)


# ==================== 去噪算法 ====================

class Denoiser:
    """图像去噪算法集合"""

    @staticmethod
    def bilateral_denoise(image, d=9, sigma_color=75, sigma_space=75):
        """
        双边滤波去噪：保持边缘的同时去除噪声
        
        参数:
            d: 滤波直径
            sigma_color: 颜色空间高斯标准差
            sigma_space: 坐标空间高斯标准差
        """
        return cv2.bilateralFilter(image, d, sigma_color, sigma_space)

    @staticmethod
    def nlm_denoise(image, h=10, template_window=7, search_window=21):
        """
        非局部均值去噪（NLM）：效果最佳的OpenCV去噪
        
        参数:
            h: 滤波强度
            template_window: 模板窗口大小
            search_window: 搜索窗口大小
        """
        return cv2.fastNlMeansDenoisingColored(image, None, h, h,
                                                template_window, search_window)

    @staticmethod
    def gaussian_denoise(image, kernel_size=5):
        """高斯滤波去噪"""
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

    @staticmethod
    def median_denoise(image, kernel_size=5):
        """中值滤波去噪（对椒盐噪声效果好）"""
        return cv2.medianBlur(image, kernel_size)

    @staticmethod
    def wavelet_denoise(image, sigma=25):
        """
        小波去噪（使用DWT近似）
        利用图像金字塔模拟小波分解去噪
        """
        # 高斯金字塔分解
        levels = 3
        pyramid = [image.astype(np.float32)]
        current = image.copy()

        for _ in range(levels):
            current = cv2.pyrDown(current)
            pyramid.append(current.astype(np.float32))

        # 对每一层去噪
        for i in range(1, len(pyramid)):
            layer = pyramid[i]
            # 对高频分量阈值处理
            blurred = cv2.GaussianBlur(layer, (3, 3), 0)
            detail = layer - blurred
            # 软阈值
            detail = np.sign(detail) * np.maximum(np.abs(detail) - sigma / (i + 1), 0)
            pyramid[i] = blurred + detail

        # 重建
        result = pyramid[-1]
        for i in range(len(pyramid) - 2, -1, -1):
            result = cv2.pyrUp(result, dstsize=(
                pyramid[i].shape[1], pyramid[i].shape[0]))
            result = result + pyramid[i] - cv2.GaussianBlur(result, (3, 3), 0)

        return np.clip(result, 0, 255).astype(np.uint8)

    @staticmethod
    def adaptive_denoise(image, noise_level='auto'):
        """
        自适应去噪：根据噪声水平自动选择参数
        
        参数:
            noise_level: 'low', 'medium', 'high', 'auto'
        """
        if noise_level == 'auto':
            # 估计噪声水平
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # 使用拉普拉斯算子估计噪声
            laplacian = cv2.Laplacian(gray, cv2.CV_64F)
            noise_est = np.std(laplacian) * 0.5
            if noise_est < 5:
                noise_level = 'low'
            elif noise_est < 15:
                noise_level = 'medium'
            else:
                noise_level = 'high'

        params = {
            'low': {'h': 5, 'd': 5, 'sigma': 30},
            'medium': {'h': 10, 'd': 9, 'sigma': 50},
            'high': {'h': 20, 'd': 12, 'sigma': 75},
        }

        p = params.get(noise_level, params['medium'])

        # 轻度噪声用双边，重度用NLM
        if noise_level == 'low':
            return Denoiser.bilateral_denoise(image, d=p['d'],
                                              sigma_color=p['sigma'])
        else:
            return Denoiser.nlm_denoise(image, h=p['h'])


# ==================== 超分辨率 ====================

class SuperResolution:
    """图像超分辨率"""

    @staticmethod
    def bicubic_upscale(image, scale=2):
        """双三次插值放大"""
        h, w = image.shape[:2]
        return cv2.resize(image, (w * scale, h * scale),
                          interpolation=cv2.INTER_CUBIC)

    @staticmethod
    def laplacian_sharpen_upscale(image, scale=2, alpha=1.5):
        """
        拉普拉斯锐化+放大
        先放大再锐化，提升清晰度
        """
        # 放大
        h, w = image.shape[:2]
        upscaled = cv2.resize(image, (w * scale, h * scale),
                              interpolation=cv2.INTER_CUBIC)

        # 拉普拉斯锐化
        laplacian = cv2.Laplacian(upscaled, cv2.CV_64F)
        sharpened = upscaled.astype(np.float64) - alpha * laplacian
        return np.clip(sharpened, 0, 255).astype(np.uint8)

    @staticmethod
    def edsr_upscale(image, scale=2, model_path=None):
        """
        EDSR深度学习超分（需要OpenCV DNN + 模型文件）
        模型下载: https://github.com/Saafke/EDSR_Tensorflow
        """
        if model_path is None:
            print("需要EDSR模型文件，降级到双三次插值")
            return SuperResolution.bicubic_upscale(image, scale)

        try:
            sr = cv2.dnn_superres.DnnSuperResImpl_create()
            sr.readModel(model_path)
            sr.setModel("edsr", scale)
            return sr.upsample(image)
        except Exception as e:
            print(f"EDSR加载失败: {e}，降级到双三次插值")
            return SuperResolution.bicubic_upscale(image, scale)

    @staticmethod
    def detail_enhance_upscale(image, scale=2):
        """
        细节增强放大：放大后增强纹理细节
        """
        h, w = image.shape[:2]
        upscaled = cv2.resize(image, (w * scale, h * scale),
                              interpolation=cv2.INTER_CUBIC)

        # 使用OpenCV的detailEnhance
        enhanced = cv2.detailEnhance(upscaled, sigma_s=10, sigma_r=0.15)
        return enhanced


# ==================== HDR色调映射 ====================

class HDRToneMapper:
    """HDR色调映射"""

    @staticmethod
    def drago_tonemap(image, gamma=1.0, saturation=1.0, bias=0.85):
        """
        Drago色调映射
        
        参数:
            image: LDR图像（将被当作HDR输入处理）
            gamma: 伽马值
            saturation: 饱和度
            bias: 偏置参数
        """
        img = image.astype(np.float32) / 255.0
        tonemap = cv2.createTonemapDrago(gamma, saturation, bias)
        result = tonemap.process(img)
        return np.clip(result * 255, 0, 255).astype(np.uint8)

    @staticmethod
    def reinhard_tonemap(image, gamma=1.0, intensity=0.0,
                         light_adapt=1.0, color_adapt=0.0):
        """
        Reinhard色调映射
        """
        img = image.astype(np.float32) / 255.0
        tonemap = cv2.createTonemapReinhard(gamma, intensity,
                                            light_adapt, color_adapt)
        result = tonemap.process(img)
        return np.clip(result * 255, 0, 255).astype(np.uint8)

    @staticmethod
    def mantiuk_tonemap(image, gamma=1.0, saturation=1.0, scale=0.7):
        """
        Mantiuk色调映射
        """
        img = image.astype(np.float32) / 255.0
        tonemap = cv2.createTonemapMantiuk(gamma, saturation, scale)
        result = tonemap.process(img)
        return np.clip(result * 255, 0, 255).astype(np.uint8)

    @staticmethod
    def simple_tonemap(image, gamma=2.2):
        """
        简单伽马色调映射
        """
        img = image.astype(np.float32) / 255.0
        result = np.power(img, 1.0 / gamma)
        return np.clip(result * 255, 0, 255).astype(np.uint8)


# ==================== 自适应直方图均衡 ====================

class AdaptiveHistogram:
    """自适应直方图均衡化"""

    @staticmethod
    def clahe(image, clip_limit=2.0, grid_size=(8, 8)):
        """
        CLAHE自适应直方图均衡（最常用）
        在LAB空间只均衡亮度通道，保持颜色不失真
        
        参数:
            image: BGR图像
            clip_limit: 对比度限制
            grid_size: 网格大小
        """
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)

        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=grid_size)
        l = clahe.apply(l)

        lab = cv2.merge([l, a, b])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    @staticmethod
    def clahe_adaptive(image, method='auto'):
        """
        自适应CLAHE：自动选择最佳参数
        
        参数:
            method: 'auto' 自动, 'low' 低对比度, 'high' 高对比度
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if method == 'auto':
            # 根据直方图分布自动选择
            hist = cv2.calcHist([gray], [0], None, [256], [0, 256]).ravel()
            hist_norm = hist / hist.sum()
            # 计算信息熵
            entropy = -np.sum(hist_norm * np.log2(hist_norm + 1e-10))
            if entropy < 5:
                method = 'low'
            else:
                method = 'high'

        params = {
            'low': {'clip_limit': 3.0, 'grid_size': (8, 8)},
            'high': {'clip_limit': 1.5, 'grid_size': (16, 16)},
        }
        p = params.get(method, params['high'])
        return AdaptiveHistogram.clahe(image, **p)

    @staticmethod
    def multi_scale_retinex(image, sigma_list=None):
        """
        多尺度Retinex（MSR）
        模拟人眼对光照的自适应
        
        参数:
            image: BGR图像
            sigma_list: 高斯模糊尺度列表
        """
        if sigma_list is None:
            sigma_list = [15, 80, 250]

        img = image.astype(np.float64) + 1.0  # 避免log(0)

        result = np.zeros_like(img)
        for sigma in sigma_list:
            blurred = cv2.GaussianBlur(img, (0, 0), sigma)
            result += np.log(img) - np.log(blurred + 1.0)

        result /= len(sigma_list)

        # 归一化到0~255
        for ch in range(3):
            channel = result[:, :, ch]
            min_val, max_val = channel.min(), channel.max()
            result[:, :, ch] = (channel - min_val) / (max_val - min_val + 1e-6) * 255

        return np.clip(result, 0, 255).astype(np.uint8)

    @staticmethod
    def local_contrast_enhancement(image, block_size=32, gain=2.0):
        """
        局部对比度增强
        """
        h, w = image.shape[:2]
        result = image.copy().astype(np.float32)

        for y in range(0, h, block_size):
            for x in range(0, w, block_size):
                y2 = min(y + block_size, h)
                x2 = min(x + block_size, w)
                block = result[y:y2, x:x2]

                mean = np.mean(block, axis=(0, 1), keepdims=True)
                result[y:y2, x:x2] = (block - mean) * gain + mean

        return np.clip(result, 0, 255).astype(np.uint8)


# ==================== 综合增强管线 ====================

class ImageEnhancer:
    """
    综合图像增强器
    组合多种算法的增强管线
    """

    def __init__(self):
        self.defogger = Defogger()
        self.denoiser = Denoiser()
        self.sr = SuperResolution()
        self.tone_mapper = HDRToneMapper()
        self.hist = AdaptiveHistogram()

    def enhance_pipeline(self, image, operations=None):
        """
        增强管线：按顺序执行多种增强操作
        
        参数:
            image: 输入图像
            operations: 操作列表，如 ['clahe', 'denoise', 'sharpen']
                        可选: 'clahe', 'denoise', 'sharpen', 'defog', 'tone'
            
        返回:
            result: 增强后的图像
        """
        if operations is None:
            operations = ['clahe', 'denoise', 'sharpen']

        result = image.copy()

        for op in operations:
            if op == 'clahe':
                result = self.hist.clahe(result)
            elif op == 'denoise':
                result = self.denoiser.adaptive_denoise(result, 'low')
            elif op == 'sharpen':
                kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
                result = cv2.filter2D(result, -1, kernel)
            elif op == 'defog':
                result = self.defogger.simple_defog(result)
            elif op == 'tone':
                result = self.tone_mapper.simple_tonemap(result, gamma=1.8)

        return result

    def auto_enhance(self, image):
        """
        全自动增强：自动判断并应用合适的增强操作
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 评估图像质量
        mean_brightness = np.mean(gray)
        contrast = np.std(gray)
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()  # 清晰度

        operations = []

        # 暗图像→CLAHE
        if mean_brightness < 80:
            operations.append('clahe')
            operations.append('defog')

        # 低对比度→CLAHE
        if contrast < 40:
            if 'clahe' not in operations:
                operations.append('clahe')

        # 模糊→锐化
        if laplacian_var < 100:
            operations.append('sharpen')

        # 噪点多→去噪
        noise = np.std(cv2.Laplacian(gray, cv2.CV_64F))
        if noise > 20:
            operations.append('denoise')

        if not operations:
            operations = ['clahe', 'sharpen']

        print(f"自动增强: 亮度={mean_brightness:.0f}, 对比度={contrast:.0f}, "
              f"清晰度={laplacian_var:.0f}, 噪声={noise:.0f}")
        print(f"应用操作: {operations}")

        return self.enhance_pipeline(image, operations)


# ==================== 使用示例 ====================

def example_defog():
    """去雾示例"""
    image = cv2.imread("foggy.jpg")
    if image is None:
        # 模拟雾天图像
        image = np.random.randint(150, 220, (300, 400, 3), dtype=np.uint8)
        image = cv2.GaussianBlur(image, (15, 15), 5)

    # 暗通道先验去雾
    result_dcp = Defogger.dark_channel_prior(image)
    # 简单去雾
    result_simple = Defogger.simple_defog(image)

    cv2.imshow("Foggy", image)
    cv2.imshow("DCP Defog", result_dcp)
    cv2.imshow("Simple Defog", result_simple)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def example_denoise():
    """去噪示例"""
    image = cv2.imread("noisy.jpg")
    if image is None:
        # 创建带噪声图像
        clean = np.ones((200, 300, 3), dtype=np.uint8) * 128
        noise = np.random.normal(0, 25, clean.shape)
        image = np.clip(clean.astype(float) + noise, 0, 255).astype(np.uint8)

    # 不同去噪方法
    result_bilateral = Denoiser.bilateral_denoise(image)
    result_nlm = Denoiser.nlm_denoise(image)
    result_adaptive = Denoiser.adaptive_denoise(image)

    cv2.imshow("Noisy", image)
    cv2.imshow("Bilateral", result_bilateral)
    cv2.imshow("NLM", result_nlm)
    cv2.imshow("Adaptive", result_adaptive)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def example_clahe():
    """CLAHE自适应直方图均衡示例"""
    image = cv2.imread("low_contrast.jpg")
    if image is None:
        image = np.ones((200, 300, 3), dtype=np.uint8) * 100
        cv2.putText(image, "Low Contrast", (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (120, 120, 120), 2)

    # 普通直方图均衡
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hist_eq = cv2.equalizeHist(gray)

    # CLAHE
    clahe_result = AdaptiveHistogram.clahe(image)

    # MSR
    msr_result = AdaptiveHistogram.multi_scale_retinex(image)

    cv2.imshow("Original", image)
    cv2.imshow("CLAHE", clahe_result)
    cv2.imshow("MSR", msr_result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def example_auto_enhance():
    """全自动增强示例"""
    image = cv2.imread("bad_quality.jpg")
    if image is None:
        image = np.ones((200, 300, 3), dtype=np.uint8) * 80

    enhancer = ImageEnhancer()
    result = enhancer.auto_enhance(image)

    cv2.imshow("Original", image)
    cv2.imshow("Enhanced", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print("=== 图像增强模块 ===")
    print("功能:")
    print("  - 去雾: 暗通道先验 / 直方图拉伸 / 白平衡")
    print("  - 去噪: 双边/NLM/高斯/中值/小波/自适应")
    print("  - 超分: 双三次/锐化放大/EDSR/细节增强")
    print("  - HDR: Drago/Reinhard/Mantiuk/伽马映射")
    print("  - 均衡: CLAHE/自适应CLAHE/MSR/局部对比度")
    print("  - 综合: 自动增强管线")
    example_auto_enhance()
