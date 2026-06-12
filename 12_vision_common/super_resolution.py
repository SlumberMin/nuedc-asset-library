# -*- coding: utf-8 -*-
"""
超分辨率模块 - 双三次插值 + ESPCN轻量级上采样
适用于电赛中低分辨率图像增强、远距离目标细节恢复
"""

import cv2
import numpy as np
import os

# ESPCN模型文件路径（OpenCV DNN模块内置ESPCN支持）
# 下载地址: https://github.com/Saafke/EDSR_Tensorflow/tree/master/models
# 或使用OpenCV自带的超分模型
ESPCN_MODELS = {
    2: 'ESPCN_x2.pb',
    3: 'ESPCN_x3.pb',
    4: 'ESPCN_x4.pb',
}


class SuperResolution:
    """超分辨率上采样器：支持双三次插值和ESPCN神经网络"""

    def __init__(self, scale=4, model_dir='models', method='auto'):
        """
        参数：
            scale: 放大倍数 (2, 3, 4)
            model_dir: ESPCN模型存放目录
            method: 'bicubic'双三次 | 'espcn'神经网络 | 'auto'自动选择
        """
        self.scale = scale
        self.model_dir = model_dir
        self.method = method
        self.net = None
        self._load_model()

    def _load_model(self):
        """加载ESPCN模型"""
        model_file = ESPCN_MODELS.get(self.scale)
        if model_file is None:
            print(f"[警告] 不支持{self.scale}x放大，仅支持2/3/4x")
            return

        model_path = os.path.join(self.model_dir, model_file)
        if os.path.exists(model_path):
            try:
                self.net = cv2.dnn.readNetFromCaffe  # placeholder
                # 实际使用TensorFlow格式
                self.net = cv2.dnn.readNet(model_path)
                print(f"[信息] ESPCN {self.scale}x模型加载成功: {model_path}")
            except Exception as e:
                print(f"[警告] 模型加载失败: {e}，将使用双三次插值")
                self.net = None
        else:
            if self.method == 'espcn':
                print(f"[警告] 模型文件不存在: {model_path}")
                print(f"[提示] 请从GitHub下载ESPCN_x{self.scale}.pb到{self.model_dir}/目录")

    def upscale_bicubic(self, image):
        """双三次插值上采样"""
        h, w = image.shape[:2]
        return cv2.resize(image, (w * self.scale, h * self.scale),
                          interpolation=cv2.INTER_CUBIC)

    def upscale_espcn(self, image):
        """ESPCN神经网络上采样"""
        if self.net is None:
            print("[回退] 使用双三次插值")
            return self.upscale_bicubic(image)

        # ESPCN输入要求：单通道或三通道，归一化到[0,1]
        if len(image.shape) == 3:
            # YCbCr色彩空间：只对Y通道超分
            ycbcr = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
            y_channel = ycbcr[:, :, 0].astype(np.float32) / 255.0
        else:
            y_channel = image.astype(np.float32) / 255.0

        h, w = y_channel.shape

        # DNN前向传播
        blob = cv2.dnn.blobFromImage(y_channel, 1.0, (w, h))
        self.net.setInput(blob)
        output = self.net.forward()

        # 输出处理
        sr_y = output[0, 0] * 255.0
        sr_y = np.clip(sr_y, 0, 255).astype(np.uint8)

        if len(image.shape) == 3:
            # 上采样Cr/Cb通道并合并
            sr_cr = cv2.resize(ycbcr[:, :, 1], (sr_y.shape[1], sr_y.shape[0]),
                               interpolation=cv2.INTER_CUBIC)
            sr_cb = cv2.resize(ycbcr[:, :, 2], (sr_y.shape[1], sr_y.shape[0]),
                               interpolation=cv2.INTER_CUBIC)
            sr_ycbcr = cv2.merge([sr_y, sr_cr, sr_cb])
            result = cv2.cvtColor(sr_ycbcr, cv2.COLOR_YCrCb2BGR)
        else:
            result = sr_y

        return result

    def upscale_enhanced(self, image):
        """
        增强型上采样：双三次 + 锐化 + 细节增强
        即使没有ESPCN模型也能获得较好效果
        """
        # 双三次插值
        upscaled = self.upscale_bicubic(image)

        # 非锐化掩蔽(USM)锐化
        blurred = cv2.GaussianBlur(upscaled, (0, 0), 3)
        sharpened = cv2.addWeighted(upscaled, 1.5, blurred, -0.5, 0)

        # CLAHE对比度增强（在LAB空间的L通道）
        lab = cv2.cvtColor(sharpened, cv2.COLOR_BGR2LAB)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        lab[:, :, 0] = clahe.apply(lab[:, :, 0])
        result = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        return result

    def upscale(self, image):
        """
        统一接口：根据配置选择上采样方法
        """
        if self.method == 'espcn' and self.net is not None:
            return self.upscale_espcn(image)
        elif self.method == 'bicubic':
            return self.upscale_bicubic(image)
        elif self.method == 'auto':
            if self.net is not None:
                return self.upscale_espcn(image)
            else:
                return self.upscale_enhanced(image)
        else:
            return self.upscale_enhanced(image)

    @staticmethod
    def compare_methods(image, scale=4, output_path='sr_comparison.jpg'):
        """对比不同上采样方法的效果"""
        h, w = image.shape[:2]
        target_size = (w * scale, h * scale)

        # 最近邻
        nn = cv2.resize(image, target_size, interpolation=cv2.INTER_NEAREST)
        # 双线性
        bilinear = cv2.resize(image, target_size, interpolation=cv2.INTER_LINEAR)
        # 双三次
        bicubic = cv2.resize(image, target_size, interpolation=cv2.INTER_CUBIC)
        # Lanczos
        lanczos = cv2.resize(image, target_size, interpolation=cv2.INTER_LANCZOS4)

        # 拼接对比图
        top = np.hstack([nn, bilinear])
        bottom = np.hstack([bicubic, lanczos])
        comparison = np.vstack([top, bottom])

        # 添加标签
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(comparison, 'Nearest', (10, 30), font, 1, (0, 255, 0), 2)
        cv2.putText(comparison, 'Bilinear', (w * scale + 10, 30), font, 1, (0, 255, 0), 2)
        cv2.putText(comparison, 'Bicubic', (10, h * scale + 30), font, 1, (0, 255, 0), 2)
        cv2.putText(comparison, 'Lanczos', (w * scale + 10, h * scale + 30), font, 1, (0, 255, 0), 2)

        cv2.imwrite(output_path, comparison)
        print(f"对比图已保存: {output_path}")
        return comparison

    @staticmethod
    def psnr(original, upscaled):
        """计算PSNR（峰值信噪比）评估上采样质量"""
        mse = np.mean((original.astype(float) - upscaled.astype(float)) ** 2)
        if mse == 0:
            return float('inf')
        return 10 * np.log10(255.0 ** 2 / mse)


# ========== 使用示例 ==========
if __name__ == '__main__':
    print("SuperResolution 超分辨率模块")
    print("=" * 40)

    # 创建测试图像
    test_img = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.putText(test_img, 'TEST', (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
    cv2.circle(test_img, (80, 60), 30, (0, 200, 255), -1)
    # 添加细线条测试细节恢复
    for i in range(0, 160, 5):
        cv2.line(test_img, (i, 100), (i + 3, 115), (100, 255, 100), 1)

    sr = SuperResolution(scale=4, method='auto')
    result = sr.upscale(test_img)
    cv2.imwrite("sr_result.jpg", result)
    print(f"原图尺寸: {test_img.shape[1]}x{test_img.shape[0]}")
    print(f"超分尺寸: {result.shape[1]}x{result.shape[0]}")

    # 方法对比
    SuperResolution.compare_methods(test_img, scale=4)
    print("\n编程调用示例：")
    print("  sr = SuperResolution(scale=4)")
    print("  high_res = sr.upscale(low_res_image)")
