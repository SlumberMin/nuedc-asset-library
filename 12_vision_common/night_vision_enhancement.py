#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
夜视增强模块 (Night Vision Enhancement)

功能：
    - 直方图均衡化（全局/CLAHE局部自适应均衡）
    - 去噪处理（高斯/双边/非局部均值去噪）
    - 红外伪彩色映射（多种伪彩色方案）
    - 自适应亮度调节（基于场景亮度的自动增益）

适用场景：夜间监控、暗光环境视觉增强、红外/夜视系统、电赛夜间巡线
"""

import cv2
import numpy as np
from typing import Tuple, Optional, Dict
from enum import Enum


class DenoiseMethod(Enum):
    """去噪方法"""
    GAUSSIAN = "gaussian"
    BILATERAL = "bilateral"
    NLM = "nlm"          # 非局部均值（效果最好，速度最慢）
    MEDIAN = "median"


class ColorMap(Enum):
    """伪彩色映射方案"""
    IRON = "iron"          # 铁红色（热成像经典配色）
    RAINBOW = "rainbow"    # 彩虹色
    JET = "jet"            # Jet色
    INFERNO = "inferno"    # 暗火色
    THERMAL = "thermal"    # 自定义热成像配色


class NightVisionEnhancement:
    """夜视增强处理系统"""

    # 预设配置
    PRESETS = {
        "mild": {"clahe_clip": 2.0, "denoise_strength": 5, "brightness_gain": 1.3},
        "moderate": {"clahe_clip": 3.0, "denoise_strength": 10, "brightness_gain": 1.6},
        "aggressive": {"clahe_clip": 4.0, "denoise_strength": 15, "brightness_gain": 2.0},
        "extreme": {"clahe_clip": 6.0, "denoise_strength": 20, "brightness_gain": 2.5},
    }

    def __init__(self,
                 clahe_clip: float = 3.0,
                 clahe_grid: Tuple[int, int] = (8, 8),
                 denoise_method: DenoiseMethod = DenoiseMethod.BILATERAL,
                 denoise_strength: int = 10,
                 color_map: ColorMap = ColorMap.IRON,
                 brightness_gain: float = 1.5,
                 auto_exposure: bool = True,
                 target_brightness: float = 120.0):
        """
        初始化夜视增强系统

        参数：
            clahe_clip: CLAHE对比度限制因子（越大增强越强）
            clahe_grid: CLAHE网格大小
            denoise_method: 去噪方法
            denoise_strength: 去噪强度
            color_map: 伪彩色方案
            brightness_gain: 亮度增益（手动模式）
            auto_exposure: 是否自动调节曝光/亮度
            target_brightness: 目标亮度值（自动模式，0-255）
        """
        self.clahe_clip = clahe_clip
        self.clahe_grid = clahe_grid
        self.denoise_method = denoise_method
        self.denoise_strength = denoise_strength
        self.color_map = color_map
        self.brightness_gain = brightness_gain
        self.auto_exposure = auto_exposure
        self.target_brightness = target_brightness

        # 自适应增益平滑
        self._gain_history = []
        self._gain_smooth_window = 5

        # 自定义热成像LUT
        self._thermal_lut = self._create_thermal_lut()

    @staticmethod
    def _create_thermal_lut() -> np.ndarray:
        """创建自定义热成像查找表（黑->紫->红->黄->白）"""
        lut = np.zeros((256, 1, 3), dtype=np.uint8)

        # 分段线性插值
        # 黑 -> 深紫 (0-64)
        for i in range(64):
            t = i / 64.0
            lut[i, 0] = [int(40 * t), 0, int(80 * t)]

        # 深紫 -> 红 (64-128)
        for i in range(64, 128):
            t = (i - 64) / 64.0
            lut[i, 0] = [int(40 + 215 * t), 0, int(80 * (1 - t))]

        # 红 -> 黄 (128-192)
        for i in range(128, 192):
            t = (i - 128) / 64.0
            lut[i, 0] = [255, int(255 * t), 0]

        # 黄 -> 白 (192-255)
        for i in range(192, 256):
            t = (i - 192) / 63.0
            lut[i, 0] = [255, 255, int(255 * t)]

        return lut

    def _estimate_brightness(self, gray: np.ndarray) -> float:
        """估计图像平均亮度"""
        return float(np.mean(gray))

    def _adaptive_brightness(self, gray: np.ndarray) -> np.ndarray:
        """
        自适应亮度调节

        根据当前图像亮度自动计算增益，使输出接近目标亮度
        """
        current_brightness = self._estimate_brightness(gray)

        if current_brightness < 1:
            current_brightness = 1  # 防止除零

        # 计算所需增益
        if self.auto_exposure:
            gain = self.target_brightness / current_brightness
            # 限制增益范围，防止过曝
            gain = np.clip(gain, 1.0, 5.0)
        else:
            gain = self.brightness_gain

        # 增益平滑（减少帧间闪烁）
        self._gain_history.append(gain)
        if len(self._gain_history) > self._gain_smooth_window:
            self._gain_history.pop(0)
        smooth_gain = np.mean(self._gain_history)

        # 应用增益
        enhanced = np.clip(gray.astype(np.float32) * smooth_gain, 0, 255).astype(np.uint8)

        return enhanced

    def _histogram_equalization(self, gray: np.ndarray) -> np.ndarray:
        """
        直方图均衡化增强

        使用CLAHE（对比度受限自适应直方图均衡），
        比全局均衡效果更好，避免过度增强
        """
        clahe = cv2.createCLAHE(
            clipLimit=self.clahe_clip,
            tileGridSize=self.clahe_grid
        )
        equalized = clahe.apply(gray)
        return equalized

    def _denoise(self, image: np.ndarray) -> np.ndarray:
        """
        去噪处理

        参数：
            image: 输入图像（灰度或彩色）

        返回：
            去噪后的图像
        """
        if self.denoise_method == DenoiseMethod.GAUSSIAN:
            ksize = max(3, self.denoise_strength // 2 * 2 + 1)  # 确保奇数
            return cv2.GaussianBlur(image, (ksize, ksize), 0)

        elif self.denoise_method == DenoiseMethod.BILATERAL:
            return cv2.bilateralFilter(
                image,
                d=self.denoise_strength,
                sigmaColor=self.denoise_strength * 10,
                sigmaSpace=self.denoise_strength * 10
            )

        elif self.denoise_method == DenoiseMethod.NLM:
            if len(image.shape) == 2:
                return cv2.fastNlMeansDenoising(
                    image, None,
                    h=self.denoise_strength,
                    templateWindowSize=7,
                    searchWindowSize=21
                )
            else:
                return cv2.fastNlMeansDenoisingColored(
                    image, None,
                    h=self.denoise_strength,
                    hForColorComponents=self.denoise_strength,
                    templateWindowSize=7,
                    searchWindowSize=21
                )

        elif self.denoise_method == DenoiseMethod.MEDIAN:
            ksize = max(3, self.denoise_strength // 2 * 2 + 1)
            if ksize % 2 == 0:
                ksize += 1
            return cv2.medianBlur(image, ksize)

        return image

    def _apply_pseudocolor(self, gray: np.ndarray) -> np.ndarray:
        """
        伪彩色映射

        将灰度图像转换为伪彩色图像，模拟热成像/红外效果
        """
        if self.color_map == ColorMap.THERMAL:
            # 使用自定义热成像LUT
            return cv2.LUT(gray, self._thermal_lut)

        # OpenCV内置伪彩色映射
        colormap_map = {
            ColorMap.IRON: cv2.COLORMAP_JET,      # 铁红近似
            ColorMap.RAINBOW: cv2.COLORMAP_RAINBOW,
            ColorMap.JET: cv2.COLORMAP_JET,
            ColorMap.INFERNO: cv2.COLORMAP_INFERNO,
        }

        cv_cmap = colormap_map.get(self.color_map, cv2.COLORMAP_JET)
        colored = cv2.applyColorMap(gray, cv_cmap)

        if self.color_map == ColorMap.IRON:
            # 铁红效果：通道重排 BGR -> IRON
            b, g, r = cv2.split(colored)
            colored = cv2.merge([r, g, b])  # 反转通道得到暖色调

        return colored

    def enhance_brightness(self, frame: np.ndarray) -> np.ndarray:
        """
        仅增强亮度（不伪彩色）

        参数：
            frame: BGR输入图像

        返回：
            增强后的BGR图像
        """
        # 转HSV，在V通道增强
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # 亮度增强
        v = self._adaptive_brightness(v)

        # CLAHE增强
        v = self._histogram_equalization(v)

        # 合并
        hsv = cv2.merge([h, s, v])
        enhanced = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        # 去噪
        enhanced = self._denoise(enhanced)

        return enhanced

    def enhance_nightvision(self, frame: np.ndarray) -> np.ndarray:
        """
        夜视增强（亮度增强 + 伪彩色）

        参数：
            frame: BGR输入图像

        返回：
            伪彩色夜视增强图像
        """
        # 灰度处理链
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. 自适应亮度
        gray = self._adaptive_brightness(gray)

        # 2. CLAHE直方图均衡
        gray = self._histogram_equalization(gray)

        # 3. 去噪
        gray = self._denoise(gray)

        # 4. 伪彩色映射
        result = self._apply_pseudocolor(gray)

        return result

    def enhance_full(self, frame: np.ndarray) -> Dict[str, np.ndarray]:
        """
        完整增强流水线，返回各阶段结果

        参数：
            frame: BGR输入图像

        返回：
            包含各阶段结果的字典:
            - "original": 原图
            - "brightness": 亮度增强
            - "equalized": 直方图均衡
            - "denoised": 去噪结果
            - "pseudocolor": 伪彩色
            - "final": 最终结果（亮度增强+去噪+伪彩色）
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = {"original": frame.copy()}

        # 阶段1：亮度增强
        bright = self._adaptive_brightness(gray)
        results["brightness"] = cv2.cvtColor(bright, cv2.COLOR_GRAY2BGR)

        # 阶段2：直方图均衡
        equalized = self._histogram_equalization(bright)
        results["equalized"] = cv2.cvtColor(equalized, cv2.COLOR_GRAY2BGR)

        # 阶段3：去噪
        denoised = self._denoise(equalized)
        results["denoised"] = cv2.cvtColor(denoised, cv2.COLOR_GRAY2BGR)

        # 阶段4：伪彩色
        pseudocolor = self._apply_pseudocolor(denoised)
        results["pseudocolor"] = pseudocolor

        # 最终结果
        results["final"] = pseudocolor

        return results

    @classmethod
    def from_preset(cls, preset_name: str, **kwargs) -> 'NightVisionEnhancement':
        """
        从预设配置创建实例

        参数：
            preset_name: 预设名称 ("mild"/"moderate"/"aggressive"/"extreme")
            **kwargs: 覆盖参数

        返回：
            配置好的 NightVisionEnhancement 实例
        """
        if preset_name not in cls.PRESETS:
            raise ValueError(f"未知预设: {preset_name}. 可用: {list(cls.PRESETS.keys())}")

        params = cls.PRESETS[preset_name].copy()
        params.update(kwargs)

        return cls(
            clahe_clip=params.get("clahe_clip", 3.0),
            denoise_strength=params.get("denoise_strength", 10),
            brightness_gain=params.get("brightness_gain", 1.5),
            **{k: v for k, v in params.items()
               if k not in ("clahe_clip", "denoise_strength", "brightness_gain")}
        )

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        处理单帧（一体化接口，默认夜视增强+伪彩色）

        参数：
            frame: BGR格式输入图像

        返回：
            增强后的图像
        """
        return self.enhance_nightvision(frame)


def create_comparison(image_dict: Dict[str, np.ndarray],
                      cell_size: Tuple[int, int] = (320, 240)) -> np.ndarray:
    """
    将多个处理阶段结果拼接为对比图

    参数：
        image_dict: {名称: 图像} 字典
        cell_size: 每个单元格大小

    返回：
        拼接后的对比图
    """
    cw, ch = cell_size
    images = list(image_dict.items())
    n = len(images)

    # 2行排列
    cols = (n + 1) // 2
    canvas = np.zeros((ch * 2, cw * cols, 3), dtype=np.uint8)

    for idx, (name, img) in enumerate(images):
        row = idx // cols
        col = idx % cols

        resized = cv2.resize(img, (cw, ch))
        if len(resized.shape) == 2:
            resized = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)

        y1, y2 = row * ch, (row + 1) * ch
        x1, x2 = col * cw, (col + 1) * cw
        canvas[y1:y2, x1:x2] = resized

        # 标注名称
        cv2.putText(canvas, name, (x1 + 5, y1 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    return canvas


# ======================== 使用示例 ========================

def example_image():
    """单张图片夜视增强示例"""
    img_path = "night_scene.jpg"
    frame = cv2.imread(img_path)

    if frame is None:
        print(f"无法读取图像: {img_path}")
        print("生成模拟暗光图像...")
        # 创建模拟暗光场景
        frame = np.random.randint(5, 40, (480, 640, 3), dtype=np.uint8)
        # 添加一些亮目标（模拟车灯/路灯）
        cv2.circle(frame, (200, 300), 15, (200, 200, 200), -1)
        cv2.circle(frame, (450, 250), 10, (180, 180, 180), -1)
        cv2.rectangle(frame, (300, 350), (340, 450), (150, 150, 150), -1)
        # 添加噪声
        noise = np.random.randint(0, 15, frame.shape, dtype=np.uint8)
        frame = cv2.add(frame, noise)

    # 创建不同增强级别
    mild = NightVisionEnhancement.from_preset("mild")
    moderate = NightVisionEnhancement.from_preset("moderate")
    aggressive = NightVisionEnhancement.from_preset("aggressive")

    # 获取完整处理流水线
    nv = NightVisionEnhancement(
        clahe_clip=3.0,
        denoise_method=DenoiseMethod.BILATERAL,
        denoise_strength=10,
        color_map=ColorMap.THERMAL,
        auto_exposure=True,
        target_brightness=120
    )
    pipeline = nv.enhance_full(frame)

    print("=== 夜视增强结果 ===")
    print(f"原图平均亮度: {np.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)):.1f}")
    print(f"增强后平均亮度: {np.mean(cv2.cvtColor(pipeline['final'], cv2.COLOR_BGR2GRAY)):.1f}")

    # 对比不同预设
    results = {
        "Original": frame,
        "Mild": mild.process_frame(frame),
        "Moderate": moderate.process_frame(frame),
        "Aggressive": aggressive.process_frame(frame),
    }

    comparison = create_comparison(results, cell_size=(320, 240))
    cv2.imshow("Night Vision Comparison", comparison)

    # 显示完整流水线
    pipeline_view = create_comparison(pipeline, cell_size=(260, 200))
    cv2.imshow("Processing Pipeline", pipeline_view)

    print("\n按任意键退出...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def example_camera():
    """摄像头实时夜视增强"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 使用中等预设
    nv = NightVisionEnhancement(
        clahe_clip=3.0,
        denoise_method=DenoiseMethod.BILATERAL,
        denoise_strength=8,
        color_map=ColorMap.THERMAL,
        auto_exposure=True,
        target_brightness=130
    )

    # 可切换的模式
    modes = ["original", "brightness", "pseudocolor"]
    current_mode = 0

    print("夜视增强系统已启动")
    print("  'q' - 退出")
    print("  'm' - 切换显示模式")
    print("  '1-4' - 切换预设 (1=mild, 2=moderate, 3=aggressive, 4=extreme)")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        mode = modes[current_mode]

        if mode == "original":
            result = frame.copy()
            label = "Original"
        elif mode == "brightness":
            result = nv.enhance_brightness(frame)
            label = "Enhanced Brightness"
        else:
            result = nv.enhance_nightvision(frame)
            label = "Night Vision (Pseudocolor)"

        # 显示当前模式
        cv2.putText(result, f"Mode: {label}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 显示亮度信息
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)
        cv2.putText(result, f"Brightness: {brightness:.0f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        cv2.imshow("Night Vision", result)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('m'):
            current_mode = (current_mode + 1) % len(modes)
        elif key == ord('1'):
            nv = NightVisionEnhancement.from_preset("mild", color_map=ColorMap.THERMAL)
            print("预设: mild")
        elif key == ord('2'):
            nv = NightVisionEnhancement.from_preset("moderate", color_map=ColorMap.THERMAL)
            print("预设: moderate")
        elif key == ord('3'):
            nv = NightVisionEnhancement.from_preset("aggressive", color_map=ColorMap.THERMAL)
            print("预设: aggressive")
        elif key == ord('4'):
            nv = NightVisionEnhancement.from_preset("extreme", color_map=ColorMap.THERMAL)
            print("预设: extreme")

    cap.release()
    cv2.destroyAllWindows()


def example_adaptive():
    """自适应亮度调节对比示例"""
    # 模拟不同亮度的场景
    scenes = {
        "极暗": np.random.randint(2, 15, (240, 320, 3), dtype=np.uint8),
        "昏暗": np.random.randint(10, 50, (240, 320, 3), dtype=np.uint8),
        "微光": np.random.randint(30, 80, (240, 320, 3), dtype=np.uint8),
        "正常": np.random.randint(80, 180, (240, 320, 3), dtype=np.uint8),
    }

    nv = NightVisionEnhancement(auto_exposure=True, target_brightness=120)

    results = {}
    for name, scene in scenes.items():
        enhanced = nv.enhance_brightness(scene)
        orig_bright = np.mean(cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY))
        enh_bright = np.mean(cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY))
        results[f"{name}({orig_bright:.0f}->{enh_bright:.0f})"] = enhanced

    comparison = create_comparison(results, cell_size=(320, 240))
    cv2.imshow("Adaptive Brightness", comparison)
    print("自适应亮度对比（按任意键退出）")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--camera":
        example_camera()
    elif len(sys.argv) > 1 and sys.argv[1] == "--adaptive":
        example_adaptive()
    else:
        example_image()
