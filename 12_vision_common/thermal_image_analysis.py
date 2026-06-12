#!/usr/bin/env python3
"""
热成像分析模块 - 温度映射 + 热点检测 + 伪彩色
适用于电赛热成像检测、温度监控、故障诊断等任务
依赖: numpy, opencv-python
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List, Dict


class ThermalImageAnalyzer:
    """热成像分析：温度映射、热点检测、伪彩色增强"""

    # 常用伪彩色映射
    COLORMAPS = {
        'ironbow': cv2.COLORMAP_JET,
        'rainbow': cv2.COLORMAP_RAINBOW,
        'white_hot': cv2.COLORMAP_BONE,
        'black_hot': cv2.COLORMAP_BONE,  # 反转
        'inferno': cv2.COLORMAP_INFERNO,
        'hot': cv2.COLORMAP_HOT,
        'turbo': cv2.COLORMAP_TURBO,
    }

    def __init__(self, temp_min: float = 20.0, temp_max: float = 50.0,
                 bit_depth: int = 16):
        """
        初始化热成像分析器
        Args:
            temp_min: 最低温度（摄氏度）
            temp_max: 最高温度（摄氏度）
            bit_depth: 热成像位深度（8或16）
        """
        self.temp_min = temp_min
        self.temp_max = temp_max
        self.bit_depth = bit_depth
        self.max_raw = (1 << bit_depth) - 1

    def raw_to_temperature(self, raw_img: np.ndarray) -> np.ndarray:
        """
        原始热成像数据转温度
        Args:
            raw_img: 原始热图 (H,W), uint8或uint16
        Returns:
            温度图 (H,W), float, 单位摄氏度
        """
        raw = raw_img.astype(np.float64)
        if raw_img.dtype == np.uint16:
            raw /= 65535.0
        elif raw_img.dtype == np.uint8:
            raw /= 255.0

        temperature = self.temp_min + raw * (self.temp_max - self.temp_min)
        return temperature

    def temperature_to_raw(self, temperature: np.ndarray) -> np.ndarray:
        """温度转原始值"""
        raw = (temperature - self.temp_min) / (self.temp_max - self.temp_min)
        raw = np.clip(raw, 0, 1)

        if self.bit_depth == 16:
            return (raw * 65535).astype(np.uint16)
        else:
            return (raw * 255).astype(np.uint8)

    def apply_pseudocolor(self, raw_img: np.ndarray,
                          colormap: str = 'ironbow',
                          invert: bool = False) -> np.ndarray:
        """
        应用伪彩色映射
        Args:
            raw_img: 原始热图 (H,W)
            colormap: 伪彩色类型
            invert: 是否反转（用于black_hot效果）
        Returns:
            伪彩色图 (H,W,3) BGR
        """
        # 归一化到0-255
        if raw_img.dtype == np.uint16:
            normalized = (raw_img / 256).astype(np.uint8)
        else:
            normalized = raw_img.copy()

        if invert:
            normalized = 255 - normalized

        cm = self.COLORMAPS.get(colormap, cv2.COLORMAP_JET)
        colored = cv2.applyColorMap(normalized, cm)
        return colored

    def detect_hotspots(self, temperature: np.ndarray,
                        threshold_temp: Optional[float] = None,
                        threshold_method: str = 'absolute',
                        n_std: float = 2.0) -> Dict:
        """
        检测热点区域
        Args:
            temperature: 温度图 (H,W)
            threshold_temp: 温度阈值（absolute模式下使用）
            threshold_method: 阈值方法 ('absolute', 'relative', 'adaptive')
            n_std: relative模式下高于均值N个标准差
        Returns:
            {'mask': 热点掩码, 'contours': 轮廓, 'bboxes': 边界框,
             'stats': 统计信息}
        """
        if threshold_method == 'absolute':
            if threshold_temp is None:
                threshold_temp = self.temp_max * 0.8
            mask = (temperature >= threshold_temp).astype(np.uint8) * 255

        elif threshold_method == 'relative':
            mean_t = temperature.mean()
            std_t = temperature.std()
            threshold_temp = mean_t + n_std * std_t
            mask = (temperature >= threshold_temp).astype(np.uint8) * 255

        elif threshold_method == 'adaptive':
            temp_u8 = ((temperature - temperature.min()) /
                       (temperature.max() - temperature.min()) * 255).astype(np.uint8)
            mask = cv2.adaptiveThreshold(
                temp_u8, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 11, -2
            )
        else:
            raise ValueError(f"未知阈值方法: {threshold_method}")

        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # 分析每个热点
        bboxes = []
        stats = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 20:  # 过滤小区域
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            roi_temp = temperature[y:y + h, x:x + w]

            bboxes.append((x, y, w, h))
            stats.append({
                'center': (x + w // 2, y + h // 2),
                'area': area,
                'bbox': (x, y, w, h),
                'temp_min': float(roi_temp.min()),
                'temp_max': float(roi_temp.max()),
                'temp_mean': float(roi_temp.mean()),
                'temp_std': float(roi_temp.std()),
            })

        return {
            'mask': mask,
            'contours': contours,
            'bboxes': bboxes,
            'stats': stats,
            'threshold': threshold_temp
        }

    def analyze_temperature_distribution(self, temperature: np.ndarray) -> Dict:
        """
        温度分布分析
        Args:
            temperature: 温度图 (H,W)
        Returns:
            统计信息字典
        """
        valid = temperature[np.isfinite(temperature)]

        # 直方图
        hist, bin_edges = np.histogram(valid, bins=50)
        bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2

        return {
            'min': float(valid.min()),
            'max': float(valid.max()),
            'mean': float(valid.mean()),
            'median': float(np.median(valid)),
            'std': float(valid.std()),
            'percentile_5': float(np.percentile(valid, 5)),
            'percentile_95': float(np.percentile(valid, 95)),
            'histogram': hist,
            'hist_bins': bin_centers,
        }

    def temperature_gradient(self, temperature: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算温度梯度（温度变化率）
        Args:
            temperature: 温度图
        Returns:
            (梯度幅值, 梯度方向)
        """
        grad_x = cv2.Sobel(temperature, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(temperature, cv2.CV_64F, 0, 1, ksize=3)

        magnitude = np.sqrt(grad_x ** 2 + grad_y ** 2)
        direction = np.arctan2(grad_y, grad_x)

        return magnitude, direction

    def thermal_roi_analysis(self, temperature: np.ndarray,
                             roi: Tuple[int, int, int, int]) -> Dict:
        """
        感兴趣区域温度分析
        Args:
            temperature: 温度图
            roi: (x, y, w, h)
        Returns:
            区域温度分析结果
        """
        x, y, w, h = roi
        roi_temp = temperature[y:y + h, x:x + w]

        grad_mag, grad_dir = self.temperature_gradient(roi_temp)

        return {
            'roi': roi,
            'temp_min': float(roi_temp.min()),
            'temp_max': float(roi_temp.max()),
            'temp_mean': float(roi_temp.mean()),
            'temp_std': float(roi_temp.std()),
            'gradient_mean': float(grad_mag.mean()),
            'gradient_max': float(grad_mag.max()),
            'n_hot_pixels': int((roi_temp > self.temp_max * 0.9).sum()),
        }

    def visualize_analysis(self, raw_img: np.ndarray,
                           temperature: np.ndarray,
                           hotspots: Dict,
                           colormap: str = 'ironbow') -> np.ndarray:
        """
        可视化分析结果
        Args:
            raw_img: 原始热图
            temperature: 温度图
            hotspots: 热点检测结果
            colormap: 伪彩色类型
        Returns:
            可视化图像
        """
        # 伪彩色底图
        vis = self.apply_pseudocolor(raw_img, colormap)

        # 绘制热点框
        for stat in hotspots['stats']:
            x, y, w, h = stat['bbox']
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 0, 255), 2)
            label = f"{stat['temp_max']:.1f}C"
            cv2.putText(vis, label, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 添加温度标尺
        h, w = vis.shape[:2]
        scale_bar = np.linspace(self.temp_max, self.temp_min, h).reshape(-1, 1)
        scale_bar = np.tile(scale_bar, (1, 30))
        scale_raw = self.temperature_to_raw(scale_bar)
        scale_color = self.apply_pseudocolor(scale_raw, colormap)
        vis[:, w - 35:w - 5] = scale_color

        return vis

    def thermal_difference(self, temp1: np.ndarray,
                           temp2: np.ndarray) -> np.ndarray:
        """
        两帧热图差异分析
        Args:
            temp1, temp2: 温度图
        Returns:
            温度差图
        """
        return temp1 - temp2

    def emissivity_correction(self, temperature: np.ndarray,
                              emissivity: float = 0.95) -> np.ndarray:
        """
        发射率校正
        Args:
            temperature: 原始温度读数
            emissivity: 物体发射率 (0~1)
        Returns:
            校正后的温度
        """
        # 简化的发射率校正公式
        # T_real = T_measured / emissivity^(1/4)
        corrected = temperature / (emissivity ** 0.25)
        return corrected


if __name__ == '__main__':
    # ==================== 使用示例 ====================
    print("=== 热成像分析模块使用示例 ===\n")

    # 1. 创建模拟热成像数据
    h, w = 240, 320
    # 基础温度场（背景约25度）
    temp_field = np.ones((h, w), dtype=np.float64) * 25.0

    # 添加热点（模拟故障发热）
    # 热点1: 电路板上某元件过热
    cv2.circle(temp_field, (100, 120), 15, 45.0, -1)
    # 热点2: 电机发热
    cv2.ellipse(temp_field, (220, 80), (30, 20), 0, 0, 360, 42.0, -1)
    # 热点3: 电源模块
    cv2.rectangle(temp_field, (50, 180), (120, 220), 38.0, -1)

    # 添加温度梯度和噪声
    gradient = np.linspace(0, 5, w)[None, :].repeat(h, axis=0)
    temp_field += gradient
    noise = np.random.normal(0, 0.5, (h, w))
    temp_field += noise

    # 转为16位原始数据
    analyzer = ThermalImageAnalyzer(temp_min=20.0, temp_max=50.0)
    raw_16bit = analyzer.temperature_to_raw(temp_field)

    print(f"热图尺寸: {raw_16bit.shape}, 数据类型: {raw_16bit.dtype}")

    # 2. 原始数据转温度
    temperature = analyzer.raw_to_temperature(raw_16bit)
    print(f"温度范围: {temperature.min():.1f}°C ~ {temperature.max():.1f}°C")

    # 3. 温度分布分析
    dist = analyzer.analyze_temperature_distribution(temperature)
    print(f"\n温度分布:")
    print(f"  均值: {dist['mean']:.1f}°C, 中位数: {dist['median']:.1f}°C")
    print(f"  标准差: {dist['std']:.1f}°C")
    print(f"  5%分位: {dist['percentile_5']:.1f}°C, 95%分位: {dist['percentile_95']:.1f}°C")

    # 4. 热点检测
    print(f"\n热点检测:")
    for method in ['absolute', 'relative']:
        hotspots = analyzer.detect_hotspots(temperature,
                                            threshold_method=method)
        print(f"  {method}方法: 检测到{len(hotspots['stats'])}个热点, "
              f"阈值={hotspots['threshold']:.1f}°C")
        for i, stat in enumerate(hotspots['stats']):
            print(f"    热点{i}: 中心{stat['center']}, "
                  f"温度{stat['temp_min']:.1f}~{stat['temp_max']:.1f}°C, "
                  f"面积{stat['area']:.0f}像素")

    # 5. 温度梯度分析
    grad_mag, grad_dir = analyzer.temperature_gradient(temperature)
    print(f"\n温度梯度:")
    print(f"  最大梯度: {grad_mag.max():.2f}°C/像素")
    print(f"  平均梯度: {grad_mag.mean():.2f}°C/像素")

    # 6. ROI分析
    roi_result = analyzer.thermal_roi_analysis(temperature, (80, 100, 60, 60))
    print(f"\nROI分析 (80,100,60,60):")
    print(f"  温度: {roi_result['temp_min']:.1f}~{roi_result['temp_max']:.1f}°C")
    print(f"  平均梯度: {roi_result['gradient_mean']:.2f}")

    # 7. 伪彩色可视化
    print(f"\n伪彩色映射:")
    for cmap in ['ironbow', 'rainbow', 'white_hot', 'inferno', 'hot']:
        colored = analyzer.apply_pseudocolor(raw_16bit, cmap)
        print(f"  {cmap}: 输出尺寸{colored.shape}")

    # 8. 发射率校正
    corrected = analyzer.emissivity_correction(temperature, emissivity=0.85)
    print(f"\n发射率校正 (ε=0.85):")
    print(f"  校正前均温: {temperature.mean():.1f}°C")
    print(f"  校正后均温: {corrected.mean():.1f}°C")

    # 9. 热差异分析
    temp2 = temp_field + np.random.normal(0, 0.3, (h, w))
    diff = analyzer.thermal_difference(temperature, temp2)
    print(f"\n帧间温差: 均值={np.abs(diff).mean():.3f}°C, 最大={np.abs(diff).max():.2f}°C")

    print("\n示例完成！")
