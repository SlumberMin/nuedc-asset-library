"""
图像质量评估模块 - 清晰度/曝光/噪声/色彩/综合评分
功能：基于多种指标对图像质量进行全方位评估
依赖：opencv-python, numpy
适用：电赛中图像预处理质量筛选、自动曝光反馈等场景
"""

import cv2
import numpy as np
from collections import OrderedDict


class ImageQualityAssessor:
    """
    图像质量评估器
    提供清晰度、曝光、噪声、色彩等多维度评估
    """

    def __init__(self):
        """初始化评估器"""
        # 评分权重
        self.weights = {
            'sharpness': 0.30,
            'exposure': 0.25,
            'noise': 0.20,
            'color': 0.15,
            'contrast': 0.10,
        }

    def assess(self, frame):
        """
        全面评估图像质量
        Args:
            frame: BGR彩色图像
        Returns:
            report: 评估报告字典
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        report = OrderedDict()

        # 各维度评分
        report['sharpness'] = self.assess_sharpness(gray)
        report['exposure'] = self.assess_exposure(gray)
        report['noise'] = self.assess_noise(gray)
        report['color'] = self.assess_color(frame)
        report['contrast'] = self.assess_contrast(gray)

        # 综合评分
        total_score = 0.0
        for dim, info in report.items():
            w = self.weights.get(dim, 0)
            total_score += info['score'] * w
        report['overall'] = {'score': total_score, 'level': self._score_level(total_score)}

        return report

    def assess_sharpness(self, gray):
        """
        清晰度评估（基于拉普拉斯方差和梯度能量）
        Args:
            gray: 灰度图
        Returns:
            dict: {'score': 0~100, 'laplacian_var': float, 'gradient_energy': float}
        """
        # 方法1: 拉普拉斯方差（经典清晰度指标）
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        lap_var = laplacian.var()

        # 方法2: Sobel梯度能量
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient_energy = np.mean(sobel_x**2 + sobel_y**2)

        # 方法3: Tenengrad（Sobel梯度幅值均值）
        gradient_mag = np.sqrt(sobel_x**2 + sobel_y**2)
        tenengrad = np.mean(gradient_mag)

        # 映射到 0~100 分（阈值基于经验值）
        # 拉普拉斯方差: <50模糊, 50~500清晰, >500非常清晰
        score_lap = np.clip(lap_var / 5.0, 0, 100)
        # 梯度能量: 归一化
        score_grad = np.clip(gradient_energy / 50.0, 0, 100)

        score = score_lap * 0.6 + score_grad * 0.4

        return {
            'score': round(float(np.clip(score, 0, 100)), 1),
            'laplacian_var': round(float(lap_var), 2),
            'gradient_energy': round(float(gradient_energy), 2),
            'tenengrad': round(float(tenengrad), 2),
            'level': self._score_level(score),
        }

    def assess_exposure(self, gray):
        """
        曝光评估（亮度分布和直方图分析）
        Args:
            gray: 灰度图
        Returns:
            dict: {'score': 0~100, 'mean_brightness': float, 'histogram_balance': float}
        """
        mean_brightness = np.mean(gray)
        std_brightness = np.std(gray)

        # 理想亮度范围 80~180
        ideal_mean = 128
        deviation = abs(mean_brightness - ideal_mean)

        # 亮度得分
        brightness_score = max(0, 100 - deviation * 1.0)

        # 过曝/欠曝比例
        overexposed = np.sum(gray > 245) / gray.size
        underexposed = np.sum(gray < 10) / gray.size
        clipping_penalty = (overexposed + underexposed) * 500  # 放大惩罚

        # 直方图均匀度（信息量）
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256]).flatten()
        hist = hist / hist.sum()
        # 有效灰度级数（非零bin数）
        effective_bins = np.sum(hist > 0.001)
        histogram_score = (effective_bins / 256) * 100

        score = brightness_score * 0.5 + histogram_score * 0.3 - clipping_penalty * 0.2

        return {
            'score': round(float(np.clip(score, 0, 100)), 1),
            'mean_brightness': round(float(mean_brightness), 1),
            'std_brightness': round(float(std_brightness), 1),
            'overexposed_pct': round(float(overexposed * 100), 2),
            'underexposed_pct': round(float(underexposed * 100), 2),
            'effective_bins': int(effective_bins),
            'level': self._score_level(score),
        }

    def assess_noise(self, gray):
        """
        噪声评估（基于高斯差分和平坦区域分析）
        Args:
            gray: 灰度图
        Returns:
            dict: {'score': 0~100, 'noise_level': float, 'snr': float}
        """
        # 方法1: 高频分量估计（高斯差分）
        blur = cv2.GaussianBlur(gray, (5, 5), 1.0)
        noise = cv2.absdiff(gray, blur)
        noise_level = np.mean(noise)

        # 方法2: 平坦区域的局部方差
        h, w = gray.shape
        block_size = 32
        local_vars = []
        for y in range(0, h - block_size, block_size):
            for x in range(0, w - block_size, block_size):
                block = gray[y:y+block_size, x:x+block_size].astype(float)
                local_vars.append(np.var(block))

        if local_vars:
            # 选取方差最低的10%块（最平坦区域）
            local_vars.sort()
            n_flat = max(1, len(local_vars) // 10)
            flat_noise = np.mean(local_vars[:n_flat])
        else:
            flat_noise = 0

        # 信噪比
        signal = np.mean(gray.astype(float))
        snr = signal / max(noise_level, 1e-6)

        # 评分（噪声越低越好）
        score_noise = max(0, 100 - noise_level * 10)
        score_flat = max(0, 100 - flat_noise * 2)
        score_snr = min(snr * 2, 100)

        score = score_noise * 0.4 + score_flat * 0.3 + score_snr * 0.3

        return {
            'score': round(float(np.clip(score, 0, 100)), 1),
            'noise_level': round(float(noise_level), 3),
            'flat_region_noise': round(float(flat_noise), 2),
            'snr': round(float(snr), 1),
            'level': self._score_level(score),
        }

    def assess_color(self, frame):
        """
        色彩质量评估（饱和度、色彩丰富度、白平衡）
        Args:
            frame: BGR彩色图像
        Returns:
            dict: {'score': 0~100, 'saturation': float, 'color_diversity': float}
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 饱和度分析
        saturation = np.mean(hsv[:, :, 1])
        # 理想饱和度 50~150
        sat_score = max(0, 100 - abs(saturation - 100) * 1.0)

        # 色彩丰富度（色相直方图的分散程度）
        hue = hsv[:, :, 0]
        hue_hist = cv2.calcHist([hue], [0], None, [180], [0, 180]).flatten()
        hue_hist = hue_hist / hue_hist.sum()
        # 熵作为多样性指标
        nonzero = hue_hist[hue_hist > 0]
        entropy = -np.sum(nonzero * np.log2(nonzero))
        max_entropy = np.log2(180)
        color_diversity = (entropy / max_entropy) * 100

        # 白平衡评估（各通道均值的均衡程度）
        b_mean = np.mean(frame[:, :, 0].astype(float))
        g_mean = np.mean(frame[:, :, 1].astype(float))
        r_mean = np.mean(frame[:, :, 2].astype(float))
        channel_std = np.std([b_mean, g_mean, r_mean])
        # 通道标准差越小白平衡越好
        wb_score = max(0, 100 - channel_std * 2)

        score = sat_score * 0.35 + color_diversity * 0.35 + wb_score * 0.3

        return {
            'score': round(float(np.clip(score, 0, 100)), 1),
            'saturation': round(float(saturation), 1),
            'color_diversity': round(float(color_diversity), 1),
            'white_balance': round(float(wb_score), 1),
            'bgr_means': (round(float(b_mean), 1),
                          round(float(g_mean), 1),
                          round(float(r_mean), 1)),
            'level': self._score_level(score),
        }

    def assess_contrast(self, gray):
        """
        对比度评估
        Args:
            gray: 灰度图
        Returns:
            dict: {'score': 0~100, 'rms_contrast': float, 'michelson': float}
        """
        # RMS对比度
        rms_contrast = np.std(gray.astype(float)) / 128.0 * 100

        # Michelson对比度
        max_val = np.max(gray.astype(float))
        min_val = np.min(gray.astype(float))
        if max_val + min_val > 0:
            michelson = (max_val - min_val) / (max_val + min_val) * 100
        else:
            michelson = 0

        # Weber对比度（基于局部方差）
        local_std = cv2.GaussianBlur(gray.astype(float), (15, 15), 0)
        local_mean = cv2.blur(gray.astype(float), (15, 15))
        weber = np.mean(local_std) / max(np.mean(local_mean), 1) * 100

        score = min(100, rms_contrast * 0.5 + michelson * 0.3 + weber * 0.2)

        return {
            'score': round(float(np.clip(score, 0, 100)), 1),
            'rms_contrast': round(float(rms_contrast), 1),
            'michelson_contrast': round(float(michelson), 1),
            'level': self._score_level(score),
        }

    @staticmethod
    def _score_level(score):
        """将分数映射为等级"""
        if score >= 80:
            return '优秀'
        elif score >= 60:
            return '良好'
        elif score >= 40:
            return '一般'
        elif score >= 20:
            return '较差'
        else:
            return '很差'

    def print_report(self, report):
        """打印评估报告"""
        print("=" * 50)
        print("图像质量评估报告")
        print("=" * 50)
        for dim, info in report.items():
            score = info.get('score', 0)
            level = info.get('level', '')
            print(f"  {dim:15s}: {score:5.1f}分 [{level}]")
            # 打印额外指标
            for k, v in info.items():
                if k not in ('score', 'level'):
                    print(f"    {k}: {v}")
        print("=" * 50)

    def draw_report(self, frame, report):
        """
        在图像上绘制评估结果
        Args:
            frame: 原始图像
            report: assess()的返回结果
        Returns:
            vis: 可视化图像
        """
        vis = frame.copy()
        h, w = vis.shape[:2]

        # 半透明背景
        overlay = vis.copy()
        cv2.rectangle(overlay, (10, 10), (350, 280), (0, 0, 0), -1)
        vis = cv2.addWeighted(overlay, 0.6, vis, 0.4, 0)

        y_pos = 35
        cv2.putText(vis, "Quality Report", (20, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        y_pos += 30

        # 颜色映射
        def score_color(score):
            if score >= 80: return (0, 255, 0)
            elif score >= 60: return (0, 255, 255)
            elif score >= 40: return (0, 165, 255)
            else: return (0, 0, 255)

        for dim, info in report.items():
            if dim == 'overall':
                continue
            score = info.get('score', 0)
            level = info.get('level', '')
            color = score_color(score)

            # 绘制文字
            text = f"{dim}: {score:.0f} [{level}]"
            cv2.putText(vis, text, (20, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # 绘制评分条
            bar_w = int(score * 2.0)
            cv2.rectangle(vis, (200, y_pos - 12), (200 + bar_w, y_pos - 2),
                          color, -1)
            y_pos += 30

        # 综合评分（大字）
        overall = report.get('overall', {})
        ov_score = overall.get('score', 0)
        ov_level = overall.get('level', '')
        color = score_color(ov_score)
        cv2.putText(vis, f"Overall: {ov_score:.0f}/100 [{ov_level}]",
                    (20, y_pos + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        return vis


# ============================================================
# 批量评估工具
# ============================================================

def assess_images(image_paths):
    """
    批量评估多张图像
    Args:
        image_paths: 图片路径列表
    Returns:
        results: 评估结果列表
    """
    assessor = ImageQualityAssessor()
    results = []

    for path in image_paths:
        frame = cv2.imread(path)
        if frame is None:
            print(f"无法读取: {path}")
            continue
        report = assessor.assess(frame)
        report['_path'] = path
        results.append(report)
        assessor.print_report(report)

    # 排序
    results.sort(key=lambda r: r.get('overall', {}).get('score', 0), reverse=True)
    return results


# ============================================================
# 使用示例
# ============================================================

def demo_camera():
    """摄像头实时图像质量评估"""
    assessor = ImageQualityAssessor()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("图像质量评估 - 按 q 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        report = assessor.assess(frame)
        vis = assessor.draw_report(frame, report)

        cv2.imshow('Image Quality Assessment', vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path):
    """单张图片质量评估"""
    assessor = ImageQualityAssessor()
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"无法读取图片: {image_path}")
        return

    report = assessor.assess(frame)
    assessor.print_report(report)

    vis = assessor.draw_report(frame, report)
    cv2.imshow('Image Quality Assessment', vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        demo_image(sys.argv[1])
    else:
        demo_camera()
