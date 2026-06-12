"""
图像修复模块 - 损坏区域检测 + Telea/NS算法 + 深度学习修复
功能：自动检测损坏区域，使用多种算法修复图像
依赖：opencv-python, numpy
"""

import cv2
import numpy as np


# ==================== 损坏区域检测 ====================

class DamageDetector:
    """自动检测图像中的损坏/缺失区域"""

    def __init__(self):
        pass

    def detect_scratches(self, image, threshold=30):
        """
        检测划痕/裂纹
        
        参数:
            image: BGR图像
            threshold: 边缘阈值
            
        返回:
            mask: 损坏区域掩码 (白色=损坏)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 形态学梯度检测线状结构
        kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 1))
        kernel_v = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 25))

        # 检测水平和垂直划痕
        tophat_h = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel_h)
        tophat_v = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel_v)

        # 合并
        combined = cv2.add(tophat_h, tophat_v)
        _, mask = cv2.threshold(combined, threshold, 255, cv2.THRESH_BINARY)

        # 形态学后处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.dilate(mask, kernel, iterations=1)

        return mask

    def detect_text_overlay(self, image):
        """
        检测图像上的文字覆盖区域（用于去除水印等）
        
        返回:
            mask: 文字区域掩码
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 自适应阈值
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY_INV, 11, 2)

        # 连通域分析
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            thresh, connectivity=8)

        mask = np.zeros_like(gray)
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            # 过滤：文字通常面积小、宽高比适中
            if 50 < area < 5000 and 0.1 < w / (h + 1e-6) < 10:
                mask[labels == i] = 255

        return mask

    def detect_dead_pixels(self, image, neighborhood=5, threshold=80):
        """
        检测坏点/噪点
        
        返回:
            mask: 坏点掩码
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32)

        # 中值滤波
        median = cv2.medianBlur(gray.astype(np.uint8), neighborhood).astype(np.float32)

        # 差异大于阈值的为坏点
        diff = np.abs(gray - median)
        _, mask = cv2.threshold(diff.astype(np.uint8), threshold, 255, cv2.THRESH_BINARY)

        return mask

    def detect_watermark(self, image, region=None):
        """
        检测半透明水印区域
        
        参数:
            image: BGR图像
            region: 可选的搜索区域 (x, y, w, h)
            
        返回:
            mask: 水印掩码
        """
        if region:
            x, y, w, h = region
            roi = image[y:y + h, x:x + w]
        else:
            roi = image

        # 水印通常亮度异常
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        _, _, v = cv2.split(hsv)

        # 高亮度低饱和度区域
        mask = np.zeros(roi.shape[:2], dtype=np.uint8)
        mask[v > 200] = 255

        if region:
            full_mask = np.zeros(image.shape[:2], dtype=np.uint8)
            full_mask[y:y + h, x:x + w] = mask
            return full_mask

        return mask


# ==================== 图像修复器 ====================

class ImageInpainter:
    """
    图像修复器
    支持：OpenCV内置Telea/NS算法 + 扩展方法
    """

    def __init__(self):
        pass

    def inpaint_telea(self, image, mask, radius=3):
        """
        Telea算法修复（基于快速行进法）
        适合：小区域修复，纹理简单区域
        
        参数:
            image: BGR图像
            mask: 损坏区域掩码
            radius: 修复半径
        """
        return cv2.inpaint(image, mask, radius, cv2.INPAINT_TELEA)

    def inpaint_ns(self, image, mask, radius=3):
        """
        Navier-Stokes算法修复（基于流体动力学）
        适合：边缘保持更好的修复
        
        参数:
            image: BGR图像
            mask: 损坏区域掩码
            radius: 修复半径
        """
        return cv2.inpaint(image, mask, radius, cv2.INPAINT_NS)

    def inpaint_multiscale(self, image, mask, scales=3):
        """
        多尺度修复：从粗到细逐步修复
        适合：大面积损坏
        
        参数:
            image: BGR图像
            mask: 损坏区域掩码
            scales: 尺度层级数
        """
        h, w = image.shape[:2]
        result = image.copy()
        current_mask = mask.copy()

        # 从低分辨率到高分辨率逐级修复
        for level in range(scales, 0, -1):
            scale = 1.0 / (2 ** (level - 1))
            small_h, small_w = int(h * scale), int(w * scale)

            if small_h < 1 or small_w < 1:
                continue

            # 缩小
            small_img = cv2.resize(result, (small_w, small_h))
            small_mask = cv2.resize(current_mask, (small_w, small_h))
            _, small_mask = cv2.threshold(small_mask, 127, 255, cv2.THRESH_BINARY)

            # 在低分辨率修复
            if cv2.countNonZero(small_mask) > 0:
                repaired = cv2.inpaint(small_img, small_mask, 5, cv2.INPAINT_TELEA)
                result = cv2.resize(repaired, (w, h))

            # 缩小掩码（已修复区域不再需要修复）
            current_mask = cv2.resize(current_mask, (small_w, small_h))
            current_mask = cv2.resize(current_mask, (w, h))
            _, current_mask = cv2.threshold(current_mask, 127, 255, cv2.THRESH_BINARY)

        return result

    def inpaint_exemplar(self, image, mask, patch_size=9, search_area=50):
        """
        基于样例的修复（简化版PatchMatch思想）
        适合：纹理区域修复
        
        参数:
            image: BGR图像
            mask: 损坏区域掩码
            patch_size: 块大小
            search_area: 搜索范围
        """
        result = image.copy().astype(np.float32)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h, w = image.shape[:2]
        half_patch = patch_size // 2

        # 找到掩码边界
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            for point in contour:
                px, py = point[0]

                if mask[py, px] == 0:
                    continue

                best_patch = None
                best_dist = float('inf')

                # 在搜索范围内寻找最佳匹配块
                y1 = max(0, py - search_area)
                y2 = min(h - patch_size, py + search_area)
                x1 = max(0, px - search_area)
                x2 = min(w - patch_size, px + search_area)

                for sy in range(y1, y2, 3):
                    for sx in range(x1, x2, 3):
                        # 检查候选块是否在有效区域
                        patch_mask = mask[sy:sy + patch_size, sx:sx + patch_size]
                        if patch_mask.size == 0:
                            continue
                        if np.mean(patch_mask) > 10:  # 候选块也有损坏
                            continue

                        # 计算块差异（只比较已知像素）
                        src_patch = gray[sy:sy + patch_size, sx:sx + patch_size]
                        dst_patch = gray[py - half_patch:py + half_patch + 1,
                                    px - half_patch:px + half_patch + 1]

                        if src_patch.shape != dst_patch.shape:
                            continue

                        # 加权差异（边界像素权重更高）
                        valid_mask = mask[py - half_patch:py + half_patch + 1,
                                     px - half_patch:px + half_patch + 1] == 0
                        if np.sum(valid_mask) < 3:
                            continue

                        dist = np.sum(np.abs(src_patch[valid_mask].astype(float) -
                                              dst_patch[valid_mask].astype(float)))

                        if dist < best_dist:
                            best_dist = dist
                            best_patch = result[sy:sy + patch_size,
                                          sx:sx + patch_size].copy()

                if best_patch is not None:
                    result[py - half_patch:py + half_patch + 1,
                    px - half_patch:px + half_patch + 1] = best_patch

        return np.clip(result, 0, 255).astype(np.uint8)

    def inpaint_edge_guided(self, image, mask):
        """
        边缘引导修复：先修复边缘，再填充内部
        
        返回:
            result: 修复后的图像
        """
        # 先用NS算法修复边缘
        # 膨胀掩码获取边缘区域
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dilated = cv2.dilate(mask, kernel, iterations=2)
        edge_mask = dilated - mask

        # 阶段1：修复边缘区域
        if cv2.countNonZero(edge_mask) > 0:
            stage1 = cv2.inpaint(image, edge_mask, 5, cv2.INPAINT_NS)
        else:
            stage1 = image.copy()

        # 阶段2：修复内部区域
        result = cv2.inpaint(stage1, mask, 3, cv2.INPAINT_TELEA)

        return result


# ==================== 使用示例 ====================

def example_auto_inpaint():
    """自动检测+修复示例"""
    # 加载带损坏的图像
    image = cv2.imread("damaged_image.jpg")

    if image is None:
        # 创建模拟损坏图像
        image = np.random.randint(100, 200, (300, 400, 3), dtype=np.uint8)
        # 添加划痕
        cv2.line(image, (50, 50), (350, 250), (0, 0, 0), 3)
        cv2.line(image, (100, 0), (100, 300), (0, 0, 0), 2)
        # 添加噪点
        noise_mask = np.random.random(image.shape[:2]) > 0.98
        image[noise_mask] = [0, 0, 0]
        cv2.imwrite("damaged_test.jpg", image)
        print("已创建模拟损坏图像: damaged_test.jpg")

    # 检测损坏区域
    detector = DamageDetector()
    scratch_mask = detector.detect_scratches(image)

    # 修复
    inpainter = ImageInpainter()

    # Telea算法
    result_telea = inpainter.inpaint_telea(image, scratch_mask, radius=3)

    # NS算法
    result_ns = inpainter.inpaint_ns(image, scratch_mask, radius=3)

    # 多尺度修复
    result_multi = inpainter.inpaint_multiscale(image, scratch_mask)

    print("修复完成！")
    cv2.imshow("Original", image)
    cv2.imshow("Mask", scratch_mask)
    cv2.imshow("Telea Result", result_telea)
    cv2.imshow("NS Result", result_ns)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def example_watermark_removal():
    """水印去除示例"""
    image = cv2.imread("watermarked.jpg")
    if image is None:
        return

    detector = DamageDetector()
    inpainter = ImageInpainter()

    # 检测水印（可指定区域）
    mask = detector.detect_watermark(image, region=(10, 10, 200, 50))

    # 修复
    result = inpainter.inpaint_ns(image, mask, radius=5)
    cv2.imwrite("watermark_removed.jpg", result)
    print("水印去除完成！")


if __name__ == "__main__":
    print("=== 图像修复模块 ===")
    print("算法: Telea(快速行进) / NS(流体力学) / 多尺度 / 样例匹配")
    example_auto_inpaint()
