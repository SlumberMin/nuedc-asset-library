"""
目标大小测量模块 - 已知距离下的尺寸计算
适用于电赛中测量已知距离处目标物体的实际尺寸
"""

import cv2
import numpy as np
import math


class TargetSizeEstimator:
    """基于小孔相机模型的目标尺寸估计器"""

    def __init__(self, focal_length_px, sensor_width_mm=None, image_width_px=None):
        """
        初始化尺寸估计器

        Args:
            focal_length_px: 焦距(像素单位)
            sensor_width_mm: 传感器宽度(mm)，可选，用于与真实焦距换算
            image_width_px: 图像宽度(像素)，可选
        """
        self.focal_length_px = focal_length_px
        # 可选：从物理焦距换算像素焦距
        if sensor_width_mm and image_width_px:
            focal_length_mm = focal_length_px
            self.focal_length_px = (focal_length_mm * image_width_px) / sensor_width_mm

    def estimate_size_by_distance(self, known_distance_m, pixel_length, image_width_px=640):
        """
        已知距离时，根据像素长度计算实际尺寸

        原理: 实际尺寸 / 距离 = 像素尺寸 / 焦距(像素)

        Args:
            known_distance_m: 已知距离(米)
            pixel_length: 目标在图像中的像素长度
            image_width_px: 图像宽度(像素)

        Returns:
            实际尺寸(米)
        """
        actual_size = (pixel_length * known_distance_m) / self.focal_length_px
        return actual_size

    def estimate_width_height(self, known_distance_m, bbox):
        """
        已知距离时，根据边界框计算目标实际宽高

        Args:
            known_distance_m: 已知距离(米)
            bbox: 边界框 (x, y, w, h)

        Returns:
            (actual_width_m, actual_height_m)
        """
        x, y, w, h = bbox
        actual_w = (w * known_distance_m) / self.focal_length_px
        actual_h = (h * known_distance_m) / self.focal_length_px
        return actual_w, actual_h

    def estimate_from_two_points(self, known_distance_m, pt1, pt2):
        """
        已知距离时，根据两点计算实际距离

        Args:
            known_distance_m: 已知距离(米)
            pt1: 起点 (x1, y1)
            pt2: 终点 (x2, y2)

        Returns:
            实际距离(米)
        """
        pixel_dist = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
        return (pixel_dist * known_distance_m) / self.focal_length_px

    def estimate_by_reference(self, known_ref_size_m, known_ref_pixel, target_pixel):
        """
        已知参考物尺寸时，通过比例计算目标尺寸

        Args:
            known_ref_size_m: 参考物实际尺寸(米)
            known_ref_pixel: 参考物像素尺寸
            target_pixel: 目标像素尺寸

        Returns:
            目标实际尺寸(米)
        """
        scale = known_ref_size_m / known_ref_pixel
        return target_pixel * scale

    def estimate_area(self, known_distance_m, pixel_area):
        """
        已知距离时，根据像素面积计算实际面积

        Args:
            known_distance_m: 已知距离(米)
            pixel_area: 目标像素面积

        Returns:
            实际面积(平方米)
        """
        # 像素面积 -> 实际面积 的比例因子
        scale = (known_distance_m / self.focal_length_px) ** 2
        return pixel_area * scale

    def get_pixel_length_from_contour(self, contour):
        """
        从轮廓获取最小外接矩形的长宽(像素)

        Args:
            contour: OpenCV轮廓

        Returns:
            (width_px, height_px, angle) - 短边、长边、旋转角度
        """
        rect = cv2.minAreaRect(contour)
        w, h = rect[1]
        if w > h:
            w, h = h, w
        return w, h, rect[2]

    def measure_object(self, frame, contour, known_distance_m):
        """
        一站式测量：输入帧、轮廓和已知距离，返回目标实际尺寸

        Args:
            frame: 输入图像
            contour: 目标轮廓
            known_distance_m: 已知距离(米)

        Returns:
            dict: 包含像素和实际尺寸信息
        """
        # 像素尺寸
        x, y, w, h = cv2.boundingRect(contour)
        min_w, min_h, angle = self.get_pixel_length_from_contour(contour)
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        # 实际尺寸
        actual_w, actual_h = self.estimate_width_height(known_distance_m, (x, y, w, h))
        actual_area = self.estimate_area(known_distance_m, area)

        return {
            'bbox': (x, y, w, h),
            'pixel_width': w, 'pixel_height': h,
            'pixel_min_rect': (min_w, min_h),
            'pixel_area': area,
            'pixel_perimeter': perimeter,
            'actual_width_m': actual_w,
            'actual_height_m': actual_h,
            'actual_area_m2': actual_area,
            'distance_m': known_distance_m,
            'angle_deg': angle
        }

    @staticmethod
    def calibrate_focal_length(known_distance_m, known_size_m, pixel_size):
        """
        标定焦距：已知实际尺寸、距离和像素尺寸，反算焦距

        Args:
            known_distance_m: 已知距离(米)
            known_size_m: 已知实际尺寸(米)
            pixel_size: 像素尺寸

        Returns:
            焦距(像素)
        """
        return (pixel_size * known_distance_m) / known_size_m


# ==================== 使用示例 ====================
if __name__ == '__main__':
    # 示例：已知焦距350像素，测量1米外的目标
    estimator = TargetSizeEstimator(focal_length_px=350)

    # 假设目标在图像中占80像素宽、60像素高
    bbox = (100, 100, 80, 60)
    w_m, h_m = estimator.estimate_width_height(known_distance_m=1.0, bbox=bbox)
    print(f"目标实际尺寸: {w_m:.4f}m x {h_m:.4f}m")

    # 使用参考物标定
    size = estimator.estimate_by_reference(
        known_ref_size_m=0.05,  # 参考物5cm
        known_ref_pixel=70,     # 参考物70像素
        target_pixel=80         # 目标80像素
    )
    print(f"通过参考物估计目标尺寸: {size:.4f}m = {size*100:.2f}cm")

    # 标定焦距
    f = TargetSizeEstimator.calibrate_focal_length(1.0, 0.1, 350)
    print(f"标定焦距: {f:.1f} 像素")
