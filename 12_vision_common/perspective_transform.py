"""
透视变换模块 - 四点标定 + 鸟瞰图 + 逆变换
适用于电赛中平面校正、鸟瞰视角、几何测量等场景

功能:
- 四点透视变换 (手动标定)
- 鸟瞰图变换 (俯视图)
- 逆透视变换
- 自动四点检测 (轮廓+霍夫)
- 交互式标定
- 透视校正 + 量测
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict


class PerspectiveTransformer:
    """透视变换器"""

    def __init__(self):
        self.H = None          # 当前单应性矩阵
        self.H_inv = None      # 逆矩阵
        self.src_pts = None    # 源四点
        self.dst_pts = None    # 目标四点
        self.dst_size = None   # 输出尺寸

    def calibrate(self, src_pts: np.ndarray, dst_pts: np.ndarray,
                  dst_size: Optional[Tuple[int, int]] = None) -> np.ndarray:
        """
        四点标定
        Args:
            src_pts: 源图四点 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
                     顺序: 左上, 右上, 右下, 左下
            dst_pts: 目标四点 (同上)
            dst_size: 输出图像尺寸 (w, h), None则自动计算
        Returns:
            变换矩阵 H (3x3)
        """
        src = np.float32(src_pts).reshape(-1, 2)
        dst = np.float32(dst_pts).reshape(-1, 2)

        self.src_pts = src
        self.dst_pts = dst
        self.H = cv2.getPerspectiveTransform(src, dst)
        self.H_inv = cv2.getPerspectiveTransform(dst, src)

        if dst_size is None:
            w = int(max(dst[:, 0]) - min(dst[:, 0]))
            h = int(max(dst[:, 1]) - min(dst[:, 1]))
            self.dst_size = (w, h)
        else:
            self.dst_size = dst_size

        return self.H

    def transform(self, image: np.ndarray,
                  dst_size: Optional[Tuple[int, int]] = None,
                  border_value: Tuple[int, int, int] = (0, 0, 0)) -> np.ndarray:
        """
        执行透视变换
        Args:
            image: 输入图像
            dst_size: 输出尺寸 (w, h), None则使用标定时的尺寸
            border_value: 边界填充值
        Returns:
            变换后的图像
        """
        if self.H is None:
            raise ValueError("请先调用 calibrate() 进行标定")

        size = dst_size if dst_size is not None else self.dst_size
        return cv2.warpPerspective(image, self.H, size,
                                   borderValue=border_value)

    def inverse_transform(self, image: np.ndarray,
                          dst_size: Optional[Tuple[int, int]] = None) -> np.ndarray:
        """
        逆透视变换
        Args:
            image: 已变换的图像
            dst_size: 输出尺寸 (w, h)
        Returns:
            还原后的图像
        """
        if self.H_inv is None:
            raise ValueError("请先调用 calibrate() 进行标定")

        if dst_size is None:
            dst_size = (int(max(self.src_pts[:, 0]) - min(self.src_pts[:, 0])),
                        int(max(self.src_pts[:, 1]) - min(self.src_pts[:, 1])))

        return cv2.warpPerspective(image, self.H_inv, dst_size)

    def transform_point(self, point: Tuple[float, float]) -> Tuple[float, float]:
        """变换单个点"""
        if self.H is None:
            raise ValueError("请先标定")
        pt = np.float32([[[point[0], point[1]]]])
        transformed = cv2.perspectiveTransform(pt, self.H)
        return (float(transformed[0, 0, 0]), float(transformed[0, 0, 1]))

    def inverse_transform_point(self, point: Tuple[float, float]) -> Tuple[float, float]:
        """逆变换单个点"""
        if self.H_inv is None:
            raise ValueError("请先标定")
        pt = np.float32([[[point[0], point[1]]]])
        transformed = cv2.perspectiveTransform(pt, self.H_inv)
        return (float(transformed[0, 0, 0]), float(transformed[0, 0, 1]))

    @staticmethod
    def draw_points(image: np.ndarray, points: np.ndarray,
                    color: Tuple[int, int, int] = (0, 0, 255),
                    radius: int = 5) -> np.ndarray:
        """绘制标定点"""
        vis = image.copy()
        pts = np.int32(points).reshape(-1, 2)
        labels = ['LT', 'RT', 'RB', 'LB']
        for i, pt in enumerate(pts):
            cv2.circle(vis, tuple(pt), radius, color, -1)
            cv2.putText(vis, labels[i % 4], (pt[0] + 8, pt[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        # 连线
        cv2.polylines(vis, [pts], True, color, 2)
        return vis


class BirdEyeView(PerspectiveTransformer):
    """鸟瞰图变换器"""

    def __init__(self):
        super().__init__()

    def setup_from_region(self, image: np.ndarray,
                          src_corners: np.ndarray,
                          output_width: int = 0,
                          output_height: int = 0,
                          pixels_per_meter: float = 0) -> np.ndarray:
        """
        从地面四角设置鸟瞰图
        Args:
            image: 原始图像
            src_corners: 地面上的四角点 (左上, 右上, 右下, 左下)
            output_width: 输出宽度 (像素), 0=自动
            output_height: 输出高度 (像素), 0=自动
            pixels_per_meter: 像素/米比例 (用于物理量测)
        """
        src = np.float32(src_corners).reshape(-1, 2)

        # 计算输出尺寸
        if output_width == 0:
            w_top = np.linalg.norm(src[1] - src[0])
            w_bottom = np.linalg.norm(src[2] - src[3])
            output_width = int(max(w_top, w_bottom))
        if output_height == 0:
            h_left = np.linalg.norm(src[3] - src[0])
            h_right = np.linalg.norm(src[2] - src[1])
            output_height = int(max(h_left, h_right))

        # 目标矩形
        dst = np.float32([
            [0, 0],
            [output_width, 0],
            [output_width, output_height],
            [0, output_height]
        ])

        return self.calibrate(src, dst, (output_width, output_height))

    def setup_simple(self, image: np.ndarray,
                     top_width_ratio: float = 0.3,
                     bottom_width_ratio: float = 1.0,
                     top_y_ratio: float = 0.4,
                     bottom_y_ratio: float = 1.0) -> np.ndarray:
        """
        简易鸟瞰图设置 (基于图像比例)
        典型用于: 道路/车道的俯视变换
        Args:
            top_width_ratio: 顶部(远处)宽度占图像宽度比例
            bottom_width_ratio: 底部(近处)宽度占图像宽度比例
            top_y_ratio: 顶部Y坐标占图像高度比例
            bottom_y_ratio: 底部Y坐标占图像高度比例
        """
        h, w = image.shape[:2]

        cx = w / 2
        tw = w * top_width_ratio / 2
        bw = w * bottom_width_ratio / 2
        ty = h * top_y_ratio
        by = h * bottom_y_ratio

        src_corners = np.float32([
            [cx - tw, ty],   # 左上
            [cx + tw, ty],   # 右上
            [cx + bw, by],   # 右下
            [cx - bw, by]    # 左下
        ])

        out_w = int(max(bw * 2, tw * 2))
        out_h = int(by - ty)

        return self.setup_from_region(image, src_corners, out_w, out_h)

    def get_bird_eye(self, image: np.ndarray) -> np.ndarray:
        """获取鸟瞰图"""
        return self.transform(image)

    def restore_from_bird_eye(self, bird_eye: np.ndarray,
                              original_size: Tuple[int, int]) -> np.ndarray:
        """从鸟瞰图恢复原始视角"""
        return self.inverse_transform(bird_eye, original_size)

    def measure_distance(self, pt1: Tuple[float, float],
                         pt2: Tuple[float, float],
                         pixels_per_meter: float = 1.0) -> float:
        """
        在鸟瞰图上测量两点间的实际距离
        Args:
            pt1, pt2: 鸟瞰图上的两个点
            pixels_per_meter: 比例尺 (像素/米)
        Returns:
            距离 (米)
        """
        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]
        pixel_dist = np.sqrt(dx * dx + dy * dy)
        return pixel_dist / pixels_per_meter


class AutoPerspectiveCorrector:
    """自动透视校正器"""

    def __init__(self, min_area_ratio: float = 0.05,
                 approx_epsilon: float = 0.02):
        """
        Args:
            min_area_ratio: 最小轮廓面积占图像面积比例
            approx_epsilon: 多边形逼近精度
        """
        self.min_area_ratio = min_area_ratio
        self.approx_epsilon = approx_epsilon

    def detect_quadrilateral(self, image: np.ndarray) -> Optional[np.ndarray]:
        """
        自动检测图像中的四边形
        Args:
            image: 输入图像
        Returns:
            四边形四个顶点 (左上, 右上, 右下, 左下) 或 None
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        h, w = gray.shape
        img_area = h * w

        # 边缘检测
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # 膨胀连接边缘
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.dilate(edges, kernel, iterations=1)

        # 查找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 按面积排序
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < img_area * self.min_area_ratio:
                continue

            # 多边形逼近
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, self.approx_epsilon * peri, True)

            if len(approx) == 4:
                # 找到四边形
                pts = approx.reshape(-1, 2)
                return self._order_points(pts)

        return None

    def auto_correct(self, image: np.ndarray,
                     output_size: Optional[Tuple[int, int]] = None) -> Optional[np.ndarray]:
        """
        自动透视校正
        Args:
            image: 输入图像
            output_size: 输出尺寸
        Returns:
            校正后的图像 或 None
        """
        pts = self.detect_quadrilateral(image)
        if pts is None:
            return None

        if output_size is None:
            w = int(max(np.linalg.norm(pts[1] - pts[0]),
                        np.linalg.norm(pts[2] - pts[3])))
            h = int(max(np.linalg.norm(pts[3] - pts[0]),
                        np.linalg.norm(pts[2] - pts[1])))
            output_size = (w, h)

        dst = np.float32([
            [0, 0],
            [output_size[0], 0],
            [output_size[0], output_size[1]],
            [0, output_size[1]]
        ])

        H = cv2.getPerspectiveTransform(np.float32(pts), dst)
        return cv2.warpPerspective(image, H, output_size)

    @staticmethod
    def _order_points(pts: np.ndarray) -> np.ndarray:
        """
        将四点排序为: 左上, 右上, 右下, 左下
        """
        rect = np.zeros((4, 2), dtype=np.float32)

        # 左上: x+y 最小; 右下: x+y 最大
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        # 右上: y-x 最小; 左下: y-x 最大
        d = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(d)]
        rect[3] = pts[np.argmax(d)]

        return rect


class InteractiveCalibrator:
    """交互式透视标定 (点击选取四点)"""

    def __init__(self):
        self.points = []
        self.image = None
        self.window_name = 'Calibration'

    def calibrate(self, image: np.ndarray,
                  window_size: Tuple[int, int] = (800, 600)) -> Optional[np.ndarray]:
        """
        交互式标定: 点击选择4个点
        Args:
            image: 输入图像
            window_size: 显示窗口大小
        Returns:
            变换矩阵 H 或 None
        """
        self.points = []
        self.image = image.copy()

        # 缩放显示
        scale = min(window_size[0] / image.shape[1],
                    window_size[1] / image.shape[0])
        display = cv2.resize(image, None, fx=scale, fy=scale)
        self.scale = scale

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)

        print("请点击选择4个点 (顺序: 左上->右上->右下->左下)")
        print("按 'r' 重置, 按 'q' 取消, 按回车确认")

        while True:
            show = display.copy()
            for i, pt in enumerate(self.points):
                ox, oy = int(pt[0] / scale), int(pt[1] / scale)
                cv2.circle(show, (ox, oy), 5, (0, 0, 255), -1)
                cv2.putText(show, str(i), (ox + 5, oy - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            if len(self.points) == 4:
                pts_show = np.int32([(int(p[0] / scale), int(p[1] / scale))
                                     for p in self.points])
                cv2.polylines(show, [pts_show], True, (0, 255, 0), 2)
                cv2.putText(show, "Press ENTER to confirm", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow(self.window_name, show)
            key = cv2.waitKey(30) & 0xFF

            if key == ord('q'):
                cv2.destroyWindow(self.window_name)
                return None
            elif key == ord('r'):
                self.points = []
            elif key == 13 and len(self.points) == 4:  # Enter
                break

        cv2.destroyWindow(self.window_name)

        src = np.float32(self.points)
        w = int(max(np.linalg.norm(src[1] - src[0]),
                    np.linalg.norm(src[2] - src[3])))
        h = int(max(np.linalg.norm(src[3] - src[0]),
                    np.linalg.norm(src[2] - src[1])))
        dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])

        H = cv2.getPerspectiveTransform(src, dst)
        return H

    def _mouse_callback(self, event, x, y, flags, param):
        """鼠标回调"""
        if event == cv2.EVENT_LBUTTONDOWN and len(self.points) < 4:
            self.points.append((x / self.scale, y / self.scale))
            print(f"  点{len(self.points)}: ({x / self.scale:.0f}, {y / self.scale:.0f})")


# ==================== 快捷函数 ====================

def warp_perspective(image: np.ndarray,
                     src_pts: np.ndarray,
                     dst_size: Tuple[int, int]) -> np.ndarray:
    """快速透视变换"""
    src = np.float32(src_pts).reshape(-1, 2)
    w, h = dst_size
    dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    H = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(image, H, dst_size)


def get_bird_eye(image: np.ndarray,
                 top_ratio: float = 0.4,
                 width_ratio: float = 0.3) -> np.ndarray:
    """快速鸟瞰图"""
    bev = BirdEyeView()
    bev.setup_simple(image, top_width_ratio=width_ratio, top_y_ratio=top_ratio)
    return bev.get_bird_eye(image)


def auto_correct_perspective(image: np.ndarray) -> Optional[np.ndarray]:
    """自动透视校正"""
    corrector = AutoPerspectiveCorrector()
    return corrector.auto_correct(image)


def rectify_document(image: np.ndarray,
                     target_ratio: float = 1.414) -> Optional[np.ndarray]:
    """
    文档校正 (A4纸等)
    Args:
        image: 含文档的照片
        target_ratio: 目标宽高比 (A4=sqrt(2)≈1.414)
    Returns:
        校正后的图像 或 None
    """
    corrector = AutoPerspectiveCorrector(min_area_ratio=0.1)
    pts = corrector.detect_quadrilateral(image)
    if pts is None:
        return None

    w = int(max(np.linalg.norm(pts[1] - pts[0]),
                np.linalg.norm(pts[2] - pts[3])))
    h = int(w / target_ratio)
    return warp_perspective(image, pts, (w, h))


# ==================== 示例与测试 ====================

if __name__ == '__main__':
    # 创建测试图像: 透视变形的矩形
    img = np.zeros((400, 600, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)

    # 在透视变形的区域画图案
    src_pts = np.float32([[150, 50], [450, 80], [480, 350], [120, 320]])
    cv2.fillPoly(img, [np.int32(src_pts)], (200, 200, 200))
    cv2.putText(img, "TEST", (250, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

    print("=== 四点透视变换测试 ===")
    pt = PerspectiveTransformer()
    dst_pts = np.float32([[0, 0], [300, 0], [300, 270], [0, 270]])
    pt.calibrate(src_pts, dst_pts, (300, 270))
    warped = pt.transform(img)
    print(f"  原图: {img.shape}, 变换后: {warped.shape}")

    # 逆变换
    restored = pt.inverse_transform(warped, (600, 400))
    print(f"  逆变换: {restored.shape}")

    # 点变换测试
    pt_src = (300, 200)
    pt_dst = pt.transform_point(pt_src)
    pt_back = pt.inverse_transform_point(pt_dst)
    print(f"  点变换: {pt_src} -> {pt_dst} -> {pt_back}")

    print("\n=== 鸟瞰图测试 ===")
    bev = BirdEyeView()
    bev.setup_from_region(img, src_pts, 300, 270)
    bird = bev.get_bird_eye(img)
    print(f"  鸟瞰图: {bird.shape}")

    # 距离测量
    dist = bev.measure_distance((50, 50), (250, 200), pixels_per_meter=10)
    print(f"  测量距离: {dist:.2f} 米")

    print("\n=== 自动四边形检测测试 ===")
    corrector = AutoPerspectiveCorrector()
    detected = corrector.detect_quadrilateral(img)
    if detected is not None:
        print(f"  检测到四边形: {detected}")
        corrected = corrector.auto_correct(img)
        print(f"  校正后尺寸: {corrected.shape}")
    else:
        print("  未检测到四边形")

    # 绘制标定点可视化
    vis = PerspectiveTransformer.draw_points(img, src_pts)
    print(f"  标定可视化: {vis.shape}")

    print("\n透视变换模块测试完成!")
