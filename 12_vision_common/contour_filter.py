"""
轮廓过滤模块 - 按面积/圆度/长宽比等条件过滤轮廓
====================================================
功能:
  - 面积过滤 (最小/最大面积)
  - 圆度过滤 (接近圆形的程度)
  - 长宽比过滤 (矩形比例)
  - 凸性过滤 (凸包占比)
  - 周长过滤
  - 边界框过滤
  - 多条件组合过滤
  - 轮廓排序 (按面积/位置/圆度等)

适用场景:
  - 从复杂背景中筛选目标形状
  - 去除噪声轮廓
  - 按几何特征分类轮廓
  - 电赛中目标物筛选

用法:
  cf = ContourFilter()
  good = cf.filter_by_area(contours, min_area=100, max_area=10000)
  good = cf.filter_multi(contours, min_area=100, min_circularity=0.5)
  ranked = cf.sort_by_area(contours, reverse=True)
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional


class ContourFilter:
    """轮廓过滤器"""

    # ──────────────── 单条件过滤 ────────────────
    @staticmethod
    def filter_by_area(contours: list, min_area: float = 0,
                       max_area: float = float('inf')) -> list:
        """按面积过滤轮廓"""
        return [c for c in contours if min_area <= cv2.contourArea(c) <= max_area]

    @staticmethod
    def filter_by_circularity(contours: list, min_circ: float = 0.0,
                              max_circ: float = 1.0) -> list:
        """
        按圆度过滤。
        圆度 = 4π * area / perimeter², 完美圆=1.0
        """
        result = []
        for c in contours:
            area = cv2.contourArea(c)
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            circ = 4 * np.pi * area / (perimeter ** 2)
            if min_circ <= circ <= max_circ:
                result.append(c)
        return result

    @staticmethod
    def filter_by_aspect_ratio(contours: list, min_ratio: float = 0.0,
                               max_ratio: float = float('inf')) -> list:
        """
        按长宽比过滤 (宽/高)。
        min_ratio/max_ratio: 边界框的宽/高比范围
        """
        result = []
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if h == 0:
                continue
            ratio = w / h
            if min_ratio <= ratio <= max_ratio:
                result.append(c)
        return result

    @staticmethod
    def filter_by_solidity(contours: list, min_solidity: float = 0.0,
                           max_solidity: float = 1.0) -> list:
        """
        按凸性 (solidity) 过滤。
        solidity = contour_area / convex_hull_area
        值越接近1越凸 (越规则)
        """
        result = []
        for c in contours:
            area = cv2.contourArea(c)
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0:
                continue
            solidity = area / hull_area
            if min_solidity <= solidity <= max_solidity:
                result.append(c)
        return result

    @staticmethod
    def filter_by_perimeter(contours: list, min_peri: float = 0,
                            max_peri: float = float('inf')) -> list:
        """按周长过滤"""
        return [c for c in contours
                if min_peri <= cv2.arcLength(c, True) <= max_peri]

    @staticmethod
    def filter_by_extent(contours: list, min_extent: float = 0.0,
                         max_extent: float = 1.0) -> list:
        """
        按填充率 (extent) 过滤。
        extent = contour_area / bounding_rect_area
        矩形=1.0, 不规则形状<1.0
        """
        result = []
        for c in contours:
            area = cv2.contourArea(c)
            x, y, w, h = cv2.boundingRect(c)
            rect_area = w * h
            if rect_area == 0:
                continue
            extent = area / rect_area
            if min_extent <= extent <= max_extent:
                result.append(c)
        return result

    @staticmethod
    def filter_by_vertices(contours: list, min_vertices: int = 3,
                           max_vertices: int = 100,
                           epsilon_factor: float = 0.02) -> list:
        """
        按近似多边形顶点数过滤。
        epsilon_factor: 越大近似越粗糙, 顶点数越少
        """
        result = []
        for c in contours:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, epsilon_factor * peri, True)
            n = len(approx)
            if min_vertices <= n <= max_vertices:
                result.append(c)
        return result

    # ──────────────── 多条件组合过滤 ────────────────
    @staticmethod
    def filter_multi(contours: list,
                     min_area: float = 0, max_area: float = float('inf'),
                     min_circularity: float = 0.0, max_circularity: float = 1.0,
                     min_aspect: float = 0.0, max_aspect: float = float('inf'),
                     min_solidity: float = 0.0, max_solidity: float = 1.0,
                     min_extent: float = 0.0, max_extent: float = 1.0,
                     min_perimeter: float = 0, max_perimeter: float = float('inf'),
                     min_vertices: int = 0, max_vertices: int = 999) -> list:
        """
        多条件同时过滤 (AND逻辑)。
        所有参数都有默认值 (不过滤), 只设置需要的即可。
        """
        result = []
        for c in contours:
            area = cv2.contourArea(c)
            perimeter = cv2.arcLength(c, True)

            # 面积
            if not (min_area <= area <= max_area):
                continue
            # 周长
            if not (min_perimeter <= perimeter <= max_perimeter):
                continue
            # 圆度
            if perimeter > 0:
                circ = 4 * np.pi * area / (perimeter ** 2)
                if not (min_circularity <= circ <= max_circularity):
                    continue
            # 长宽比
            x, y, w, h = cv2.boundingRect(c)
            if h > 0:
                ratio = w / h
                if not (min_aspect <= ratio <= max_aspect):
                    continue
            # 凸性
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            if hull_area > 0:
                solidity = area / hull_area
                if not (min_solidity <= solidity <= max_solidity):
                    continue
            # 填充率
            rect_area = w * h if w > 0 and h > 0 else 1
            extent = area / rect_area
            if not (min_extent <= extent <= max_extent):
                continue
            # 顶点数
            if min_vertices > 0 or max_vertices < 999:
                approx = cv2.approxPolyDP(c, 0.02 * perimeter, True)
                if not (min_vertices <= len(approx) <= max_vertices):
                    continue

            result.append(c)
        return result

    # ──────────────── 排序 ────────────────
    @staticmethod
    def sort_by_area(contours: list, reverse: bool = True) -> list:
        """按面积排序, reverse=True 从大到小"""
        return sorted(contours, key=cv2.contourArea, reverse=reverse)

    @staticmethod
    def sort_by_circularity(contours: list, reverse: bool = True) -> list:
        """按圆度排序"""
        def _circ(c):
            a = cv2.contourArea(c)
            p = cv2.arcLength(c, True)
            return 4 * np.pi * a / (p ** 2) if p > 0 else 0
        return sorted(contours, key=_circ, reverse=reverse)

    @staticmethod
    def sort_by_position(contours: list, axis: str = 'x') -> list:
        """
        按位置排序。
        axis: 'x' (左到右) | 'y' (上到下) | 'cx' (重心x) | 'cy' (重心y)
        """
        def _pos(c):
            if axis in ('cx', 'cy'):
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    return cx if axis == 'cx' else cy
            x, y, w, h = cv2.boundingRect(c)
            return x if axis == 'x' else y
        return sorted(contours, key=_pos)

    # ──────────────── 提取最大/最小轮廓 ────────────────
    @staticmethod
    def get_largest(contours: list, n: int = 1) -> list:
        """获取面积最大的N个轮廓"""
        return ContourFilter.sort_by_area(contours, reverse=True)[:n]

    @staticmethod
    def get_smallest(contours: list, n: int = 1) -> list:
        """获取面积最小的N个轮廓"""
        return ContourFilter.sort_by_area(contours, reverse=False)[:n]

    @staticmethod
    def get_most_circular(contours: list, n: int = 1) -> list:
        """获取最圆的N个轮廓"""
        return ContourFilter.sort_by_circularity(contours, reverse=True)[:n]

    # ──────────────── 轮廓特征提取 ────────────────
    @staticmethod
    def analyze_contour(contour) -> dict:
        """提取单个轮廓的全部几何特征"""
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        x, y, w, h = cv2.boundingRect(contour)

        # 圆度
        circularity = 4 * np.pi * area / (perimeter ** 2) if perimeter > 0 else 0
        # 长宽比
        aspect_ratio = w / h if h > 0 else 0
        # 凸性
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area > 0 else 0
        # 填充率
        rect_area = w * h
        extent = area / rect_area if rect_area > 0 else 0
        # 最小外接圆
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        # 拟合椭圆
        ellipse = None
        if len(contour) >= 5:
            ellipse = cv2.fitEllipse(contour)
        # 顶点数
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)

        return {
            'area': area,
            'perimeter': perimeter,
            'circularity': circularity,
            'aspect_ratio': aspect_ratio,
            'solidity': solidity,
            'extent': extent,
            'bounding_rect': (x, y, w, h),
            'center': (int(cx), int(cy)),
            'enclosing_radius': radius,
            'n_vertices': len(approx),
            'ellipse': ellipse,
        }

    # ──────────────── 可视化 ────────────────
    @staticmethod
    def draw_filtered(img: np.ndarray, contours: list,
                      passed: list, color_pass: tuple = (0, 255, 0),
                      color_fail: tuple = (0, 0, 255)) -> np.ndarray:
        """可视化过滤结果: 通过的绿色, 未通过的红色"""
        vis = img.copy()
        failed = [c for c in contours if c not in passed]
        cv2.drawContours(vis, failed, -1, color_fail, 1)
        cv2.drawContours(vis, passed, -1, color_pass, 2)
        return vis


# ──────────────── Demo ────────────────
if __name__ == '__main__':
    import sys

    img_path = sys.argv[1] if len(sys.argv) > 1 else 'test.jpg'
    img = cv2.imread(img_path)
    if img is None:
        print(f"无法读取图像: {img_path}")
        sys.exit(1)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"找到 {len(contours)} 个轮廓")

    cf = ContourFilter()

    # 多条件过滤
    good = cf.filter_multi(contours,
                           min_area=500, max_area=50000,
                           min_circularity=0.3,
                           min_solidity=0.5)
    print(f"过滤后 {len(good)} 个轮廓")

    # 打印特征
    for i, c in enumerate(good[:5]):
        info = cf.analyze_contour(c)
        print(f"轮廓{i}: 面积={info['area']:.0f}, 圆度={info['circularity']:.2f}, "
              f"长宽比={info['aspect_ratio']:.2f}, 凸性={info['solidity']:.2f}")

    # 可视化
    vis = cf.draw_filtered(img, contours, good)
    cv2.imshow('Contour Filter', vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
