"""
轮廓分析模块 - 面积/周长/圆度/矩形度/长宽比
适用于电赛中的形状识别、目标分类和几何特征提取
"""

import cv2
import numpy as np
import math


class ContourAnalyzer:
    """轮廓特征分析器"""

    def __init__(self, min_area=100):
        """
        Args:
            min_area: 最小面积过滤阈值
        """
        self.min_area = min_area

    def filter_contours(self, contours):
        """
        按面积过滤轮廓

        Args:
            contours: 轮廓列表

        Returns:
            过滤后的轮廓列表
        """
        return [c for c in contours if cv2.contourArea(c) >= self.min_area]

    def get_area(self, contour):
        """计算轮廓面积(像素²)"""
        return cv2.contourArea(contour)

    def get_perimeter(self, contour, closed=True):
        """计算轮廓周长(像素)"""
        return cv2.arcLength(contour, closed)

    def get_circularity(self, contour):
        """
        计算圆度 (4π·A / P²)
        完美圆=1, 越不规则越接近0

        Returns:
            圆度 [0, 1]
        """
        area = cv2.contourArea(contour)
        peri = cv2.arcLength(contour, True)
        if peri == 0:
            return 0
        return (4 * math.pi * area) / (peri ** 2)

    def get_rectangularity(self, contour):
        """
        计算矩形度 (面积 / 最小外接矩形面积)
        完美矩形=1

        Returns:
            矩形度 [0, 1]
        """
        area = cv2.contourArea(contour)
        rect = cv2.minAreaRect(contour)
        rect_area = rect[1][0] * rect[1][1]
        if rect_area == 0:
            return 0
        return area / rect_area

    def get_aspect_ratio(self, contour):
        """
        计算长宽比 (最小外接矩形)

        Returns:
            长宽比 (短边/长边, [0, 1])
        """
        rect = cv2.minAreaRect(contour)
        w, h = rect[1]
        if w == 0 or h == 0:
            return 0
        return min(w, h) / max(w, h)

    def get_extent(self, contour):
        """
        计算延展度 (面积 / 边界矩形面积)

        Returns:
            延展度 [0, 1]
        """
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        rect_area = w * h
        if rect_area == 0:
            return 0
        return area / rect_area

    def get_solidity(self, contour):
        """
        计算实心度 (面积 / 凸包面积)

        Returns:
            实心度 [0, 1]
        """
        area = cv2.contourArea(contour)
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            return 0
        return area / hull_area

    def get_compactness(self, contour):
        """
        计算紧凑度 (面积 / 周长²)
        与圆度类似但未归一化

        Returns:
            紧凑度
        """
        area = cv2.contourArea(contour)
        peri = cv2.arcLength(contour, True)
        if peri == 0:
            return 0
        return area / (peri ** 2)

    def get_eccentricity(self, contour):
        """
        计算离心率(通过拟合椭圆)
        圆=0, 越扁越大

        Returns:
            离心率 [0, 1)
        """
        if len(contour) < 5:
            return 0
        ellipse = cv2.fitEllipse(contour)
        (cx, cy), (ma, MA), angle = ellipse
        a = max(ma, MA) / 2
        b = min(ma, MA) / 2
        if a == 0:
            return 0
        return math.sqrt(1 - (b / a) ** 2)

    def get_hu_moments(self, contour):
        """
        计算Hu矩(尺度、旋转、平移不变矩)

        Returns:
            7个Hu矩值的数组
        """
        moments = cv2.moments(contour)
        hu = cv2.HuMoments(moments).flatten()
        # 取对数便于比较
        for i in range(len(hu)):
            if hu[i] != 0:
                hu[i] = -np.sign(hu[i]) * np.log10(abs(hu[i]))
            else:
                hu[i] = 0
        return hu

    def get_centroid(self, contour):
        """
        获取轮廓质心

        Returns:
            (cx, cy) 或 None
        """
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return (cx, cy)

    def get_bounding_info(self, contour):
        """
        获取边界框信息

        Returns:
            dict: bbox, min_rect, min_rect_area, center
        """
        x, y, w, h = cv2.boundingRect(contour)
        min_rect = cv2.minAreaRect(contour)
        return {
            'bbox': (x, y, w, h),
            'min_rect': min_rect,
            'min_rect_area': min_rect[1][0] * min_rect[1][1],
            'center': (x + w // 2, y + h // 2)
        }

    def analyze(self, contour):
        """
        全面分析一个轮廓的所有特征

        Args:
            contour: OpenCV轮廓

        Returns:
            dict: 所有特征值
        """
        area = self.get_area(contour)
        peri = self.get_perimeter(contour)

        result = {
            'area': area,
            'perimeter': peri,
            'circularity': self.get_circularity(contour),
            'rectangularity': self.get_rectangularity(contour),
            'aspect_ratio': self.get_aspect_ratio(contour),
            'extent': self.get_extent(contour),
            'solidity': self.get_solidity(contour),
            'compactness': self.get_compactness(contour),
            'eccentricity': self.get_eccentricity(contour),
            'centroid': self.get_centroid(contour),
        }

        bbox_info = self.get_bounding_info(contour)
        result.update(bbox_info)

        result['hu_moments'] = self.get_hu_moments(contour)

        return result

    def classify_shape(self, contour):
        """
        简单形状分类

        Args:
            contour: OpenCV轮廓

        Returns:
            形状名称字符串
        """
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        vertices = len(approx)
        circ = self.get_circularity(contour)

        if vertices == 3:
            return "triangle"
        elif vertices == 4:
            ar = self.get_aspect_ratio(contour)
            if ar > 0.9:
                return "square"
            else:
                return "rectangle"
        elif vertices == 5:
            return "pentagon"
        elif vertices == 6:
            return "hexagon"
        elif circ > 0.8:
            return "circle"
        else:
            return f"polygon_{vertices}"

    def compare_contours(self, c1, c2):
        """
        比较两个轮廓的形状相似度

        Args:
            c1, c2: 两个轮廓

        Returns:
            相似度值(越小越相似)
        """
        return cv2.matchShapes(c1, c2, cv2.CONTOURS_MATCH_I2, 0)

    def draw_analysis(self, frame, contour, color=(0, 255, 0)):
        """
        在图像上绘制轮廓分析结果

        Args:
            frame: 输入图像
            contour: 轮廓
            color: 颜色

        Returns:
            绘制后的图像
        """
        info = self.analyze(contour)

        # 绘制轮廓
        cv2.drawContours(frame, [contour], -1, color, 2)

        # 边界框
        x, y, w, h = info['bbox']
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 1)

        # 最小外接矩形
        box = cv2.boxPoints(info['min_rect'])
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 1)

        # 质心
        cx, cy = info['centroid']
        cv2.circle(frame, (cx, cy), 4, (0, 255, 255), -1)

        # 文字标注
        texts = [
            f"Shape: {self.classify_shape(contour)}",
            f"Area: {info['area']:.0f}",
            f"Circ: {info['circularity']:.2f}",
            f"Rect: {info['rectangularity']:.2f}",
            f"AR: {info['aspect_ratio']:.2f}",
            f"Solid: {info['solidity']:.2f}",
        ]
        for i, text in enumerate(texts):
            cv2.putText(frame, text, (x, y - 10 - 18 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        return frame

    def analyze_all(self, contours):
        """
        分析多个轮廓

        Args:
            contours: 轮廓列表

        Returns:
            list of dict
        """
        return [self.analyze(c) for c in contours]

    def find_by_feature(self, contours, feature, min_val=None, max_val=None):
        """
        按特征值筛选轮廓

        Args:
            contours: 轮廓列表
            feature: 特征名 ('circularity', 'rectangularity', etc.)
            min_val: 最小值
            max_val: 最大值

        Returns:
            符合条件的轮廓列表
        """
        results = []
        for c in contours:
            info = self.analyze(c)
            val = info.get(feature)
            if val is None:
                continue
            if min_val is not None and val < min_val:
                continue
            if max_val is not None and val > max_val:
                continue
            results.append(c)
        return results


# ==================== 使用示例 ====================
if __name__ == '__main__':
    analyzer = ContourAnalyzer(min_area=50)

    # 创建测试图像
    img = np.zeros((500, 500, 3), dtype=np.uint8)

    # 画不同形状
    cv2.circle(img, (100, 100), 50, (255, 255, 255), -1)
    cv2.rectangle(img, (200, 50), (350, 150), (255, 255, 255), -1)
    cv2.drawContours(img, [np.array([[400, 50], [450, 150], [350, 150]])], 0, (255, 255, 255), -1)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        info = analyzer.analyze(c)
        shape = analyzer.classify_shape(c)
        print(f"\n--- {shape} ---")
        print(f"  面积: {info['area']:.0f}")
        print(f"  周长: {info['perimeter']:.1f}")
        print(f"  圆度: {info['circularity']:.3f}")
        print(f"  矩形度: {info['rectangularity']:.3f}")
        print(f"  长宽比: {info['aspect_ratio']:.3f}")
        print(f"  实心度: {info['solidity']:.3f}")
        print(f"  离心率: {info['eccentricity']:.3f}")

        img = analyzer.draw_analysis(img, c)

    # 圆形筛选
    circles = analyzer.find_by_feature(contours, 'circularity', min_val=0.8)
    print(f"\n找到 {len(circles)} 个近似圆形")

    cv2.imshow("Contour Analysis", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
