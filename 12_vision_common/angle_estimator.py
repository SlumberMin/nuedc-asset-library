"""
角度估计模块 - 基于视觉的倾斜角/旋转角测量
适用于电赛中测量目标的倾斜角度、旋转角度
"""

import cv2
import numpy as np
import math


class AngleEstimator:
    """基于视觉的角度估计器"""

    @staticmethod
    def get_rotation_angle_minrect(contour):
        """
        通过最小外接矩形获取旋转角度

        Args:
            contour: OpenCV轮廓

        Returns:
            旋转角度(度), 通常范围 [-90, 0)
        """
        rect = cv2.minAreaRect(contour)
        angle = rect[2]
        return angle

    @staticmethod
    def get_normalized_angle(contour):
        """
        获取归一化角度: 将最小外接矩形角度转为 [-90, 90] 范围
        表示目标相对于水平方向的倾斜

        Args:
            contour: OpenCV轮廓

        Returns:
            归一化角度(度) [-90, 90]
        """
        rect = cv2.minAreaRect(contour)
        w, h = rect[1]
        angle = rect[2]

        # OpenCV minAreaRect: w < h 时 angle ∈ [-90, 0)
        # 归一化到 [-90, 90]
        if w < h:
            angle = angle  # [-90, 0)
        else:
            angle = angle + 90  # 转换

        return angle

    @staticmethod
    def get_orientation_by_moments(contour):
        """
        通过图像矩计算目标主轴方向

        Args:
            contour: OpenCV轮廓

        Returns:
            主轴角度(度), 相对于水平方向
        """
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return 0.0

        # 中心矩
        mu20 = moments['mu20']
        mu02 = moments['mu02']
        mu11 = moments['mu11']

        # 主轴角度
        angle_rad = 0.5 * math.atan2(2 * mu11, mu20 - mu02)
        angle_deg = math.degrees(angle_rad)

        return angle_deg

    @staticmethod
    def estimate_tilt_from_ellipse(contour):
        """
        通过拟合椭圆估计倾斜角

        Args:
            contour: OpenCV轮廓(需要至少5个点)

        Returns:
            (angle_deg, (cx, cy), (a, b)) - 角度、中心、长短轴
        """
        if len(contour) < 5:
            return None

        ellipse = cv2.fitEllipse(contour)
        center, axes, angle = ellipse
        return angle, center, axes

    @staticmethod
    def estimate_angle_from_two_points(pt1, pt2):
        """
        两点之间的连线角度

        Args:
            pt1: 起点 (x1, y1)
            pt2: 终点 (x2, y2)

        Returns:
            角度(度), 相对于水平方向
        """
        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]
        angle_rad = math.atan2(dy, dx)
        return math.degrees(angle_rad)

    @staticmethod
    def estimate_angle_from_lines(lines):
        """
        从多条线段估计主要方向角度(霍夫线检测结果)

        Args:
            lines: 线段列表, 每条 [[x1,y1,x2,y2]] 或 None

        Returns:
            主要方向角度(度)
        """
        if lines is None or len(lines) == 0:
            return None

        angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
            angles.append(angle)

        # 取角度中值
        return float(np.median(angles))

    @staticmethod
    def get_longest_line_angle(contour):
        """
        获取轮廓中最长线段的角度(通过凸包缺陷或骨架)

        Args:
            contour: OpenCV轮廓

        Returns:
            主方向角度(度)
        """
        # 使用最小外接矩形的长边方向
        rect = cv2.minAreaRect(contour)
        w, h = rect[1]
        angle = rect[2]

        # 长边方向
        if w > h:
            return angle
        else:
            return angle + 90

    @staticmethod
    def estimate_perspective_angle(src_pts, dst_pts):
        """
        通过透视变换估计平面倾斜角

        Args:
            src_pts: 源图像4个点 (4,2)
            dst_pts: 目标图像4个点 (4,2)

        Returns:
            (homography_matrix, pitch_deg, yaw_deg)
        """
        src = np.array(src_pts, dtype=np.float32)
        dst = np.array(dst_pts, dtype=np.float32)

        H, _ = cv2.findHomography(src, dst)

        # 分解单应矩阵(简化估算)
        h = H / H[2, 2]
        # 旋转角度近似
        pitch = math.degrees(math.atan2(h[1, 0], h[0, 0]))
        yaw = math.degrees(math.atan2(h[2, 0], h[2, 1]))

        return H, pitch, yaw

    @staticmethod
    def draw_angle_info(frame, contour, color=(0, 255, 0)):
        """
        在图像上绘制角度信息

        Args:
            frame: 输入图像
            contour: 目标轮廓
            color: 绘制颜色

        Returns:
            绘制后的图像, 角度值
        """
        angle = AngleEstimator.get_normalized_angle(contour)

        # 最小外接矩形
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, color, 2)

        # 中心点
        cx = int(rect[0][0])
        cy = int(rect[0][1])

        # 绘制方向线
        rad = math.radians(angle)
        length = max(int(rect[1][0]), int(rect[1][1])) // 2
        ex = int(cx + length * math.cos(rad))
        ey = int(cy + length * math.sin(rad))
        cv2.arrowedLine(frame, (cx, cy), (ex, ey), color, 2, tipLength=0.2)

        # 标注角度
        cv2.putText(frame, f"{angle:.1f} deg", (cx - 40, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        return frame, angle


# ==================== 使用示例 ====================
if __name__ == '__main__':
    # 创建测试图像
    img = np.zeros((400, 400, 3), dtype=np.uint8)

    # 画一个倾斜的矩形
    center = (200, 200)
    size = (120, 60)
    angle = 30
    rect = ((center[0], center[1]), (size[0], size[1]), angle)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(img, [box], 0, (255, 255, 255), -1)

    # 转灰度并找轮廓
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)

        estimator = AngleEstimator()
        print(f"minAreaRect角度: {estimator.get_rotation_angle_minrect(c):.1f}°")
        print(f"归一化角度: {estimator.get_normalized_angle(c):.1f}°")
        print(f"矩方法角度: {estimator.get_orientation_by_moments(c):.1f}°")
        print(f"最长边角度: {estimator.get_longest_line_angle(c):.1f}°")

        result = estimator.estimate_tilt_from_ellipse(c)
        if result:
            print(f"椭圆拟合角度: {result[0]:.1f}°")

        # 绘制
        img2, a = estimator.draw_angle_info(img.copy(), c)
        cv2.imshow("Angle", img2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
