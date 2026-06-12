#!/usr/bin/env python3
"""
单应性变换(Homography)工具
功能：图像透视变换 + 平面校正 + 鸟瞰图生成
适用：OpenCV + Orange Pi 5

应用场景:
  - 将倾斜拍摄的平面校正为正视图
  - 生成鸟瞰图(Bird's Eye View)
  - 文档扫描/名片扫描
  - 地面标定
"""

import cv2
import numpy as np


class HomographyEstimator:
    """单应性变换估计器"""

    def __init__(self, camera_matrix=None, dist_coeffs=None):
        """
        参数:
            camera_matrix: 3x3相机内参矩阵
            dist_coeffs: 畸变系数
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros((5, 1))
        self.homography_matrix = None
        self.homography_inv = None

    def find_homography(self, src_points, dst_points, method='ransac',
                         ransac_threshold=5.0):
        """
        计算单应性矩阵
        参数:
            src_points: 源点列表 Nx2或Nx1x2
            dst_points: 目标点列表 Nx2或Nx1x2
            method: 'ransac' | 'least_squares' | 'lmeds'
            ransac_threshold: RANSAC内点阈值(像素)
        返回:
            H: 3x3单应性矩阵, None表示失败
        """
        src = np.array(src_points, dtype=np.float32).reshape(-1, 1, 2)
        dst = np.array(dst_points, dtype=np.float32).reshape(-1, 1, 2)

        if len(src) < 4:
            print("[Homography] 错误: 至少需要4个点对")
            return None

        if method == 'ransac':
            method_flag = cv2.RANSAC
        elif method == 'lmeds':
            method_flag = cv2.LMEDS
        else:
            method_flag = 0

        H, mask = cv2.findHomography(src, dst, method_flag, ransac_threshold)
        if H is not None:
            self.homography_matrix = H
            self.homography_inv = np.linalg.inv(H)
        return H

    def compute_homography_from_corners(self, src_corners, dst_corners):
        """
        从四角点计算单应性矩阵
        参数:
            src_corners: 源四角 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
            dst_corners: 目标四角 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
        """
        return self.find_homography(src_corners, dst_corners)

    def warp_perspective(self, image, output_size=None):
        """
        透视变换
        参数:
            image: 输入图像
            output_size: (width, height) 输出尺寸
        返回:
            变换后的图像
        """
        if self.homography_matrix is None:
            print("[Homography] 错误: 未计算单应性矩阵")
            return image

        if output_size is None:
            h, w = image.shape[:2]
            output_size = (w, h)

        return cv2.warpPerspective(image, self.homography_matrix, output_size)

    def warp_point(self, point):
        """
        将点从源平面映射到目标平面
        参数:
            point: (x, y) 或 np.array
        返回:
            (x', y') 映射后的点
        """
        if self.homography_matrix is None:
            return point

        pt = np.array([point[0], point[1], 1.0], dtype=np.float64)
        pt_warped = self.homography_matrix @ pt
        if pt_warped[2] != 0:
            pt_warped /= pt_warped[2]
        return (float(pt_warped[0]), float(pt_warped[1]))

    def warp_point_inverse(self, point):
        """逆映射: 从目标平面映射回源平面"""
        if self.homography_inv is None:
            return point

        pt = np.array([point[0], point[1], 1.0], dtype=np.float64)
        pt_warped = self.homography_inv @ pt
        if pt_warped[2] != 0:
            pt_warped /= pt_warped[2]
        return (float(pt_warped[0]), float(pt_warped[1]))


class BirdEyeView:
    """鸟瞰图生成器"""

    def __init__(self, camera_matrix=None, dist_coeffs=None):
        self.estimator = HomographyEstimator(camera_matrix, dist_coeffs)
        self.road_width = None
        self.road_height = None

    def calibrate_from_road(self, frame, road_corners, bird_size=(600, 400)):
        """
        从道路四角点标定鸟瞰图
        参数:
            frame: 输入帧
            road_corners: 道路四角 [(x1,y1)...] 按左上、右上、右下、左下顺序
            bird_size: (width, height) 鸟瞰图尺寸
        """
        self.road_width = bird_size[0]
        self.road_height = bird_size[1]

        dst_corners = np.array([
            [0, 0],
            [bird_size[0] - 1, 0],
            [bird_size[0] - 1, bird_size[1] - 1],
            [0, bird_size[1] - 1]
        ], dtype=np.float32)

        return self.estimator.find_homography(road_corners, dst_corners)

    def get_bird_eye(self, frame):
        """获取鸟瞰图"""
        if self.road_width is None or self.road_height is None:
            return frame
        return self.estimator.warp_perspective(frame, (self.road_width, self.road_height))


class DocumentScanner:
    """文档扫描器 - 将倾斜文档校正为正视图"""

    def __init__(self, camera_matrix=None, dist_coeffs=None):
        self.estimator = HomographyEstimator(camera_matrix, dist_coeffs)

    def scan_document(self, frame, doc_corners):
        """
        扫描文档
        参数:
            frame: 输入帧
            doc_corners: 文档四角 [(x1,y1)...] 按左上、右上、右下、左下顺序
        返回:
            校正后的文档图像
        """
        if len(doc_corners) != 4:
            print("[Scanner] 错误: 需要4个角点")
            return frame

        src = np.array(doc_corners, dtype=np.float32)

        # 计算输出尺寸
        w_top = np.linalg.norm(src[1] - src[0])
        w_bottom = np.linalg.norm(src[2] - src[3])
        h_left = np.linalg.norm(src[3] - src[0])
        h_right = np.linalg.norm(src[2] - src[1])

        out_w = int(max(w_top, w_bottom))
        out_h = int(max(h_left, h_right))

        dst = np.array([
            [0, 0],
            [out_w - 1, 0],
            [out_w - 1, out_h - 1],
            [0, out_h - 1]
        ], dtype=np.float32)

        H = self.estimator.find_homography(src, dst)
        if H is None:
            return frame

        return cv2.warpPerspective(frame, H, (out_w, out_h))


def auto_detect_document_corners(frame, min_area=10000):
    """
    自动检测文档/矩形区域的四个角点
    返回: [(x1,y1), (x2,y2), (x3,y3), (x4,y4)] 或 None
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 75, 200)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges = cv2.dilate(edges, kernel, iterations=2)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            break

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        if len(approx) == 4:
            pts = approx.reshape(4, 2)
            # 排序: 左上、右上、右下、左下
            s = pts.sum(axis=1)
            d = np.diff(pts, axis=1)
            corners = np.array([
                pts[np.argmin(s)],     # 左上
                pts[np.argmin(d)],     # 右上
                pts[np.argmax(s)],     # 右下
                pts[np.argmax(d)],     # 左下
            ], dtype=np.float32)
            return corners.tolist()

    return None


def demo():
    """演示文档扫描"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    scanner = DocumentScanner()
    print("文档扫描器启动 (按q退出, 按s扫描)")

    corners = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        vis = frame.copy()

        # 尝试自动检测
        auto_corners = auto_detect_document_corners(frame)
        if auto_corners is not None:
            pts = np.array(auto_corners, dtype=np.int32)
            cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
            for i, (x, y) in enumerate(auto_corners):
                cv2.circle(vis, (int(x), int(y)), 5, (0, 0, 255), -1)
                cv2.putText(vis, str(i), (int(x)+10, int(y)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            corners = auto_corners

        cv2.putText(vis, "Press 's' to scan, 'q' to quit", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Document Scanner", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s') and corners is not None:
            scanned = scanner.scan_document(frame, corners)
            cv2.imshow("Scanned Document", scanned)
            cv2.waitKey(0)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo()
