"""
人脸交换模块 - 人脸检测 + 特征点 + 仿射变换 + 泊松融合
功能：检测人脸、提取特征点、变换对齐、无缝融合
依赖：opencv-python, numpy
"""

import cv2
import numpy as np


# ==================== 人脸检测器 ====================

class FaceDetector:
    """
    多种方式的人脸检测器
    支持Haar级联 / DNN深度学习
    """

    def __init__(self, method='dnn', confidence=0.5):
        """
        参数:
            method: 'haar' 或 'dnn'
            confidence: DNN检测置信度阈值
        """
        self.method = method
        self.confidence = confidence

        if method == 'haar':
            # Haar级联分类器（OpenCV自带）
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.detector = cv2.CascadeClassifier(cascade_path)
        elif method == 'dnn':
            # 使用OpenCV DNN人脸检测（需要模型文件，这里提供备选）
            self.detector = None

    def detect(self, image):
        """
        检测图像中的人脸
        
        参数:
            image: BGR图像
            
        返回:
            faces: 人脸矩形列表 [(x, y, w, h), ...]
        """
        if self.method == 'haar':
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = self.detector.detectMultiScale(gray, 1.3, 5, minSize=(30, 30))
            return [(x, y, w, h) for (x, y, w, h) in faces]

        elif self.method == 'dnn' and self.detector is not None:
            blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300),
                                          (104.0, 177.0, 123.0))
            self.detector.setInput(blob)
            detections = self.detector.forward()
            faces = []
            h, w = image.shape[:2]
            for i in range(detections.shape[2]):
                conf = detections[0, 0, i, 2]
                if conf > self.confidence:
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    x1, y1, x2, y2 = box.astype(int)
                    faces.append((x1, y1, x2 - x1, y2 - y1))
            return faces

        return []


# ==================== 面部特征点检测 ====================

class FacialLandmarkDetector:
    """
    面部68/5点特征点检测
    使用OpenCV内置或dlib方式
    """

    def __init__(self):
        # 使用简化的5点检测（眼睛、鼻子、嘴角）
        pass

    def detect_landmarks_5(self, image, face_rect):
        """
        检测5个关键特征点（简化版，基于几何估算）
        
        参数:
            image: BGR图像
            face_rect: (x, y, w, h) 人脸矩形
            
        返回:
            5个关键点: 左眼、右眼、鼻尖、左嘴角、右嘴角
        """
        x, y, w, h = face_rect

        # 基于人脸几何比例估算关键点位置
        left_eye = (x + int(w * 0.3), y + int(h * 0.35))
        right_eye = (x + int(w * 0.7), y + int(h * 0.35))
        nose_tip = (x + int(w * 0.5), y + int(h * 0.55))
        left_mouth = (x + int(w * 0.35), y + int(h * 0.75))
        right_mouth = (x + int(w * 0.65), y + int(h * 0.75))

        return np.array([left_eye, right_eye, nose_tip, left_mouth, right_mouth],
                        dtype=np.float32)

    def detect_landmarks_dlib(self, image, face_rect):
        """
        使用dlib检测68个特征点（需安装dlib）
        
        返回:
            68个特征点坐标数组
        """
        try:
            import dlib
            predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
            x, y, w, h = face_rect
            rect = dlib.rectangle(x, y, x + w, y + h)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            shape = predictor(gray, rect)
            landmarks = np.array([(shape.part(i).x, shape.part(i).y)
                                  for i in range(68)], dtype=np.float32)
            return landmarks
        except (ImportError, RuntimeError):
            # 降级到几何估算
            return self.detect_landmarks_5(image, face_rect)


# ==================== 人脸对齐与变换 ====================

class FaceAligner:
    """基于特征点的人脸对齐与仿射变换"""

    @staticmethod
    def get_alignment_matrix(src_points, dst_points):
        """
        计算仿射变换矩阵（相似变换）
        
        参数:
            src_points: 源关键点 (N, 2)
            dst_points: 目标关键点 (N, 2)
            
        返回:
            M: 2x3仿射变换矩阵
        """
        # 最小二乘仿射变换
        src = src_points.astype(np.float32)
        dst = dst_points.astype(np.float32)
        M = cv2.estimateAffinePartial2D(src, dst)[0]
        if M is None:
            M = np.float32([[1, 0, 0], [0, 1, 0]])
        return M

    @staticmethod
    def get_triangles(rect, points):
        """
        获取Delaunay三角剖分（用于分片仿射变换）
        
        参数:
            rect: (x, y, w, h) 包围矩形
            points: 特征点数组
            
        返回:
            triangles: 三角形索引列表 [(i,j,k), ...]
        """
        subdiv = cv2.Subdiv2D(rect)
        for p in points:
            subdiv.insert((float(p[0]), float(p[1])))

        triangles = []
        for t in subdiv.getTriangleList():
            pts = []
            for i in range(0, 6, 2):
                for j, p in enumerate(points):
                    if abs(t[i] - p[0]) < 1 and abs(t[i + 1] - p[1]) < 1:
                        pts.append(j)
                        break
            if len(pts) == 3:
                triangles.append(tuple(pts))

        return triangles

    @staticmethod
    def warp_triangle(src, dst, src_tri, dst_tri):
        """
        对单个三角形区域进行仿射变换
        
        参数:
            src: 源图像
            dst: 目标图像
            src_tri: 源三角形三点坐标 (3, 2)
            dst_tri: 目标三角形三点坐标 (3, 2)
        """
        # 计算包围矩形
        r1 = cv2.boundingRect(np.float32([src_tri]))
        r2 = cv2.boundingRect(np.float32([dst_tri]))

        # 裁剪三角形区域
        src_tri_offset = [(p[0] - r1[0], p[1] - r1[1]) for p in src_tri]
        dst_tri_offset = [(p[0] - r2[0], p[1] - r2[1]) for p in dst_tri]

        src_crop = src[r1[1]:r1[1] + r1[3], r1[0]:r1[0] + r1[2]]

        if src_crop.size == 0:
            return

        # 仿射变换
        M = cv2.getAffineTransform(np.float32(src_tri_offset),
                                    np.float32(dst_tri_offset))
        warped = cv2.warpAffine(src_crop, M, (r2[2], r2[3]),
                                 borderMode=cv2.BORDER_REFLECT_101)

        # 创建三角形掩码
        mask = np.zeros((r2[3], r2[2], 3), dtype=np.float32)
        cv2.fillConvexPoly(mask, np.int32(dst_tri_offset), (1.0, 1.0, 1.0), 16)

        # 融合到目标
        dst_roi = dst[r2[1]:r2[1] + r2[3], r2[0]:r2[0] + r2[2]]
        if dst_roi.shape == warped.shape == mask.shape:
            dst[r2[1]:r2[1] + r2[3], r2[0]:r2[0] + r2[2]] = \
                dst_roi * (1 - mask) + warped * mask


# ==================== 人脸交换主类 ====================

class FaceSwapper:
    """
    人脸交换器：将源人脸换到目标图像上
    流程：检测→特征点→三角剖分→仿射变换→泊松融合
    """

    def __init__(self, method='haar'):
        self.detector = FaceDetector(method=method)
        self.landmark_detector = FacialLandmarkDetector()
        self.aligner = FaceAligner()

    def swap(self, src_image, dst_image, blend_mode='seamless'):
        """
        执行人脸交换
        
        参数:
            src_image: 源人脸图像
            dst_image: 目标图像
            blend_mode: 'seamless'(泊松融合) 或 'mask'(掩码融合)
            
        返回:
            result: 交换后的图像
        """
        # 1. 检测人脸
        src_faces = self.detector.detect(src_image)
        dst_faces = self.detector.detect(dst_image)

        if not src_faces or not dst_faces:
            print("未检测到人脸！")
            return dst_image

        src_face = max(src_faces, key=lambda f: f[2] * f[3])  # 取最大的脸
        dst_face = max(dst_faces, key=lambda f: f[2] * f[3])

        # 2. 获取特征点
        src_landmarks = self.landmark_detector.detect_landmarks_5(src_image, src_face)
        dst_landmarks = self.landmark_detector.detect_landmarks_5(dst_image, dst_face)

        # 3. 计算变换矩阵
        M = self.aligner.get_alignment_matrix(src_landmarks, dst_landmarks)

        # 4. 变换源人脸
        warped_face = cv2.warpAffine(src_image, M, (dst_image.shape[1], dst_image.shape[0]))

        # 5. 创建掩码
        mask = np.zeros(dst_image.shape[:2], dtype=np.uint8)
        # 使用凸包作为掩码
        hull = cv2.convexHull(dst_landmarks.astype(np.int32))
        cv2.fillConvexPoly(mask, hull, 255)

        # 6. 融合
        if blend_mode == 'seamless':
            result = self._seamless_blend(warped_face, dst_image, mask, dst_face)
        else:
            result = self._mask_blend(warped_face, dst_image, mask)

        return result

    def _seamless_blend(self, warped_src, dst, mask, face_rect):
        """
        泊松融合（seamlessClone）
        实现自然的颜色过渡和光照匹配
        """
        x, y, w, h = face_rect
        center = (x + w // 2, y + h // 2)

        try:
            # 使用OpenCV泊松融合
            result = cv2.seamlessClone(warped_src, dst, mask, center,
                                        cv2.MIXED_CLONE)
        except cv2.error:
            # 降级到掩码融合
            result = self._mask_blend(warped_src, dst, mask)

        return result

    def _mask_blend(self, warped_src, dst, mask):
        """掩码融合（简单的alpha混合）"""
        # 羽化掩码边缘
        mask_blur = cv2.GaussianBlur(mask, (21, 21), 10)
        mask_3ch = cv2.merge([mask_blur, mask_blur, mask_blur]).astype(np.float32) / 255.0

        result = dst.astype(np.float32) * (1 - mask_3ch) + \
                 warped_src.astype(np.float32) * mask_3ch
        return result.astype(np.uint8)

    def color_correct(self, src_face, dst_face_region):
        """
        颜色校正：匹配源人脸到目标肤色
        
        参数:
            src_face: 源人脸区域
            dst_face_region: 目标人脸区域
            
        返回:
            corrected: 颜色校正后的源人脸
        """
        # 转换到LAB颜色空间
        src_lab = cv2.cvtColor(src_face, cv2.COLOR_BGR2LAB).astype(np.float32)
        dst_lab = cv2.cvtColor(dst_face_region, cv2.COLOR_BGR2LAB).astype(np.float32)

        # 计算均值和标准差
        src_mean, src_std = cv2.meanStdDev(src_face)
        dst_mean, dst_std = cv2.meanStdDev(dst_face_region)

        # 逐通道标准化
        result = np.zeros_like(src_lab)
        for i in range(3):
            result[:, :, i] = (src_lab[:, :, i] - src_mean[i][0]) * \
                               (dst_std[i][0] / (src_std[i][0] + 1e-6)) + dst_mean[i][0]

        result = np.clip(result, 0, 255).astype(np.uint8)
        return cv2.cvtColor(result, cv2.COLOR_LAB2BGR)


# ==================== 使用示例 ====================

def example_face_swap():
    """人脸交换完整示例"""
    # 加载图像
    src = cv2.imread("face_source.jpg")
    dst = cv2.imread("face_target.jpg")

    if src is None or dst is None:
        print("请准备两张含人脸的图像: face_source.jpg, face_target.jpg")
        # 创建模拟测试
        src = np.zeros((200, 200, 3), dtype=np.uint8)
        dst = np.zeros((400, 400, 3), dtype=np.uint8)
        cv2.circle(src, (100, 100), 80, (200, 180, 160), -1)
        cv2.circle(dst, (200, 200), 100, (160, 180, 200), -1)

    # 创建交换器
    swapper = FaceSwapper(method='haar')

    # 执行交换
    result = swapper.swap(src, dst, blend_mode='seamless')

    # 显示
    cv2.imshow("Source", src)
    cv2.imshow("Destination", dst)
    cv2.imshow("Result", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 保存
    cv2.imwrite("face_swap_result.jpg", result)
    print("结果已保存: face_swap_result.jpg")


def example_color_correction():
    """颜色校正示例"""
    face1 = cv2.imread("face1.jpg")
    face2 = cv2.imread("face2.jpg")

    if face1 is not None and face2 is not None:
        swapper = FaceSwapper()
        corrected = swapper.color_correct(face1, face2)
        cv2.imshow("Original", face1)
        cv2.imshow("Corrected", corrected)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    print("=== 人脸交换模块 ===")
    print("功能: 人脸检测→特征点→仿射变换→泊松融合")
    example_face_swap()
