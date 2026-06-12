"""
image_qr_v13.py - QR码识别V13
支持QR码检测、解码、定位及姿态估计，适用于电赛定位/引导场景
"""
import cv2
import numpy as np
import math


class QRCodeDetector:
    """QR码识别器 V13"""

    def __init__(self):
        self._detector = cv2.QRCodeDetector()

    def decode_single(self, img):
        """
        检测并解码单个QR码
        Returns:
            dict: {data, bbox_points, straight_qr} 或 None
        """
        retval, decoded_info, points, straight_qr = self._detector.detectAndDecode(img)
        if retval and decoded_info:
            return {
                "data": decoded_info,
                "points": points.tolist() if points is not None else [],
                "bbox": cv2.boundingRect(points.astype(np.int32)) if points is not None else None,
                "center": tuple(np.mean(points[0], axis=0).astype(int)) if points is not None else None,
                "straight_qr": straight_qr
            }
        return None

    def decode_multi(self, img):
        """
        检测并解码多个QR码
        Returns:
            list of dict
        """
        retval, decoded_info, points, straight_qr_list = self._detector.detectAndDecodeMulti(img)
        results = []
        if retval and decoded_info:
            for i, info in enumerate(decoded_info):
                if info:
                    pts = points[i] if points is not None else None
                    results.append({
                        "data": info,
                        "points": pts.tolist() if pts is not None else [],
                        "bbox": cv2.boundingRect(pts.astype(np.int32)) if pts is not None else None,
                        "center": tuple(np.mean(pts, axis=0).astype(int)) if pts is not None else None
                    })
        return results

    def get_pose(self, img, qr_size_mm=50.0, camera_matrix=None, dist_coeffs=None):
        """
        估计QR码的6DoF姿态
        Args:
            img: 输入图像
            qr_size_mm: QR码实际物理尺寸(mm)
            camera_matrix: 相机内参 3x3, None则使用默认值
            dist_coeffs: 畸变系数
        Returns:
            dict: {rvec, tvec, distance_mm, angles_deg} 或 None
        """
        retval, decoded_info, points, _ = self._detector.detectAndDecode(img)
        if not retval or points is None:
            return None

        h, w = img.shape[:2]
        if camera_matrix is None:
            # 默认相机内参估算
            focal = max(w, h)
            camera_matrix = np.array([
                [focal, 0, w / 2],
                [0, focal, h / 2],
                [0, 0, 1]], dtype=np.float64)
        if dist_coeffs is None:
            dist_coeffs = np.zeros(5)

        # QR码四个角的世界坐标（以中心为原点）
        half = qr_size_mm / 2.0
        obj_points = np.array([
            [-half, -half, 0],
            [half, -half, 0],
            [half, half, 0],
            [-half, half, 0]
        ], dtype=np.float64)

        img_points = points[0].astype(np.float64)
        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
        if not success:
            return None

        # 计算欧拉角
        rot_mat, _ = cv2.Rodrigues(rvec)
        sy = math.sqrt(rot_mat[0, 0] ** 2 + rot_mat[1, 0] ** 2)
        if sy > 1e-6:
            x_angle = math.atan2(rot_mat[2, 1], rot_mat[2, 2])
            y_angle = math.atan2(-rot_mat[2, 0], sy)
            z_angle = math.atan2(rot_mat[1, 0], rot_mat[0, 0])
        else:
            x_angle = math.atan2(-rot_mat[1, 2], rot_mat[1, 1])
            y_angle = math.atan2(-rot_mat[2, 0], sy)
            z_angle = 0

        distance = np.linalg.norm(tvec)

        return {
            "data": decoded_info,
            "rvec": rvec.flatten().tolist(),
            "tvec": tvec.flatten().tolist(),
            "distance_mm": float(distance),
            "angles_deg": {
                "roll": math.degrees(x_angle),
                "pitch": math.degrees(y_angle),
                "yaw": math.degrees(z_angle)
            },
            "center_px": tuple(np.mean(img_points, axis=0).astype(int))
        }

    def preprocess_for_decode(self, img):
        """
        针对模糊/低对比度QR码的预处理
        Returns:
            list of 预处理图像变体
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        variants = []
        # CLAHE增强
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        variants.append(clahe.apply(gray))
        # 自适应二值化
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY, 51, 10)
        variants.append(binary)
        # 锐化
        kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
        variants.append(cv2.filter2D(gray, -1, kernel))
        return variants

    def robust_decode(self, img):
        """
        鲁棒解码：尝试多种预处理后解码
        Returns:
            dict 或 None
        """
        # 直接尝试
        result = self.decode_single(img)
        if result:
            return result
        # 多预处理尝试
        for variant in self.preprocess_for_decode(img):
            result = self.decode_single(cv2.cvtColor(variant, cv2.COLOR_GRAY2BGR) if len(variant.shape) == 2 else variant)
            if result:
                result["preprocess_used"] = True
                return result
        # 多尺度尝试
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        for scale in [1.5, 2.0, 0.5, 0.75]:
            h, w = gray.shape
            resized = cv2.resize(gray, (int(w * scale), int(h * scale)))
            bgr = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
            result = self.decode_single(bgr)
            if result:
                result["scale_used"] = scale
                return result
        return None

    def draw_results(self, img, results):
        """
        在图像上绘制检测结果
        Args:
            img: 原图
            results: decode_single/decode_multi的返回值
        """
        if not isinstance(results, list):
            results = [results]
        vis = img.copy()
        for r in results:
            if r is None:
                continue
            pts = np.array(r["points"], dtype=np.int32)
            cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
            cx, cy = r.get("center", (0, 0))
            cv2.putText(vis, r.get("data", ""), (cx - 30, cy - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return vis


if __name__ == "__main__":
    img = cv2.imread("test_qr.png")
    if img is not None:
        detector = QRCodeDetector()
        result = detector.robust_decode(img)
        if result:
            print(f"QR内容: {result['data']}")
            print(f"中心: {result['center']}")
            # 姿态估计
            pose = detector.get_pose(img, qr_size_mm=50)
            if pose:
                print(f"距离: {pose['distance_mm']:.1f}mm")
                print(f"角度: {pose['angles_deg']}")
        else:
            print("未检测到QR码")
    else:
        print("请准备测试图片 test_qr.png")
