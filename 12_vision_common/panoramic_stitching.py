"""
全景拼接模块 - 多图拼接 + 自动裁剪 + 曝光补偿
功能：基于特征匹配和单应变换实现图像拼接
依赖：opencv-python, numpy
适用：电赛中全景成像、宽视野场景重建等场景
"""

import cv2
import numpy as np
from collections import OrderedDict


class PanoramicStitcher:
    """
    全景图像拼接器
    支持多图像自动拼接、曝光补偿、自动裁剪
    """

    def __init__(self, feature_method='orb', match_ratio=0.75, min_matches=10):
        """
        初始化拼接器
        Args:
            feature_method: 特征检测方法 'orb' / 'sift' / 'akaze'
            match_ratio: 特征匹配比率阈值（Lowe's ratio test）
            min_matches: 最少匹配点数
        """
        self.feature_method = feature_method
        self.match_ratio = match_ratio
        self.min_matches = min_matches

        # 初始化特征检测器
        if feature_method == 'sift':
            self.detector = cv2.SIFT_create(nfeatures=3000)
            self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        elif feature_method == 'akaze':
            self.detector = cv2.AKAZE_create()
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        else:  # orb
            self.detector = cv2.ORB_create(nfeatures=3000)
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    def detect_and_compute(self, img):
        """
        检测特征点并计算描述子
        Args:
            img: 输入图像
        Returns:
            keypoints: 特征点列表
            descriptors: 描述子数组
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        keypoints, descriptors = self.detector.detectAndCompute(gray, None)
        return keypoints, descriptors

    def match_features(self, desc1, desc2):
        """
        特征匹配
        Args:
            desc1: 描述子1
            desc2: 描述子2
        Returns:
            good_matches: 优质匹配列表
        """
        if desc1 is None or desc2 is None:
            return []

        # kNN匹配
        k = min(2, len(desc2))
        matches = self.matcher.knnMatch(desc1, desc2, k=k)

        # Lowe's ratio test
        good_matches = []
        for m in matches:
            if len(m) == 2:
                if m[0].distance < self.match_ratio * m[1].distance:
                    good_matches.append(m[0])
            elif len(m) == 1:
                good_matches.append(m[0])

        return good_matches

    def compute_homography(self, img1, img2):
        """
        计算两张图像间的单应矩阵
        Args:
            img1: 图像1
            img2: 图像2
        Returns:
            H: 单应矩阵 (3x3)
            mask: 内点掩码
            num_inliers: 内点数量
        """
        kp1, desc1 = self.detect_and_compute(img1)
        kp2, desc2 = self.detect_and_compute(img2)

        good_matches = self.match_features(desc1, desc2)

        if len(good_matches) < self.min_matches:
            return None, None, 0

        # 提取匹配点坐标
        pts1 = np.float32([kp1[m.queryIdx].pt for m in good_matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good_matches])

        # RANSAC估计单应矩阵
        H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 5.0)

        if H is None:
            return None, None, 0

        num_inliers = int(mask.sum()) if mask is not None else 0
        return H, mask, num_inliers

    def warp_and_blend(self, img1, img2, H):
        """
        透视变换和图像融合
        Args:
            img1: 待变换图像
            img2: 参考图像
            H: img1到img2的单应矩阵
        Returns:
            result: 拼接结果
        """
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]

        # 计算变换后图像的边界
        corners1 = np.float32([[0, 0], [w1, 0], [w1, h1], [0, h1]]).reshape(-1, 1, 2)
        corners1_transformed = cv2.perspectiveTransform(corners1, H)
        corners2 = np.float32([[0, 0], [w2, 0], [w2, h2], [0, h2]]).reshape(-1, 1, 2)

        all_corners = np.concatenate([corners1_transformed, corners2], axis=0)

        # 计算输出图像大小和偏移
        x_min, y_min = np.int32(all_corners.min(axis=0).ravel())
        x_max, y_max = np.int32(all_corners.max(axis=0).ravel())

        # 平移矩阵（确保没有负坐标）
        translation = np.array([[1, 0, -x_min],
                                 [0, 1, -y_min],
                                 [0, 0, 1]], dtype=np.float64)

        # 透视变换
        output_size = (x_max - x_min, y_max - y_min)
        warped1 = cv2.warpPerspective(img1, translation @ H, output_size)
        warped2 = cv2.warpPerspective(img2, translation, output_size)

        # 融合（加权混合重叠区域）
        mask1 = cv2.warpPerspective(
            np.ones(img1.shape[:2], dtype=np.uint8) * 255,
            translation @ H, output_size
        )
        mask2 = cv2.warpPerspective(
            np.ones(img2.shape[:2], dtype=np.uint8) * 255,
            translation, output_size
        )

        result = self._blend_images(warped1, warped2, mask1, mask2)

        return result

    def _blend_images(self, img1, img2, mask1, mask2):
        """
        渐入渐出融合
        Args:
            img1: 变换后的图像1
            img2: 变换后的图像2
            mask1: 图像1的有效区域
            mask2: 图像2的有效区域
        Returns:
            blended: 融合结果
        """
        # 非重叠区域直接复制
        only1 = (mask1 > 0) & (mask2 == 0)
        only2 = (mask2 > 0) & (mask1 == 0)
        overlap = (mask1 > 0) & (mask2 > 0)

        result = np.zeros_like(img1, dtype=np.float64)
        result[only1] = img1[only1].astype(np.float64)
        result[only2] = img2[only2].astype(np.float64)

        # 重叠区域加权混合
        if np.any(overlap):
            # 距离变换计算权重
            mask1_f = mask1.astype(np.float64) / 255.0
            mask2_f = mask2.astype(np.float64) / 255.0

            dist1 = cv2.distanceTransform(mask1.astype(np.uint8), cv2.DIST_L2, 5)
            dist2 = cv2.distanceTransform(mask2.astype(np.uint8), cv2.DIST_L2, 5)

            # 归一化
            dist1_norm = dist1 / max(dist1.max(), 1)
            dist2_norm = dist2 / max(dist2.max(), 1)

            total = dist1_norm + dist2_norm + 1e-6
            w1 = dist1_norm / total
            w2 = dist2_norm / total

            for c in range(3):
                result[:, :, c] = np.where(
                    overlap,
                    img1[:, :, c].astype(np.float64) * w1 +
                    img2[:, :, c].astype(np.float64) * w2,
                    result[:, :, c]
                )

        return np.clip(result, 0, 255).astype(np.uint8)

    def exposure_compensation(self, img1, img2, mask1, mask2):
        """
        曝光补偿（消除两张图之间的亮度差异）
        Args:
            img1: 图像1
            img2: 图像2
            mask1: 图像1有效区域
            mask2: 图像2有效区域
        Returns:
            img1_comp: 补偿后的图像1
        """
        # 在重叠区域计算亮度差异
        overlap = (mask1 > 0) & (mask2 > 0)
        if not np.any(overlap):
            return img1

        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY).astype(float)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY).astype(float)

        mean1 = np.mean(gray1[overlap])
        mean2 = np.mean(gray2[overlap])

        if mean1 > 0:
            ratio = mean2 / mean1
            # 限制补偿幅度
            ratio = np.clip(ratio, 0.5, 2.0)
            img1_comp = np.clip(img1.astype(float) * ratio, 0, 255).astype(np.uint8)
            return img1_comp

        return img1

    def auto_crop(self, img):
        """
        自动裁剪黑色边框
        Args:
            img: 拼接后的图像
        Returns:
            cropped: 裁剪后的图像
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        _, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

        # 查找最大内接矩形
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return img

        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)

        # 稍微收缩避免边缘黑边
        margin = 5
        x = min(x + margin, img.shape[1] - 1)
        y = min(y + margin, img.shape[0] - 1)
        w = max(w - 2 * margin, 1)
        h = max(h - 2 * margin, 1)

        return img[y:y+h, x:x+w]

    def stitch_pair(self, img1, img2):
        """
        拼接两张图像
        Args:
            img1: 左侧图像
            img2: 右侧图像
        Returns:
            result: 拼接结果
            success: 是否成功
        """
        # 调整大小（提高效率）
        max_dim = 1000
        scale1 = min(max_dim / max(img1.shape[:2]), 1.0)
        scale2 = min(max_dim / max(img2.shape[:2]), 1.0)
        scale = min(scale1, scale2)

        if scale < 1.0:
            img1_s = cv2.resize(img1, None, fx=scale, fy=scale)
            img2_s = cv2.resize(img2, None, fx=scale, fy=scale)
        else:
            img1_s, img2_s = img1, img2

        # 计算单应矩阵
        H, mask, inliers = self.compute_homography(img1_s, img2_s)

        if H is None or inliers < self.min_matches:
            return None, False

        # 如果缩放了，需要调整H
        if scale < 1.0:
            S = np.array([[1/scale, 0, 0], [0, 1/scale, 0], [0, 0, 1]])
            H = S @ H @ np.linalg.inv(S)

        # 透视变换和融合
        result = self.warp_and_blend(img1, img2, H)

        # 自动裁剪
        result = self.auto_crop(result)

        return result, True

    def stitch_multiple(self, images, order='left_to_right'):
        """
        拼接多张图像
        Args:
            images: 图像列表
            order: 拼接顺序 'left_to_right' / 'center_out'
        Returns:
            result: 全景图
        """
        if len(images) < 2:
            return images[0] if images else None

        # 确定拼接顺序
        if order == 'center_out':
            n = len(images)
            indices = [n // 2]
            left, right = n // 2 - 1, n // 2 + 1
            while left >= 0 or right < n:
                if left >= 0:
                    indices.append(left)
                    left -= 1
                if right < n:
                    indices.append(right)
                    right += 1
        else:
            indices = list(range(len(images)))

        # 逐对拼接
        result = images[indices[0]]
        for i in range(1, len(indices)):
            idx = indices[i]
            img = images[idx]

            # 尝试两种拼接方向
            res1, ok1 = self.stitch_pair(result, img)
            res2, ok2 = self.stitch_pair(img, result)

            if ok1 and ok2:
                # 选择更大的结果
                result = res1 if res1.size >= res2.size else res2
            elif ok1:
                result = res1
            elif ok2:
                result = res2
            else:
                print(f"警告: 无法拼接第 {idx} 张图像")
                continue

        return result


# ============================================================
# 实时全景拼接（视频流逐帧拼接）
# ============================================================

class RealTimePanorama:
    """
    实时全景拼接
    从视频流中逐步构建全景图
    """

    def __init__(self, threshold=0.3):
        """
        Args:
            threshold: 触发新拼接的运动阈值
        """
        self.stitcher = PanoramicStitcher()
        self.panorama = None
        self.prev_frame = None
        self.threshold = threshold

    def process_frame(self, frame):
        """
        处理新帧
        Args:
            frame: 当前帧
        Returns:
            panorama: 当前全景图
            updated: 是否有更新
        """
        if self.panorama is None:
            self.panorama = frame.copy()
            self.prev_frame = frame.copy()
            return self.panorama, True

        # 检测运动幅度
        motion = self._compute_motion(self.prev_frame, frame)

        if motion < self.threshold:
            return self.panorama, False

        # 拼接
        result, success = self.stitcher.stitch_pair(self.panorama, frame)

        if success and result is not None:
            self.panorama = result
            self.prev_frame = frame.copy()
            return self.panorama, True

        return self.panorama, False

    def _compute_motion(self, img1, img2):
        """计算两帧之间的运动幅度"""
        g1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        g2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        g1 = cv2.resize(g1, (160, 120))
        g2 = cv2.resize(g2, (160, 120))

        # 特征点匹配数作为运动指标
        orb = cv2.ORB_create(500)
        kp1, d1 = orb.detectAndCompute(g1, None)
        kp2, d2 = orb.detectAndCompute(g2, None)

        if d1 is None or d2 is None:
            return 0.0

        bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        matches = bf.knnMatch(d1, d2, k=2)
        good = [m for m in matches if len(m) == 2 and m[0].distance < 0.7 * m[1].distance]

        return len(good) / max(len(kp1), 1)


# ============================================================
# 使用示例
# ============================================================

def demo_image_stitching(image_paths):
    """多图拼接演示"""
    stitcher = PanoramicStitcher(feature_method='orb')

    images = []
    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            print(f"无法读取: {path}")
            continue
        images.append(img)

    if len(images) < 2:
        print("需要至少2张图片")
        return

    print(f"开始拼接 {len(images)} 张图像...")
    result = stitcher.stitch_multiple(images, order='left_to_right')

    if result is not None:
        cv2.imshow('Panorama', result)
        cv2.imwrite('panorama_result.jpg', result)
        print(f"全景图大小: {result.shape[1]}x{result.shape[0]}")
        print("已保存为 panorama_result.jpg")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("拼接失败")


def demo_camera_panorama():
    """摄像头实时全景拼接演示"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    rtp = RealTimePanorama(threshold=0.2)
    print("实时全景拼接 - 按 s 拼接当前帧, 按 q 退出, 按 r 重置")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow('Camera', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            panorama, updated = rtp.process_frame(frame)
            if updated:
                cv2.imshow('Panorama', panorama)
                print(f"全景图大小: {panorama.shape[1]}x{panorama.shape[0]}")
        elif key == ord('r'):
            rtp = RealTimePanorama()
            cv2.destroyWindow('Panorama')

    cap.release()
    cv2.destroyAllWindows()

    if rtp.panorama is not None:
        cv2.imwrite('panorama_result.jpg', rtp.panorama)
        print("全景图已保存")


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 2:
        demo_image_stitching(sys.argv[1:])
    else:
        demo_camera_panorama()
