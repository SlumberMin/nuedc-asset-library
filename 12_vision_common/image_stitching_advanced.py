"""
高级图像拼接模块 - 特征匹配 + RANSAC + 全景拼接
适用于电赛中全景图像生成、多相机画面融合等任务。
"""

import cv2
import numpy as np
from threading import Thread, Lock
import time


class ImageStitcher:
    """
    高级图像拼接器。
    支持多种特征检测器、RANSAC变换估计和多图全景拼接。
    """

    def __init__(
        self,
        feature_method="orb",
        matcher_type="bf",
        ransac_threshold=5.0,
        min_matches=10,
        blend_alpha=0.5,
    ):
        """
        初始化图像拼接器。

        参数:
            feature_method: 特征检测方法 ('orb', 'sift', 'akaze')
            matcher_type: 匹配器类型 ('bf'=暴力匹配, 'flann'=FLANN)
            ransac_threshold: RANSAC重投影误差阈值
            min_matches: 最少匹配点数
            blend_alpha: 图像混合透明度
        """
        self.feature_method = feature_method
        self.matcher_type = matcher_type
        self.ransac_threshold = ransac_threshold
        self.min_matches = min_matches
        self.blend_alpha = blend_alpha

        # 初始化特征检测器
        self._init_detector()

        # 初始化匹配器
        self._init_matcher()

        # 结果缓存
        self._lock = Lock()
        self._last_homography = None
        self._last_matches = None
        self._inlier_count = 0

    def _init_detector(self):
        """初始化特征检测器。"""
        if self.feature_method == "orb":
            self.detector = cv2.ORB_create(nfeatures=3000)
        elif self.feature_method == "sift":
            self.detector = cv2.SIFT_create(nfeatures=3000)
        elif self.feature_method == "akaze":
            self.detector = cv2.AKAZE_create()
        else:
            raise ValueError(f"不支持的特征检测方法: {self.feature_method}")

    def _init_matcher(self):
        """初始化特征匹配器。"""
        if self.matcher_type == "bf":
            if self.feature_method in ("orb", "akaze"):
                self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
            else:
                self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        elif self.matcher_type == "flann":
            if self.feature_method in ("orb", "akaze"):
                # 二进制描述子用LSH索引
                index_params = dict(
                    algorithm=6, table_number=6, key_size=12, multi_probe_level=1
                )
            else:
                # 浮点描述子用KD树
                index_params = dict(algorithm=1, trees=5)
            search_params = dict(checks=50)
            self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

    def detect_and_compute(self, img):
        """
        检测特征点并计算描述子。

        参数:
            img: 输入图像（BGR或灰度）
        返回:
            keypoints: 特征点列表
            descriptors: 描述子数组
        """
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        keypoints, descriptors = self.detector.detectAndCompute(gray, None)
        return keypoints, descriptors

    def match_features(self, desc1, desc2):
        """
        特征匹配（带Lowe比率测试）。

        参数:
            desc1, desc2: 两组描述子
        返回:
            good_matches: 筛选后的优质匹配列表
        """
        if desc1 is None or desc2 is None:
            return []
        if len(desc1) < 2 or len(desc2) < 2:
            return []

        # KNN匹配（k=2用于比率测试）
        matches = self.matcher.knnMatch(desc1, desc2, k=2)

        # Lowe比率测试
        good_matches = []
        for m_pair in matches:
            if len(m_pair) == 2:
                m, n = m_pair
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)

        return good_matches

    def estimate_homography(self, kp1, kp2, matches):
        """
        使用RANSAC估计单应性矩阵。

        参数:
            kp1, kp2: 两组特征点
            matches: 匹配列表
        返回:
            H: 3x3单应性矩阵（失败返回None）
            mask: 内点掩码
        """
        if len(matches) < self.min_matches:
            return None, None

        # 提取匹配点坐标
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # RANSAC估计
        H, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC, self.ransac_threshold)

        if H is not None and mask is not None:
            self._inlier_count = int(mask.sum())
            return H, mask

        return None, None

    def stitch_two_images(self, img_left, img_right):
        """
        拼接两张图像（img_right变换到img_left坐标系）。

        参数:
            img_left: 左侧（参考）图像
            img_right: 右侧（待变换）图像
        返回:
            result: 拼接结果图像
            info: 拼接信息字典
        """
        # 检测特征
        kp1, desc1 = self.detect_and_compute(img_left)
        kp2, desc2 = self.detect_and_compute(img_right)

        # 特征匹配
        good_matches = self.match_features(desc1, desc2)

        info = {
            "kp_left_count": len(kp1),
            "kp_right_count": len(kp2),
            "match_count": len(good_matches),
            "inlier_count": 0,
            "success": False,
        }

        if len(good_matches) < self.min_matches:
            print(f"[警告] 匹配点不足: {len(good_matches)} < {self.min_matches}")
            return None, info

        # 估计单应性矩阵
        H, mask = self.estimate_homography(kp1, kp2, good_matches)

        if H is None:
            print("[警告] 单应性矩阵估计失败")
            return None, info

        info["inlier_count"] = self._inlier_count
        info["success"] = True

        with self._lock:
            self._last_homography = H
            self._last_matches = good_matches

        # 图像变换与拼接
        h_left, w_left = img_left.shape[:2]
        h_right, w_right = img_right.shape[:2]

        # 计算输出尺寸
        corners_right = np.float32(
            [[0, 0], [w_right, 0], [w_right, h_right], [0, h_right]]
        ).reshape(-1, 1, 2)
        corners_transformed = cv2.perspectiveTransform(corners_right, H)

        corners_left = np.float32(
            [[0, 0], [w_left, 0], [w_left, h_left], [0, h_left]]
        ).reshape(-1, 1, 2)

        all_corners = np.concatenate([corners_left, corners_transformed], axis=0)

        [x_min, y_min] = np.int32(all_corners.min(axis=0).ravel())
        [x_max, y_max] = np.int32(all_corners.max(axis=0).ravel())

        # 平移矩阵（处理负坐标）
        translation = np.array([[1, 0, -x_min], [0, 1, -y_min], [0, 0, 1]])

        output_size = (x_max - x_min, y_max - y_min)

        # 透视变换
        warped_right = cv2.warpPerspective(
            img_right, translation.dot(H), output_size
        )
        warped_left = cv2.warpPerspective(
            img_left, translation, output_size
        )

        # 混合拼接（简单加权混合）
        # 创建非零区域掩码
        mask_left = cv2.cvtColor(warped_left, cv2.COLOR_BGR2GRAY) > 0
        mask_right = cv2.cvtColor(warped_right, cv2.COLOR_BGR2GRAY) > 0
        overlap = mask_left & mask_right

        result = warped_left.copy()
        result[mask_right & ~mask_left] = warped_right[mask_right & ~mask_left]

        # 重叠区域渐变混合
        if np.any(overlap):
            # 简单alpha混合
            result[overlap] = (
                warped_left[overlap] * self.blend_alpha
                + warped_right[overlap] * (1 - self.blend_alpha)
            ).astype(np.uint8)

        return result, info

    def draw_matches(self, img1, kp1, img2, kp2, matches, mask=None):
        """
        可视化特征匹配结果。

        参数:
            img1, img2: 两幅图像
            kp1, kp2: 特征点
            matches: 匹配列表
            mask: 内点掩码
        返回:
            match_img: 匹配可视化图像
        """
        if mask is not None:
            matches_mask = mask.ravel().tolist()
            draw_params = dict(
                matchColor=(0, 255, 0),
                singlePointColor=(255, 0, 0),
                matchesMask=matches_mask,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
            )
        else:
            draw_params = dict(
                matchColor=(0, 255, 0),
                singlePointColor=(255, 0, 0),
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
            )

        match_img = cv2.drawMatches(
            img1, kp1, img2, kp2, matches, None, **draw_params
        )
        return match_img

    def stitch_panorama(self, images):
        """
        多图全景拼接（顺序拼接）。

        参数:
            images: 图像列表（按顺序排列）
        返回:
            panorama: 全景图像
        """
        if len(images) < 2:
            return images[0] if images else None

        result = images[0]
        for i in range(1, len(images)):
            print(f"[信息] 拼接第 {i}/{len(images)-1} 对...")
            stitched, info = self.stitch_two_images(result, images[i])
            if stitched is not None:
                result = stitched
                print(f"  成功！内点数: {info['inlier_count']}")
            else:
                print(f"  失败，跳过第 {i+1} 张图像")

        return result

    def auto_crop(self, image):
        """
        自动裁剪拼接结果的黑色边框。

        参数:
            image: 拼接结果图像
        返回:
            裁剪后的图像
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
            return image[y : y + h, x : x + w]
        return image


class RealTimeStitcher:
    """实时双摄拼接器（多线程）。"""

    def __init__(self, src_left=0, src_right=1, width=640, height=480):
        """
        初始化实时拼接器。

        参数:
            src_left, src_right: 左右摄像头索引
            width, height: 画面尺寸
        """
        self.src_left = src_left
        self.src_right = src_right
        self.width = width
        self.height = height

        self.stitcher = ImageStitcher(feature_method="orb", matcher_type="bf")

        self._lock = Lock()
        self._frame_left = None
        self._frame_right = None
        self._result = None
        self._running = False
        self._fps = 0.0

    def _capture_loop(self):
        """双摄捕获主循环。"""
        cap_l = cv2.VideoCapture(self.src_left)
        cap_r = cv2.VideoCapture(self.src_right)

        cap_l.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap_l.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap_r.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap_r.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not cap_l.isOpened() or not cap_r.isOpened():
            print("[错误] 无法打开摄像头")
            self._running = False
            return

        fps_counter = 0
        fps_timer = time.time()

        while self._running:
            ret_l, frame_l = cap_l.read()
            ret_r, frame_r = cap_r.read()
            if not ret_l or not ret_r:
                break

            frame_l = cv2.resize(frame_l, (self.width, self.height))
            frame_r = cv2.resize(frame_r, (self.width, self.height))

            # 拼接
            stitched, info = self.stitcher.stitch_two_images(frame_l, frame_r)

            fps_counter += 1
            if time.time() - fps_timer >= 1.0:
                self._fps = fps_counter / (time.time() - fps_timer)
                fps_counter = 0
                fps_timer = time.time()

            with self._lock:
                self._frame_left = frame_l
                self._frame_right = frame_r
                self._result = stitched

        cap_l.release()
        cap_r.release()

    def start(self):
        """启动实时拼接。"""
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        print("[信息] 实时图像拼接已启动")

    def stop(self):
        """停止。"""
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=2.0)
        print("[信息] 实时图像拼接已停止")

    def get_result(self):
        """获取拼接结果。"""
        with self._lock:
            return self._result.copy() if self._result is not None else None

    def get_source_frames(self):
        """获取左右原始帧。"""
        with self._lock:
            l = self._frame_left.copy() if self._frame_left is not None else None
            r = self._frame_right.copy() if self._frame_right is not None else None
            return l, r


def stitch_from_files(image_paths, output_path=None, **kwargs):
    """
    从文件列表拼接全景图。

    参数:
        image_paths: 图像文件路径列表
        output_path: 输出路径（可选）
        **kwargs: ImageStitcher参数
    返回:
        panorama: 全景图像
    """
    images = []
    for p in image_paths:
        img = cv2.imread(p)
        if img is not None:
            images.append(img)
        else:
            print(f"[警告] 无法读取: {p}")

    if len(images) < 2:
        print("[错误] 至少需要2张图像")
        return None

    stitcher = ImageStitcher(**kwargs)
    panorama = stitcher.stitch_panorama(images)

    if panorama is not None:
        panorama = stitcher.auto_crop(panorama)
        if output_path:
            cv2.imwrite(output_path, panorama)
            print(f"[信息] 全景图已保存: {output_path}")

    return panorama


def main():
    """
    使用示例：实时双摄拼接。
    按 'q' 退出。
    """
    stitcher = RealTimeStitcher(src_left=0, src_right=1, width=640, height=480)
    stitcher.start()

    try:
        while True:
            result = stitcher.get_result()
            if result is not None:
                # 缩放到显示尺寸
                h, w = result.shape[:2]
                max_w = 1200
                if w > max_w:
                    scale = max_w / w
                    result = cv2.resize(result, (max_w, int(h * scale)))
                cv2.imshow("Panorama Stitching", result)

            # 显示源帧
            l, r = stitcher.get_source_frames()
            if l is not None and r is not None:
                combined = np.hstack([l, r])
                cv2.resize(combined, (640, 240))
                cv2.imshow("Source L+R", cv2.resize(combined, (640, 240)))

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        stitcher.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
