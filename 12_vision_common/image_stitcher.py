"""
图像拼接模块
用于全景视野拼接，支持多张图像自动拼接
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
from enum import Enum


class StitchMethod(Enum):
    """拼接方法"""
    ORB = "orb"         # ORB特征匹配
    SIFT = "sift"       # SIFT特征匹配
    OPENCV = "opencv"   # OpenCV内置拼接器


class ImageStitcher:
    """
    图像拼接器
    支持多种拼接方法，用于全景视野生成
    """
    
    def __init__(self, 
                 method: StitchMethod = StitchMethod.OPENCV,
                 max_features: int = 1000,
                 match_ratio: float = 0.75,
                 min_match_count: int = 10):
        """
        Args:
            method: 拼接方法
            max_features: 最大特征数
            match_ratio: 匹配比率阈值
            min_match_count: 最小匹配数
        """
        self.method = method
        self.max_features = max_features
        self.match_ratio = match_ratio
        self.min_match_count = min_match_count
        
        # 特征检测器
        if method == StitchMethod.ORB:
            self.detector = cv2.ORB_create(nfeatures=max_features)
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        elif method == StitchMethod.SIFT:
            self.detector = cv2.SIFT_create(nfeatures=max_features)
            self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        elif method == StitchMethod.OPENCV:
            self.stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    
    def stitch_two(self, img1: np.ndarray, img2: np.ndarray) -> Tuple[bool, np.ndarray]:
        """
        拼接两张图像
        Args:
            img1: 图像1（左）
            img2: 图像2（右）
        Returns:
            (是否成功, 拼接结果)
        """
        if self.method == StitchMethod.OPENCV:
            return self._stitch_opencv([img1, img2])
        else:
            return self._stitch_feature(img1, img2)
    
    def stitch_multiple(self, images: List[np.ndarray]) -> Tuple[bool, np.ndarray]:
        """
        拼接多张图像
        Args:
            images: 图像列表
        Returns:
            (是否成功, 拼接结果)
        """
        if len(images) < 2:
            return False, images[0] if images else None
        
        if self.method == StitchMethod.OPENCV:
            return self._stitch_opencv(images)
        else:
            return self._stitch_multiple_feature(images)
    
    def _stitch_opencv(self, images: List[np.ndarray]) -> Tuple[bool, np.ndarray]:
        """使用OpenCV内置拼接器"""
        # 确保图像大小一致（可选）
        status, result = self.stitcher.stitch(images)
        
        if status == cv2.Stitcher_OK:
            return True, result
        else:
            error_msgs = {
                cv2.Stitcher_ERR_NEED_MORE_IMGS: "需要更多图像",
                cv2.Stitcher_ERR_HOMOGRAPHY_EST_FAIL: "单应性估计失败",
                cv2.Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL: "相机参数调整失败"
            }
            print(f"拼接失败: {error_msgs.get(status, '未知错误')}")
            return False, None
    
    def _stitch_feature(self, img1: np.ndarray, img2: np.ndarray) -> Tuple[bool, np.ndarray]:
        """使用特征匹配拼接两张图像"""
        # 灰度化
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
        
        # 检测特征
        kp1, desc1 = self.detector.detectAndCompute(gray1, None)
        kp2, desc2 = self.detector.detectAndCompute(gray2, None)
        
        if desc1 is None or desc2 is None:
            return False, None
        
        # 匹配
        matches = self.matcher.knnMatch(desc1, desc2, k=2)
        
        # Lowe's ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < self.match_ratio * n.distance:
                good_matches.append(m)
        
        if len(good_matches) < self.min_match_count:
            print(f"匹配点不足: {len(good_matches)} < {self.min_match_count}")
            return False, None
        
        # 获取匹配点坐标
        pts1 = np.array([kp1[m.queryIdx].pt for m in good_matches])
        pts2 = np.array([kp2[m.trainIdx].pt for m in good_matches])
        
        pts1 = pts1.reshape(-1, 1, 2).astype(np.float32)
        pts2 = pts2.reshape(-1, 1, 2).astype(np.float32)
        
        # 计算单应性矩阵
        H, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC, 5.0)
        
        if H is None:
            return False, None
        
        # 透视变换
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        
        # 计算输出图像大小
        corners2 = np.array([
            [0, 0],
            [w2, 0],
            [w2, h2],
            [0, h2]
        ], dtype=np.float32).reshape(-1, 1, 2)
        
        corners2_transformed = cv2.perspectiveTransform(corners2, H)
        
        corners1 = np.array([
            [0, 0],
            [w1, 0],
            [w1, h1],
            [0, h1]
        ], dtype=np.float32).reshape(-1, 1, 2)
        
        all_corners = np.concatenate([corners1, corners2_transformed], axis=0)
        
        x_min = int(np.floor(all_corners[:, 0, 0].min()))
        x_max = int(np.ceil(all_corners[:, 0, 0].max()))
        y_min = int(np.floor(all_corners[:, 0, 1].min()))
        y_max = int(np.ceil(all_corners[:, 0, 1].max()))
        
        # 平移变换
        translation = np.array([
            [1, 0, -x_min],
            [0, 1, -y_min],
            [0, 0, 1]
        ], dtype=np.float64)
        
        output_size = (x_max - x_min, y_max - y_min)
        
        # 变换图像
        result = cv2.warpPerspective(img2, translation @ H, output_size)
        
        # 放置第一张图
        result[-y_min:h1 - y_min, -x_min:w1 - x_min] = img1
        
        return True, result
    
    def _stitch_multiple_feature(self, images: List[np.ndarray]) -> Tuple[bool, np.ndarray]:
        """使用特征匹配拼接多张图像"""
        result = images[0]
        
        for i in range(1, len(images)):
            success, result = self._stitch_feature(result, images[i])
            if not success:
                print(f"拼接第 {i+1} 张图像失败")
                return False, result
        
        return True, result
    
    def stitch_vertical(self, img_top: np.ndarray, img_bottom: np.ndarray) -> Tuple[bool, np.ndarray]:
        """
        垂直拼接（上下）
        Args:
            img_top: 上方图像
            img_bottom: 下方图像
        Returns:
            (是否成功, 拼接结果)
        """
        # 旋转90度后水平拼接，再旋转回来
        top_rot = cv2.rotate(img_top, cv2.ROTATE_90_CLOCKWISE)
        bottom_rot = cv2.rotate(img_bottom, cv2.ROTATE_90_CLOCKWISE)
        
        success, result_rot = self.stitch_two(top_rot, bottom_rot)
        
        if success:
            result = cv2.rotate(result_rot, cv2.ROTATE_90_COUNTERCLOCKWISE)
            return True, result
        
        return False, None
    
    def create_panorama(self, 
                       images: List[np.ndarray],
                       overlap_ratio: float = 0.3) -> Tuple[bool, np.ndarray]:
        """
        创建全景图
        Args:
            images: 按顺序排列的图像列表（从左到右）
            overlap_ratio: 预期重叠比例
        Returns:
            (是否成功, 全景图)
        """
        if len(images) < 2:
            return False, images[0] if images else None
        
        # 尝试使用OpenCV拼接器
        if self.method == StitchMethod.OPENCV:
            return self._stitch_opencv(images)
        
        # 使用特征匹配
        return self._stitch_multiple_feature(images)
    
    def stitch_with_blend(self, 
                         img1: np.ndarray, 
                         img2: np.ndarray,
                         blend_width: int = 50) -> Tuple[bool, np.ndarray]:
        """
        带混合的拼接（消除接缝）
        Args:
            img1: 图像1
            img2: 图像2
            blend_width: 混合区域宽度
        Returns:
            (是否成功, 拼接结果)
        """
        success, result = self.stitch_two(img1, img2)
        
        if not success:
            return False, None
        
        # 找到重叠区域进行混合
        # 这里使用简单的渐变混合
        h, w = result.shape[:2]
        
        # 转换为灰度查找有效区域
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY) if len(result.shape) == 3 else result
        mask = (gray > 0).astype(np.float32)
        
        # 创建渐变混合掩码
        blend_mask = np.zeros_like(mask)
        
        # 对重叠区域进行混合
        # 查找水平方向的渐变
        for y in range(h):
            row = mask[y, :]
            # 找到第一个和最后一个非零像素
            nonzero = np.nonzero(row)[0]
            if len(nonzero) > 0:
                x_start = nonzero[0]
                x_end = nonzero[-1]
                
                # 创建渐变
                if x_end - x_start > blend_width:
                    gradient = np.linspace(0, 1, blend_width)
                    blend_mask[y, x_start:x_start+blend_width] = gradient
        
        return True, result
    
    def detect_and_crop_black(self, image: np.ndarray) -> np.ndarray:
        """裁剪黑色边框"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # 查找非零区域
        coords = cv2.findNonZero(gray)
        if coords is not None:
            x, y, w, h = cv2.boundingRect(coords)
            return image[y:y+h, x:x+w]
        
        return image
    
    def visualize_matches(self, 
                         img1: np.ndarray, 
                         img2: np.ndarray,
                         max_display: int = 50) -> np.ndarray:
        """可视化特征匹配"""
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
        
        kp1, desc1 = self.detector.detectAndCompute(gray1, None)
        kp2, desc2 = self.detector.detectAndCompute(gray2, None)
        
        if desc1 is None or desc2 is None:
            return np.hstack([img1, img2])
        
        matches = self.matcher.knnMatch(desc1, desc2, k=2)
        
        good_matches = []
        for m, n in matches:
            if m.distance < self.match_ratio * n.distance:
                good_matches.append(m)
        
        # 限制显示数量
        good_matches = sorted(good_matches, key=lambda x: x.distance)[:max_display]
        
        # 绘制匹配
        vis = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, None,
                            matchColor=(0, 255, 0),
                            singlePointColor=(255, 0, 0),
                            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        
        return vis
    
    def get_stitch_info(self, 
                       img1: np.ndarray, 
                       img2: np.ndarray) -> dict:
        """获取拼接信息"""
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
        
        kp1, desc1 = self.detector.detectAndCompute(gray1, None)
        kp2, desc2 = self.detector.detectAndCompute(gray2, None)
        
        if desc1 is None or desc2 is None:
            return {'keypoints1': 0, 'keypoints2': 0, 'matches': 0, 'good_matches': 0}
        
        matches = self.matcher.knnMatch(desc1, desc2, k=2)
        
        good_matches = []
        for m, n in matches:
            if m.distance < self.match_ratio * n.distance:
                good_matches.append(m)
        
        return {
            'keypoints1': len(kp1),
            'keypoints2': len(kp2),
            'total_matches': len(matches),
            'good_matches': len(good_matches),
            'match_ratio': len(good_matches) / max(len(matches), 1)
        }


class PanoramaBuilder:
    """
    全景图构建器
    支持实时拼接和增量构建
    """
    
    def __init__(self, 
                 method: StitchMethod = StitchMethod.OPENCV,
                 max_images: int = 10):
        """
        Args:
            method: 拼接方法
            max_images: 最大图像数量
        """
        self.stitcher = ImageStitcher(method=method)
        self.images: List[np.ndarray] = []
        self.max_images = max_images
        self.panorama = None
    
    def add_image(self, image: np.ndarray) -> bool:
        """
        添加一张图像
        Args:
            image: 输入图像
        Returns:
            是否添加成功
        """
        if len(self.images) >= self.max_images:
            return False
        
        self.images.append(image.copy())
        return True
    
    def build(self) -> Tuple[bool, np.ndarray]:
        """构建全景图"""
        if len(self.images) < 2:
            return False, self.images[0] if self.images else None
        
        success, panorama = self.stitcher.stitch_multiple(self.images)
        
        if success:
            # 裁剪黑色边框
            panorama = self.stitcher.detect_and_crop_black(panorama)
            self.panorama = panorama
        
        return success, panorama
    
    def build_incremental(self, image: np.ndarray) -> Tuple[bool, np.ndarray]:
        """
        增量构建全景图
        Args:
            image: 新图像
        Returns:
            (是否成功, 当前全景图)
        """
        self.images.append(image.copy())
        
        if len(self.images) < 2:
            return True, image
        
        return self.build()
    
    def clear(self):
        """清空图像列表"""
        self.images.clear()
        self.panorama = None
    
    def get_images(self) -> List[np.ndarray]:
        """获取所有图像"""
        return self.images


# 使用示例
if __name__ == "__main__":
    stitcher = ImageStitcher(method=StitchMethod.ORB)
    
    cap = cv2.VideoCapture(0)
    
    panorama_builder = PanoramaBuilder(method=StitchMethod.ORB)
    
    print("按空格键拍照，按回车键拼接，按q退出")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            cv2.imshow("Camera", frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' '):  # 空格键
                panorama_builder.add_image(frame)
                print(f"已添加 {len(panorama_builder.get_images())} 张图像")
            
            elif key == 13:  # 回车键
                success, panorama = panorama_builder.build()
                if success:
                    cv2.imshow("Panorama", panorama)
                    print("全景图生成成功")
                else:
                    print("全景图生成失败")
            
            elif key == ord('q'):
                break
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
