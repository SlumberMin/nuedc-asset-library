#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多相机融合模块 - Multi-Camera Fusion
======================================
针对 Orange Pi 5 优化的多相机处理算法
包含：双目标定、鱼眼校正、图像拼接、立体匹配

技术栈：OpenCV + NumPy + 多线程优化
适配：Orange Pi 5 (RK3588S) / Linux ARM64

作者：nuedc-asset-library
"""

import cv2
import numpy as np
import threading
import time
import os
import json


class StereoCalibrator:
    """
    双目相机标定器

    功能：
    - 单目标定
    - 双目标定
    - 立体校正
    - 标定结果保存/加载

    使用示例：
        calibrator = StereoCalibrator()
        calibrator.calibrate(left_images, right_images, pattern_size=(9,6))
        calibrator.save('stereo_calib.npz')
    """

    def __init__(self, pattern_size=(9, 6), square_size=25.0):
        """
        初始化标定器

        参数：
            pattern_size: 棋盘格内角点数 (列, 行)
            square_size: 方格尺寸（毫米）
        """
        self.pattern_size = pattern_size
        self.square_size = square_size

        # 生成棋盘格世界坐标
        self.obj_points = []  # 3D点
        self.img_points_l = []  # 左图2D点
        self.img_points_r = []  # 右图2D点

        # 标定结果
        self.camera_matrix_l = None
        self.dist_coeffs_l = None
        self.camera_matrix_r = None
        self.dist_coeffs_r = None
        self.R = None  # 旋转矩阵
        self.T = None  # 平移向量
        self.E = None  # 本征矩阵
        self.F = None  # 基础矩阵

        # 校正映射
        self.rectify_map_l = None
        self.rectify_map_r = None
        self.Q = None  # 视差-深度映射矩阵

        print(f"[双目标定] 棋盘格: {pattern_size}, 方格大小: {square_size}mm")

    def _generate_object_points(self, n_images):
        """
        生成棋盘格3D坐标

        参数：
            n_images: 图像数量

        返回：
            obj_points: 3D点列表
        """
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0],
                                0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        return [objp.copy() for _ in range(n_images)]

    def find_chessboard(self, image):
        """
        检测棋盘格角点

        参数：
            image: 输入图像（灰度或BGR）

        返回：
            found: 是否找到
            corners: 角点坐标
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        # 查找棋盘格角点
        found, corners = cv2.findChessboardCorners(
            gray, self.pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                  cv2.CALIB_CB_NORMALIZE_IMAGE +
                  cv2.CALIB_CB_FAST_CHECK
        )

        if found:
            # 亚像素精化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        return found, corners

    def calibrate_single(self, images):
        """
        单目标定

        参数：
            images: 图像列表

        返回：
            camera_matrix: 相机内参矩阵
            dist_coeffs: 畸变系数
            rvecs: 旋转向量
            tvecs: 平移向量
        """
        obj_points = []
        img_points = []

        for img in images:
            found, corners = self.find_chessboard(img)
            if found:
                obj_points.append(
                    np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
                )
                obj_points[-1][:, :2] = np.mgrid[0:self.pattern_size[0],
                                                   0:self.pattern_size[1]].T.reshape(-1, 2)
                obj_points[-1] *= self.square_size
                img_points.append(corners)

        if len(obj_points) < 3:
            print("[标定] 图像不足，至少需要3张")
            return None, None, None, None

        h, w = images[0].shape[:2]
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, (w, h), None, None
        )

        print(f"[单目标定] 重投影误差: {ret:.4f}")
        return camera_matrix, dist_coeffs, rvecs, tvecs

    def calibrate_stereo(self, left_images, right_images):
        """
        双目标定

        参数：
            left_images: 左相机图像列表
            right_images: 右相机图像列表

        返回：
            success: 是否成功
        """
        assert len(left_images) == len(right_images), "左右图像数量必须相同"

        obj_points_all = []
        img_points_l_all = []
        img_points_r_all = []

        h, w = left_images[0].shape[:2]
        image_size = (w, h)

        for left_img, right_img in zip(left_images, right_images):
            found_l, corners_l = self.find_chessboard(left_img)
            found_r, corners_r = self.find_chessboard(right_img)

            if found_l and found_r:
                objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:self.pattern_size[0],
                                        0:self.pattern_size[1]].T.reshape(-1, 2)
                objp *= self.square_size

                obj_points_all.append(objp)
                img_points_l_all.append(corners_l)
                img_points_r_all.append(corners_r)

        if len(obj_points_all) < 3:
            print("[双目标定] 有效图像对不足")
            return False

        # 单目标定
        print("[双目标定] 左相机标定...")
        ret_l, self.camera_matrix_l, self.dist_coeffs_l, _, _ = cv2.calibrateCamera(
            obj_points_all, img_points_l_all, image_size, None, None
        )

        print("[双目标定] 右相机标定...")
        ret_r, self.camera_matrix_r, self.dist_coeffs_r, _, _ = cv2.calibrateCamera(
            obj_points_all, img_points_r_all, image_size, None, None
        )

        # 双目标定
        print("[双目标定] 立体标定...")
        ret, self.camera_matrix_l, self.dist_coeffs_l, \
            self.camera_matrix_r, self.dist_coeffs_r, \
            self.R, self.T, self.E, self.F = cv2.stereoCalibrate(
            obj_points_all, img_points_l_all, img_points_r_all,
            self.camera_matrix_l, self.dist_coeffs_l,
            self.camera_matrix_r, self.dist_coeffs_r,
            image_size,
            flags=cv2.CALIB_FIX_INTRINSIC
        )

        print(f"[双目标定] 重投影误差: {ret:.4f}")

        # 立体校正
        self._compute_rectification(image_size)

        return True

    def _compute_rectification(self, image_size):
        """
        计算立体校正映射

        参数：
            image_size: 图像尺寸 (w, h)
        """
        # 立体校正
        (self.R1, self.R2, self.P1, self.P2, self.Q,
         roi1, roi2) = cv2.stereoRectify(
            self.camera_matrix_l, self.dist_coeffs_l,
            self.camera_matrix_r, self.dist_coeffs_r,
            image_size, self.R, self.T,
            alpha=0, newImageSize=image_size
        )

        # 计算校正映射
        self.rectify_map_l = cv2.initUndistortRectifyMap(
            self.camera_matrix_l, self.dist_coeffs_l,
            self.R1, self.P1, image_size, cv2.CV_32FC1
        )

        self.rectify_map_r = cv2.initUndistortRectifyMap(
            self.camera_matrix_r, self.dist_coeffs_r,
            self.R2, self.P2, image_size, cv2.CV_32FC1
        )

        self.image_size = image_size
        print("[双目标定] 校正映射已计算")

    def rectify(self, left_img, right_img):
        """
        立体校正

        参数：
            left_img: 左图像
            right_img: 右图像

        返回：
            rectified_l: 校正后的左图
            rectified_r: 校正后的右图
        """
        if self.rectify_map_l is None:
            print("[校正] 未标定，无法校正")
            return left_img, right_img

        rectified_l = cv2.remap(left_img, self.rectify_map_l[0], self.rectify_map_l[1],
                                 cv2.INTER_LINEAR)
        rectified_r = cv2.remap(right_img, self.rectify_map_r[0], self.rectify_map_r[1],
                                 cv2.INTER_LINEAR)

        return rectified_l, rectified_r

    def save(self, filepath):
        """
        保存标定结果

        参数：
            filepath: 文件路径 (.npz)
        """
        np.savez(filepath,
                  camera_matrix_l=self.camera_matrix_l,
                  dist_coeffs_l=self.dist_coeffs_l,
                  camera_matrix_r=self.camera_matrix_r,
                  dist_coeffs_r=self.dist_coeffs_r,
                  R=self.R, T=self.T, E=self.E, F=self.F,
                  R1=self.R1, R2=self.R2,
                  P1=self.P1, P2=self.P2, Q=self.Q,
                  image_size=np.array(self.image_size))
        print(f"[标定] 已保存到: {filepath}")

    def load(self, filepath):
        """
        加载标定结果

        参数：
            filepath: 文件路径 (.npz)
        """
        data = np.load(filepath)
        self.camera_matrix_l = data['camera_matrix_l']
        self.dist_coeffs_l = data['dist_coeffs_l']
        self.camera_matrix_r = data['camera_matrix_r']
        self.dist_coeffs_r = data['dist_coeffs_r']
        self.R = data['R']
        self.T = data['T']
        self.E = data['E']
        self.F = data['F']
        self.R1 = data['R1']
        self.R2 = data['R2']
        self.P1 = data['P1']
        self.P2 = data['P2']
        self.Q = data['Q']

        # 重新计算校正映射
        # 从保存的image_size加载（npz中已保存）
        if 'image_size' in data:
            img_size = tuple(data['image_size'])
            self.image_size = img_size
            self._compute_rectification(img_size)
        else:
            # 旧版npz可能未保存image_size，需用户手动设置
            print(f"[标定] 已从 {filepath} 加载（需手动调用rectify前设置image_size）")
            return
        print(f"[标定] 已从 {filepath} 加载")


class FisheyeCorrector:
    """
    鱼眼镜头校正器

    功能：
    - 鱼眼畸变校正
    - 视场角保持
    - 多种校正模式

    使用示例：
        corrector = FisheyeCorrector()
        corrected = corrector.correct(fisheye_image)
    """

    def __init__(self, camera_matrix=None, dist_coeffs=None, balance=0.0):
        """
        初始化鱼眼校正器

        参数：
            camera_matrix: 相机内参矩阵
            dist_coeffs: 鱼眼畸变系数 (k1, k2, k3, k4)
            balance: 平衡参数 (0=裁剪, 1=保留全部)
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.balance = balance
        self.map1 = None
        self.map2 = None

        print("[鱼眼校正] 初始化完成")

    def calibrate(self, images, pattern_size=(9, 6), square_size=25.0):
        """
        鱼眼相机标定

        参数：
            images: 标定图像列表
            pattern_size: 棋盘格大小
            square_size: 方格尺寸

        返回：
            success: 是否成功
        """
        obj_points = []
        img_points = []

        for img in images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
            found, corners = cv2.findChessboardCorners(gray, pattern_size)

            if found:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
                objp *= square_size

                obj_points.append(objp)
                img_points.append(corners)

        if len(obj_points) < 3:
            print("[鱼眼标定] 图像不足")
            return False

        h, w = images[0].shape[:2]
        image_size = (w, h)

        # 鱼眼标定
        N_OK = len(obj_points)
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(N_OK)]

        rms, self.camera_matrix, self.dist_coeffs, _, _ = cv2.fisheye.calibrate(
            obj_points, img_points, image_size, K, D, rvecs, tvecs,
            flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC +
                  cv2.fisheye.CALIB_CHECK_COND +
                  cv2.fisheye.CALIB_FIX_SKEW,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        )

        print(f"[鱼眼标定] RMS误差: {rms:.4f}")

        # 预计算校正映射
        self._compute_maps(image_size)

        return True

    def _compute_maps(self, image_size):
        """
        预计算校正映射

        参数：
            image_size: 图像尺寸 (w, h)
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        # 新的相机矩阵
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.camera_matrix, self.dist_coeffs, image_size,
            np.eye(3), balance=self.balance
        )

        # 计算映射
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs,
            np.eye(3), new_K, image_size, cv2.CV_16SC2
        )

        self.image_size = image_size
        print("[鱼眼校正] 校正映射已计算")

    def correct(self, image):
        """
        校正鱼眼图像

        参数：
            image: 鱼眼图像

        返回：
            corrected: 校正后的图像
        """
        if self.map1 is None or self.map2 is None:
            print("[鱼眼校正] 未标定，无法校正")
            return image

        corrected = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)
        return corrected

    def correct_with_params(self, image, camera_matrix, dist_coeffs, balance=0.0):
        """
        使用指定参数校正（不依赖预计算）

        参数：
            image: 输入图像
            camera_matrix: 相机矩阵
            dist_coeffs: 畸变系数
            balance: 平衡参数

        返回：
            corrected: 校正后的图像
        """
        h, w = image.shape[:2]
        image_size = (w, h)

        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            camera_matrix, dist_coeffs, image_size,
            np.eye(3), balance=balance
        )

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            camera_matrix, dist_coeffs,
            np.eye(3), new_K, image_size, cv2.CV_16SC2
        )

        corrected = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
        return corrected


class ImageStitcher:
    """
    图像拼接器

    功能：
    - 两图拼接
    - 多图全景拼接
    - 多种拼接模式

    使用示例：
        stitcher = ImageStitcher()
        result = stitcher.stitch([img1, img2, img3])
    """

    def __init__(self, mode='panorama', blend_alpha=0.5):
        """
        初始化拼接器

        参数：
            mode: 拼接模式 ('panorama', 'horizontal', 'vertical')
            blend_alpha: 混合透明度
        """
        self.mode = mode
        self.blend_alpha = blend_alpha

        # OpenCV拼接器
        self.stitcher = None
        try:
            self.stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA
                                                  if mode == 'panorama'
                                                  else cv2.Stitcher_SCANS)
        except Exception:
            print("[拼接器] Stitcher创建失败，使用手动拼接")

        # 特征检测器（用于手动拼接）
        self.feature_detector = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        print(f"[图像拼接] 模式: {mode}")

    def stitch(self, images):
        """
        拼接图像

        参数：
            images: 图像列表

        返回：
            result: 拼接结果
            status: 状态码
        """
        if len(images) < 2:
            return images[0] if images else None, 0

        # 尝试使用OpenCV Stitcher
        if self.stitcher is not None:
            try:
                status, result = self.stitcher.stitch(images)
                if status == cv2.Stitcher_OK:
                    return result, status
                else:
                    print(f"[拼接] Stitcher失败 (status={status})，使用手动方法")
            except Exception:
                pass

        # 手动拼接（两图）
        if len(images) == 2:
            return self._manual_stitch(images[0], images[1])

        # 多图：逐对拼接
        result = images[0]
        for img in images[1:]:
            result, _ = self._manual_stitch(result, img)
            if result is None:
                return None, -1

        return result, 0

    def _manual_stitch(self, img1, img2):
        """
        手动拼接两图

        参数：
            img1: 图像1
            img2: 图像2

        返回：
            result: 拼接结果
            status: 状态
        """
        # 特征检测
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        kp1, des1 = self.feature_detector.detectAndCompute(gray1, None)
        kp2, des2 = self.feature_detector.detectAndCompute(gray2, None)

        if des1 is None or des2 is None or len(kp1) < 10 or len(kp2) < 10:
            print("[拼接] 特征点不足")
            return None, -1

        # 特征匹配
        matches = self.matcher.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)

        # 保留好的匹配
        good_matches = matches[:min(50, len(matches))]

        if len(good_matches) < 10:
            print("[拼接] 匹配点不足")
            return None, -1

        # 计算单应性矩阵
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        if H is None:
            print("[拼接] 单应性计算失败")
            return None, -1

        # 透视变换拼接
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]

        # 计算输出尺寸
        corners1 = np.float32([[0, 0], [w1, 0], [w1, h1], [0, h1]]).reshape(-1, 1, 2)
        corners2 = np.float32([[0, 0], [w2, 0], [w2, h2], [0, h2]]).reshape(-1, 1, 2)

        warped_corners = cv2.perspectiveTransform(corners1, H)
        all_corners = np.concatenate([corners2, warped_corners], axis=0)

        [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel())
        [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel())

        # 平移矩阵
        translate = np.array([[1, 0, -xmin], [0, 1, -ymin], [0, 0, 1]])

        # 透视变换
        output_size = (xmax - xmin, ymax - ymin)
        warped1 = cv2.warpPerspective(img1, translate.dot(H), output_size)
        warped2 = cv2.warpPerspective(img2, translate, output_size)

        # 简单混合
        mask1 = (warped1 > 0).any(axis=2).astype(np.float32)
        mask2 = (warped2 > 0).any(axis=2).astype(np.float32)

        overlap = mask1 * mask2

        result = warped2.copy()
        result = np.where(warped1 > 0, warped1, result)

        # 重叠区域加权混合
        for c in range(3):
            overlap_area = warped1[:, :, c] * 0.5 + warped2[:, :, c] * 0.5
            result[:, :, c] = np.where(overlap > 0,
                                        overlap_area.astype(np.uint8),
                                        result[:, :, c])

        return result, 0

    def draw_matches(self, img1, img2, max_matches=50):
        """
        绘制特征匹配

        参数：
            img1: 图像1
            img2: 图像2
            max_matches: 最大匹配数

        返回：
            match_img: 匹配可视化图像
        """
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        kp1, des1 = self.feature_detector.detectAndCompute(gray1, None)
        kp2, des2 = self.feature_detector.detectAndCompute(gray2, None)

        matches = self.matcher.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)[:max_matches]

        match_img = cv2.drawMatches(img1, kp1, img2, kp2, matches, None,
                                     matchColor=(0, 255, 0), flags=2)

        return match_img


class StereoDepthEstimator:
    """
    立体深度估计器

    功能：
    - 立体匹配（SGBM/BM）
    - 视差图计算
    - 深度图生成
    - 3D点云

    使用示例：
        estimator = StereoDepthEstimator()
        depth = estimator.compute_depth(left_img, right_img)
    """

    def __init__(self, method='sgbm', num_disparities=64, block_size=11):
        """
        初始化深度估计器

        参数：
            method: 匹配方法 ('sgbm', 'bm')
            num_disparities: 最大视差（必须是16的倍数）
            block_size: 匹配块大小（奇数）
        """
        self.method = method
        self.num_disparities = num_disparities
        self.block_size = block_size

        if method == 'sgbm':
            self.stereo = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=num_disparities,
                blockSize=block_size,
                P1=8 * 3 * block_size ** 2,
                P2=32 * 3 * block_size ** 2,
                disp12MaxDiff=1,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32,
                preFilterCap=63,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
        else:
            self.stereo = cv2.StereoBM_create(
                numDisparities=num_disparities,
                blockSize=block_size
            )

        # WLS滤波器（可选，用于平滑视差图）
        self.wls_filter = None
        try:
            self.stereo_right = cv2.ximgproc.createRightMatcher(self.stereo)
            self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
            self.wls_filter.setLambda(8000)
            self.wls_filter.setSigmaColor(1.5)
        except Exception:
            self.stereo_right = None

        print(f"[深度估计] 方法: {method}, 视差范围: {num_disparities}")

    def compute_disparity(self, left_gray, right_gray):
        """
        计算视差图

        参数：
            left_gray: 左灰度图
            right_gray: 右灰度图

        返回：
            disparity: 视差图 (float32)
        """
        # 计算视差
        disparity = self.stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

        # WLS滤波
        if self.wls_filter is not None and self.stereo_right is not None:
            disparity_right = self.stereo_right.compute(right_gray, left_gray).astype(np.float32) / 16.0
            disparity = self.wls_filter.filter(disparity, left_gray, None, disparity_right)

        return disparity

    def compute_depth(self, left_img, right_img, baseline=None, focal_length=None):
        """
        计算深度图

        参数：
            left_img: 左图像
            right_img: 右图像
            baseline: 基线距离（米）
            focal_length: 焦距（像素）

        返回：
            depth: 深度图（米）
            disparity: 视差图
            depth_vis: 深度可视化
        """
        # 转灰度
        if len(left_img.shape) == 3:
            left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left_img

        if len(right_img.shape) == 3:
            right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        else:
            right_gray = right_img

        # 计算视差
        disparity = self.compute_disparity(left_gray, right_gray)

        # 计算深度
        depth = None
        if baseline is not None and focal_length is not None:
            # depth = (focal_length * baseline) / disparity
            with np.errstate(divide='ignore', invalid='ignore'):
                depth = (focal_length * baseline) / disparity
                depth[disparity <= 0] = 0
                depth[depth > 100] = 0  # 限制最大深度100m

        # 深度可视化
        depth_vis = self.visualize_disparity(disparity)

        return depth, disparity, depth_vis

    def visualize_disparity(self, disparity):
        """
        视差图可视化

        参数：
            disparity: 视差图

        返回：
            vis: 可视化图像（BGR）
        """
        # 归一化
        disp_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_norm = np.uint8(disp_norm)

        # 应用色彩映射
        vis = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)

        # 无效区域标记为黑色
        vis[disparity <= 0] = [0, 0, 0]

        return vis

    def compute_point_cloud(self, disparity, Q, max_depth=50):
        """
        计算3D点云

        参数：
            disparity: 视差图
            Q: 视差-深度映射矩阵
            max_depth: 最大深度

        返回：
            points_3d: 3D点云 (H, W, 3)
            colors: 颜色（如果有）
        """
        # 重投影到3D
        points_3d = cv2.reprojectImageTo3D(disparity, Q)

        # 过滤无效点
        mask = (disparity > 0) & (points_3d[:, :, 2] < max_depth) & (points_3d[:, :, 2] > 0)
        points_3d = points_3d[mask]

        return points_3d


class MultiCameraFusionPipeline:
    """
    多相机融合流水线

    整合双目标定、校正、深度估计、拼接

    使用示例：
        pipeline = MultiCameraFusionPipeline()
        pipeline.start(left_cam=0, right_cam=1)

        while True:
            result = pipeline.get_result()
            if result:
                cv2.imshow('Stereo', result['side_by_side'])
                cv2.imshow('Depth', result['depth_vis'])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        pipeline.stop()
    """

    def __init__(self, left_cam=0, right_cam=1, resolution=(640, 480)):
        """
        初始化多相机流水线

        参数：
            left_cam: 左摄像头ID
            right_cam: 右摄像头ID
            resolution: 分辨率
        """
        self.left_cam = left_cam
        self.right_cam = right_cam
        self.resolution = resolution

        # 组件
        self.calibrator = StereoCalibrator()
        self.depth_estimator = StereoDepthEstimator(method='sgbm')
        self.stitcher = ImageStitcher()

        # 线程
        self._cap_l = None
        self._cap_r = None
        self._frame_l = None
        self._frame_r = None
        self._result = None
        self._running = False
        self._lock = threading.Lock()
        self._threads = []

        # 标定文件
        self.calib_file = os.path.expanduser('~/.hermes/stereo_calib.npz')

    def start(self):
        """启动流水线"""
        self._cap_l = cv2.VideoCapture(self.left_cam)
        self._cap_r = cv2.VideoCapture(self.right_cam)

        for cap in [self._cap_l, self._cap_r]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

        self._running = True

        # 采集线程
        t1 = threading.Thread(target=self._capture_loop, daemon=True)
        t1.start()
        self._threads.append(t1)

        # 处理线程
        t2 = threading.Thread(target=self._process_loop, daemon=True)
        t2.start()
        self._threads.append(t2)

        # 尝试加载标定
        if os.path.exists(self.calib_file):
            self.calibrator.load(self.calib_file)

        print("[多相机流水线] 已启动")

    def stop(self):
        """停止流水线"""
        self._running = False
        for t in self._threads:
            t.join(timeout=2)
        if self._cap_l:
            self._cap_l.release()
        if self._cap_r:
            self._cap_r.release()
        print("[多相机流水线] 已停止")

    def _capture_loop(self):
        """采集循环"""
        while self._running:
            ret_l, frame_l = self._cap_l.read()
            ret_r, frame_r = self._cap_r.read()

            if ret_l and ret_r:
                with self._lock:
                    self._frame_l = frame_l
                    self._frame_r = frame_r

    def _process_loop(self):
        """处理循环"""
        while self._running:
            frame_l, frame_r = None, None
            with self._lock:
                if self._frame_l is not None and self._frame_r is not None:
                    frame_l = self._frame_l.copy()
                    frame_r = self._frame_r.copy()

            if frame_l is not None and frame_r is not None:
                self._process_stereo(frame_l, frame_r)

    def _process_stereo(self, left, right):
        """
        处理立体图像对

        参数：
            left: 左图像
            right: 右图像
        """
        # 校正
        if self.calibrator.rectify_map_l is not None:
            left_rect, right_rect = self.calibrator.rectify(left, right)
        else:
            left_rect, right_rect = left, right

        # 深度估计
        left_gray = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)

        disparity = self.depth_estimator.compute_disparity(left_gray, right_gray)
        depth_vis = self.depth_estimator.visualize_disparity(disparity)

        # 并排显示
        side_by_side = np.hstack([left_rect, right_rect])

        # 绘制极线（如果已标定）
        if self.calibrator.rectify_map_l is not None:
            for y in range(0, side_by_side.shape[0], 50):
                cv2.line(side_by_side, (0, y), (side_by_side.shape[1], y),
                         (0, 255, 0), 1)

        with self._lock:
            self._result = {
                'left': left_rect,
                'right': right_rect,
                'disparity': disparity,
                'depth_vis': depth_vis,
                'side_by_side': side_by_side,
            }

    def get_result(self):
        """获取最新结果"""
        with self._lock:
            return self._result

    def calibrate(self, left_images, right_images):
        """
        执行标定

        参数：
            left_images: 左图像列表
            right_images: 右图像列表
        """
        success = self.calibrator.calibrate_stereo(left_images, right_images)
        if success:
            os.makedirs(os.path.dirname(self.calib_file), exist_ok=True)
            self.calibrator.save(self.calib_file)
        return success


def draw_stereo_info(frame, fps=0, disparity_range=None):
    """
    在立体图像上绘制信息

    参数：
        frame: 输入图像
        fps: 帧率
        disparity_range: 视差范围

    返回：
        annotated: 标注后的图像
    """
    annotated = frame.copy()

    if fps > 0:
        cv2.putText(annotated, f'FPS: {fps:.1f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    if disparity_range is not None:
        cv2.putText(annotated, f'Disparity: {disparity_range[0]}-{disparity_range[1]}',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return annotated


# ================================================================
#                          使用示例
# ================================================================
if __name__ == '__main__':
    print("=" * 60)
    print("多相机融合 - Multi-Camera Fusion")
    print("针对 Orange Pi 5 优化")
    print("=" * 60)

    print("\n选择功能:")
    print("  1 - 双目深度估计")
    print("  2 - 鱼眼校正")
    print("  3 - 图像拼接")
    print("  4 - 双目标定")

    choice = input("请选择 (1-4, 默认1): ").strip() or '1'

    if choice == '1':
        # 双目深度估计
        estimator = StereoDepthEstimator(method='sgbm')

        cap_l = cv2.VideoCapture(0)
        cap_r = cv2.VideoCapture(1)

        print("\n按 'q' 退出")

        while True:
            ret_l, frame_l = cap_l.read()
            ret_r, frame_r = cap_r.read()

            if not ret_l or not ret_r:
                print("无法读取摄像头")
                break

            t_start = time.time()
            depth, disparity, depth_vis = estimator.compute_depth(frame_l, frame_r)
            fps = 1.0 / max(time.time() - t_start, 1e-6)

            # 并排显示
            side_by_side = np.hstack([frame_l, frame_r])

            cv2.putText(depth_vis, f'FPS: {fps:.1f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            cv2.imshow('Stereo', side_by_side)
            cv2.imshow('Depth', depth_vis)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap_l.release()
        cap_r.release()

    elif choice == '2':
        # 鱼眼校正
        corrector = FisheyeCorrector()

        cap = cv2.VideoCapture(0)
        print("\n按 'q' 退出")
        print("按 'c' 标定鱼眼镜头")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            cv2.imshow('Fisheye', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cap.release()

    elif choice == '3':
        # 图像拼接
        stitcher = ImageStitcher()

        cap = cv2.VideoCapture(0)
        images = []

        print("\n按 'c' 捕获图像")
        print("按 's' 拼接")
        print("按 'q' 退出")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            cv2.imshow('Capture', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                images.append(frame.copy())
                print(f"已捕获 {len(images)} 张图像")
            elif key == ord('s') and len(images) >= 2:
                result, status = stitcher.stitch(images)
                if result is not None:
                    cv2.imshow('Stitched', result)
                    print("拼接成功")
                else:
                    print("拼接失败")

        cap.release()

    elif choice == '4':
        # 双目标定
        calibrator = StereoCalibrator()
        cap_l = cv2.VideoCapture(0)
        cap_r = cv2.VideoCapture(1)

        left_images = []
        right_images = []

        print("\n按 'c' 捕获标定图像")
        print("按 's' 开始标定")
        print("按 'q' 退出")

        while True:
            ret_l, frame_l = cap_l.read()
            ret_r, frame_r = cap_r.read()

            if not ret_l or not ret_r:
                break

            side_by_side = np.hstack([frame_l, frame_r])
            cv2.imshow('Calibration', side_by_side)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                left_images.append(frame_l.copy())
                right_images.append(frame_r.copy())
                print(f"已捕获 {len(left_images)} 对图像")
            elif key == ord('s') and len(left_images) >= 3:
                print("开始标定...")
                success = calibrator.calibrate_stereo(left_images, right_images)
                if success:
                    calibrator.save('stereo_calib.npz')
                    print("标定完成并保存")

        cap_l.release()
        cap_r.release()

    cv2.destroyAllWindows()
