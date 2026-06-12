# -*- coding: utf-8 -*-
"""
模块4: 3D视觉 - 双目测距/结构光
=================================
嵌入式平台3D视觉方案

技术方案:
  方案A: 双目立体匹配 → 深度图 (最通用)
  方案B: 单目深度估计 (Monocular Depth) → 精度有限
  方案C: 结构光(激光线+单目) → 电赛常用★

双目测距原理:
  深度 Z = f × B / d
  f: 焦距(像素), B: 基线距离(mm), d: 视差(像素)

优化:
  1. SGBM比BM精度高, 但慢; 嵌入式用BM或精简SGBM
  2. ROI只计算感兴趣区域
  3. 视差图后处理: WLS滤波
  4. 预计算查找表加速坐标转换

电赛场景:
  - 避障测距: 测量前方障碍物距离
  - 抓取定位: 目标三维坐标获取
  - 路径规划: 3D地图构建
"""

import cv2
import numpy as np


class StereoCalibrator:
    """
    双目标定工具
    
    标定流程:
      1. 打印棋盘格(如9x6)
      2. 拍摄15-20组不同角度的棋盘格图片
      3. 调用 calibrate() 获取内参和畸变系数
      4. 调用 save() 保存标定结果
    """
    
    def __init__(self, board_size=(9, 6), square_size=25.0):
        """
        Args:
            board_size: 棋盘格内角点数 (列, 行)
            square_size: 格子尺寸(mm)
        """
        self.board_size = board_size
        self.square_size = square_size
        self.obj_points = []  # 3D世界坐标
        self.img_points_l = []  # 左图角点
        self.img_points_r = []  # 右图角点
    
    def add_images(self, img_left, img_right):
        """
        添加一组标定图片对
        
        Returns:
            found: 是否检测到棋盘格
        """
        gray_l = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
        
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret_l, corners_l = cv2.findChessboardCorners(gray_l, self.board_size, flags=flags)
        ret_r, corners_r = cv2.findChessboardCorners(gray_r, self.board_size, flags=flags)
        
        if ret_l and ret_r:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_l = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), criteria)
            corners_r = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), criteria)
            
            # 生成3D坐标
            objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2)
            objp *= self.square_size
            
            self.obj_points.append(objp)
            self.img_points_l.append(corners_l)
            self.img_points_r.append(corners_r)
            return True
        return False
    
    def calibrate(self, image_size):
        """
        执行双目标定
        
        Args:
            image_size: (width, height)
        Returns:
            stereo_params: 标定参数字典
        """
        # 单目标定
        ret_l, mtx_l, dist_l, _, _ = cv2.calibrateCamera(
            self.obj_points, self.img_points_l, image_size, None, None)
        ret_r, mtx_r, dist_r, _, _ = cv2.calibrateCamera(
            self.obj_points, self.img_points_r, image_size, None, None)
        
        # 双目标定
        flags = cv2.CALIB_FIX_INTRINSIC
        ret, mtx_l, dist_l, mtx_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
            self.obj_points, self.img_points_l, self.img_points_r,
            mtx_l, dist_l, mtx_r, dist_r, image_size, flags=flags)
        
        # 立体校正
        R1, R2, P1, P2, Q, roi_l, roi_r = cv2.stereoRectify(
            mtx_l, dist_l, mtx_r, dist_r, image_size, R, T,
            alpha=0, newImageSize=image_size)
        
        # 计算重映射表
        map_lx, map_ly = cv2.initUndistortRectifyMap(
            mtx_l, dist_l, R1, P1, image_size, cv2.CV_32FC1)
        map_rx, map_ry = cv2.initUndistortRectifyMap(
            mtx_r, dist_r, R2, P2, image_size, cv2.CV_32FC1)
        
        return {
            "mtx_l": mtx_l, "dist_l": dist_l,
            "mtx_r": mtx_r, "dist_r": dist_r,
            "R": R, "T": T, "Q": Q,
            "map_lx": map_lx, "map_ly": map_ly,
            "map_rx": map_rx, "map_ry": map_ry,
            "roi_l": roi_l, "roi_r": roi_r,
            "image_size": image_size,
            "reprojection_error": ret
        }


class StereoDepth:
    """
    双目深度估计器
    
    输入: 左右图像对
    输出: 视差图 / 深度图 / 3D点云
    
    用法:
        stereo = StereoDepth(calib_params)
        disparity = stereo.compute_disparity(img_l, img_r)
        depth = stereo.get_depth(disparity)
        points3d = stereo.get_point_cloud(disparity, img_l)
    """
    
    def __init__(self, calib_params=None, num_disparities=64, block_size=15):
        """
        Args:
            calib_params: StereoCalibrator的标定结果
            num_disparities: 最大视差(16的倍数)
            block_size: 匹配块大小(奇数)
        """
        self.params = calib_params
        self.num_disp = num_disparities
        self.block_size = block_size
        
        # SGBM立体匹配(精度与速度平衡)
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
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY  # 速度优化
        )
        
        # WLS滤波(后处理, 提升质量)
        try:
            self.wls = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
            self.right_matcher = cv2.ximgproc.createRightMatcher(self.stereo)
            self.use_wls = True
        except (cv2.error, AttributeError):
            self.use_wls = False
            print("[双目] WLS滤波不可用, 使用原始SGBM")
    
    def preprocess(self, img_left, img_right):
        """
        预处理: 校正+灰度化
        
        Returns:
            rect_l, rect_r: 校正后的灰度图
        """
        gray_l = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
        
        if self.params:
            gray_l = cv2.remap(gray_l, self.params["map_lx"], 
                              self.params["map_ly"], cv2.INTER_LINEAR)
            gray_r = cv2.remap(gray_r, self.params["map_rx"], 
                              self.params["map_ry"], cv2.INTER_LINEAR)
        
        return gray_l, gray_r
    
    def compute_disparity(self, img_left, img_right):
        """
        计算视差图
        
        Args:
            img_left, img_right: 左右图像(BGR或灰度)
        Returns:
            disparity: 视差图(float32, 单位:像素)
        """
        if len(img_left.shape) == 3:
            gray_l, gray_r = self.preprocess(img_left, img_right)
        else:
            gray_l, gray_r = img_left, img_right
        
        disp_l = self.stereo.compute(gray_l, gray_r)
        
        if self.use_wls:
            disp_r = self.right_matcher.compute(gray_r, gray_l)
            disp_l = self.wls.filter(disp_l, img_left, None, disp_r)
        
        # 转为float32, 除以16(SGBM输出16倍精度)
        disparity = disp_l.astype(np.float32) / 16.0
        return disparity
    
    def get_depth(self, disparity):
        """
        视差→深度图
        
        公式: depth = f * B / disparity
        f: 焦距, B: 基线距离
        
        Args:
            disparity: 视差图
        Returns:
            depth: 深度图(mm)
        """
        if self.params is None:
            print("[警告] 无标定参数, 返回视差图作为深度")
            return disparity
        
        Q = self.params["Q"]
        # Q[2,3]=f*Tx, Q[3,3]=-1/Tx → f*B = -Q[2,3]/Q[3,3]
        # 简化: depth = focal_length * baseline / disparity
        T = self.params["T"]
        baseline = np.linalg.norm(T)  # 基线距离(mm)
        f = self.params["P1"][0, 0]   # 焦距(像素)
        
        depth = np.zeros_like(disparity)
        valid = disparity > 0
        depth[valid] = f * baseline / disparity[valid]
        
        return depth
    
    def get_3d_points(self, disparity, img_color=None):
        """
        视差图→3D点云
        
        Args:
            disparity: 视差图
            img_color: 彩色图(可选, 用于着色)
        Returns:
            points: Nx3 (x,y,z)
            colors: Nx3 (b,g,r) 或 None
        """
        if self.params is None:
            return np.array([]), None
        
        points_3d = cv2.reprojectImageTo3D(disparity, self.params["Q"])
        
        mask = disparity > 0
        points = points_3d[mask]
        
        colors = None
        if img_color is not None:
            colors = img_color[mask]
        
        return points, colors
    
    def measure_distance(self, disparity, roi=None):
        """
        测量ROI区域的距离
        
        Args:
            disparity: 视差图
            roi: (x, y, w, h) 感兴趣区域, None=全图
        Returns:
            distance_mm: 距离(mm), -1表示无效
        """
        depth = self.get_depth(disparity)
        
        if roi:
            x, y, w, h = roi
            region = depth[y:y+h, x:x+w]
        else:
            h, w = depth.shape
            region = depth[h//4:3*h//4, w//4:3*w//4]  # 中心区域
        
        valid = region[region > 0]
        if len(valid) == 0:
            return -1
        
        # 用中值滤波去除噪声
        return float(np.median(valid))


class StructureLight:
    """
    结构光测距(单目+激光线)
    
    原理:
      1. 投射激光线到物体表面
      2. 摄像头拍摄激光线图像
      3. 分析激光线偏移 → 计算深度
    
    电赛常用方案: 成本低, 只需一个摄像头+激光模块
    
    配置:
      - 激光模块: 线状激光(一字线), 波长650nm(红色)
      - 安装: 激光与摄像头固定间距B, 平行安装
      - 标定: 需标定激光-摄像头相对位置
    """
    
    def __init__(self, camera_mtx=None, baseline_mm=50.0, laser_angle=30.0):
        """
        Args:
            camera_mtx: 摄像头内参矩阵
            baseline_mm: 激光与摄像头间距(mm)
            laser_angle: 激光与光轴夹角(度)
        """
        self.mtx = camera_mtx
        self.baseline = baseline_mm
        self.laser_angle = np.radians(laser_angle)
    
    def detect_laser_line(self, frame):
        """
        检测激光线位置
        
        激光特征: 红色、高亮度
        Returns:
            line_points: Nx2 激光线坐标数组
            line_mask: 二值掩码
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 红色激光检测(H范围窄, S/V高)
        mask1 = cv2.inRange(hsv, np.array([0, 150, 200]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 150, 200]), np.array([180, 255, 255]))
        laser_mask = mask1 | mask2
        
        # 形态学清理
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        laser_mask = cv2.morphologyEx(laser_mask, cv2.MORPH_OPEN, kernel)
        
        # 提取激光线点
        points = np.column_stack(np.where(laser_mask > 0))  # (row, col)
        if len(points) > 0:
            points = points[:, ::-1]  # 转为(x, y)
        
        return points, laser_mask
    
    def compute_depth(self, laser_points, frame_height):
        """
        根据激光线偏移计算深度
        
        原理: 深度 = B / (tan(α) + dy/f)
        dy: 激光点与基准线的垂直偏移
        
        Args:
            laser_points: Nx2 激光点坐标
            frame_height: 帧高度
        Returns:
            depths: N个深度值(mm)
        """
        if self.mtx is None:
            return np.full(len(laser_points), -1.0)
        
        fy = self.mtx[1, 1]  # 焦距y
        cy = self.mtx[1, 2]  # 光心y
        
        # 基准线: 图像中线
        baseline_y = frame_height / 2
        
        dy = laser_points[:, 1] - baseline_y  # 垂直偏移
        
        # 深度计算
        depths = self.baseline * fy / (dy + fy * np.tan(self.laser_angle))
        depths = np.abs(depths)
        depths[depths < 10] = 0  # 过滤异常值
        depths[depths > 2000] = 0
        
        return depths
    
    def scan_profile(self, frame):
        """
        扫描激光线获取物体截面轮廓
        
        Returns:
            profile: Nx3 (x, y, depth_mm) 轮廓点
        """
        points, mask = self.detect_laser_line(frame)
        if len(points) == 0:
            return np.array([]).reshape(0, 3)
        
        depths = self.compute_depth(points, frame.shape[0])
        
        # 组合结果
        profile = np.column_stack([points, depths])
        
        # 按x排序
        profile = profile[profile[:, 0].argsort()]
        
        return profile


# ===== 电赛应用示例 =====
def demo_obstacle_distance():
    """
    双目测距示例
    
    场景: 测量前方障碍物距离
    输出: 障碍物中心距离(mm)
    """
    from .platform_utils import optimize_opencv, FrameCounter
    optimize_opencv()
    
    # 使用双目摄像头
    cap_l = cv2.VideoCapture(0)  # 左目
    cap_r = cv2.VideoCapture(2)  # 右目(编号可能不同)
    
    stereo = StereoDepth(num_disparities=64, block_size=15)
    counter = FrameCounter()
    
    print("[双目测距] 启动... 按q退出")
    while True:
        ret_l, frame_l = cap_l.read()
        ret_r, frame_r = cap_r.read()
        if not ret_l or not ret_r:
            continue
        
        disparity = stereo.compute_disparity(frame_l, frame_r)
        
        # 测量画面中心距离
        dist = stereo.measure_distance(disparity)
        
        # 可视化视差
        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
        
        counter.tick()
        label = f"FPS:{counter.fps:.1f} Dist:{dist:.0f}mm" if dist > 0 else f"FPS:{counter.fps:.1f}"
        cv2.putText(disp_color, label, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.imshow("Stereo Depth", disp_color)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap_l.release()
    cap_r.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo_obstacle_distance()
