# -*- coding: utf-8 -*-
"""
模块3: 场景理解 - 语义分割简化版
==================================
嵌入式平台轻量级语义分割

方案选择:
  方案A: 颜色+边缘特征分割 (最快, ~60fps, 精度低)
  方案B: MobileNetV2-DeepLabV3 (ONNX, ~10fps)
  方案C: BiSeNet/RKNN INT8 (~25fps) ★推荐

优化策略:
  1. 降分辨率: 320x320代替512x512
  2. 隔帧推理: 每2-3帧推理一次, 中间帧用光流补偿
  3. 感兴趣区域: 只处理ROI区域
  4. INT8量化: NPU推理速度提升3x

电赛场景:
  - 路面检测: 区分道路/障碍/边界
  - 色块分拣: 不同颜色区域识别
  - 简易SLAM: 障碍物区域标记
"""

import cv2
import numpy as np


class ColorSegmentor:
    """
    基于颜色的快速语义分割(无需模型)
    
    原理: HSV颜色空间阈值分割
    性能: ~60fps (640x480, Orange Pi 5)
    
    用法:
        seg = ColorSegmentor()
        mask = seg.segment(frame)
        # mask: 0=背景, 1=红色, 2=绿色, 3=蓝色, 4=白色, 5=黑色
    """
    
    # 类别颜色映射 (用于可视化)
    CLASS_COLORS = {
        0: (0, 0, 0),       # 背景 - 黑
        1: (0, 0, 255),     # 红色
        2: (0, 255, 0),     # 绿色
        3: (255, 0, 0),     # 蓝色
        4: (255, 255, 255), # 白色
        5: (64, 64, 64),    # 黑色
        6: (0, 255, 255),   # 黄色
        7: (255, 0, 255),   # 紫色
    }
    
    CLASS_NAMES = {
        0: "背景", 1: "红色", 2: "绿色", 3: "蓝色",
        4: "白色", 5: "黑色", 6: "黄色", 7: "紫色"
    }
    
    def __init__(self, custom_ranges=None):
        """
        Args:
            custom_ranges: 自定义HSV范围 {class_id: (lower, upper)}
        """
        # 默认HSV范围(可调)
        self.ranges = {
            1: (np.array([0, 80, 80]), np.array([10, 255, 255])),     # 红色1
            11: (np.array([160, 80, 80]), np.array([180, 255, 255])), # 红色2
            2: (np.array([35, 80, 80]), np.array([85, 255, 255])),    # 绿色
            3: (np.array([100, 80, 80]), np.array([130, 255, 255])),  # 蓝色
            4: (np.array([0, 0, 180]), np.array([180, 30, 255])),     # 白色
            5: (np.array([0, 0, 0]), np.array([180, 255, 40])),       # 黑色
            6: (np.array([20, 80, 80]), np.array([35, 255, 255])),    # 黄色
            7: (np.array([130, 80, 80]), np.array([160, 255, 255])),  # 紫色
        }
        if custom_ranges:
            self.ranges.update(custom_ranges)
    
    def segment(self, frame):
        """
        颜色分割
        
        Args:
            frame: BGR图像
        Returns:
            mask: 分割掩码(HxW, uint8, 0-7对应类别)
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        result = np.zeros((h, w), dtype=np.uint8)
        
        # 按优先级分割(数字大的优先)
        for cls_id, (lower, upper) in sorted(self.ranges.items(), reverse=True):
            mask = cv2.inRange(hsv, lower, upper)
            # 形态学去噪
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            actual_cls = 1 if cls_id == 11 else cls_id  # 红色合并
            result[mask > 0] = actual_cls
        
        return result
    
    def visualize(self, mask):
        """将分割掩码可视化为彩色图"""
        h, w = mask.shape
        vis = np.zeros((h, w, 3), dtype=np.uint8)
        for cls_id, color in self.CLASS_COLORS.items():
            vis[mask == cls_id] = color
        return vis
    
    def get_region_info(self, mask):
        """
        获取各区域信息(面积比、重心)
        
        Returns:
            {class_id: {"name": str, "ratio": float, "center": (x,y)}}
        """
        h, w = mask.shape
        total = h * w
        info = {}
        for cls_id in np.unique(mask):
            if cls_id == 0:
                continue
            cls_mask = (mask == cls_id).astype(np.uint8)
            ratio = np.sum(cls_mask) / total
            
            # 重心
            moments = cv2.moments(cls_mask)
            if moments['m00'] > 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
            else:
                cx, cy = 0, 0
            
            name = self.CLASS_NAMES.get(cls_id, f"class_{cls_id}")
            info[cls_id] = {"name": name, "ratio": ratio, "center": (cx, cy)}
        
        return info


class LiteSegmentor:
    """
    轻量级神经网络语义分割 (ONNX推理)
    
    模型: MobileNetV2-DeepLabV3 (Cityscapes 21类)
    输入: 320x320 RGB
    输出: 320x320 类别掩码
    
    部署:
      1. PyTorch导出: torch.onnx.export(deeplabv3, ...)
      2. 或用PaddleSeg/MMSeg导出ONNX
      3. RKNN转换获得最优性能
    
    电赛用法:
        seg = LiteSegmentor("deeplabv3_mobilenetv2.onnx")
        mask = seg.segment(frame)
        # 分析: 前方可通行区域 / 障碍物区域
    """
    
    # Cityscapes简化类名(电赛常用子集)
    CLASS_NAMES = {
        0: "背景", 1: "道路", 2: "人行道", 3: "建筑",
        4: "植被", 5: "天空", 6: "行人", 7: "车辆",
        8: "障碍物"
    }
    
    def __init__(self, model_path=None, input_size=320, use_rknn=False):
        self.input_size = input_size
        self.use_rknn = use_rknn
        self.net = None
        
        if model_path:
            if use_rknn:
                from rknnlite.api import RKNNLite
                self.rknn = RKNNLite()
                self.rknn.load_rknn(model_path.replace('.onnx', '.rknn'))
                self.rknn.init_runtime()
            else:
                self.net = cv2.dnn.readNetFromONNX(model_path)
    
    def segment(self, frame):
        """语义分割推理"""
        if self.net is None and not self.use_rknn:
            # 后备: 颜色分割
            return ColorSegmentor().segment(frame)
        
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, 
                                      (self.input_size, self.input_size),
                                      (0.485, 0.456, 0.406), 
                                      swapRB=True, crop=False)
        
        self.net.setInput(blob)
        output = self.net.forward()  # [1, C, H, W]
        
        mask = np.argmax(output[0], axis=0)  # [H, W]
        mask = cv2.resize(mask.astype(np.uint8), (w, h), 
                          interpolation=cv2.INTER_NEAREST)
        return mask


class RoadDetector:
    """
    道路/可通行区域检测(电赛专用)
    
    结合颜色分割+边缘检测, 识别:
      - 可通行区域(道路/地面)
      - 边界线(黑线/白线)
      - 障碍物(非地面区域)
    
    应用: 智能小车循迹
    """
    
    def __init__(self):
        self.color_seg = ColorSegmentor()
    
    def detect(self, frame):
        """
        检测道路信息
        
        Returns:
            dict: {
                "mask": 分割掩码,
                "drivable_area": 可通行区域占比,
                "center_offset": 中心偏移量(-1~1, 负=偏左),
                "obstacles": [(x,y,w,h), ...],
                "line_angle": 线条角度(度)
            }
        """
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 颜色分割
        mask = self.color_seg.segment(frame)
        
        # 边缘检测(识别黑线)
        edges = cv2.Canny(gray, 50, 150)
        
        # 可通行区域: 非障碍、非黑线
        drivable = (mask == 0) | (mask == 4)  # 背景或白色
        drivable_ratio = np.sum(drivable) / (h * w)
        
        # 中心偏移: 可通行区域的重心偏移
        moments = cv2.moments(drivable.astype(np.uint8))
        if moments['m00'] > 0:
            cx = moments['m10'] / moments['m00']
            center_offset = (cx - w/2) / (w/2)  # -1~1
        else:
            center_offset = 0
        
        # 线条角度(霍夫变换)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, 
                                 minLineLength=30, maxLineGap=10)
        line_angle = 0
        if lines is not None:
            angles = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.degrees(np.arctan2(y2-y1, x2-x1))
                if abs(angle) < 45:  # 只取近水平线
                    angles.append(angle)
            if angles:
                line_angle = np.median(angles)
        
        # 障碍物轮廓
        obstacle_mask = (mask == 1) | (mask == 2) | (mask == 3) | (mask == 5)
        obstacle_mask = obstacle_mask.astype(np.uint8) * 255
        contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, 
                                        cv2.CHAIN_APPROX_SIMPLE)
        obstacles = [cv2.boundingRect(c) for c in contours 
                    if cv2.contourArea(c) > 500]
        
        return {
            "mask": mask,
            "drivable_area": drivable_ratio,
            "center_offset": center_offset,
            "obstacles": obstacles,
            "line_angle": line_angle
        }


# ===== 电赛应用示例 =====
def demo_line_following():
    """
    智能小车循迹示例
    
    场景: 沿黑线行驶, 避开障碍物
    输出: 转向指令 (左转/右转/直行/停止)
    """
    from .platform_utils import CameraThread, FrameCounter, optimize_opencv
    optimize_opencv()
    
    cam = CameraThread(src=0, width=640, height=480).start()
    road = RoadDetector()
    counter = FrameCounter()
    
    print("[循迹] 启动... 按q退出")
    while True:
        frame = cam.read()
        if frame is None:
            continue
        
        info = road.detect(frame)
        
        # 转向决策
        offset = info["center_offset"]
        if info["drivable_area"] < 0.2:
            cmd = "停止 - 前方无路"
        elif offset < -0.3:
            cmd = "左转"
        elif offset > 0.3:
            cmd = "右转"
        else:
            cmd = "直行"
        
        vis = road.color_seg.visualize(info["mask"])
        vis = cv2.addWeighted(frame, 0.5, vis, 0.5, 0)
        
        counter.tick()
        cv2.putText(vis, f"FPS:{counter.fps:.1f} {cmd}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.imshow("Line Following", vis)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo_line_following()
