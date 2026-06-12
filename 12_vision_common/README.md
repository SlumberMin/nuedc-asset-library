# 12_vision_common

## 模块概述

本模块是电赛视觉处理的核心代码库，包含颜色检测、形状识别、直线检测、二维码/条码识别、ArUco/AprilTag标记检测、图像测量、OCR、目标跟踪、人脸检测、手势识别、车道检测、运动检测、多相机融合等通用视觉算法。所有算法均基于 OpenCV 实现，可直接嵌入比赛项目。

## 目录结构

```
12_vision_common/
├── README.md                          # 本文档
├── embedded_vision/                   # 嵌入式视觉高级模块
│   ├── README.md                      # 嵌入式视觉说明
│   ├── __init__.py
│   ├── quickstart.py                  # 快速入门
│   ├── object_detector.py             # 目标检测器
│   ├── deepsort_tracker.py            # DeepSORT跟踪器
│   ├── stereo_depth.py                # 双目深度估计
│   ├── visual_slam.py                 # 视觉SLAM
│   ├── semantic_segmentation.py       # 语义分割
│   └── platform_utils.py              # 平台工具
├── # === 核心视觉算法（最新版） ===
├── color_tracker.py                   # 颜色追踪
├── hough_line_detector.py             # 霍夫直线检测
├── apriltag_detector.py               # AprilTag检测
├── homography.py                      # 单应性变换
├── advanced_lane_detection.py         # 高级车道检测
├── face_detection_opi5.py             # 人脸检测（OPi5优化）
├── gesture_recognition.py             # 手势识别
├── traffic_sign_recognition.py        # 交通标志识别
├── motion_detection_advanced.py       # 高级运动检测
├── multi_camera_fusion.py             # 多相机融合
├── # === 迭代版本模块 ===
├── image_qr_v7~v13.py                 # 二维码识别（多版本）
├── image_barcode_v2~v13.py            # 条码识别（多版本）
├── image_measurement.py~v12.py        # 图像测量（多版本）
├── image_fiducial.py~v12.py           # 标记识别（多版本）
├── image_ocr.py~v12.py                # OCR文字识别（多版本）
├── # === 图像处理基础模块 ===
├── image_filter.py                    # 图像滤波
├── image_morphology.py                # 形态学操作
├── image_threshold.py                 # 阈值处理
├── image_edge.py                      # 边缘检测
├── image_contour.py                   # 轮廓检测
├── image_histogram.py                 # 直方图分析
├── image_enhance_v2.py                # 图像增强
├── image_noise.py                     # 噪声处理
├── image_feature.py                   # 特征提取
├── image_match.py                     # 模板匹配
├── image_transformation.py            # 图像变换
├── image_segment_v2.py                # 图像分割
├── image_detect_v2.py                 # 目标检测
├── image_track_v2.py                  # 目标跟踪
├── image_registration.py              # 图像配准
├── image_annotation.py                # 图像标注
├── image_quality.py                   # 图像质量评估
├── image_frequency.py                 # 频域处理
├── image_batch.py                     # 批量处理
├── image_watermark.py                 # 水印处理
└── image_compression.py               # 图像压缩
```

## 文件清单和说明

### 核心算法模块（推荐使用最新版）

| 文件 | 说明 |
|------|------|
| color_tracker.py | HSV颜色空间目标追踪 |
| hough_line_detector.py | 霍夫变换直线检测 |
| apriltag_detector.py | AprilTag标记检测与姿态估计 |
| homography.py | 透视变换与坐标映射 |
| advanced_lane_detection.py | 车道线检测（透视+多项式拟合） |
| face_detection_opi5.py | OPi5优化的人脸检测 |
| gesture_recognition.py | 手势识别 |
| motion_detection_advanced.py | 背景减除运动检测 |
| multi_camera_fusion.py | 多相机数据融合 |

### 嵌入式视觉模块

| 文件 | 说明 |
|------|------|
| embedded_vision/object_detector.py | 轻量级目标检测器 |
| embedded_vision/deepsort_tracker.py | DeepSORT多目标跟踪 |
| embedded_vision/stereo_depth.py | 双目深度估计 |
| embedded_vision/visual_slam.py | 视觉同步定位与建图 |

### 版本化模块（取最新版本号使用）

| 模块 | 最新版 | 功能 |
|------|--------|------|
| image_qr | v13 | 二维码识别 |
| image_barcode | v13 | 条码识别 |
| image_measurement | v12 | 距离/尺寸测量 |
| image_fiducial | v12 | 标记/基准点检测 |
| image_ocr | v12 | OCR文字识别 |

## 使用方法

```python
# 颜色追踪
from color_tracker import ColorTracker
tracker = ColorTracker(target_color='red')
result = tracker.track(frame)

# 二维码检测（使用最新版）
from image_qr_v13 import QRDetector
detector = QRDetector()
codes = detector.detect(frame)

# AprilTag检测
from apriltag_detector import AprilTagDetector
detector = AprilTagDetector()
tags = detector.detect(frame)

# 图像测量
from image_measurement_v12 import ImageMeasurer
measurer = ImageMeasurer(pixels_per_cm=10.0)
distance = measurer.measure_distance(point1, point2)
```

## 依赖说明

- Python 3.8+
- OpenCV (`pip install opencv-python`)
- NumPy
- 部分模块需要：apriltag (`pip install apriltag`), pyzbar (`pip install pyzbar`)
- 人脸检测需要 OpenCV DNN 模型文件

## 常见问题

**Q: 版本号这么多，用哪个？**
A: 使用**最新版本号**（数字最大的）。旧版本保留供参考和兼容性。

**Q: 二维码检测不出来？**
A: 确保图像清晰、二维码占比适中（不要太大或太小）。尝试调整 `image_qr_v13.py` 中的检测参数。

**Q: 颜色检测在不同光照下不稳定？**
A: 使用自适应HSV范围，或先做白平衡。参考 `image_enhance_v2.py` 进行预处理。
