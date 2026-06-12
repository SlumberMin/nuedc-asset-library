# 嵌入式视觉通用代码库

> 电赛专用 | Orange Pi 5 / RK3588 平台 | 目标 30fps+

## 目录结构

```
embedded_vision/
├── __init__.py              # 包说明
├── platform_utils.py        # 平台优化工具(OpenCV/多线程/后端选择)
├── object_detector.py       # 模块1: 轻量级目标检测(MobileNet-SSD/YOLOv8)
├── deepsort_tracker.py      # 模块2: 实时目标跟踪(DeepSORT简化版)
├── semantic_segmentation.py # 模块3: 场景理解(颜色分割/语义分割)
├── stereo_depth.py          # 模块4: 3D视觉(双目测距/结构光)
├── visual_slam.py           # 模块5: 视觉SLAM(ORB里程计)
└── quickstart.py            # 快速启动示例
```

## 技术搜索笔记

### YOLOv8 边缘部署(2024-2025)
- **ultralytics** 官方支持导出 ONNX/TensorRT/CoreML/RKNN
- YOLOv8n (nano) 320×320 → RK3588 NPU INT8: **~40fps**
- 导出: `yolo export model=yolov8n.pt format=onnx imgsz=320`
- RKNN转换: `RKNN-Toolkit2` 可直接从ONNX转INT8

### OpenCV ARM优化
- `cv2.setNumThreads(4)` 开启多线程
- OpenCV 4.8+ NEON加速已默认开启
- `cv2.ocl.setUseOpenCL(True)` 启用GPU加速
- SGBM用 `STEREO_SGBM_MODE_SGBM_3WAY` 模式提速50%

### RK3588 NPU (RKNN)
- 6 TOPS INT8算力, 支持CNN/Transformer
- RKNN-Toolkit2: pip install rknn-toolkit2
- 支持模型: YOLOv5/v8, MobileNet, ResNet, PaddleDetection等
- 推理API: `RKNNLite` (轻量运行时)

### 电赛优秀实现参考
- OpenMV: MicroPython视觉, 适合简单任务
- K210/K230: 平头哥NPU, 性价比高
- Jetson Nano: NVIDIA生态, 但功耗高
- **Orange Pi 5 + RK3588**: 6 TOPS + 低功耗, 电赛首选

## 快速开始

### 1. 安装依赖
```bash
pip install opencv-python numpy scipy
# 可选: pip install onnxruntime rknn-toolkit2
```

### 2. 运行示例
```python
from embedded_vision.object_detector import MobileNetSSDDetector
from embedded_vision.platform_utils import CameraThread

cam = CameraThread(src=0).start()
detector = MobileNetSSDDetector()

while True:
    frame = cam.read()
    results = detector.detect(frame)  # [(class, conf, (x,y,w,h)), ...]
    for cls, conf, box in results:
        print(f"{cls}: {conf:.2f}")
```

### 3. 综合演示
```bash
cd embedded_vision
python quickstart.py
```

## 各模块详细用法

### 模块1: 目标检测
```python
# 方案A: OpenCV DNN (无需额外依赖)
det = MobileNetSSDDetector(conf_threshold=0.5)

# 方案B: YOLOv8 ONNX
det = YOLOv8Detector("yolov8n.onnx", input_size=320)

# 方案C: YOLOv8 RKNN (最快)
det = YOLOv8Detector("yolov8n.rknn", use_rknn=True)

results = det.detect(frame)  # → [(class, conf, (x,y,w,h))]
```

### 模块2: 目标跟踪
```python
tracker = SimpleDeepSORT(iou_threshold=0.3, max_age=30)
detections = det.detect(frame)
tracks = tracker.update(detections)  # → [(track_id, class, (x,y,w,h))]
```

### 模块3: 语义分割
```python
# 颜色分割(无需模型, ~60fps)
seg = ColorSegmentor()
mask = seg.segment(frame)  # 0=背景, 1=红, 2=绿, 3=蓝...

# 循迹检测
road = RoadDetector()
info = road.detect(frame)  # → {drivable_area, center_offset, obstacles, line_angle}
```

### 模块4: 双目测距
```python
stereo = StereoDepth(calib_params, num_disparities=64)
disparity = stereo.compute_disparity(img_left, img_right)
depth = stereo.get_depth(disparity)  # 深度图(mm)
distance = stereo.measure_distance(disparity, roi=(200,100,240,280))

# 结构光(单目+激光)
sl = StructureLight(baseline_mm=50, laser_angle=30)
profile = sl.scan_profile(frame)  # → Nx3 (x, y, depth)
```

### 模块5: 视觉SLAM
```python
K = np.array([[600,0,320],[0,600,240],[0,0,1]], dtype=np.float64)
slam = MonoSLAM(K)
slam.process_frame(frame)
pos = slam.get_position()  # [x, y, z] mm
yaw = slam.get_yaw()       # 度
traj = slam.get_trajectory()  # Nx3 轨迹
```

## 性能参考 (Orange Pi 5, RK3588)

| 模块 | 方案 | 分辨率 | 帧率 |
|------|------|--------|------|
| 目标检测 | YOLOv8n RKNN INT8 | 320×320 | ~40fps |
| 目标检测 | MobileNet-SSD OpenCV | 300×300 | ~25fps |
| 目标跟踪 | DeepSORT简化 | 640×480 | ~45fps |
| 颜色分割 | HSV阈值 | 640×480 | ~60fps |
| 双目SGBM | SGBM-3WAY | 640×480 | ~20fps |
| 视觉VO | ORB+PnP | 640×480 | ~30fps |

## 电赛应用方案

| 题目类型 | 推荐模块组合 |
|----------|-------------|
| 智能小车循迹 | 颜色分割 + RoadDetector |
| 目标识别追踪 | YOLOv8 + DeepSORT |
| 避障测距 | 双目测距 + 目标检测 |
| 自主导航 | Visual SLAM + 双目 |
| 物体分拣 | 目标检测 + 颜色分割 |
| 机械臂引导 | 目标检测 + 结构光测距 |
