# 视觉代码库 API 文档

> 版本: 1.0 | 更新日期: 2026-06-10  
> 路径: `10_视觉通用代码库/` + `06_OrangePi5_视觉驱动/`  
> 运行环境: Orange Pi 5 (RK3588S) / PC (Ubuntu/Windows)  
> 依赖: OpenCV 4.x, NumPy

---

## 目录

1. [颜色追踪器](#颜色追踪器)
2. [循迹线检测器](#循迹线检测器)
3. [圆形目标检测器](#圆形目标检测器)
4. [ArUco 标记检测器](#aruco-标记检测器)
5. [QR 码解码器](#qr-码解码器)
6. [相机标定工具](#相机标定工具)
7. [坐标变换](#坐标变换)
8. [距离估算器](#距离估算器)
9. [颜色阈值工具](#颜色阈值工具)
10. [相机驱动层](#相机驱动层)

---

## 颜色追踪器

**文件**: `10_视觉通用代码库/color_tracker.py`  
**功能**: 多目标颜色追踪 + Kalman 滤波预测 + 轨迹绘制

### `KalmanTracker` 类

单目标 Kalman 滤波跟踪器。

| 方法 | 参数 | 返回 | 说明 |
|---|---|---|---|
| `__init__(init_x, init_y, dt)` | 初始位置和时间步 | - | 状态: [x, y, vx, vy] |
| `predict()` | - | `(x, y)` | 预测下一帧位置 |
| `update(x, y)` | 观测位置 | - | 用观测值更新滤波器 |
| `get_position()` | - | `(x, y)` | 当前估计位置 |
| `get_velocity()` | - | `(vx, vy)` | 当前速度估计 |
| `is_lost()` | - | `bool` | 是否已丢失 (超过 max_lost 帧) |

**属性**: `trajectory` (deque) - 轨迹历史; `lost_count` - 丢失帧计数; `max_lost` - 最大允许丢失帧数 (默认 15)

### `ColorTracker` 类

多目标颜色追踪器。

#### 预设颜色 (`PRESETS`)

| 颜色 | HSV 范围 |
|---|---|
| `red` | H: [0,10]∪[160,180], S: [100,255], V: [100,255] |
| `blue` | H: [100,130], S: [100,255], V: [100,255] |
| `green` | H: [35,85], S: [80,255], V: [80,255] |
| `yellow` | H: [20,35], S: [100,255], V: [100,255] |
| `black` | H: [0,180], S: [0,255], V: [0,50] |

#### 构造函数

```python
ColorTracker(color_name='red', min_area=300, max_targets=5,
             use_kalman=True, trail_length=64)
```

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `color_name` | str / dict | `'red'` | 预设名或自定义 HSV 字典 |
| `min_area` | int | `300` | 最小轮廓面积 (像素²) |
| `max_targets` | int | `5` | 最大同时追踪目标数 |
| `use_kalman` | bool | `True` | 是否启用 Kalman 滤波 |
| `trail_length` | int | `64` | 轨迹历史长度 |

#### 方法

| 方法 | 参数 | 返回 | 说明 |
|---|---|---|---|
| `update(frame)` | BGR 图像 | `(results, mask)` | 处理一帧，返回追踪结果 |
| `draw(frame, results)` | BGR 图像, 结果列表 | BGR 图像 | 可视化追踪结果 |

**results 字段**: `{'id', 'cx', 'cy', 'vx', 'vy', 'trajectory', 'lost'}`

#### 使用示例

```python
tracker = ColorTracker(color_name='red', min_area=200)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    results, mask = tracker.update(frame)
    vis = tracker.draw(frame, results)
    for r in results:
        print(f"目标 {r['id']}: ({r['cx']}, {r['cy']}) 速度: ({r['vx']:.1f}, {r['vy']:.1f})")
    cv2.imshow('Tracker', vis)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

---

## 循迹线检测器

**文件**: `10_视觉通用代码库/line_detector.py`  
**功能**: 检测循迹线 (黑/红/蓝/绿)，支持直线和曲线

### `LineDetector` 类

#### 构造函数

```python
LineDetector(line_color='black', roi_ratio=0.5, binary_thresh=0,
             min_line_length=30, max_line_gap=10)
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `line_color` | `'black'` | 线条颜色 ('black'/'red'/'blue'/'green') |
| `roi_ratio` | `0.5` | ROI 占画面比例 (从底部开始) |
| `binary_thresh` | `0` | 二值化阈值 (0=自适应) |
| `min_line_length` | `30` | HoughLinesP 最小线段长度 |
| `max_line_gap` | `10` | HoughLinesP 最大线段间隙 |

#### 方法

| 方法 | 返回 | 说明 |
|---|---|---|
| `detect(frame)` | `dict` | 完整检测流程 |
| `detect_lines_hough(mask)` | `list[(x1,y1,x2,y2)]` | Hough 直线检测 |
| `detect_line_center(mask, n_slices)` | `list[(x,y)]` | 切片法检测线中心 |
| `calculate_deviation(center_points, width)` | `(deviation, angle)` | 偏移量计算 |
| `fit_curve(center_points)` | `(coeffs, curve_points)` | 多项式曲线拟合 |
| `draw(frame, result)` | BGR 图像 | 可视化 |

**detect() 返回**: `{'lines', 'center_points', 'curve_points', 'curve_coeffs', 'deviation', 'angle', 'mask', 'roi_offset'}`

#### 使用示例

```python
detector = LineDetector(line_color='black', roi_ratio=0.5)
result = detector.detect(frame)
if result['deviation'] is not None:
    if result['deviation'] > 30:
        print("偏右，左转")
    elif result['deviation'] < -30:
        print("偏左，右转")
```

---

## 圆形目标检测器

**文件**: `10_视觉通用代码库/circle_detector.py`  
**功能**: 霍夫圆检测 + 轮廓拟合圆检测

### `CircleDetector` 类

#### 构造函数

```python
CircleDetector(method='contour', min_radius=10, max_radius=200,
               min_area=200, color_filter=None)
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `method` | `'contour'` | 'hough' / 'contour' / 'both' |
| `min_radius` | `10` | 最小半径 (像素) |
| `max_radius` | `200` | 最大半径 (像素) |
| `min_area` | `200` | 最小轮廓面积 |
| `color_filter` | `None` | HSV 颜色过滤 `{'lower': [H,S,V], 'upper': [H,S,V]}` |

#### 方法

| 方法 | 返回 | 说明 |
|---|---|---|
| `detect(frame)` | `list[dict]` | 完整检测流程 |
| `detect_hough(blurred)` | `list[dict]` | 霍夫圆检测 |
| `detect_contour(frame, mask)` | `list[dict]` | 轮廓拟合圆检测 |
| `draw(frame, results, show_info)` | BGR 图像 | 可视化 |

**结果字段**: `{'cx', 'cy', 'radius', 'method', 'confidence', 'circularity', 'area', 'ellipse'}`

---

## ArUco 标记检测器

**文件**: `10_视觉通用代码库/aruco_detector.py`  
**功能**: ArUco/AprilTag 检测 + 6DOF 位姿估计

### `ArucoDetector` 类

#### 构造函数

```python
ArucoDetector(dict_type='5x5_100', marker_length=0.05,
              camera_matrix=None, dist_coeffs=None)
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `dict_type` | `'5x5_100'` | 字典类型 (见下表) |
| `marker_length` | `0.05` | 标记物理边长 (米) |
| `camera_matrix` | `None` | 3×3 相机内参矩阵 |
| `dist_coeffs` | `None` | 畸变系数 |

#### 支持的字典

| 字典 | 说明 |
|---|---|
| `4x4_50` ~ `4x4_1000` | 4×4 位标记 |
| `5x5_50` ~ `5x5_1000` | 5×5 位标记 |
| `6x6_50` ~ `6x6_1000` | 6×6 位标记 |
| `7x7_50` ~ `7x7_1000` | 7×7 位标记 |
| `apriltag_16h5` ~ `apriltag_36h11` | AprilTag |

#### 方法

| 方法 | 返回 | 说明 |
|---|---|---|
| `detect(frame)` | `list[dict]` | 检测标记 |
| `draw(frame, results)` | BGR 图像 | 可视化 |
| `estimate_pose(corners)` | `list[dict]` | 6DOF 位姿估计 |
| `load_camera_params(filepath)` | `bool` | 加载相机参数 |
| `generate_marker(id, dict_type, size, path)` | 图像 (静态) | 生成标记图片 |

**结果字段**: `{'id', 'corners', 'center', 'pose'}`  
**pose 字段**: `{'rvec', 'tvec', 'rotation_matrix', 'euler', 'distance'}`

---

## QR 码解码器

**文件**: `10_视觉通用代码库/qr_decoder.py`

### 主要函数

| 函数 | 说明 |
|---|---|
| `decode_qr(frame)` | 检测并解码 QR 码 |
| `decode_multi_qr(frame)` | 多 QR 码同时解码 |

---

## 相机标定工具

**文件**: `10_视觉通用代码库/camera_calibration.py`

### `CameraCalibrator` 类

#### 构造函数

```python
CameraCalibrator(chessboard_size=(9, 6), square_size=0.025)
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `chessboard_size` | `(9, 6)` | 棋盘格内角点数 (列, 行) |
| `square_size` | `0.025` | 每格物理边长 (米) |

#### 方法

| 方法 | 返回 | 说明 |
|---|---|---|
| `find_corners(image)` | `(found, corners)` | 查找棋盘格角点 |
| `add_image(image)` | `bool` | 添加标定图像 |
| `add_images_from_dir(dir, pattern)` | `int` | 批量添加标定图像 |
| `calibrate(image_size)` | `bool` | 执行标定 |
| `undistort(image)` | 图像 | 校正畸变 |
| `undistort_with_roi(image)` | 图像 | 校正畸变 + 裁剪有效区域 |
| `save(filepath)` | - | 保存标定参数 (.npz) |
| `load(filepath)` | `bool` | 加载标定参数 |
| `get_undistort_maps()` | `(map1, map2)` | 获取畸变校正映射表 |
| `draw_corners(image, corners, found)` | 图像 | 绘制角点 |
| `print_report()` | - | 打印标定报告 |

**标定结果属性**: `camera_matrix`, `dist_coeffs`, `rvecs`, `tvecs`, `reprojection_error`

### `RealtimeCalibrator` 类

实时标定流程封装。

```python
RealtimeCalibrator(camera_id=0, chessboard_size=(9, 6), square_size=0.025)
```

| 方法 | 说明 |
|---|---|
| `capture_and_calibrate(n_images=20, auto_capture=True)` | 实时采集 + 标定 |

---

## 坐标变换

**文件**: `10_视觉通用代码库/coordinate_transform.py`

### 主要功能

| 函数 | 说明 |
|---|---|
| `pixel_to_world(px, py, camera_matrix, dist_coeffs, rvec, tvec)` | 像素坐标转世界坐标 |
| `world_to_pixel(wx, wy, wz, camera_matrix, rvec, tvec)` | 世界坐标转像素坐标 |
| `pixel_to_camera(px, py, Z, camera_matrix)` | 像素坐标转相机坐标 |

---

## 距离估算器

**文件**: `10_视觉通用代码库/distance_estimator.py`

### 主要功能

| 函数 | 说明 |
|---|---|
| `estimate_distance_by_size(obj_pixel_size, obj_real_size, focal_length)` | 基于已知尺寸估算距离 |
| `estimate_distance_by_aruco(marker_corners, marker_length, camera_matrix)` | 基于 ArUco 标记估算距离 |

---

## 颜色阈值工具

**文件**: `10_视觉通用代码库/color_threshold_tool.py`

交互式 HSV 阈值调节工具，提供滑动条实时调参。

---

## 相机驱动层

**路径**: `06_OrangePi5_视觉驱动/camera/`

### 模块列表

| 文件 | 说明 |
|---|---|
| `camera_manager.py` | 相机管理器 (多相机统一接口) |
| `camera_v4l2.py` | V4L2 驱动 (Linux 原生) |
| `camera_libuvc.py` | libuvc 驱动 (USB UVC) |
| `camera_config.py` | 相机参数配置 |
| `fast_capture.py` | 高速采集 (零拷贝) |
| `auto_exposure.py` | 自动曝光控制 |
| `hdr_capture.py` | HDR 合成拍摄 |
| `multi_frame_denoise.py` | 多帧降噪 |
| `multi_camera.py` | 多相机同步采集 |
| `yuyv_processor.py` | YUYV 格式处理 |

### `CameraManager` 类 (核心接口)

```python
from camera.camera_manager import CameraManager

cam = CameraManager(camera_id=0, width=640, height=480, fps=30)
cam.open()
frame = cam.read()
cam.release()
```

### 性能优化建议

1. 使用 `fast_capture.py` 的零拷贝模式减少内存分配
2. 多线程采集: 采集线程和处理线程分离
3. 使用 YUYV 格式避免色彩空间转换开销
4. 启用硬件加速 (RK3588S RGA/NPU)
