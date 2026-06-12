# 08_OrangePi5视觉驱动

## 模块概述

本模块提供 Orange Pi 5 平台的相机驱动和图像采集优化方案，支持 V4L2、libuvc、OpenCV 三种后端，适配 USB 摄像头和 CSI 相机，提供高速抓拍、多相机同步等高级功能。

## 目录结构

```
08_vision_drivers/
├── README.md                          # 本文档
├── v4l2_camera_driver.py              # V4L2后端相机驱动
├── libuvc_camera_driver.py            # libuvc后端相机驱动
├── opencv_camera_driver.py            # OpenCV后端相机驱动
├── camera_base.py                     # 相机驱动基类
├── yuyv_processor.py                  # YUYV格式图像处理器
├── multi_camera_manager.py            # 多相机管理器
├── camera_config.py                   # 相机参数配置
├── high_speed_capture.py              # 高速抓拍模块
├── hdr_capture.py                     # HDR合成模块
├── multi_frame_denoise.py             # 多帧降噪模块
├── camera_calibration_tool.py         # 相机标定工具
├── camera_benchmark.py                # 相机性能基准测试
├── configs/
│   ├── usb_camera_default.json        # USB相机默认配置
│   ├── csi_camera_default.json        # CSI相机默认配置
│   └── multi_camera_config.json       # 多相机配置
├── examples/
│   ├── basic_capture.py               # 基础采集示例
│   ├── multi_camera_demo.py           # 多相机示例
│   └── high_speed_demo.py             # 高速采集示例
└── docs/
    └── camera_troubleshooting.md      # 相机故障排查
```

## 使用方法

```python
# 基础使用
from v4l2_camera_driver import V4L2Camera

cam = V4L2Camera(device=0, width=640, height=480, fps=30)
frame = cam.read()
cam.release()

# 多相机使用
from multi_camera_manager import MultiCameraManager

manager = MultiCameraManager()
manager.add_camera(0, name="front")
manager.add_camera(2, name="back")
frames = manager.read_all()
```

## 依赖说明

- Python 3.8+
- OpenCV (`pip install opencv-python`)
- V4L2 (Linux内核自带)
- libuvc (`pip install libuvc`，可选)
- NumPy

## 常见问题

**Q: USB摄像头打不开？**
A: 检查 `/dev/video*` 设备节点，确认用户有读写权限（`sudo chmod 666 /dev/video0`）。

**Q: 帧率上不去？**
A: 降低分辨率，或使用 V4L2 后端替代 OpenCV 后端（减少一次格式转换）。参考 `camera_benchmark.py` 测试。

**Q: 多相机同步采集有延迟？**
A: 使用 `MultiCameraManager` 的同步模式，或考虑硬件触发同步。

> ⚠️ **注意**：本模块为规划中的内容，部分文件待开发完善。
