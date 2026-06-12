# 09_RK3588S硬件加速

## 模块概述

本模块提供 RK3588S 芯片的硬件加速方案，包括 RGA（2D图像处理加速）、GPU（OpenCL通用计算）和 NPU（RKNN神经网络推理）三大硬件加速引擎的封装和使用示例。

## 目录结构

```
09_hw_acceleration/
├── README.md                          # 本文档
├── rga_accelerator.py                 # RGA图像处理加速封装
├── rga_resize.py                      # RGA缩放
├── rga_crop_rotate.py                 # RGA裁剪与旋转
├── rga_color_convert.py               # RGA色彩空间转换
├── gpu_opencl_accelerator.py          # OpenCL GPU加速封装
├── gpu_threshold.py                   # GPU加速二值化
├── gpu_filter.py                      # GPU加速滤波
├── npu_rknn_inference.py              # RKNN NPU推理封装
├── npu_model_loader.py                # NPU模型加载器
├── npu_preprocess.py                  # NPU推理前处理
├── npu_postprocess.py                 # NPU推理后处理
├── performance_benchmark.py           # 性能基准测试
├── hardware_capability.py             # 硬件能力检测
├── examples/
│   ├── rga_demo.py                    # RGA使用示例
│   ├── gpu_demo.py                    # GPU使用示例
│   └── npu_demo.py                    # NPU使用示例
└── docs/
    ├── rk3588s_architecture.md        # RK3588S架构说明
    └── performance_comparison.md      # 硬件加速性能对比
```

## 使用方法

```python
# RGA加速缩放
from rga_accelerator import RGAAccelerator

rga = RGAAccelerator()
resized = rga.resize(frame, 320, 240)
cropped = rga.crop(frame, x=100, y=100, w=200, h=200)

# NPU推理
from npu_rknn_inference import RKNPUInference

npu = RKNPUInference(model_path="yolov5s.rknn")
results = npu.inference(frame)
```

## 依赖说明

- Orange Pi 5 / RK3588S 开发板
- RKNN-Toolkit2（模型转换）
- RKNN-Lite2（板端推理）
- librkgpu（GPU加速）
- librga（RGA加速）
- Python 3.8+, NumPy, OpenCV

## 常见问题

**Q: RGA和GPU加速怎么选？**
A: 简单图像操作（缩放/裁剪/旋转）用RGA，复杂计算（滤波/二值化）用GPU。

**Q: NPU推理速度慢？**
A: 确认模型已量化为INT8，检查NPU核心是否全部启用。

> ⚠️ **注意**：本模块为规划中的内容，部分文件待开发完善。
