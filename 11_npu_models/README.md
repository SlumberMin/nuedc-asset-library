# 11_npu_models

## 模块概述

本模块收录适用于 RK3588S NPU 的目标检测和图像分类模型，包括 YOLOv5/v8、PicoDet、MobileNet、EfficientNet 等，提供 RKNN 格式模型文件、转换脚本和部署指南。

## 目录结构

```
11_npu_models/
├── README.md                          # 本文档
├── yolov5/
│   ├── yolov5n.rknn                   # YOLOv5-Nano (最快)
│   ├── yolov5s.rknn                   # YOLOv5-Small (推荐)
│   ├── yolov5m.rknn                   # YOLOv5-Medium
│   ├── export_rknn.py                 # 模型转换脚本
│   └── inference_demo.py              # 推理示例
├── yolov8/
│   ├── yolov8n.rknn                   # YOLOv8-Nano
│   ├── yolov8s.rknn                   # YOLOv8-Small
│   ├── export_rknn.py                 # 模型转换脚本
│   └── inference_demo.py              # 推理示例
├── picodet/
│   ├── picodet_s.rknn                 # PicoDet-Small
│   ├── picodet_m.rknn                 # PicoDet-Medium
│   └── inference_demo.py              # 推理示例
├── mobilenet/
│   ├── mobilenetv3_large.rknn         # MobileNetV3-Large (分类)
│   ├── mobilenetv3_small.rknn         # MobileNetV3-Small (分类)
│   └── inference_demo.py              # 推理示例
├── efficientnet/
│   ├── efficientnet_b0.rknn           # EfficientNet-B0
│   └── inference_demo.py              # 推理示例
├── tools/
│   ├── rknn_converter.py              # 通用RKNN转换工具
│   ├── model_quantize.py              # INT8量化工具
│   ├── benchmark_all.py               # 全模型性能对比
│   └── dataset_preparer.py            # 数据集准备工具
├── docs/
│   ├── rknn_model_conversion.md       # RKNN模型转换教程
│   ├── quantization_guide.md          # 量化指南
│   └── deployment_guide.md            # 部署指南
└── labels/
    └── coco_labels.txt                # COCO类别标签
```

## 使用方法

```python
# YOLOv5推理示例
from rknnlite.api import RKNNLite

rknn = RKNNLite()
rknn.load_rknn('yolov5/yolov5s.rknn')
rknn.init_runtime()

# 推理
img = cv2.resize(frame, (640, 640))
outputs = rknn.inference(inputs=[img])

# 后处理获取检测框
boxes, scores, classes = postprocess(outputs)
```

## 依赖说明

- RKNN-Toolkit2（PC端模型转换）
- RKNN-Lite2（板端推理）
- NumPy, OpenCV
- 模型来源：Ultralytics YOLO, PaddleDetection

## 常见问题

**Q: 模型怎么选？**
A: 速度优先选YOLOv5n/YOLOv8n，精度优先选YOLOv5s/YOLOv8s。见 `docs/deployment_guide.md`。

**Q: 如何训练自定义数据集？**
A: 1) 在PC端用YOLO训练 → 2) 导出ONNX → 3) 用 `rknn_converter.py` 转RKNN → 4) 板端部署。

**Q: INT8量化后精度下降严重？**
A: 使用校准数据集（100-500张），参考 `quantization_guide.md`。

> ⚠️ **注意**：本模块为规划中的内容，部分文件待开发完善。当前RKNN模型文件需自行转换生成。
