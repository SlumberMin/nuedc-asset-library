# -*- coding: utf-8 -*-
"""
嵌入式视觉通用代码库 - Orange Pi 5 平台
========================================
目标: 30fps+ 实时视觉处理

模块列表:
  1. mobilenet_ssd_detector  - 轻量级目标检测
  2. deepsort_tracker        - 实时目标跟踪
  3. semantic_segmentation   - 场景理解/语义分割
  4. stereo_depth            - 3D视觉/双目测距
  5. visual_slam             - 视觉SLAM简化版

技术要点(2024-2025):
  - YOLOv8 ONNX/TensorRT/NCNN 部署 → 边缘设备推理
  - OpenCV ARM NEON + VPI 加速 → SIMD优化
  - RK3588 NPU (RKNN-Toolkit2) → 6 TOPS INT8推理
  - 双线程流水线: 采集线程 + 推理线程, 降低延迟
  - INT8量化 + 模型剪枝 → 2-4x加速, 精度损失<2%

电赛应用场景:
  - 智能小车障碍物检测与避障
  - 目标识别、追踪与抓取
  - 视觉循迹与路径规划
  - 双目测距辅助导航
"""
__version__ = "1.0.0"
