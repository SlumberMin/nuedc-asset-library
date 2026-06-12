# 10_opencv

## 模块概述

本模块提供 OpenCV 视觉算法的优化版本，针对嵌入式平台（Orange Pi 5）进行性能调优，包含优化版颜色检测、形状识别、直线检测等算法，以及编译优化指南和性能对比报告。

## 目录结构

```
10_opencv/
├── README.md                          # 本文档
├── optimized_color_detect.py          # 优化版颜色检测
├── optimized_shape_detect.py          # 优化版形状识别
├── optimized_line_detect.py           # 优化版直线检测
├── optimized_qr_detect.py             # 优化版二维码识别
├── optimized_template_match.py        # 优化版模板匹配
├── optimized_contour.py               # 优化版轮廓检测
├── optimized_threshold.py             # 优化版自适应阈值
├── roi_preprocess.py                  # ROI区域预处理加速
├── parallel_pipeline.py               # 多线程并行处理管线
├── compile_optimization_guide.md      # OpenCV编译优化指南
├── performance_comparison.md          # 优化前后性能对比
├── platform_tuning_guide.md           # 平台调优指南
├── benchmarks/
│   ├── benchmark_color.py             # 颜色检测性能测试
│   ├── benchmark_shape.py             # 形状识别性能测试
│   └── benchmark_pipeline.py          # 管线整体性能测试
└── docs/
    └── opencv_build_from_source.md    # OpenCV源码编译指南
```

## 使用方法

```python
# 使用优化版颜色检测
from optimized_color_detect import FastColorDetector

detector = FastColorDetector(hsv_lower=(0, 100, 100), hsv_upper=(10, 255, 255))
contours = detector.detect(frame)

# 使用并行处理管线
from parallel_pipeline import ParallelPipeline

pipeline = ParallelPipeline()
pipeline.add_stage("resize", lambda f: cv2.resize(f, (320, 240)))
pipeline.add_stage("detect", detector.detect)
results = pipeline.process(frame)
```

## 依赖说明

- Python 3.8+
- OpenCV (`pip install opencv-python`)
- NumPy
- 建议自行编译OpenCV以启用NEON优化

## 常见问题

**Q: 优化后速度提升多少？**
A: 典型场景下颜色检测提升2-5倍，直线检测提升3-8倍。详见 `performance_comparison.md`。

**Q: 需要重新编译OpenCV吗？**
A: 不强制，但建议编译时开启NEON、VFPv3优化。参考 `compile_optimization_guide.md`。

> ⚠️ **注意**：本模块为规划中的内容，部分文件待开发完善。
