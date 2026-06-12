# RK3588S NPU 部署完整指南

> **相关文档**：
> - [RK3588S开发环境配置指南](../../07_RK3588S_硬件加速/docs/RK3588S开发环境配置指南.md) - 开发板环境配置
> - [硬件加速使用手册](../../07_RK3588S_硬件加速/docs/硬件加速使用手册.md) - RGA/GPU/NPU综合使用
> - [模型转换教程](模型转换教程.md) - ONNX到RKNN转换流程
> - [电赛视觉系统设计指南](../../06_OrangePi5_视觉驱动/docs/电赛视觉系统设计指南.md) - 视觉系统整体设计

## 1. RK3588S NPU 概述

| 参数 | 规格 |
|------|------|
| NPU算力 | 6 TOPS (INT8) |
| NPU核心 | 3核 (可独立/联合使用) |
| 支持框架 | TensorFlow/PyTorch/PaddlePaddle (通过RKNN) |
| 支持算子 | Conv, DWConv, FC, Pool, BN, ReLU, Sigmoid等主流算子 |
| 量化支持 | INT8/INT16/FP16 |

## 2. 开发板环境配置

### 2.1 系统刷写

```bash
# 下载官方Ubuntu/Debian固件
# 使用RKDevTool刷写到eMMC/SD卡
# 确认内核版本 >= 5.10, NPU驱动已加载
dmesg | grep rknpu
```

### 2.2 安装 RKNN-Lite2 (板端)

```bash
# 拷贝到开发板
scp rknnlite2-2.x.x-aarch64.deb user@board:~/

# 板端安装
sudo dpkg -i rknnlite2-2.x.x-aarch64.deb
# 或
pip install rknnlite2-2.x.x-cp3x-cp3x-linux_aarch64.whl
```

### 2.3 验证安装

```python
from rknnlite.api import RKNNLite
rknn = RKNNLite()
print('RKNN-Lite2 加载成功')
rknn.release()
```

## 3. 模型部署流程

### 3.1 完整部署步骤

```
PC端 (x86)                           板端 (RK3588S)
┌─────────────────┐                 ┌─────────────────┐
│ 1. 训练模型      │                 │                  │
│ 2. 导出ONNX      │                 │                  │
│ 3. RKNN转换+量化  │ ──传输.rknn──> │ 4. 加载RKNN模型   │
│                  │                 │ 5. 初始化NPU运行时 │
│                  │                 │ 6. 图像预处理      │
│                  │                 │ 7. NPU推理         │
│                  │                 │ 8. 后处理+可视化   │
└─────────────────┘                 └─────────────────┘
```

### 3.2 部署代码模板

```python
#!/usr/bin/env python3
"""RK3588S NPU 部署模板"""
import cv2
import numpy as np
import time
from rknnlite.api import RKNNLite

class NPUDeployer:
    def __init__(self, model_path, core_mask=RKNNLite.NPU_CORE_0_1_2):
        self.rknn = RKNNLite()
        ret = self.rknn.load_rknn(model_path)
        assert ret == 0, f"模型加载失败: {ret}"
        
        ret = self.rknn.init_runtime(core_mask=core_mask)
        assert ret == 0, f"运行时初始化失败: {ret}"
        
        print(f"[OK] NPU初始化完成, 使用全部3核")
    
    def preprocess(self, img, target_size=640):
        img = cv2.resize(img, (target_size, target_size))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        data = np.expand_dims(np.transpose(img, (2,0,1)), 0).astype(np.float32)
        return data / 255.0
    
    def infer(self, input_data):
        t0 = time.perf_counter()
        outputs = self.rknn.inference(inputs=[input_data])
        dt = (time.perf_counter() - t0) * 1000
        return outputs, dt
    
    def release(self):
        self.rknn.release()

# 使用示例
if __name__ == '__main__':
    deployer = NPUDeployer('yolov8n.rknn')
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        input_data = deployer.preprocess(frame)
        outputs, dt = deployer.infer(input_data)
        # ... 后处理和显示
    
    deployer.release()
    cap.release()
```

## 4. NPU 核心使用策略

### 4.1 核心掩码

```python
# 单核模式 (功耗低, 延迟高)
rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)

# 双核模式
rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1)

# 三核模式 (最高性能)
rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)

# 自动分配
rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
```

### 4.2 推荐策略

| 场景 | 核心数 | 说明 |
|------|--------|------|
| 单路视频 | 1核 | 节省功耗, 30fps足够 |
| 双路视频 | 2核 | 并行推理 |
| 三路视频 | 3核 | 每路独立一核 |
| 高帧率单路 | 3核 | 极致性能 |

## 5. 输入源适配

### 5.1 摄像头

```python
# USB摄像头
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# MIPI CSI摄像头 (需GStreamer)
cap = cv2.VideoCapture(
    'v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! appsink',
    cv2.CAP_GSTREAMER
)
```

### 5.2 视频文件/RTSP流

```python
# 视频文件
cap = cv2.VideoCapture('test.mp4')

# RTSP流
cap = cv2.VideoCapture('rtsp://admin:password@192.168.1.100:554/stream')
```

## 6. 性能优化技巧

### 6.1 零拷贝优化

```python
# 使用DMA-BUF共享内存, 避免CPU拷贝
outputs = self.rknn.inference(inputs=[input_data], lazy=True)
```

### 6.2 异步推理

```python
# 流水线: 预处理/推理/后处理并行
import threading

class AsyncInference:
    def __init__(self, deployer):
        self.deployer = deployer
        self.result = None
        self.lock = threading.Lock()
    
    def run_async(self, input_data):
        thread = threading.Thread(target=self._infer, args=(input_data,))
        thread.start()
        return thread
    
    def _infer(self, input_data):
        outputs, dt = self.deployer.infer(input_data)
        with self.lock:
            self.result = outputs
```

### 6.3 多模型串联

```python
# 检测 -> 跟踪, 两个模型交替使用NPU
# 方案1: 同步串行
det_result = det_model.infer(frame)
track_result = track_model.infer(det_result)

# 方案2: 分核并行 (检测用2核, 跟踪用1核)
```

## 7. 常见部署问题

### 7.1 NPU未加载

```bash
# 检查NPU驱动
ls /dev/rknpu*
dmesg | grep rknpu

# 如果没有设备节点, 需要更新固件/内核
```

### 7.2 推理性能不达标

- 确认使用NPU而非CPU推理 (检查init_runtime返回值)
- 降低输入分辨率
- 使用INT8量化模型
- 确认NPU核心数足够

### 7.3 内存不足

```bash
# 查看内存使用
free -h
cat /proc/rknpu/mem

# 优化: 使用单核推理, 减少batch_size, 及时释放模型
```

### 7.4 温度控制

```bash
# 监控温度
cat /sys/class/thermal/thermal_zone*/temp

# RK3588S NPU温度阈值: < 85°C
# 超温会自动降频, 影响推理性能
# 建议: 加散热片 + 风扇
```

## 8. 电赛应用场景

### 8.1 巡线小车

```
摄像头 → YOLOv8n(检测赛道线) → PID控制
输入: 320x320, 延迟<15ms, FPS>60
```

### 8.2 目标识别与抓取

```
摄像头 → YOLOv8n(目标检测) → 坐标映射 → 机械臂控制
输入: 640x640, FPS>30
```

### 8.3 多路监控

```
摄像头1 ─┐
摄像头2 ─┤→ NPU分核推理 → 综合判断
摄像头3 ─┘
```
