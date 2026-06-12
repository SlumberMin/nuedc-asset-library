# NPU 部署最佳实践 (RKNN)

## 1. RKNN 工具链概述

### 1.1 RKNN 架构
```
┌─────────────────────────────────────────────────────┐
│                   应用层 (Application)               │
├─────────────────────────────────────────────────────┤
│              RKNN Runtime (C/C++/Python)            │
├──────────┬──────────┬──────────┬────────────────────┤
│ NPU Core0│ NPU Core1│ NPU Core2│   CPU Fallback     │
└──────────┴──────────┴──────────┴────────────────────┘
│               RK3588S 硬件 (6 TOPS)                 │
└─────────────────────────────────────────────────────┘
```

### 1.2 工具链组件
- **RKNN Toolkit2**: PC端模型转换与量化
- **RKNN Runtime**: 板端推理引擎
- **RKNN Model Zoo**: 预训练模型和示例

---

## 2. 模型转换

### 2.1 基本转换流程
```python
from rknn.api import RKNN

def convert_model():
    rknn = RKNN(verbose=True)
    
    # 1. 配置
    rknn.config(
        mean_values=[[123.675, 116.28, 103.53]],
        std_values=[[58.395, 57.12, 57.375]],
        target_platform='rk3588',
        quantized_dtype='asymmetric_quantized-8',
        optimization_level=3
    )
    
    # 2. 加载模型
    ret = rknn.load_onnx(model='model.onnx')
    assert ret == 0, 'Load model failed!'
    
    # 3. 构建（含量化）
    ret = rknn.build(
        do_quantization=True,
        dataset='calibration_dataset.txt'
    )
    assert ret == 0, 'Build model failed!'
    
    # 4. 导出
    rknn.export_rknn('model.rknn')
    
    # 5. 评估精度
    perf_results = rknn.eval_perf()
    accuracy_results = rknn.eval_accuracy()
    
    rknn.release()
```

### 2.2 校准数据集准备
```python
# calibration_dataset.txt - 每行一个图片路径
# 建议 500-1000 张代表性图片
# 覆盖各种场景、光照、角度

import os

def prepare_calibration(data_dir, output_txt, num_images=500):
    images = []
    for f in os.listdir(data_dir):
        if f.lower().endswith(('.jpg', '.png', '.bmp')):
            images.append(os.path.join(data_dir, f))
    
    # 随机采样
    import random
    random.shuffle(images)
    images = images[:num_images]
    
    with open(output_txt, 'w') as f:
        for img in images:
            f.write(img + '\n')

prepare_calibration('train_images/', 'calibration_dataset.txt')
```

### 2.3 常见模型转换
```python
# YOLOv8
rknn.config(
    mean_values=[[0, 0, 0]],
    std_values=[[255, 255, 255]],
    target_platform='rk3588'
)
rknn.load_onnx(model='yolov8n.onnx')

# MobileNetV3
rknn.config(
    mean_values=[[123.675, 116.28, 103.53]],
    std_values=[[58.395, 57.12, 57.375]],
    target_platform='rk3588'
)
rknn.load_onnx(model='mobilenetv3.onnx')

# PPOCR
rknn.config(
    mean_values=[[123.675, 116.28, 103.53]],
    std_values=[[58.395, 57.12, 57.375]],
    target_platform='rk3588',
    quantized_algorithm='mmse'  # OCR用mmse算法
)
rknn.load_onnx(model='ppocr_det.onnx')
```

---

## 3. 量化优化

### 3.1 量化算法选择
| 算法 | 适用场景 | 精度 | 速度 |
|------|----------|------|------|
| normal | 通用 | ★★★ | ★★★★★ |
| kl_divergence | 检测/分割 | ★★★★ | ★★★★ |
| mmse | OCR/细粒度 | ★★★★★ | ★★★ |
| asymmetric_quantized-16 | 高精度需求 | ★★★★★ | ★★ |

```python
# 选择量化算法
rknn.build(
    do_quantization=True,
    dataset='calibration.txt',
    quantized_algorithm='normal',  # 或 'kl_divergence' / 'mmse'
)
```

### 3.2 混合量化
```python
# 对精度敏感层使用高精度
rknn.build(
    do_quantization=True,
    dataset='calibration.txt',
    quantized_dtype='asymmetric_quantized-u8',  # 默认INT8
)

# 部分层指定为 FP16
# 在 RKNN Toolkit2 高级配置中设置
rknn.config(
    target_platform='rk3588',
    quantized_dtype='asymmetric_quantized-u8',
    # 可通过 custom_string 指定特定层的量化方式
)
```

### 3.3 量化精度调优
```python
# 评估量化前后精度差异
def evaluate_quantization(rknn, test_images, test_labels):
    rknn.init_runtime()
    
    correct = 0
    total = len(test_images)
    
    for img_path, label in zip(test_images, test_labels):
        img = cv2.imread(img_path)
        img = cv2.resize(img, (224, 224))
        
        outputs = rknn.inference(inputs=[img])
        pred = np.argmax(outputs[0])
        
        if pred == label:
            correct += 1
    
    accuracy = correct / total
    return accuracy

# 如果精度损失 > 1%，尝试：
# 1. 增加校准数据量
# 2. 使用 kl_divergence / mmse 算法
# 3. 混合精度量化
```

---

## 4. 板端部署

### 4.1 C++ 部署
```cpp
#include "rknn_api.h"
#include <opencv2/opencv.hpp>

class RKNNInference {
public:
    int init(const char* model_path) {
        int ret = rknn_init(&ctx, model_path, 0, 0, NULL);
        if (ret < 0) {
            printf("rknn_init failed! ret=%d\n", ret);
            return -1;
        }
        
        // 查询输入输出信息
        rknn_input_output_num io_num;
        rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
        
        return 0;
    }
    
    int inference(cv::Mat& img) {
        // 设置输入
        rknn_input inputs[1];
        memset(inputs, 0, sizeof(inputs));
        inputs[0].index = 0;
        inputs[0].type = RKNN_TENSOR_UINT8;
        inputs[0].size = img.total() * img.elemSize();
        inputs[0].fmt = RKNN_TENSOR_NHWC;
        inputs[0].buf = img.data;
        
        rknn_inputs_set(ctx, 1, inputs);
        
        // 推理
        rknn_run(ctx, NULL);
        
        // 获取输出
        rknn_output outputs[1];
        memset(outputs, 0, sizeof(outputs));
        outputs[0].want_float = true;
        rknn_outputs_get(ctx, 1, outputs, NULL);
        
        // 处理结果
        float* result = (float*)outputs[0].buf;
        
        rknn_outputs_release(ctx, 1, outputs);
        return 0;
    }
    
    void release() {
        rknn_destroy(ctx);
    }
    
private:
    rknn_context ctx;
};
```

### 4.2 Python 部署
```python
from rknnlite.api import RKNNLite

def deploy_on_board():
    rknn = RKNNLite()
    rknn.load_rknn('model.rknn')
    rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
    
    # 推理
    img = cv2.imread('test.jpg')
    img = cv2.resize(img, (640, 640))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    outputs = rknn.inference(inputs=[img])
    
    # 解析结果
    boxes, classes, scores = post_process(outputs)
    
    rknn.release()
    return boxes, classes, scores
```

### 4.3 多线程推理
```python
import threading
from queue import Queue

class MultiModelRunner:
    def __init__(self, model_paths):
        self.models = []
        for path in model_paths:
            rknn = RKNNLite()
            rknn.load_rknn(path)
            rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
            self.models.append(rknn)
        
        # 不同模型分配不同核心
        cores = [
            RKNNLite.NPU_CORE_0,
            RKNNLite.NPU_CORE_1,
            RKNNLite.NPU_CORE_2
        ]
        for i, rknn in enumerate(self.models):
            rknn.init_runtime(core_mask=cores[i % 3])
    
    def inference_pipeline(self, frame):
        """流水线推理"""
        results = {}
        threads = []
        
        for i, rknn in enumerate(self.models):
            def worker(idx, model, img):
                results[idx] = model.inference(inputs=[img])
            t = threading.Thread(target=worker, args=(i, rknn, frame))
            threads.append(t)
            t.start()
        
        for t in threads:
            t.join()
        
        return results
```

---

## 5. 性能优化

### 5.1 零拷贝推理
```c
// 使用 DMA-BUF 实现零拷贝
rknn_mem_info mem_info;
mem_info.flags = RKNN_MEM_FLAG_DMA;
mem_info.size = img_size;
mem_info.fd = dma_fd;  // 外部 DMA-BUF fd

rknn_set_io_mem(ctx, &mem_info, &input);

// 推理时无需内存拷贝
rknn_run(ctx, NULL);
```

### 5.2 异步推理
```c
// 非阻塞推理
rknn_run_async(ctx, NULL);

// 做其他事情...

// 等待推理完成
rknn_wait(ctx);

// 获取结果
rknn_outputs_get(ctx, 1, outputs, NULL);
```

### 5.3 多核并行策略
```
方案1: 单模型三核并行
  模型A → Core 0 + Core 1 + Core 2

方案2: 多模型独立核心
  模型A → Core 0
  模型B → Core 1
  模型C → Core 2

方案3: 流水线
  Stage1(Core 0) → Stage2(Core 1) → Stage3(Core 2)
```

### 5.4 DDR 带宽优化
```bash
# 监控 DDR 带宽使用
cat /sys/class/devfreq/dmc/cur_freq
cat /sys/class/devfreq/dmc/load

# 优化策略:
# 1. 减少输入输出数据量 (降低分辨率)
# 2. 使用 INT8 代替 FP32 (4x 减少)
# 3. 零拷贝 (避免额外内存搬运)
```

---

## 6. 常见问题解决

### 6.1 不支持的算子
```python
# 检查不支持的算子
ret = rknn.build(do_quantization=False)
# 查看日志中的 WARNING: Unsupported op: xxx

# 解决方案:
# 1. 修改模型，替换为支持的算子
# 2. 使用 CPU Fallback (性能会下降)
# 3. 自定义算子 (高级)
```

### 6.2 精度下降排查
```python
# 步骤1: 不量化测试
rknn.build(do_quantization=False)
# 如果精度正常 → 量化问题

# 步骤2: 检查预处理
# mean_values 和 std_values 是否正确

# 步骤3: 增加校准数据
# 从 200 张增加到 1000 张

# 步骤4: 尝试不同量化算法
# normal → kl_divergence → mmse
```

### 6.3 内存不足
```bash
# 检查内存使用
cat /proc/meminfo
cat /proc/buddyinfo

# 优化方案:
# 1. 模型输入尺寸减小
# 2. 多模型共享内存池
# 3. 动态加载/卸载模型
```

### 6.4 温度过高
```bash
# 监控温度
cat /sys/class/thermal/thermal_zone*/temp

# 解决方案:
# 1. 添加散热片/风扇
# 2. 降低 NPU 频率
# 3. 减少推理频率
```

---

## 7. 模型性能基准

### 7.1 RK3588S 基准数据
| 模型 | 输入尺寸 | 量化 | FPS | 精度 |
|------|----------|------|-----|------|
| MobileNetV3-S | 224×224 | INT8 | 850 | 67.5% |
| ResNet-50 | 224×224 | INT8 | 210 | 76.1% |
| YOLOv5s | 640×640 | INT8 | 42 | 37.4mAP |
| YOLOv8n | 640×640 | INT8 | 55 | 37.3mAP |
| NanoDet-Plus | 320×320 | INT8 | 120 | 27.0mAP |
| PPOCR-Det | 960×960 | INT8 | 35 | - |
| PPOCR-Rec | 48×320 | INT8 | 280 | 90%+ |
| BiSeNetV2 | 1024×512 | INT8 | 38 | 73.4mIoU |

### 7.2 优化前后对比
| 优化项 | 优化前(ms) | 优化后(ms) | 提升 |
|--------|-----------|-----------|------|
| 原始推理 | 45 | - | - |
| +零拷贝 | 45 | 38 | 15% |
| +三核并行 | 38 | 18 | 53% |
| +输入优化 | 18 | 14 | 22% |
| +后处理优化 | 14 | 11 | 21% |

---

## 8. 电赛实战模板

### 8.1 视觉检测系统
```python
class VisionSystem:
    def __init__(self, model_path):
        self.rknn = RKNNLite()
        self.rknn.load_rknn(model_path)
        self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
    
    def detect(self, frame):
        # 预处理
        img = cv2.resize(frame, (640, 640))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # 推理
        t0 = time.perf_counter()
        outputs = self.rknn.inference(inputs=[img])
        t1 = time.perf_counter()
        
        # 后处理
        boxes, scores, classes = self.postprocess(outputs)
        
        print(f"Inference: {(t1-t0)*1000:.1f}ms")
        return boxes, scores, classes
    
    def postprocess(self, outputs):
        # YOLO 后处理
        boxes, scores, classes = [], [], []
        for det in outputs[0]:
            if det[4] > 0.5:  # 置信度阈值
                boxes.append(det[:4])
                scores.append(det[4])
                classes.append(int(det[5]))
        return boxes, scores, classes
    
    def release(self):
        self.rknn.release()
```

---

## 9. 参考资源

| 资源 | 链接 |
|------|------|
| RKNN Toolkit2 | https://github.com/airockchip/rknn-toolkit2 |
| RKNN Runtime | https://github.com/airockchip/rknn-toolkit2/tree/master/rknn/runtime |
| RKNN Model Zoo | https://github.com/airockchip/rknn_model_zoo |
| Rockchip Wiki | https://opensource.rock-chips.com/wiki_Rockchip |
| RK3588 Datasheet | 联系 Rockchip 获取 |
