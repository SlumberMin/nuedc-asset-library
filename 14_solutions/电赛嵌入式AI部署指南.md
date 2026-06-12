# 电赛嵌入式AI部署指南

> 适用场景：电赛中需要在MCU/嵌入式平台上部署神经网络模型（图像分类、目标检测、语音识别等）

## 一、嵌入式AI技术栈总览

```
训练端                          部署端
┌──────────┐    转换    ┌──────────────────┐
│ PyTorch  │──────────→│ ONNX格式          │
│ TensorFlow│──────────→│ TFLite格式        │
│ Keras    │──────────→│ 自定义格式         │
└──────────┘           └────────┬─────────┘
                                │ 量化+优化
                         ┌──────┴──────┐
                         │ 部署运行时    │
                         ├─────────────┤
                         │ TFLite Micro │
                         │ ONNX Runtime │
                         │ NPU专用SDK   │
                         │ CMSIS-NN     │
                         │ 裸机推理     │
                         └─────────────┘
```

---

## 二、TensorFlow Lite Micro

### 2.1 模型转换流程

```python
# Step 1: 训练模型（TensorFlow/Keras）
import tensorflow as tf

model = tf.keras.Sequential([
    tf.keras.layers.Conv2D(16, 3, activation='relu', input_shape=(96,96,1)),
    tf.keras.layers.MaxPooling2D(),
    tf.keras.layers.Conv2D(32, 3, activation='relu'),
    tf.keras.layers.MaxPooling2D(),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(10, activation='softmax')
])

# Step 2: 量化转换（关键！）
converter = tf.lite.TFLiteConverter.from_keras_model(model)

# 训练后量化（PTQ）- 最简单
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_data_gen  # 校准数据
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8

tflite_model = converter.convert()
with open('model_int8.tflite', 'wb') as f:
    f.write(tflite_model)
```

### 2.2 TFLite Micro集成（STM32）

```c
// tflm_integration.c
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "model_data.h"  // 转换后的模型数组

// 分配Tensor Arena（在SRAM中）
constexpr int kTensorArenaSize = 64 * 1024;  // 64KB
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];

// 注册算子
tflite::MicroMutableOpResolver<10> resolver;
resolver.AddConv2D();
resolver.AddMaxPool2D();
resolver.AddReshape();
resolver.AddFullyConnected();
resolver.AddSoftmax();

// 创建解释器
const tflite::Model* model = tflite::GetModel(g_model_data);
tflite::MicroInterpreter interpreter(
    model, resolver, tensor_arena, kTensorArenaSize);
interpreter.AllocateTensors();

// 推理
int8_t* input = interpreter.input(0)->data.int8;
memcpy(input, sensor_data, input_size);
interpreter.Invoke();
int8_t* output = interpreter.output(0)->data.int8;

// 反量化结果
float scale = interpreter.output(0)->params.scale;
int zero_point = interpreter.output(0)->params.zero_point;
float result = (output[0] - zero_point) * scale;
```

### 2.3 TFLite Micro性能参考

| MCU | RAM | Flash | 模型大小 | 推理时间 |
|-----|-----|-------|---------|---------|
| STM32F407 | 192KB | 1MB | ~50KB INT8 | ~50ms |
| STM32F746 | 320KB | 1MB | ~100KB INT8 | ~30ms |
| STM32H743 | 1MB | 2MB | ~200KB INT8 | ~15ms |
| ESP32-S3 | 512KB | - | ~100KB INT8 | ~20ms |

---

## 三、ONNX Runtime部署

### 3.1 PyTorch → ONNX 导出

```python
import torch
import onnx

# 训练好的PyTorch模型
model.eval()
dummy_input = torch.randn(1, 1, 96, 96)

# 导出ONNX
torch.onnx.export(
    model, dummy_input, "model.onnx",
    input_names=['input'], output_names=['output'],
    dynamic_axes={'input': {0: 'batch'}, 'output': {0: 'batch'}},
    opset_version=13
)

# 验证
onnx_model = onnx.load("model.onnx")
onnx.checker.check_model(onnx_model)
```

### 3.2 ONNX量化

```python
from onnxruntime.quantization import quantize_dynamic, QuantType

# 动态量化（简单快速）
quantize_dynamic(
    "model.onnx",
    "model_int8.onnx",
    weight_type=QuantType.QUInt8
)

# 静态量化（更精确，需要校准数据）
from onnxruntime.quantization import quantize_static, CalibrationDataReader

class MyCalibrationDataReader(CalibrationDataReader):
    def __init__(self, data):
        self.data = data
        self.idx = 0
    def get_next(self):
        if self.idx >= len(self.data):
            return None
        result = {"input": self.data[self.idx]}
        self.idx += 1
        return result

quantize_static(
    "model.onnx", "model_int8_static.onnx",
    calibration_data_reader=MyCalibrationDataReader(cal_data)
)
```

### 3.3 ONNX Runtime Micro（嵌入式）

```c
// 某些平台支持onnxruntime-micro
// 或使用ONNX→TFLite二次转换

// 策略：ONNX用于桌面验证，TFLite Micro用于MCU部署
// 桥接方案：ONNX → ONNX简化 → TFLite → TFLite Micro
```

---

## 四、NPU加速部署

### 4.1 常见NPU平台

| NPU | 芯片 | 算力 | 支持框架 |
|-----|------|------|---------|
| NPU (ST) | STM32N6 | 0.6 TOPS | ST Edge AI |
| Ethos-U | Cortex-M85搭配 | 0.5 TOPS | Vela编译器 |
| KPU | K210/K230 | 0.8/2 TOPS | nncase |
| NNIE | Hi3516/Hi3519 | 2/4 TOPS | RKNPU |
| APU | 某些全志芯片 | 0.5 TOPS | ACE |

### 4.2 STM32 NPU部署流程

```
1. 模型训练（PyTorch/TF）
2. 导出ONNX
3. STM32Cube.AI转换：
   stedgeai generate --model model.onnx --type onnx --output stm32_output/
4. 集成到STM32工程：
   - 添加生成的C模型文件
   - 链接STM32 AI运行时库
5. 编程调用：
```

```c
// STM32 NPU推理
#include "network.h"  // Cube.AI生成

AI_ALIGNED(4) ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(4) ai_i8 input_data[AI_NETWORK_IN_1_SIZE];
AI_ALIGNED(4) ai_i8 output_data[AI_NETWORK_OUT_1_SIZE];

ai_handle network = AI_HANDLE_NULL;
ai_network_params params;

void AI_Init(void) {
    ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
    params.map_activations = activations;
    params.map_weights = AI_NETWORK_DATA_WEIGHTS;
    ai_network_init(network, &params);
}

int AI_Run(float *input, float *output) {
    // 量化输入
    for(int i=0; i<INPUT_SIZE; i++) {
        input_data[i] = (int8_t)(input[i] / input_scale + input_zp);
    }
    // 推理
    ai_i8 *in = ai_network_inputs_get(network, NULL)->data;
    ai_i8 *out = ai_network_outputs_get(network, NULL)->data;
    memcpy(in, input_data, INPUT_SIZE);
    ai_network_run(network, NULL);
    memcpy(output_data, out, OUTPUT_SIZE);
    // 反量化输出
    for(int i=0; i<OUTPUT_SIZE; i++) {
        output[i] = (output_data[i] - output_zp) * output_scale;
    }
    return 0;
}
```

### 4.3 K210 KPU部署

```python
# 使用nncase编译
import nncase

# 加载ONNX模型
compile_options = nncase.CompileOptions()
compile_options.target = "k210"
compile_options.input_range = [0, 1]
compile_options.input_type = "uint8"
compile_options.output_type = "float32"

compiler = nncase.Compiler(compile_options)
with open("model.onnx", "rb") as f:
    compiler.import_onnx(f.read())
compiler.compile()
kmodel = compiler.gencode_tobytes()

with open("model.kmodel", "wb") as f:
    f.write(kmodel)
```

```c
// K210 KPU推理（MaixPy/C SDK）
#include "kpu.h"

static kpu_model_context_t task;
volatile static uint8_t g_ai_done_flag;

static void ai_done(void *ctx) {
    g_ai_done_flag = 1;
}

void kpu_init(void) {
    kpu_load_kmodel(&task, model_data);
    kpu_model_set_output(&task, 0, output_buffer, sizeof(output_buffer));
}

void kpu_run(uint8_t *img) {
    g_ai_done_flag = 0;
    kpu_run_kmodel(&task, img, KPU_FORMAT_RGB888, ai_done, NULL);
    while(!g_ai_done_flag);  // 等待完成
}
```

---

## 五、模型优化技巧

### 5.1 轻量模型设计原则

```
电赛嵌入式AI设计准则：
├── 参数量 < 100K（MCU）/ < 1M（MPU）
├── FLOPs < 50M（MCU）/ < 500M（MPU）
├── 模型大小 < 200KB（MCU）/ < 5MB（MPU）
├── 输入分辨率尽量小（32×32 或 64×64）
├── 优先使用Depthwise Separable Conv
└── 避免过深网络（< 15层）
```

### 5.2 轻量网络架构

```python
# MobileNet风格的轻量网络
def create_light_model():
    return tf.keras.Sequential([
        # 标准卷积
        tf.keras.layers.Conv2D(8, 3, padding='same', activation='relu'),
        tf.keras.layers.BatchNormalization(),
        tf.keras.layers.MaxPooling2D(2),

        # Depthwise Separable Conv（参数量大幅减少）
        tf.keras.layers.SeparableConv2D(16, 3, padding='same', activation='relu'),
        tf.keras.layers.BatchNormalization(),
        tf.keras.layers.MaxPooling2D(2),

        tf.keras.layers.SeparableConv2D(32, 3, padding='same', activation='relu'),
        tf.keras.layers.GlobalAveragePooling2D(),

        tf.keras.layers.Dense(16, activation='relu'),
        tf.keras.layers.Dropout(0.3),
        tf.keras.layers.Dense(num_classes, activation='softmax')
    ])
```

### 5.3 量化感知训练（QAT）

```python
# 量化感知训练 - 比PTQ精度更高
import tensorflow_model_optimization as tfmot

quantize_model = tfmot.quantization.keras.quantize_model
q_model = quantize_model(model)

q_model.compile(optimizer='adam', loss='categorical_crossentropy')
q_model.fit(x_train, y_train, epochs=20, validation_split=0.1)

# 导出TFLite（已经是INT8）
converter = tf.lite.TFLiteConverter.from_keras_model(q_model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_model = converter.convert()
```

### 5.4 知识蒸馏

```python
# 大模型（Teacher）指导小模型（Student）
teacher_model = create_large_model()  # 精度高但大
student_model = create_light_model()  # 小但精度低

def distillation_loss(y_true, y_pred, teacher_pred, temperature=3, alpha=0.7):
    # 硬标签损失
    hard_loss = tf.keras.losses.categorical_crossentropy(y_true, y_pred)
    # 软标签损失
    soft_teacher = tf.nn.softmax(teacher_pred / temperature)
    soft_student = tf.nn.softmax(y_pred / temperature)
    soft_loss = tf.keras.losses.categorical_crossentropy(soft_teacher, soft_student)
    return alpha * soft_loss + (1 - alpha) * hard_loss
```

---

## 六、CMSIS-NN优化

### 6.1 什么是CMSIS-NN

```
ARM官方的神经网络库，针对Cortex-M优化：
- 支持INT8/INT16量化推理
- 利用SIMD指令加速卷积/矩阵运算
- 典型加速比：2-5x vs 未优化C代码
- 零额外内存开销
```

### 6.2 CMSIS-NN集成

```c
#include "arm_nnfunctions.h"

// INT8卷积
arm_convolve_wrapper_s8(
    &conv_params,       // 卷积参数
    &quant_params,      // 量化参数
    input_dims,         // 输入维度
    input_data,         // 输入数据 (int8)
    filter_dims,        // 卷积核维度
    filter_data,        // 权重 (int8)
    bias_dims,          // 偏置维度
    bias_data,          // 偏置 (int32)
    output_dims,        // 输出维度
    output_data         // 输出 (int8)
);

// INT8全连接
arm_fully_connected_s8(
    &fc_params, &quant_params,
    input_dims, input_data,
    filter_dims, filter_data,
    bias_dims, bias_data,
    output_dims, output_data
);

// ReLU
arm_relu_q7(buffer, size);
```

---

## 七、部署流水线总览

```
完整部署流程：

[数据采集] → [模型设计] → [训练] → [评估]
                                        ↓
                              [导出ONNX/TF SavedModel]
                                        ↓
                              [量化 (INT8/INT4)]
                                        ↓
                              [目标平台转换]
                              ├── TFLite Micro (MCU)
                              ├── ONNX Runtime (MPU)
                              ├── NPU SDK (专用加速)
                              └── CMSIS-NN (Cortex-M)
                                        ↓
                              [集成到嵌入式工程]
                                        ↓
                              [目标板验证]
                                        ↓
                              [性能优化] ←→ [精度验证]
```

---

## 八、电赛实战建议

### 8.1 推荐方案选择

| 场景 | 推荐方案 | 芯片参考 |
|------|---------|---------|
| 简单分类（≤10类） | TFLite Micro + STM32 | STM32F4/H7 |
| 目标检测 | NPU + K210 | K210/K230 |
| 图像分类（大量类） | ONNX + MPU | RK3568 |
| 语音唤醒 | TFLite Micro + ESP32 | ESP32-S3 |
| 手势识别 | NPU | STM32N6 |

### 8.2 常见踩坑

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 量化后精度暴跌 | 校准数据不具代表性 | 增加校准数据多样性 |
| Tensor Arena不够 | 模型太大或Arena太小 | 减小模型/增大Arena |
| 推理太慢 | 未使用硬件加速 | 启用CMSIS-NN/NPU |
| 内存溢出 | 模型+数据超出RAM | 用外部PSRAM/减小输入尺寸 |
| 输出不正确 | 量化参数错误 | 检查scale和zero_point |

### 8.3 快速原型验证流程

```
1. 先用Python在PC上验证模型精度
2. 用TFLite Benchmark测试模型大小和速度
3. 量化并验证精度损失是否可接受
4. 用STM32CubeMX生成基础工程
5. 集成TFLite Micro库
6. 加载模型并测试推理
7. 集成传感器输入
8. 实际环境测试和调优
```

---

*版本：v1.0 | 电赛嵌入式AI部署指南*
