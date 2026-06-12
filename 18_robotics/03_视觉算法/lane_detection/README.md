# 车道线检测算法使用文档

## 1. 算法原理

车道线检测是自动驾驶和智能车竞赛中的核心视觉算法，用于识别道路上的车道线并计算车辆相对于车道中心的偏移量。

### 1.1 处理流程

```
原始图像
    ↓
灰度转换
    ↓
图像二值化
    ↓
边缘检测（Sobel）
    ↓
车道线点查找
    ↓
直线拟合（最小二乘法）
    ↓
偏移量计算
```

### 1.2 关键算法

**Sobel边缘检测：**
```
Gx = [[-1,0,1], [-2,0,2], [-1,0,1]] * Image
Gy = [[-1,-2,-1], [0,0,0], [1,2,1]] * Image
Magnitude = |Gx| + |Gy|
```

**最小二乘法直线拟合：**
```
给定点集 {(x1,y1), (x2,y2), ..., (xn,yn)}
求解 y = kx + b 使得 Σ(yi - kxi - b)² 最小

k = (nΣxiyi - ΣxiΣyi) / (nΣxi² - (Σxi)²)
b = (Σyi - kΣxi) / n
```

## 2. 硬件配置

### 2.1 摄像头选择
- **OV7670**: 30万像素，适合简单场景
- **OV2640**: 200万像素，适合复杂场景
- **MT9V034**: 全局快门，适合高速场景

### 2.2 安装位置
- 安装在车辆前方
- 向下倾斜15-30度
- 高度：30-50cm

### 2.3 图像参数
- 分辨率：320×240 或 640×480
- 帧率：30-60fps
- 格式：灰度图或YUV

## 3. 参数配置

### 3.1 二值化阈值
```c
hdetector.threshold = 128;  // 默认值
```
- 根据环境光线调整
- 光线强时增大阈值
- 光线弱时减小阈值

### 3.2 感兴趣区域（ROI）
```c
#define ROI_TOP     120     // 从图像中间开始
#define ROI_BOTTOM  240     // 到图像底部
```
- 只处理ROI区域可以提高处理速度
- 根据摄像头安装角度调整

### 3.3 边缘检测阈值
```c
if (hdetector->edge[y][x] > 100) {
    // 认为是边缘点
}
```
- 根据图像质量调整
- 噪声大时增大阈值

## 4. 使用示例

### 4.1 基本使用
```c
#include "lane_detection.h"

LaneDetector_HandleTypeDef hdetector;

void main()
{
    // 初始化检测器
    LaneDetector_Init(&hdetector);
    
    // 设置参数
    hdetector.threshold = 128;
    
    while (1) {
        // 采集图像
        uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH];
        Camera_Capture(image);
        
        // 设置图像
        LaneDetector_SetImage(&hdetector, (uint8_t*)image);
        
        // 处理图像
        LaneDetector_Binarize(&hdetector);
        LaneDetector_EdgeDetect(&hdetector);
        LaneDetector_FindLaneLines(&hdetector);
        
        // 获取结果
        LaneDetectionResult *result = LaneDetector_GetResult(&hdetector);
        
        if (result->valid) {
            // 使用偏移量控制车辆
            float steering = result->offset * 0.1f;  // 比例控制
            Motor_SetSteering(steering);
        }
    }
}
```

### 4.2 PID控制
```c
PositionPID_HandleTypeDef steering_pid;

void main()
{
    // 初始化PID
    PositionPID_Init(&steering_pid, 0.5f, 0.01f, 0.1f);
    
    while (1) {
        // 检测车道线
        LaneDetectionResult *result = LaneDetector_GetResult(&hdetector);
        
        if (result->valid) {
            // PID控制转向
            float steering = PositionPID_Compute(&steering_pid, 0, result->offset);
            Motor_SetSteering(steering);
        }
    }
}
```

## 5. 常见问题

### 5.1 检测不到车道线
- 检查摄像头是否正常工作
- 调整二值化阈值
- 检查ROI设置是否正确
- 确保车道线清晰可见

### 5.2 检测不稳定
- 增加滤波处理
- 使用卡尔曼滤波器平滑结果
- 增加历史帧的权重

### 5.3 处理速度慢
- 减小图像分辨率
- 缩小ROI区域
- 使用DMA传输图像
- 优化算法实现

## 6. 性能优化

### 6.1 自适应阈值
```c
uint8_t AdaptiveThreshold(uint8_t *image, uint16_t x, uint16_t y)
{
    // 计算局部平均值
    uint32_t sum = 0;
    uint16_t count = 0;
    
    for (int16_t ky = -5; ky <= 5; ky++) {
        for (int16_t kx = -5; kx <= 5; kx++) {
            if (y+ky >= 0 && y+ky < IMAGE_HEIGHT && x+kx >= 0 && x+kx < IMAGE_WIDTH) {
                sum += image[(y+ky)*IMAGE_WIDTH + (x+kx)];
                count++;
            }
        }
    }
    
    return (uint8_t)(sum / count) - 10;  // 减去偏移量
}
```

### 6.2 图像滤波
```c
void GaussianFilter(uint8_t *input, uint8_t *output, uint16_t width, uint16_t height)
{
    const int8_t kernel[3][3] = {
        {1, 2, 1},
        {2, 4, 2},
        {1, 2, 1}
    };
    
    for (uint16_t y = 1; y < height - 1; y++) {
        for (uint16_t x = 1; x < width - 1; x++) {
            int32_t sum = 0;
            for (int8_t ky = -1; ky <= 1; ky++) {
                for (int8_t kx = -1; kx <= 1; kx++) {
                    sum += input[(y+ky)*width + (x+kx)] * kernel[ky+1][kx+1];
                }
            }
            output[y*width + x] = (uint8_t)(sum / 16);
        }
    }
}
```

### 6.3 路径预测
```c
float PredictOffset(LaneDetectionResult *history, uint8_t history_count)
{
    // 使用历史数据预测未来偏移量
    float predicted = 0.0f;
    float weight_sum = 0.0f;
    
    for (uint8_t i = 0; i < history_count; i++) {
        float weight = (float)(i + 1);
        predicted += history[i].offset * weight;
        weight_sum += weight;
    }
    
    return predicted / weight_sum;
}
```

## 7. 扩展功能

### 7.1 弯道检测
```c
bool DetectCurve(LaneDetectionResult *result)
{
    // 检查车道线斜率变化
    if (fabsf(result->left_lane.slope) > 0.5f || fabsf(result->right_lane.slope) > 0.5f) {
        return true;  // 检测到弯道
    }
    return false;
}
```

### 7.2 车道线丢失处理
```c
void HandleLaneLost(LaneDetectionResult *result, LaneDetectionResult *history)
{
    if (!result->valid) {
        // 使用历史数据
        result->offset = history->offset;
        result->angle = history->angle;
        
        // 降低控制增益
        steering_pid.kp *= 0.5f;
    } else {
        // 恢复正常增益
        steering_pid.kp = original_kp;
    }
}
```
