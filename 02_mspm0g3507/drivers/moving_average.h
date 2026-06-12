/**
 * @file    moving_average.h
 * @brief   滑动平均滤波器（多种类型）
 *
 * 参考: GitHub高星项目 dsp-filter / embedded-filters 优秀实现
 *
 * 提供三种滤波器:
 *   1. SimpleMA  - 简单滑动平均（等权）
 *   2. WeightedMA - 加权滑动平均（近期数据权重更大）
 *   3. ExponentialMA - 指数滑动平均（单参数，内存最小）
 *
 * 特点:
 *   - 纯C实现，无动态内存分配
 *   - 零拷贝，O(1)更新
 *   - 编译时确定窗口大小
 *   - 支持int16_t和float两种数据类型
 *
 * 典型应用:
 *   - 传感器噪声滤除（ADC、陀螺仪、超声波）
 *   - 信号平滑（循线偏差、PID输出）
 *   - 实时数据预处理
 */
#ifndef __MOVING_AVERAGE_H
#define __MOVING_AVERAGE_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 * 1. 简单滑动平均滤波器（等权平均）
 *
 * 原理: y[n] = (x[n] + x[n-1] + ... + x[n-W+1]) / W
 *   其中 W 为窗口大小
 *
 * 特点: 延迟 = (W-1)/2 个采样周期
 * ================================================================ */

/** 简单滑动平均滤波器（float版本） */
typedef struct {
    float    *buffer;      /**< 数据缓冲区 */
    uint16_t  size;        /**< 窗口大小 */
    uint16_t  index;       /**< 当前写入索引 */
    uint16_t  count;       /**< 已填充的数据量 */
    float     sum;         /**< 窗口内数据总和 */
    float     last_output; /**< 上次输出值 */
} SimpleMA_t;

/**
 * @brief 初始化简单滑动平均滤波器
 * @param fma     滤波器控制块
 * @param buf     用户提供的缓冲区（大小 >= window_size）
 * @param window  窗口大小
 */
void SimpleMA_Init(SimpleMA_t *fma, float *buf, uint16_t window);

/**
 * @brief 输入新数据并获取滤波输出
 * @param fma  滤波器控制块
 * @param input 新的采样值
 * @return 滤波后的输出值
 *
 * 窗口未填满时返回累积平均值
 */
float SimpleMA_Update(SimpleMA_t *fma, float input);

/**
 * @brief 获取当前滤波输出（不更新）
 */
static inline float SimpleMA_GetOutput(const SimpleMA_t *fma)
{
    return fma->last_output;
}

/**
 * @brief 重置滤波器
 */
void SimpleMA_Reset(SimpleMA_t *fma);

/**
 * @brief 检查窗口是否已满
 */
static inline bool SimpleMA_IsReady(const SimpleMA_t *fma)
{
    return fma->count >= fma->size;
}


/* ================================================================
 * 2. 指数滑动平均滤波器（EMA）
 *
 * 原理: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *   其中 alpha ∈ (0, 1]
 *   alpha 越大，跟踪越快但滤波越弱
 *   alpha 越小，滤波越强但延迟越大
 *
 * 特点:
 *   - 只需1个float存储，内存最小
 *   - 无窗口缓冲区
 *   - 适合实时嵌入式场景
 * ================================================================ */

/** 指数滑动平均滤波器 */
typedef struct {
    float alpha;         /**< 平滑系数 (0, 1] */
    float one_minus_alpha; /**< 1 - alpha（预计算，减少浮点除法） */
    float output;        /**< 当前输出值 */
    bool  initialized;   /**< 首次输入标志 */
} EMA_t;

/**
 * @brief 初始化指数滑动平均滤波器
 * @param ema   滤波器控制块
 * @param alpha 平滑系数 (0, 1]
 *              0.1 = 强滤波（延迟大）
 *              0.5 = 中等滤波
 *              0.9 = 弱滤波（跟踪快）
 */
static inline void EMA_Init(EMA_t *ema, float alpha)
{
    ema->alpha = alpha;
    ema->one_minus_alpha = 1.0f - alpha;
    ema->output = 0.0f;
    ema->initialized = false;
}

/**
 * @brief 输入新数据并获取滤波输出
 * @param ema   滤波器控制块
 * @param input 新的采样值
 * @return 滤波后的输出值
 *
 * 首次调用时直接返回输入值（无延迟启动）
 */
static inline float EMA_Update(EMA_t *ema, float input)
{
    if (!ema->initialized) {
        ema->output = input;
        ema->initialized = true;
    } else {
        ema->output = ema->alpha * input + ema->one_minus_alpha * ema->output;
    }
    return ema->output;
}

/**
 * @brief 获取当前滤波输出（不更新）
 */
static inline float EMA_GetOutput(const EMA_t *ema)
{
    return ema->output;
}

/**
 * @brief 重置滤波器
 */
static inline void EMA_Reset(EMA_t *ema)
{
    ema->output = 0.0f;
    ema->initialized = false;
}


/* ================================================================
 * 3. 整数滑动平均滤波器（定点版，无浮点运算）
 *
 * 原理与 SimpleMA 相同，但使用int16_t定点运算
 * 适用于: 无FPU的MCU或对速度要求极高的场景
 *
 * 注意: 窗口大小必须为 2 的幂（用移位代替除法）
 * ================================================================ */

/** 整数滑动平均滤波器 */
typedef struct {
    int16_t  *buffer;     /**< 数据缓冲区 */
    uint16_t  size;       /**< 窗口大小（必须为2的幂） */
    uint16_t  shift;      /**< 移位数 = log2(size) */
    uint16_t  index;      /**< 当前写入索引 */
    uint16_t  count;      /**< 已填充的数据量 */
    int32_t   sum;        /**< 窗口内数据总和（用int32防溢出） */
    int16_t   last_output;/**< 上次输出值 */
} IntMA_t;

/**
 * @brief 初始化整数滑动平均滤波器
 * @param ima  滤波器控制块
 * @param buf  用户提供的缓冲区（大小 >= window_size）
 * @param window 窗口大小（必须为2的幂: 2, 4, 8, 16, 32, 64, 128, 256）
 */
void IntMA_Init(IntMA_t *ima, int16_t *buf, uint16_t window);

/**
 * @brief 输入新数据并获取滤波输出
 * @param ima   滤波器控制块
 * @param input 新的采样值
 * @return 滤波后的输出值
 */
int16_t IntMA_Update(IntMA_t *ima, int16_t input);

/**
 * @brief 获取当前滤波输出
 */
static inline int16_t IntMA_GetOutput(const IntMA_t *ima)
{
    return ima->last_output;
}

/**
 * @brief 重置滤波器
 */
void IntMA_Reset(IntMA_t *ima);

#endif /* __MOVING_AVERAGE_H */
