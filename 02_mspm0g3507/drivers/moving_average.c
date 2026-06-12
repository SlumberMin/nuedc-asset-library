/**
 * @file    moving_average.c
 * @brief   滑动平均滤波器实现
 *
 * 三种滤波器的适用场景对比:
 *
 * | 滤波器    | 内存占用 | 计算量 | 延迟   | 适用场景           |
 * |-----------|----------|--------|--------|--------------------|
 * | SimpleMA  | 4*W+20B  | O(1)   | (W-1)/2| 通用传感器滤波     |
 * | EMA       | 16B      | O(1)   | ≈1/α  | 实时信号跟踪       |
 * | IntMA     | 2*W+14B  | O(1)   | (W-1)/2| 无FPU的快速滤波    |
 *
 * 其中 W = 窗口大小
 */
#include "drivers/moving_average.h"

/* ================================================================
 * 1. 简单滑动平均滤波器（float版本）
 *
 * 算法: 使用累加和技巧，避免每次重新遍历缓冲区
 *   sum += new_sample - oldest_sample
 *   output = sum / window_size
 * ================================================================ */

void SimpleMA_Init(SimpleMA_t *fma, float *buf, uint16_t window)
{
    fma->buffer = buf;
    fma->size   = window;
    fma->index  = 0;
    fma->count  = 0;
    fma->sum    = 0.0f;
    fma->last_output = 0.0f;

    /* 清零缓冲区 */
    for (uint16_t i = 0; i < window; i++) {
        buf[i] = 0.0f;
    }
}

float SimpleMA_Update(SimpleMA_t *fma, float input)
{
    /* 如果缓冲区已满，减去最旧的数据 */
    if (fma->count >= fma->size) {
        fma->sum -= fma->buffer[fma->index];
    }

    /* 写入新数据 */
    fma->buffer[fma->index] = input;
    fma->sum += input;

    /* 更新索引（环形） */
    fma->index++;
    if (fma->index >= fma->size) {
        fma->index = 0;
    }

    /* 更新计数 */
    if (fma->count < fma->size) {
        fma->count++;
    }

    /* 计算平均值 */
    fma->last_output = fma->sum / (float)fma->count;
    return fma->last_output;
}

void SimpleMA_Reset(SimpleMA_t *fma)
{
    fma->index = 0;
    fma->count = 0;
    fma->sum   = 0.0f;
    fma->last_output = 0.0f;

    for (uint16_t i = 0; i < fma->size; i++) {
        fma->buffer[i] = 0.0f;
    }
}


/* ================================================================
 * 2. 指数滑动平均滤波器（EMA）
 *
 * 实现已在头文件中以inline函数完成（减少函数调用开销）
 * 此文件仅提供编译单元，确保链接正常
 * ================================================================ */

/* EMA的所有函数都是inline的，定义在头文件中
 * 这里无需额外实现 */


/* ================================================================
 * 3. 整数滑动平均滤波器（定点版）
 *
 * 使用移位代替除法，窗口大小必须为2的幂
 * ================================================================ */

/* 辅助函数: 计算log2（用于确定移位数） */
static uint16_t log2_uint(uint16_t n)
{
    uint16_t shift = 0;
    while (n > 1) {
        n >>= 1;
        shift++;
    }
    return shift;
}

/* 辅助函数: 判断是否为2的幂 */
static bool is_power_of_two_u16(uint16_t n)
{
    return (n != 0) && ((n & (n - 1)) == 0);
}

void IntMA_Init(IntMA_t *ima, int16_t *buf, uint16_t window)
{
    /* 窗口大小必须为2的幂 */
    if (!is_power_of_two_u16(window) || buf == (void *)0 || window == 0) {
        /* 错误: 设置无效状态 */
        ima->buffer = (void *)0;
        ima->size   = 0;
        ima->shift  = 0;
        ima->index  = 0;
        ima->count  = 0;
        ima->sum    = 0;
        ima->last_output = 0;
        return;
    }

    ima->buffer = buf;
    ima->size   = window;
    ima->shift  = log2_uint(window);
    ima->index  = 0;
    ima->count  = 0;
    ima->sum    = 0;
    ima->last_output = 0;

    /* 清零缓冲区 */
    for (uint16_t i = 0; i < window; i++) {
        buf[i] = 0;
    }
}

int16_t IntMA_Update(IntMA_t *ima, int16_t input)
{
    /* 如果缓冲区已满，减去最旧的数据 */
    if (ima->count >= ima->size) {
        ima->sum -= ima->buffer[ima->index];
    }

    /* 写入新数据 */
    ima->buffer[ima->index] = input;
    ima->sum += input;  /* sum已是int32_t(头文件定义)，无需额外保护 */

    /* 更新索引（环形，用掩码取模） */
    ima->index = (ima->index + 1) & (ima->size - 1);

    /* 更新计数 */
    if (ima->count < ima->size) {
        ima->count++;
    }

    /* 计算平均值（用移位代替除法） */
    ima->last_output = (int16_t)(ima->sum >> ima->shift);
    return ima->last_output;
}

void IntMA_Reset(IntMA_t *ima)
{
    ima->index = 0;
    ima->count = 0;
    ima->sum   = 0;
    ima->last_output = 0;

    if (ima->buffer) {
        for (uint16_t i = 0; i < ima->size; i++) {
            ima->buffer[i] = 0;
        }
    }
}
