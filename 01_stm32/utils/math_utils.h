/**
 * @file    math_utils.h
 * @brief   数学工具模块 — STM32电赛通用代码库
 * @details 限幅、映射、死区、滑动平均、一阶低通滤波。
 *          所有函数均有float和int32两个版本。
 * @author  电赛通用代码库
 * @version 1.0
 * @date    2026-06
 */

#ifndef __MATH_UTILS_H
#define __MATH_UTILS_H

#include "platform/hal_stm32.h"

/* ========================================================================== */
/*                              常量定义                                       */
/* ========================================================================== */

/** @brief 滑动平均滤波器最大窗口大小 */
#define MOVING_AVG_MAX_SIZE  64

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/**
 * @brief 滑动平均滤波器结构体
 */
typedef struct {
    float   buffer[MOVING_AVG_MAX_SIZE]; /**< 数据缓冲区 */
    uint8_t size;       /**< 窗口大小 */
    uint8_t index;      /**< 当前写入索引 */
    uint8_t count;      /**< 已存入数据个数(用于初始化阶段) */
    float   sum;        /**< 窗口内数据之和 */
} MovingAvg_t;

/**
 * @brief 一阶低通滤波器结构体
 * @note  filtered = alpha * new_value + (1 - alpha) * filtered_prev
 *        alpha越大跟踪越快，越小滤波越强
 */
typedef struct {
    float alpha;        /**< 滤波系数 0~1 */
    float filtered;     /**< 当前滤波输出 */
    bool  initialized;  /**< 是否已初始化(首次调用时直接赋值) */
} LowPassFilter_t;

/* ========================================================================== */
/*                              独立函数接口                                   */
/* ========================================================================== */

/**
 * @brief 浮点数限幅
 * @param value  输入值
 * @param min    下限
 * @param max    上限
 * @return float: 限幅后的值
 */
float Math_ClampF(float value, float min, float max);

/**
 * @brief 整数限幅
 * @param value  输入值
 * @param min    下限
 * @param max    上限
 * @return int32_t: 限幅后的值
 */
int32_t Math_ClampI(int32_t value, int32_t min, int32_t max);

/**
 * @brief 浮点数映射（线性插值）
 * @param value    输入值
 * @param in_min   输入范围下限
 * @param in_max   输入范围上限
 * @param out_min  输出范围下限
 * @param out_max  输出范围上限
 * @return float: 映射后的值
 * @note   不做限幅，超出范围会外推
 */
float Math_MapF(float value, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief 整数映射
 */
int32_t Math_MapI(int32_t value, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

/**
 * @brief 浮点数死区处理
 * @param value      输入值
 * @param dead_zone  死区大小(正值)
 * @return float: |value| < dead_zone 时返回0，否则返回value
 * @note   适用于对称死区
 */
float Math_DeadZoneF(float value, float dead_zone);

/**
 * @brief 整数死区处理
 */
int32_t Math_DeadZoneI(int32_t value, int32_t dead_zone);

/**
 * @brief 符号保持死区处理（非对称死区）
 * @param value      输入值
 * @param dead_zone  死区大小
 * @return float: 在死区内返回0，否则返回 value - sign(value)*dead_zone
 * @note   用于去除死区偏移，例如电机死区补偿
 */
float Math_DeadZoneCompensateF(float value, float dead_zone);

/* ========================================================================== */
/*                              滑动平均滤波器                                 */
/* ========================================================================== */

/**
 * @brief 初始化滑动平均滤波器
 * @param avg   滑动平均结构体指针
 * @param size  窗口大小(1~MOVING_AVG_MAX_SIZE)
 * @return ErrorCode_t
 */
ErrorCode_t MovingAvg_Init(MovingAvg_t *avg, uint8_t size);

/**
 * @brief 向滑动平均滤波器输入新数据并获取滤波结果
 * @param avg    滑动平均结构体指针
 * @param value  新数据
 * @return float: 滑动平均值
 */
float MovingAvg_Update(MovingAvg_t *avg, float value);

/**
 * @brief 获取当前滑动平均值
 * @param avg  滑动平均结构体指针
 * @return float: 当前平均值
 */
float MovingAvg_GetValue(const MovingAvg_t *avg);

/**
 * @brief 重置滑动平均滤波器
 * @param avg  滑动平均结构体指针
 * @return ErrorCode_t
 */
ErrorCode_t MovingAvg_Reset(MovingAvg_t *avg);

/* ========================================================================== */
/*                              一阶低通滤波器                                 */
/* ========================================================================== */

/**
 * @brief 初始化一阶低通滤波器
 * @param lpf    低通滤波器结构体指针
 * @param alpha  滤波系数 0~1
 *               alpha=0.1 → 强滤波(慢响应)
 *               alpha=0.5 → 中等滤波
 *               alpha=0.9 → 弱滤波(快响应)
 * @return ErrorCode_t
 */
ErrorCode_t LowPassFilter_Init(LowPassFilter_t *lpf, float alpha);

/**
 * @brief 输入新数据并获取滤波结果
 * @param lpf    低通滤波器结构体指针
 * @param value  新数据
 * @return float: 滤波后的值
 * @note   首次调用时直接返回输入值（初始化内部状态）
 */
float LowPassFilter_Update(LowPassFilter_t *lpf, float value);

/**
 * @brief 获取当前滤波输出
 * @param lpf  低通滤波器结构体指针
 * @return float: 当前滤波值
 */
float LowPassFilter_GetValue(const LowPassFilter_t *lpf);

/**
 * @brief 重置滤波器
 * @param lpf  低通滤波器结构体指针
 * @return ErrorCode_t
 */
ErrorCode_t LowPassFilter_Reset(LowPassFilter_t *lpf);

#endif /* __MATH_UTILS_H */
