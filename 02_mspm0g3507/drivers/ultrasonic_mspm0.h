/**
 * @file    ultrasonic_mspm0.h
 * @brief   SR04/US-016 超声波传感器驱动 — MSPM0G3507
 * @note    适用于SR04和US-016超声波测距传感器
 *          使用GPIO触发和Echo中断检测，通过定时器计时
 *          测量范围：2cm ~ 400cm，精度：±3mm
 *
 * 接线示例:
 *   MSPM0 PA0 → Trig (触发引脚)
 *   MSPM0 PA1 → Echo (回波引脚)
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#ifndef __ULTRASONIC_MSPM0_H
#define __ULTRASONIC_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

/* ── 传感器类型 ──────────────────────────────────────────── */
typedef enum {
    ULTRASONIC_SR04 = 0,    /* SR04超声波传感器 */
    ULTRASONIC_US016 = 1    /* US-016超声波传感器 */
} UltrasonicType;

/* ── 配置结构 ──────────────────────────────────────────────── */
typedef struct {
    GPIO_Regs *port;        /* GPIO端口 */
    uint32_t   trig_pin;    /* Trig引脚 */
    uint32_t   echo_pin;    /* Echo引脚 */
    UltrasonicType type;    /* 传感器类型 */
    uint8_t    filter_size; /* 滤波窗口大小 (默认5) */
} UltrasonicConfig;

/* ── 数据结构 ──────────────────────────────────────────────── */
typedef struct {
    float distance;         /* 测量距离 (cm) */
    float filtered;         /* 滤波后的距离 (cm) */
    uint8_t valid;          /* 数据有效标志 */
    uint32_t measure_count; /* 测量计数 */
} UltrasonicData;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化超声波传感器
 * @param cfg  指向UltrasonicConfig结构
 */
void Ultrasonic_Init(const UltrasonicConfig *cfg);

/**
 * @brief 测量距离 (阻塞式)
 * @return 距离 (cm)，0表示测量失败
 */
float Ultrasonic_Measure(void);

/**
 * @brief 获取最近一次测量结果
 * @return 距离 (cm)
 */
float Ultrasonic_GetDistance(void);

/**
 * @brief 获取滤波后的距离
 * @return 滤波后的距离 (cm)
 */
float Ultrasonic_GetFilteredDistance(void);

/**
 * @brief 检查数据是否有效
 * @return 1=有效, 0=无效
 */
uint8_t Ultrasonic_IsValid(void);

/**
 * @brief 获取测量数据结构
 * @return 指向UltrasonicData结构的指针
 */
UltrasonicData* Ultrasonic_GetData(void);

#endif /* __ULTRASONIC_MSPM0_H */