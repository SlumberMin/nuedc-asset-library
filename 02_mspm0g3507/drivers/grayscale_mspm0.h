/**
 * @file    grayscale_mspm0.h
 * @brief   感为8路灰度传感器驱动 — MSPM0G3507
 * @note    适用于感为无MCU版8路灰度循迹传感器
 *          使用ADC读取模拟值，通过地址线切换通道
 *          支持数字量、模拟量、归一化值输出
 *
 * 接线示例:
 *   MSPM0 PB0 → AD0 (ADC输入)
 *   MSPM0 PB1 → AD1 (ADC输入)
 *   MSPM0 PB2 → AD2 (ADC输入)
 *   MSPM0 PA27 → OUT (数字输出)
 *   MSPM0 PA0 → 地址位0
 *   MSPM0 PA1 → 地址位1
 *   MSPM0 PA2 → 地址位2
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#ifndef __GRAYSCALE_MSPM0_H
#define __GRAYSCALE_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

/* ── 传感器数量 ──────────────────────────────────────────── */
#define GRAYSENSOR_NUM  8   /* 8路灰度传感器 */

/* ── 数据结构 ──────────────────────────────────────────────── */
typedef struct {
    uint16_t analog[GRAYSENSOR_NUM];     /* 原始模拟值 */
    uint16_t normalized[GRAYSENSOR_NUM]; /* 归一化值 */
    uint8_t  digital;                    /* 数字量 (8位，每位对应一个传感器) */
    uint16_t white_cal[GRAYSENSOR_NUM];  /* 白校准值 */
    uint16_t black_cal[GRAYSENSOR_NUM];  /* 黑校准值 */
    uint16_t threshold_high[GRAYSENSOR_NUM]; /* 高阈值 */
    uint16_t threshold_low[GRAYSENSOR_NUM];  /* 低阈值 */
    float    normal_factor[GRAYSENSOR_NUM];  /* 归一化系数 */
    uint8_t  initialized;                /* 初始化标志 */
} GrayscaleData;

/* ── 配置结构 ──────────────────────────────────────────────── */
typedef struct {
    GPIO_Regs *addr_port;     /* 地址线端口 */
    uint32_t   addr_pin_0;    /* 地址位0引脚 */
    uint32_t   addr_pin_1;    /* 地址位1引脚 */
    uint32_t   addr_pin_2;    /* 地址位2引脚 */
    ADC_Regs   *adc;          /* ADC实例 */
    uint32_t   adc_channel;   /* ADC通道 */
    uint8_t    direction;     /* 方向 (0=正序, 1=反序) */
} GrayscaleConfig;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化灰度传感器
 * @param cfg  指向GrayscaleConfig结构
 */
void Grayscale_Init(const GrayscaleConfig *cfg);

/**
 * @brief 校准传感器 (白板和黑板)
 * @param white  白板校准值数组 (长度8)
 * @param black  黑板校准值数组 (长度8)
 */
void Grayscale_Calibrate(uint16_t *white, uint16_t *black);

/**
 * @brief 读取传感器数据 (需在主循环中调用)
 */
void Grayscale_Read(void);

/**
 * @brief 获取数字量
 * @return 8位数字量，每位对应一个传感器 (1=白, 0=黑)
 */
uint8_t Grayscale_GetDigital(void);

/**
 * @brief 获取模拟值
 * @param buf  存储8路模拟值的数组
 */
void Grayscale_GetAnalog(uint16_t *buf);

/**
 * @brief 获取归一化值
 * @param buf  存储8路归一化值的数组
 */
void Grayscale_GetNormalized(uint16_t *buf);

/**
 * @brief 计算循迹偏差
 * @return 偏差值 (-1000 ~ +1000，0表示居中，负值偏左，正值偏右)
 */
int16_t Grayscale_GetTrackError(void);

/**
 * @brief 判断是否脱线
 * @return 1=脱线, 0=在线上
 */
uint8_t Grayscale_IsOffTrack(void);

/**
 * @brief 检测十字路口
 * @return 1=检测到十字路口, 0=未检测到
 */
uint8_t Grayscale_DetectCross(void);

/**
 * @brief 检测丁字路口
 * @return 1=检测到丁字路口, 0=未检测到
 */
uint8_t Grayscale_DetectTJunction(void);

/**
 * @brief 检测起跑线
 * @return 1=检测到起跑线, 0=未检测到
 */
uint8_t Grayscale_DetectStartLine(void);

#endif /* __GRAYSCALE_MSPM0_H */