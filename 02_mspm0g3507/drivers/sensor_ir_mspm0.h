/**
 * @file    sensor_ir_mspm0.h
 * @brief   红外循迹传感器驱动 — MSPM0G3507
 * @note    支持 5路/8路 红外传感器阵列 (模拟量或开关量)
 *          模拟量通过 ADC12 读取，开关量通过 GPIO 读取
 */

#ifndef __SENSOR_IR_MSPM0_H
#define __SENSOR_IR_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

#define IR_MAX_CHANNELS     8

/* ── 传感器类型 ──────────────────────────────────────────── */
typedef enum {
    IR_TYPE_DIGITAL = 0,   /* GPIO 开关量输出 */
    IR_TYPE_ANALOG  = 1    /* ADC 模拟量输出 */
} IRType;

/* ── 传感器配置 ──────────────────────────────────────────── */
typedef struct {
    IRType type;
    uint8_t num_sensors;

    /* 数字模式: GPIO 引脚 */
    GPIO_Regs *port[IR_MAX_CHANNELS];
    uint32_t   pin[IR_MAX_CHANNELS];

    /* 模拟模式: ADC 通道 (MEMx index) */
    ADC12_Regs *adc;
    uint8_t     adc_channels[IR_MAX_CHANNELS];

    /* 阈值 (模拟模式用于二值化) */
    uint16_t threshold;

    /* 是否反色 (黑底白线=0, 白底黑线=1) */
    uint8_t inverted;
} IRConfig;

/* ── API ──────────────────────────────────────────────────── */

/** 初始化红外传感器 */
void IR_Init(const IRConfig *cfg);

/**
 * @brief 读取传感器状态 (二值化)
 * @param buf   输出数组, 每元素 0 或 1
 * @param len   数组长度
 */
void IR_ReadDigital(uint8_t *buf, uint8_t len);

/**
 * @brief 读取模拟值
 * @param buf   输出数组 (12-bit ADC 值)
 * @param len   数组长度
 */
void IR_ReadAnalog(uint16_t *buf, uint8_t len);

/**
 * @brief 计算循迹偏差值
 * @return 有符号偏差, 0=居中, 负=偏左, 正=偏右
 */
int16_t IR_GetTrackError(void);

/**
 * @brief 判断是否全部脱离赛道 (全白或全黑)
 * @return 1=脱线, 0=在线上
 */
uint8_t IR_IsOffTrack(void);

#endif /* __SENSOR_IR_MSPM0_H */
