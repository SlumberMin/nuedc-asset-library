/**
 * @file    pca9685_stm32.h
 * @brief   PCA9685 16路PWM舵机驱动板 — STM32 HAL库版本（I2C）
 * @details 通过I2C控制PCA9685，输出16路PWM信号驱动舵机/LED。
 * @version 1.0
 * @date    2026-06
 */

#ifndef __PCA9685_STM32_H
#define __PCA9685_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── PCA9685 I2C 地址 ──────────────────────────────────── */
#define PCA9685_ADDR_DEFAULT    (0x40)  /* A0~A5全接GND时的默认地址 */

/* ── PCA9685 寄存器地址 ────────────────────────────────── */
#define PCA9685_REG_MODE1       (0x00)
#define PCA9685_REG_MODE2       (0x01)
#define PCA9685_REG_LED0_ON_L   (0x06)
#define PCA9685_REG_LED0_ON_H   (0x07)
#define PCA9685_REG_LED0_OFF_L  (0x08)
#define PCA9685_REG_LED0_OFF_H  (0x09)
#define PCA9685_REG_PRESCALE    (0xFE)

/* ── MODE1 寄存器位 ─────────────────────────────────────── */
#define PCA9685_MODE1_RESTART   (0x80)
#define PCA9685_MODE1_EXTCLK    (0x40)
#define PCA9685_MODE1_AI        (0x20)  /* 自动递增 */
#define PCA9685_MODE1_SLEEP     (0x10)

/* ── PWM 参数 ───────────────────────────────────────────── */
#define PCA9685_PWM_STEPS       (4096)  /* 12位分辨率 */
#define PCA9685_OSC_FREQ        (25000000U)  /* 内部振荡器频率 25MHz */

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化PCA9685
 * @param hi2c  I2C句柄指针
 * @param addr  I2C 7位地址（默认0x40）
 * @return true=成功
 */
bool PCA9685_Init(I2C_HandleTypeDef *hi2c, uint8_t addr);

/**
 * @brief 设置PWM频率（24Hz ~ 1526Hz）
 * @param freq_hz 频率Hz
 * @return true=成功
 */
bool PCA9685_SetFreq(uint16_t freq_hz);

/**
 * @brief 设置指定通道的PWM占空比
 * @param channel 通道号 0~15
 * @param on      开启时刻 0~4095
 * @param off     关闭时刻 0~4095
 * @return true=成功
 */
bool PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off);

/**
 * @brief 设置指定通道的脉宽（舵机角度控制便捷接口）
 * @param channel    通道号 0~15
 * @param pulse_us   脉宽（微秒），舵机典型值: 500~2500us
 * @return true=成功
 */
bool PCA9685_SetServoPulse(uint8_t channel, uint16_t pulse_us);

/**
 * @brief 设置所有通道为相同PWM值
 * @param on   开启时刻
 * @param off  关闭时刻
 * @return true=成功
 */
bool PCA9685_SetAllPWM(uint16_t on, uint16_t off);

#endif /* __PCA9685_STM32_H */
