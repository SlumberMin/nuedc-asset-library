/**
 * @file    pca9685.h
 * @brief   PCA9685 16路PWM舵机驱动板 I2C驱动 — MSPM0G3507
 *
 * 硬件连接:
 *   I2C0: PB2=SCL, PB3=SDA
 *   PCA9685 默认I2C地址: 0x40
 *
 * PCA9685寄存器:
 *   MODE1      (0x00) — 模式寄存器1
 *   MODE2      (0x01) — 模式寄存器2
 *   PRE_SCALE  (0xFE) — PWM频率预分频
 *   LED0_ON_L  (0x06) — 通道0 ON低字节 (每通道4字节, 共16通道)
 *
 * PWM频率:
 *   freq = 25000000 / (4096 * prescale)
 *   prescale = round(25000000 / (4096 * freq)) - 1
 *
 * 舵机控制:
 *   50Hz → prescale = 121
 *   12位分辨率: 0~4095
 *   0.5ms(0°) = ~102,  1.5ms(90°) = ~307,  2.5ms(180°) = ~512
 *
 * 依赖: ti_msp_dl_config.h (SysConfig生成)
 */

#ifndef __PCA9685_H
#define __PCA9685_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ── PCA9685 I2C 地址 ─────────────────────────────────────── */
#define PCA9685_ADDR            (0x40)

/* ── PCA9685 寄存器地址 ────────────────────────────────────── */
#define PCA9685_REG_MODE1       (0x00)
#define PCA9685_REG_MODE2       (0x01)
#define PCA9685_REG_LED0_ON_L   (0x06)
#define PCA9685_REG_LED0_ON_H   (0x07)
#define PCA9685_REG_LED0_OFF_L  (0x08)
#define PCA9685_REG_LED0_OFF_H  (0x09)
#define PCA9685_REG_ALL_LED_ON_L  (0xFA)
#define PCA9685_REG_ALL_LED_ON_H  (0xFB)
#define PCA9685_REG_ALL_LED_OFF_L (0xFC)
#define PCA9685_REG_ALL_LED_OFF_H (0xFD)
#define PCA9685_REG_PRE_SCALE   (0xFE)

/* ── MODE1 寄存器位 ────────────────────────────────────────── */
#define PCA9685_MODE1_RESTART   (0x80)
#define PCA9685_MODE1_EXTCLK    (0x40)
#define PCA9685_MODE1_AI        (0x20)  /* 自动递增 */
#define PCA9685_MODE1_SLEEP     (0x10)  /* 低功耗模式 */
#define PCA9685_MODE1_SUB1      (0x08)
#define PCA9685_MODE1_SUB2      (0x04)
#define PCA9685_MODE1_SUB3      (0x02)
#define PCA9685_MODE1_ALLCALL   (0x01)

/* ── MODE2 寄存器位 ────────────────────────────────────────── */
#define PCA9685_MODE2_INVRT     (0x10)
#define PCA9685_MODE2_OCH       (0x08)
#define PCA9685_MODE2_OUTDRV    (0x04)  /* 推挽输出 */
#define PCA9685_MODE2_OUTNE_0   (0x01)
#define PCA9685_MODE2_OUTNE_1   (0x02)

/* ── 常量 ──────────────────────────────────────────────────── */
#define PCA9685_OSC_FREQ        (25000000UL)  /* 内部振荡器25MHz */
#define PCA9685_PWM_STEPS       (4096)        /* 12位分辨率 */

/* ── API ───────────────────────────────────────────────────── */

/**
 * @brief 初始化 PCA9685
 *        设置自动递增、推挽输出、配置50Hz舵机频率
 * @return true=成功, false=I2C通信失败
 */
bool PCA9685_Init(void);

/**
 * @brief 设置PCA9685 PWM输出频率
 * @param freq_hz  目标频率(Hz), 舵机通常50Hz
 * @return true=成功
 */
bool PCA9685_SetPWMFreq(uint16_t freq_hz);

/**
 * @brief 设置单通道PWM值
 * @param channel  通道号 0~15
 * @param on       ON时刻 (0~4095)
 * @param off      OFF时刻 (0~4095)
 * @return true=成功
 */
bool PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off);

/**
 * @brief 设置单通道舵机角度
 * @param channel  通道号 0~15
 * @param angle    角度 0~180°
 * @return true=成功
 *
 * 脉宽映射: 0.5ms(0°) ~ 2.5ms(180°)
 * 在50Hz(20ms周期)下: 0.5ms/20ms*4096=102, 2.5ms/20ms*4096=512
 */
bool PCA9685_SetAngle(uint8_t channel, uint16_t angle);

/**
 * @brief 关闭所有通道PWM输出
 * @return true=成功
 */
bool PCA9685_AllOff(void);

/**
 * @brief 写PCA9685单个寄存器
 * @param reg  寄存器地址
 * @param val  写入值
 * @return true=成功
 */
bool PCA9685_WriteReg(uint8_t reg, uint8_t val);

/**
 * @brief 读PCA9685单个寄存器
 * @param reg  寄存器地址
 * @param val  输出: 读取的值
 * @return true=成功
 */
bool PCA9685_ReadReg(uint8_t reg, uint8_t *val);

#endif /* __PCA9685_H */
