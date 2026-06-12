/**
 * @file    encoder_gpio_mspm0.h
 * @brief   GPIO中断方式编码器驱动 — MSPM0G3507
 * @note    适用于N20直流减速电机等带AB相霍尔编码器的电机
 *          使用GPIO外部中断实现正交解码，不依赖硬件QEI模式
 *          支持多路编码器同时工作
 *
 * 接线示例:
 *   MSPM0 PA0 → 编码器A相   PA1 → 编码器B相 (左轮)
 *   MSPM0 PA2 → 编码器A相   PA3 → 编码器B相 (右轮)
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#ifndef __ENCODER_GPIO_MSPM0_H
#define __ENCODER_GPIO_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

/* ── 编码器标识 ────────────────────────────────────────────── */
typedef enum {
    ENCODER_GPIO_LEFT  = 0,   /* 左轮编码器 */
    ENCODER_GPIO_RIGHT = 1,   /* 右轮编码器 */
    ENCODER_GPIO_MAX
} EncoderGpioId;

/* ── 编码器配置结构 ────────────────────────────────────────── */
typedef struct {
    GPIO_Regs *port;          /* GPIO端口 (如 GPIOA) */
    uint32_t   pin_a;         /* A相引脚 (如 DL_GPIO_PIN_0) */
    uint32_t   pin_b;         /* B相引脚 (如 DL_GPIO_PIN_1) */
    uint8_t    inverted;      /* 方向反转标志 (1=反转, 0=正常) */
} EncoderGpioConfig;

/* ── 编码器数据结构 ────────────────────────────────────────── */
typedef struct {
    volatile int32_t count;           /* 当前累计脉冲数 */
    volatile int32_t last_count;      /* 上次采样值 */
    volatile int32_t speed;           /* 速度 (脉冲/采样周期) */
    volatile uint8_t dir;             /* 当前方向 (0=正转, 1=反转) */
} EncoderGpioData;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化GPIO编码器
 * @param cfg  指向 EncoderGpioConfig 数组 (长度 ENCODER_GPIO_MAX)
 * @note  自动配置GPIO中断，启用GROUP1中断
 */
void EncoderGpio_Init(const EncoderGpioConfig *cfg);

/**
 * @brief 读取编码器累计脉冲数
 * @param id  编码器编号
 * @return 累计脉冲数 (有符号，正负表示方向)
 */
int32_t EncoderGpio_Read(EncoderGpioId id);

/**
 * @brief 读取编码器速度 (脉冲/采样周期)
 * @param id  编码器编号
 * @return 速度值 (有符号)
 */
int32_t EncoderGpio_GetSpeed(EncoderGpioId id);

/**
 * @brief 清零编码器累计值
 * @param id  编码器编号
 */
void EncoderGpio_Reset(EncoderGpioId id);

/**
 * @brief 获取编码器当前方向
 * @param id  编码器编号
 * @return 0=正转, 1=反转
 */
uint8_t EncoderGpio_GetDirection(EncoderGpioId id);

/**
 * @brief 设置编码器方向修正
 * @param id    编码器编号
 * @param inv   1=反转, 0=正常
 */
void EncoderGpio_SetInverted(EncoderGpioId id, uint8_t inv);

/**
 * @brief 编码器定时采样处理 (需在定时器中断中调用)
 * @note  计算速度并清零计数器，建议调用周期10ms
 */
void EncoderGpio_Update(void);

#endif /* __ENCODER_GPIO_MSPM0_H */