/**
 * @file    ultrasonic_stm32.h
 * @brief   SR04/US-016 超声波测距驱动 — STM32 HAL库版本
 *
 * 原理:
 *   1. Trig引脚输出10µs高电平触发测距
 *   2. 模块自动发送8个40kHz超声波脉冲
 *   3. Echo引脚返回高电平脉冲，脉宽 = 距离 * 2 / 340
 *   4. 使用TIM输入捕获测量Echo脉宽，计算距离
 *
 * 测量范围: 2cm ~ 400cm
 *
 * 硬件连接:
 *   PB6 → Trig (超声波触发)
 *   PB7 → Echo (超声波回波, TIM4_CH2 输入捕获)
 *
 * 定时器: TIM4, CH2 输入捕获, 1MHz (1µs分辨率)
 */

#ifndef __ULTRASONIC_STM32_H
#define __ULTRASONIC_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── 常量定义 ─────────────────────────────────────────────── */

/** 声速换算: 距离(cm) = 时间(µs) / ULTRASONIC_US_PER_CM */
#define ULTRASONIC_US_PER_CM    58.0f

/** 最小测量距离 (cm) */
#define ULTRASONIC_MIN_CM       2.0f

/** 最大测量距离 (cm) */
#define ULTRASONIC_MAX_CM       400.0f

/** 最大Echo脉宽 (µs) */
#define ULTRASONIC_TIMEOUT_US   30000U

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化超声波传感器
 *        配置Trig引脚(PB6)输出、Echo引脚(PB7)输入、TIM4输入捕获
 * @param htim TIM4句柄指针（需要在main.c中配置好TIM4）
 */
void Ultrasonic_Init(TIM_HandleTypeDef *htim);

/**
 * @brief 触发一次超声波测距，返回距离(cm)
 * @param distance_cm  输出: 测量距离，单位cm
 * @return true=测量成功, false=超时/无回波
 * @note 阻塞函数，最大阻塞约30ms
 *       调用间隔建议 > 60ms
 */
bool Ultrasonic_Measure(float *distance_cm);

/**
 * @brief 触发一次超声波测距，返回Echo脉宽(µs)
 * @param pulse_us  输出: Echo高电平脉宽，单位µs
 * @return true=测量成功, false=超时/无回波
 */
bool Ultrasonic_MeasureRaw(uint32_t *pulse_us);

#endif /* __ULTRASONIC_STM32_H */
