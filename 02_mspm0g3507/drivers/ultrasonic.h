/**
 * @file    ultrasonic.h
 * @brief   SR04/US-016 超声波测距驱动 — MSPM0G3507
 *
 * 原理:
 *   1. Trig引脚输出10µs高电平触发测距
 *   2. 模块自动发送8个40kHz超声波脉冲
 *   3. Echo引脚返回高电平脉冲，脉宽 = 距离 * 2 / 340
 *   4. 使用Timer(1MHz)测量Echo脉宽，计算距离
 *
 * 测量范围: 2cm ~ 400cm
 *
 * 硬件连接:
 *   PB6 → Trig (超声波触发)
 *   PB7 → Echo (超声波回波)
 *
 * 依赖: ti_msp_dl_config.h (SysConfig生成)
 *       SysConfig: src/ultrasonic_test.syscfg
 */

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ── 常量定义 ─────────────────────────────────────────────── */

/** 声速换算: 距离(cm) = 时间(µs) / ULTRASONIC_US_PER_CM */
#define ULTRASONIC_US_PER_CM    58.0f

/** 最小测量距离 (cm) */
#define ULTRASONIC_MIN_CM       2.0f

/** 最大测量距离 (cm) */
#define ULTRASONIC_MAX_CM       400.0f

/** 最大Echo脉宽 (µs) = 400cm * 58 = 23200µs + 余量 */
#define ULTRASONIC_TIMEOUT_US   30000U

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化超声波传感器
 *        使能Timer和GPIO中断，Trig引脚默认低电平
 */
void Ultrasonic_Init(void);

/**
 * @brief 触发一次超声波测距，返回距离(cm)
 *
 * @param distance_cm  输出: 测量距离，单位cm
 * @return true=测量成功, false=超时/无回波
 *
 * @note 阻塞函数，最大阻塞约30ms（400cm）
 *       调用间隔建议 > 60ms
 */
bool Ultrasonic_Measure(float *distance_cm);

/**
 * @brief 触发一次超声波测距，返回Echo脉宽(µs)
 *
 * @param pulse_us  输出: Echo高电平脉宽，单位µs
 * @return true=测量成功, false=超时/无回波
 */
bool Ultrasonic_MeasureRaw(uint32_t *pulse_us);

/**
 * @brief GPIO中断回调（在GROUP1_IRQHandler中调用）
 *        处理Echo引脚的上升沿/下降沿
 */
void Ultrasonic_EchoIRQHandler(void);

#endif /* __ULTRASONIC_H */
