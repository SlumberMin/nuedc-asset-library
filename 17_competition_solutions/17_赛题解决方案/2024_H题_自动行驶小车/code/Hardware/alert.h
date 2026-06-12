/**
 * @file    alert.h
 * @brief   声光提示模块头文件
 * @author  电赛团队
 * @date    2024
 * @note    蜂鸣器(PB8) + LED指示灯(PB9)
 */

#ifndef __ALERT_H
#define __ALERT_H

#include "stm32f1xx_hal.h"
#include "user_config.h"

/* ========================================================================== */
/*                              函数声明                                       */
/* ========================================================================== */

/**
 * @brief  声光提示模块初始化
 * @retval None
 */
void Alert_Init(void);

/**
 * @brief  触发声光提示（蜂鸣器响+LED亮）
 * @param  duration_ms: 提示持续时间(ms)
 * @retval None
 */
void Alert_Start(uint32_t duration_ms);

/**
 * @brief  停止声光提示
 * @retval None
 */
void Alert_Stop(void);

/**
 * @brief  声光提示处理（在主循环或定时中断中调用）
 * @note   管理提示定时，时间到自动关闭
 * @retval None
 */
void Alert_Process(void);

/**
 * @brief  检查是否正在提示
 * @retval uint8_t 1=正在提示, 0=未提示
 */
uint8_t Alert_IsActive(void);

/**
 * @brief  蜂鸣器单独控制
 * @param  on: 1=开, 0=关
 * @retval None
 */
void Alert_BuzzerOn(void);
void Alert_BuzzerOff(void);

/**
 * @brief  LED单独控制
 * @param  on: 1=开, 0=关
 * @retval None
 */
void Alert_LEDOn(void);
void Alert_LEDOff(void);

/**
 * @brief  LED闪烁（用于状态指示）
 * @param  times: 闪烁次数
 * @param  interval_ms: 闪烁间隔(ms)
 * @retval None
 */
void Alert_LEDBlink(uint8_t times, uint16_t interval_ms);

#endif /* __ALERT_H */
