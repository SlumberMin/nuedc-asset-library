/**
 * @file    system_mspm0.h
 * @brief   MSPM0G3507 系统初始化 — 时钟/GPIO/中断
 */

#ifndef __SYSTEM_MSPM0_H
#define __SYSTEM_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 系统总初始化
 * @note  调用 SYSCFG_DL_init() 完成引脚/外设配置
 *        配置系统时钟 80MHz (PLL)
 */
void System_Init(void);

/**
 * @brief 获取系统时钟频率
 * @return Hz
 */
uint32_t System_GetSysClkFreq(void);

/**
 * @brief 配置 SysTick 定时中断
 * @param freq_hz  中断频率 (Hz)
 */
void System_SystickConfig(uint32_t freq_hz);

/** 进入低功耗模式 */
void System_Sleep(void);

/** 软件延时 (阻塞) */
void System_DelayMs(uint32_t ms);

#endif /* __SYSTEM_MSPM0_H */
