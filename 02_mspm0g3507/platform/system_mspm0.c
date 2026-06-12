/**
 * @file    system_mspm0.c
 * @brief   MSPM0G3507 系统初始化实现
 */

#include "system_mspm0.h"

/* ── 全局 tick 计数 ──────────────────────────────────────── */
static volatile uint32_t g_systick_count = 0;

/* ── API ─────────────────────────────────────────────────── */

void System_Init(void)
{
    /* 1. 由 SysConfig 自动生成的初始化 (引脚/外设/时钟) */
#ifdef SYSCFG_DL_init
    SYSCFG_DL_init();
#endif

    /* 2. 确保全局中断开启 */
    __enable_irq();

    /* 3. 配置 SysTick (1ms) */
    System_SystickConfig(1000);
}

uint32_t System_GetSysClkFreq(void)
{
    return MSPM0_SYS_CLK_HZ;
}

void System_SystickConfig(uint32_t freq_hz)
{
    /* SysTick = 系统时钟 / freq_hz - 1 */
    SysTick->LOAD = (MSPM0_SYS_CLK_HZ / freq_hz) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

void System_Sleep(void)
{
    __WFI();
}

void System_DelayMs(uint32_t ms)
{
    uint32_t start = g_systick_count;
    while ((g_systick_count - start) < ms) {
        __WFI();
    }
}

/* ── SysTick 中断处理 ────────────────────────────────────── */
void SysTick_Handler(void)
{
    g_systick_count++;
}

/** 获取当前毫秒 tick */
uint32_t System_GetTick(void)
{
    return g_systick_count;
}
