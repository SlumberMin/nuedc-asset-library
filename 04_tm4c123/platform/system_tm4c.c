/**
 * @file system_tm4c.c
 * @brief TM4C123GH6PZT7 系统初始化实现
 */
#include "platform/tivaware.h"
#include "platform/system_tm4c.h"

/* ======================== 内部变量 ======================== */
static volatile uint32_t sys_tick_ms = 0;

/* ======================== SysTick 中断处理 ======================== */
void SysTick_Handler(void)
{
    sys_tick_ms++;
}

/* ======================== 实现 ======================== */

void system_init(void)
{
    /* 1. 使能FPU (Cortex-M4F) - 必须先启用FPU再使能Lazy Stacking */
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();

    /* 2. 配置系统时钟: 16MHz晶振 -> PLL -> 80MHz */
    MAP_SysCtlClockSet(
        SYSCTL_SYSDIV_2_5 |    /* 400MHz / 2.5 = 80MHz (通过PLL) */
        SYSCTL_USE_PLL |       /* 使用PLL */
        SYSCTL_XTAL_16MHZ |   /* 16MHz外部晶振 */
        SYSCTL_OSC_MAIN       /* 主振荡器 */
    );

    /* 3. 配置SysTick为1ms中断 */
    MAP_SysTickPeriodSet(SYS_CLK_FREQ / SYSTICK_FREQ_HZ);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();

    /* 4. 配置中断优先级分组 (4位抢占 + 0位子优先级) */
    /* TM4C使用4位优先级, 数值越小优先级越高 */

    /* 5. 使能全局中断 */
    MAP_IntMasterEnable();
}

uint32_t system_get_tick(void)
{
    return sys_tick_ms;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = sys_tick_ms;
    while ((sys_tick_ms - start) < ms) {
        /* 等待 */
    }
}

void delay_us(uint32_t us)
{
    /* 80MHz下, 每us约80个时钟周期 */
    /* 考虑循环开销, 粗略估算 */
    volatile uint32_t count = us * (SYS_CLK_FREQ_MHZ / 3);
    while (count--) {
        __asm("NOP");
    }
}

uint32_t system_get_clk_freq(void)
{
    return MAP_SysCtlClockGet();
}
