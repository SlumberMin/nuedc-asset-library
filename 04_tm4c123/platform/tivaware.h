/**
 * @file tivaware.h
 * @brief TivaWare封装层 - 统一外设操作接口
 * @target TM4C123GH6PZT7 (ARM Cortex-M4F, 80MHz)
 *
 * 封装TivaWare库函数，提供简洁统一的API。
 * 所有驱动模块通过本头文件访问底层外设。
 */
#ifndef __TIVAWARE_H
#define __TIVAWARE_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== TivaWare 头文件包含 ======================== */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* TivaWare 驱动库 */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
#include "inc/hw_pwm.h"
#include "inc/hw_qei.h"
#include "inc/hw_adc.h"
#include "inc/hw_uart.h"
#include "inc/hw_sysctl.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/systick.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

/* ======================== 系统时钟定义 ======================== */
#define SYS_CLK_FREQ        80000000UL   /* 80MHz 系统时钟 */
#define SYS_CLK_FREQ_MHZ    80

/* ======================== GPIO 统一操作宏 ======================== */
#define GPIO_PIN_SET(port, pin)     MAP_GPIOPinWrite(port, pin, pin)
#define GPIO_PIN_CLR(port, pin)     MAP_GPIOPinWrite(port, pin, 0)
#define GPIO_PIN_TOGGLE(port, pin)  MAP_GPIOPinWrite(port, pin, \
                                    ~MAP_GPIOPinRead(port, pin) & (pin))
#define GPIO_PIN_READ(port, pin)    (MAP_GPIOPinRead(port, pin) & (pin))

/* GPIO模式配置封装 */
#define GPIO_OUTPUT_PP(port, pin) \
    MAP_GPIOPinTypeGPIOOutput(port, pin)
#define GPIO_INPUT(port, pin) \
    MAP_GPIOPinTypeGPIOInput(port, pin)
#define GPIO_OUTPUT_OD(port, pin) \
    do { \
        MAP_GPIOPinTypeGPIOOutput(port, pin); \
        MAP_GPIOPadConfigSet(port, pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD); \
    } while(0)

/* ======================== 外设时钟使能封装 ======================== */
#define PERIPH_ENABLE(periph)  MAP_SysCtlPeripheralEnable(periph)
#define PERIPH_READY(periph)   MAP_SysCtlPeripheralReady(periph)

/* 等待外设时钟就绪 */
static inline void periph_wait_ready(uint32_t periph) {
    while (!MAP_SysCtlPeripheralReady(periph)) {}
}

/* ======================== PWM 封装 ======================== */
/**
 * @brief 设置PWM占空比(0.0~1.0)
 * @param base  PWM外设基址 PWM0_BASE / PWM1_BASE
 * @param gen   PWM发生器 PWM_OUT_0 ~ PWM_OUT_7
 * @param duty  占空比 0.0~1.0
 */
static inline void pwm_set_duty(uint32_t base, uint32_t gen, float duty) {
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    uint32_t period = MAP_PWMPulseWidthGet(base, gen); /* 实际是load值 */
    /* 从基址和gen计算load */
    uint32_t load = HWREG(base + PWM_O_0_LOAD +
                          (gen >> 1) * 0x40);
    uint32_t pulse = (uint32_t)(duty * (float)load);
    MAP_PWMPulseWidthSet(base, gen, pulse);
}

/**
 * @brief 快速设置PWM脉宽(时钟tick数)
 */
static inline void pwm_set_pulse(uint32_t base, uint32_t out, uint32_t ticks) {
    MAP_PWMPulseWidthSet(base, out, ticks);
}

/**
 * @brief 获取PWM周期load值
 */
static inline uint32_t pwm_get_load(uint32_t base, uint32_t gen_block) {
    return HWREG(base + PWM_O_0_LOAD + gen_block * 0x40);
}

/* ======================== ADC 封装 ======================== */
/**
 * @brief 触发ADC采样并读取结果(阻塞)
 * @param adc_base ADC外设基址
 * @param seq      序列号 ADC_ACTSS_ASEN0 ~ ASEN3
 * @return 12-bit ADC值
 */
static inline uint32_t adc_read_blocking(uint32_t adc_base, uint32_t seq) {
    MAP_ADCProcessorTrigger(adc_base, seq);
    while (MAP_ADCBusy(adc_base)) {}
    uint32_t val;
    MAP_ADCSequenceDataGet(adc_base, seq, &val);
    return val;
}

/* ======================== 定时器封装 ======================== */
#define TIMER_ENABLE(base, timer) \
    MAP_SysCtlPeripheralEnable( \
        (base == TIMER0_BASE) ? SYSCTL_PERIPH_TIMER0 : \
        (base == TIMER1_BASE) ? SYSCTL_PERIPH_TIMER1 : \
        (base == TIMER2_BASE) ? SYSCTL_PERIPH_TIMER2 : \
        (base == TIMER3_BASE) ? SYSCTL_PERIPH_TIMER3 : \
        (base == TIMER4_BASE) ? SYSCTL_PERIPH_TIMER4 : \
        SYSCTL_PERIPH_TIMER5)

/* ======================== 中断优先级宏 ======================== */
#define INT_PRIORITY_HIGH       0x00    /* 最高 */
#define INT_PRIORITY_MID        0x40
#define INT_PRIORITY_LOW        0x80
#define INT_PRIORITY_SYSTICK    0xE0    /* SysTick最低 */

/* ======================== 数学工具 (FPU) ======================== */
#define CLAMP(val, min, max) \
    ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

#define ABS(x)   ((x) < 0 ? -(x) : (x))
#define SIGN(x)  ((x) > 0 ? 1.0f : ((x) < 0 ? -1.0f : 0.0f))

/* ======================== 调试串口 ======================== */
#ifndef DEBUG_UART_BASE
#define DEBUG_UART_BASE     UART0_BASE
#endif

void debug_printf_init(void);
void debug_puts(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* __TIVAWARE_H */
