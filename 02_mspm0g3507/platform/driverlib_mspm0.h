/**
 * @file    driverlib_mspm0.h
 * @brief   MSPM0G3507 DriverLib 封装层 — 统一 GPIO/PWM/ADC/UART/I2C/SPI 操作
 * @note    基于 TI MSPM0 SDK (ti_msp_dl_config.h / driverlib)
 *          提供平台无关宏，便于从 STM32 HAL 迁移
 *
 * 引脚映射说明:
 *   MSPM0G3507 使用 PORTA/PORTB + PIN0~PIN31 编址
 *   例: DL_GPIO_PIN_0  = PORTA PIN0
 *       DL_GPIO_PIN_14 = PORTA PIN14
 *       DL_GPIO_PIN_32 = 实际对应 PORTB PIN0 (芯片内部编号)
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#ifndef __DRIVERLIB_MSPM0_H
#define __DRIVERLIB_MSPM0_H

/* ── Includes ────────────────────────────────────────────── */
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/dl_gpio.h>
#include <ti/driverlib/dl_timera.h>
#include <ti/driverlib/dl_timer.h>
#include <ti/driverlib/dl_dma.h>

/* 若工程已包含 ti_msp_dl_config.h 则直接复用 */
#ifdef __TI_MSP_DL_CONFIG_H
/* init 已由 SYSCFG_DL_init() 完成 */
#endif

/* ── 数据宽度 / 常量 ─────────────────────────────────────── */
#define MSPM0_SYS_CLK_HZ       80000000UL   /* 80 MHz (PLL) */
#define MSPM0_PERIPH_CLK_HZ    80000000UL   /* 外设时钟 = 系统时钟 (BUSCLK无分频) */
#define MSPM0_PWM_FREQ_HZ      20000UL      /* 默认 PWM 频率 20 kHz */
#define MSPM0_PWM_PERIOD        (MSPM0_PERIPH_CLK_HZ / MSPM0_PWM_FREQ_HZ)

/* ── GPIO 快速宏 ─────────────────────────────────────────── */
/** 读取引脚 */
#define GPIO_READ(port, pin)    DL_GPIO_readPins((port), (pin))
/** 置高 */
#define GPIO_SET(port, pin)     DL_GPIO_setPins((port), (pin))
/** 置低 */
#define GPIO_CLR(port, pin)     DL_GPIO_clearPins((port), (pin))
/** 翻转 */
#define GPIO_TOGGLE(port, pin)  DL_GPIO_togglePins((port), (pin))
/** 写入引脚: val = 0 置低, 否则置高 */
#define GPIO_WRITE(port, pin, val) \
    do { if (val) GPIO_SET(port, pin); else GPIO_CLR(port, pin); } while(0)

/* ── PWM 宏 ──────────────────────────────────────────────── */
/**
 * 设置 PWM 占空比 (CC0 寄存器)
 * @param timer   TIMER 实例 (如 TIMG0, TIMG6)
 * @param ch      通道 DL_TIMER_CC_0_INDEX 等
 * @param duty    0 ~ period 对应 0%~100%
 */
#define PWM_SET_DUTY(timer, ch, duty) \
    DL_TimerA_setCaptureCompareValue((timer), (duty), (ch))

/** 设置 PWM 周期 (修改 ARR) */
#define PWM_SET_PERIOD(timer, period) \
    DL_TimerA_setLoadValue((timer), (period))

/* ── ADC 宏 ──────────────────────────────────────────────── */
/**
 * 启动单次 ADC 转换
 * @param adc     ADC12 实例
 * @param seqIdx  序列器索引 DL_ADC12_MEM_IDX_0
 */
#define ADC_START(adc, seqIdx) \
    DL_ADC12_startConversion(adc)

/** 读取 ADC 结果 (12-bit) */
#define ADC_READ(adc, seqIdx) \
    DL_ADC12_getMemResult(adc, seqIdx)

/** 检查 ADC 转换完成 */
#define ADC_IS_DONE(adc) \
    DL_ADC12_getRawInterruptStatus(adc, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED)

/* ── UART 宏 ─────────────────────────────────────────────── */
/** 发送单字节 */
#define UART_TX_BYTE(uart, byte) \
    DL_UART_main_transmitData(uart, byte)

/** 接收单字节 (阻塞) */
#define UART_RX_BYTE(uart) \
    DL_UART_main_receiveData(uart)

/** 检查 UART 是否有数据 (RX中断标志) */
#define UART_RX_READY(uart) \
    (DL_UART_getRawInterruptStatus(uart) & DL_UART_INTERRUPT_RX)

/** 发送字符串 (阻塞) */
static inline void UART_SendString(UART_Regs *uart, const char *str)
{
    while (*str) {
        while (!DL_UART_isTXFIFOEmpty(uart)) {}
        DL_UART_main_transmitData(uart, (uint8_t)*str++);
    }
}

/* ── I2C 快捷宏 ──────────────────────────────────────────── */
#define I2C_START(i2c)          DL_I2C_startControllerTransfer(i2c)
#define I2C_SEND_BYTE(i2c, d)   DL_I2C_transmitControllerData(i2c, d)

/* ── SPI 快捷宏 ──────────────────────────────────────────── */
#define SPI_TX_RX(spi, data)    DL_SPI_transmitData8(spi, data)
#define SPI_RX(spi)             DL_SPI_receiveData8(spi)

/* ── 中断宏 ──────────────────────────────────────────────── */
#define IRQ_ENABLE(irqn)        NVIC_EnableIRQ(irqn)
#define IRQ_DISABLE(irqn)       NVIC_DisableIRQ(irqn)
#define IRQ_SET_PRIO(irqn, p)   NVIC_SetPriority(irqn, p)

/* ── 延时 (基于 SysTick) ─────────────────────────────────── */
#define DELAY_MS(ms)            DL_Common_delayMilliseconds(ms)
#define DELAY_US(us)            DL_Common_delayMicroseconds(us)

/* ── 调试打印 ────────────────────────────────────────────── */
#ifdef DEBUG
  #include <stdio.h>
  #define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
  #define DBG_PRINTF(...) ((void)0)
#endif

#endif /* __DRIVERLIB_MSPM0_H */
