/**
 * @file    ultrasonic_tm4c.c
 * @brief   HC-SR04超声波测距驱动 实现文件 (TM4C123)
 */

#include "ultrasonic_tm4c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"

/* ========== 内部变量 ========== */
static const Ultrasonic_Config_t *g_us_cfg = 0;
static volatile bool     g_echo_done = false;   /* 测量完成标志 */
static volatile float    g_distance_cm = 0.0f;  /* 最近一次测量结果 */
static volatile uint32_t g_echo_rising = 0;     /* 上升沿Timer值 */

/* Timer捕获中断服务例程 (处理Echo信号的上升/下降沿) */
void Ultrasonic_CaptureISR(void)
{
    if (!g_us_cfg) return;

    volatile uint32_t status = TimerIntStatus(g_us_cfg->echo_timer_base, true);
    TimerIntClear(g_us_cfg->echo_timer_base, status);

    if (status & TIMER_CAPA_EVENT) {
        volatile uint32_t cap_val = TimerValueGet(g_us_cfg->echo_timer_base,
                                          g_us_cfg->echo_timer_type);
        /* 检测Echo引脚当前电平判断上升/下降沿 */
        if (GPIOPinRead(g_us_cfg->echo_gpio_base, g_us_cfg->echo_pin)) {
            /* 上升沿: 记录起始时间 */
            g_echo_rising = cap_val;
        } else {
            /* 下降沿: 计算脉宽和距离 */
            volatile uint32_t ticks = g_echo_rising - cap_val;  /* 向上计数 */
            volatile uint32_t time_us = ticks / (g_us_cfg->sys_clock_hz / 1000000);
            volatile float distance = (float)time_us / 58.0f;
            if (distance >= 2.0f && distance <= 400.0f)
                g_distance_cm = distance;
            else
                g_distance_cm = 0.0f;
            g_echo_done = true;
        }
    }
}

/* 用于软件超时的简单延时 (微秒) */
static void _DelayUs(uint32_t us)
{
    /* 基于SysCtlDelay: 每次循环3个时钟周期 */
    SysCtlDelay(us * (g_us_cfg->sys_clock_hz / 3000000));
}

/* ========================================================================== */
/*                              公共接口实现                                    */
/* ========================================================================== */

void Ultrasonic_Init(const Ultrasonic_Config_t *cfg)
{
    g_us_cfg = cfg;

    /* ---- 1. 使能外设时钟 ---- */
    SysCtlPeripheralEnable(cfg->trig_gpio_periph);
    while (!SysCtlPeripheralReady(cfg->trig_gpio_periph)) {}

    SysCtlPeripheralEnable(cfg->echo_timer_periph);
    while (!SysCtlPeripheralReady(cfg->echo_timer_periph)) {}

    SysCtlPeripheralEnable(cfg->echo_gpio_periph);
    while (!SysCtlPeripheralReady(cfg->echo_gpio_periph)) {}

    /* ---- 2. 配置Trig引脚为GPIO输出, 默认低电平 ---- */
    GPIOPinTypeGPIOOutput(cfg->trig_gpio_base, cfg->trig_pin);
    GPIOPinWrite(cfg->trig_gpio_base, cfg->trig_pin, 0);

    /* ---- 3. 配置Echo引脚为Timer捕获输入 ---- */
    GPIOPinTypeTimer(cfg->echo_gpio_base, cfg->echo_pin);
    GPIOPinConfigure(cfg->echo_pin_config);

    /* ---- 4. 配置Timer为输入捕获模式 ---- */
    /* 16位定时器 + 边沿时间捕获 */
    TimerConfigure(cfg->echo_timer_base, cfg->echo_timer_cfg);
    TimerControlEvent(cfg->echo_timer_base, cfg->echo_timer_type,
                      cfg->echo_timer_cap_mode);

    /* 设置Timer为连续递增计数 (使用32位模式) */
    TimerLoadSet(cfg->echo_timer_base, cfg->echo_timer_type, 0xFFFFFFFF);

    /* 使能Timer捕获中断 */
    TimerIntEnable(cfg->echo_timer_base, TIMER_CAPA_EVENT);
    TimerEnable(cfg->echo_timer_base, cfg->echo_timer_type);
}

float Ultrasonic_GetDistance_cm(void)
{
    if (!g_us_cfg) return 0.0f;

    /* 先用中断方式触发并等待结果 */
    Ultrasonic_Trigger();

    /* 阻塞等待中断设置g_echo_done (超时约50ms) */
    uint32_t timeout_us = 50000;
    while (!g_echo_done && timeout_us > 0) {
        _DelayUs(10);
        timeout_us -= 10;
    }

    if (g_echo_done)
        return g_distance_cm;

    return 0.0f;  /* 超时 */
}

void Ultrasonic_Trigger(void)
{
    if (!g_us_cfg) return;

    /* 发送Trig脉冲 */
    GPIOPinWrite(g_us_cfg->trig_gpio_base, g_us_cfg->trig_pin,
                 g_us_cfg->trig_pin);
    _DelayUs(15);
    GPIOPinWrite(g_us_cfg->trig_gpio_base, g_us_cfg->trig_pin, 0);

    g_echo_done = false;

    /* 重置Timer */
    TimerLoadSet(g_us_cfg->echo_timer_base, g_us_cfg->echo_timer_type,
                 0xFFFFFFFF);
}

bool Ultrasonic_IsDone(void)
{
    return g_echo_done;
}

float Ultrasonic_GetLastDistance_cm(void)
{
    return g_distance_cm;
}
