/**
 * @file    servo_tm4c.c
 * @brief   SG90舵机驱动 实现文件 (TM4C123 PWM)
 */

#include "servo_tm4c.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

/* ========== 内部变量 ========== */
static const Servo_Config_t *g_servo_cfg = 0;
static uint32_t g_servo_load = 0;   /* PWM周期计数 (20ms对应值) */

/* ========================================================================== */
/*                              公共接口实现                                    */
/* ========================================================================== */

void Servo_Init(const Servo_Config_t *cfg)
{
    g_servo_cfg = cfg;

    /* ---- 1. 使能外设时钟 ---- */
    SysCtlPeripheralEnable(cfg->pwm_periph);
    while (!SysCtlPeripheralReady(cfg->pwm_periph)) {}

    SysCtlPeripheralEnable(cfg->gpio_periph);
    while (!SysCtlPeripheralReady(cfg->gpio_periph)) {}

    /* ---- 2. 配置GPIO引脚为PWM复用功能 ---- */
    GPIOPinConfigure(cfg->pin_config);
    GPIOPinTypePWM(cfg->gpio_base, cfg->gpio_pin);

    /* ---- 3. 计算PWM周期 ---- */
    /* 20ms周期 = 系统时钟 / 50Hz */
    g_servo_load = cfg->sys_clock_hz / 50;

    /* ---- 4. 配置PWM发生器: 递减计数, 向上对齐 ---- */
    PWMGenConfigure(cfg->pwm_base, cfg->pwm_gen,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(cfg->pwm_base, cfg->pwm_gen, g_servo_load);

    /* 初始脉宽: 中间位置 */
    Servo_SetAngle(90);

    /* ---- 5. 使能PWM输出 ---- */
    PWMOutputState(cfg->pwm_base, cfg->pwm_out_bit, true);
    PWMGenEnable(cfg->pwm_base, cfg->pwm_gen);
}

void Servo_SetAngle(uint16_t angle)
{
    if (!g_servo_cfg) return;

    /* 钳位角度 */
    if (angle > g_servo_cfg->angle_range) {
        angle = g_servo_cfg->angle_range;
    }

    /* 线性映射: angle → pulse_us */
    uint32_t pulse_us = g_servo_cfg->min_pulse_us +
        (uint32_t)angle *
        (g_servo_cfg->max_pulse_us - g_servo_cfg->min_pulse_us) /
        (g_servo_cfg->angle_range > 0 ? g_servo_cfg->angle_range : 1);  /* [修复#1] 防止angle_range=0除零 */

    Servo_SetPulse((uint16_t)pulse_us);
}

void Servo_SetPulse(uint16_t pulse_us)
{
    if (!g_servo_cfg) return;

    /* 脉宽(us) → 计数值 = pulse_us * (sys_clock / 1000000) */
    uint32_t pulse_count = (uint32_t)pulse_us *
                           (g_servo_cfg->sys_clock_hz / 1000000);

    /* 防止超出周期 */
    if (pulse_count >= g_servo_load) {
        pulse_count = g_servo_load - 1;
    }

    PWMPulseWidthSet(g_servo_cfg->pwm_base, g_servo_cfg->pwm_out, pulse_count);
}

void Servo_SetCenter(void)
{
    Servo_SetAngle(90);
}
