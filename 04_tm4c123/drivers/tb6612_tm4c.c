/**
 * @file    tb6612_tm4c.c
 * @brief   TB6612FNG 双路H桥电机驱动 实现文件 (TM4C123 TivaWare)
 */

#include "tb6612_tm4c.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

/* ========== 内部变量: 保存配置副本 ========== */
static const TB6612_Config_t *g_cfg = 0;

/* 内部: PWM周期计数值 (时钟计数) */
static uint32_t g_pwm_load = 0;

/* ========================================================================== */
/*                              内部辅助函数                                    */
/* ========================================================================== */

/**
 * @brief  设置方向引脚
 * @param  in1_pin  IN1引脚
 * @param  in2_pin  IN2引脚
 * @param  dir      方向
 */
static void _SetDirPins(uint32_t in1_pin, uint32_t in2_pin, TB6612_Dir_t dir)
{
    uint32_t base = g_cfg->gpio_base;

    switch (dir) {
    case MOTOR_FWD:
        /* IN1=高, IN2=低 → 正转 */
        GPIOPinWrite(base, in1_pin, in1_pin);
        GPIOPinWrite(base, in2_pin, 0);
        break;
    case MOTOR_REV:
        /* IN1=低, IN2=高 → 反转 */
        GPIOPinWrite(base, in1_pin, 0);
        GPIOPinWrite(base, in2_pin, in2_pin);
        break;
    case MOTOR_STOP:
        /* IN1=低, IN2=低 → 滑行停止 (高阻态) */
        GPIOPinWrite(base, in1_pin, 0);
        GPIOPinWrite(base, in2_pin, 0);
        break;
    case MOTOR_BRAKE:
    default:
        /* IN1=高, IN2=高 → 制动 (短路制动) */
        GPIOPinWrite(base, in1_pin, in1_pin);
        GPIOPinWrite(base, in2_pin, in2_pin);
        break;
    }
}

/* ========================================================================== */
/*                              公共接口实现                                    */
/* ========================================================================== */

void TB6612_Init(const TB6612_Config_t *cfg)
{
    g_cfg = cfg;

    /* ---- 1. 使能GPIO外设时钟 ---- */
    SysCtlPeripheralEnable(cfg->gpio_periph);
    while (!SysCtlPeripheralReady(cfg->gpio_periph)) {}

    /* ---- 2. 使能PWM外设时钟 ---- */
    SysCtlPeripheralEnable(cfg->pwm_periph);
    while (!SysCtlPeripheralReady(cfg->pwm_periph)) {}

    /* ---- 3. 配置方向引脚为GPIO输出 ---- */
    /* 合并所有GPIO输出引脚 */
    uint32_t dir_pins = cfg->ain1_pin | cfg->ain2_pin |
                        cfg->bin1_pin | cfg->bin2_pin |
                        cfg->stby_pin;
    GPIOPinTypeGPIOOutput(cfg->gpio_base, dir_pins);

    /* 默认全部拉低 (停止状态) */
    GPIOPinWrite(cfg->gpio_base, dir_pins, 0);

    /* ---- 4. 配置PWM引脚为PWM复用功能 ---- */
    GPIOPinTypePWM(cfg->pwm_pin_port, cfg->pwm_pin_a);
    GPIOPinTypePWM(cfg->pwm_pin_port, cfg->pwm_pin_b);

    /* ---- 5. 计算PWM周期并配置PWM发生器 ---- */
    /* PWM频率 = 系统时钟 / PWM计数器重装值 */
    g_pwm_load = cfg->sys_clock_hz / cfg->pwm_freq_hz;

    PWMGenConfigure(cfg->pwm_base, cfg->pwm_gen,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(cfg->pwm_base, cfg->pwm_gen, g_pwm_load);

    /* 初始占空比0% */
    PWMPulseWidthSet(cfg->pwm_base, cfg->pwm_out_a, 0);
    PWMPulseWidthSet(cfg->pwm_base, cfg->pwm_out_b, 0);

    /* 使能PWM输出 */
    PWMOutputState(cfg->pwm_base, cfg->pwm_out_bit_a | cfg->pwm_out_bit_b, true);
    PWMGenEnable(cfg->pwm_base, cfg->pwm_gen);
}

void TB6612_SetMotor(TB6612_Motor_t motor, TB6612_Dir_t dir, uint16_t speed)
{
    if (!g_cfg) return;

    /* 限制速度范围 */
    if (speed > 1000) speed = 1000;

    /* 根据电机通道选择方向引脚和PWM输出 */
    uint32_t in1, in2, pwm_out;
    if (motor == MOTOR_A) {
        in1 = g_cfg->ain1_pin;
        in2 = g_cfg->ain2_pin;
        pwm_out = g_cfg->pwm_out_a;
    } else {
        in1 = g_cfg->bin1_pin;
        in2 = g_cfg->bin2_pin;
        pwm_out = g_cfg->pwm_out_b;
    }

    /* 设置方向 */
    _SetDirPins(in1, in2, dir);

    /* 设置占空比: speed 0~1000 → 脉宽 0~pwm_load */
    uint32_t pulse = (uint32_t)speed * g_pwm_load / 1000;
    PWMPulseWidthSet(g_cfg->pwm_base, pwm_out, pulse);
}

void TB6612_Enable(bool enable)
{
    if (!g_cfg) return;
    if (enable) {
        GPIOPinWrite(g_cfg->gpio_base, g_cfg->stby_pin, g_cfg->stby_pin);
    } else {
        GPIOPinWrite(g_cfg->gpio_base, g_cfg->stby_pin, 0);
    }
}

void TB6612_EmergencyStop(void)
{
    if (!g_cfg) return;
    /* 两路电机全部制动 */
    TB6612_SetMotor(MOTOR_A, MOTOR_BRAKE, 0);
    TB6612_SetMotor(MOTOR_B, MOTOR_BRAKE, 0);
}
