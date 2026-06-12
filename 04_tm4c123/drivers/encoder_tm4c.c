/**
 * @file    encoder_tm4c.c
 * @brief   增量式编码器驱动 实现文件 (TM4C123 QEI硬件模式)
 */

#include "encoder_tm4c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

/* ========== 内部: 保存每转脉冲数(用于RPM计算) ========== */
static uint32_t g_ppr[2] = {0, 0};
static uint32_t g_max_pos[2] = {0, 0};

/* ========================================================================== */
/*                              公共接口实现                                    */
/* ========================================================================== */

void Encoder_Init(const Encoder_Config_t *cfg)
{
    uint32_t qei_base, gpio_periph, gpio_base;
    uint32_t pha_pin, phb_pin, idx_pin;
    uint32_t config_flags;

    /* ---- 根据模块选择对应的外设基地址和引脚 ---- */
    if (cfg->qei_module == ENCODER_QEI0) {
        qei_base   = QEI0_BASE;
        gpio_periph = SYSCTL_PERIPH_GPIOD;
        gpio_base  = GPIO_PORTD_BASE;
        pha_pin    = GPIO_PIN_6;   /* PD6 = PhA0 */
        phb_pin    = GPIO_PIN_7;   /* PD7 = PhB0 */
        idx_pin    = GPIO_PIN_5;   /* PD5 = IDX0 */
    } else {
        qei_base   = QEI1_BASE;
        gpio_periph = SYSCTL_PERIPH_GPIOC;
        gpio_base  = GPIO_PORTC_BASE;
        pha_pin    = GPIO_PIN_5;   /* PC5 = PhA1 */
        phb_pin    = GPIO_PIN_4;   /* PC4 = PhB1 */
        idx_pin    = GPIO_PIN_6;   /* PC6 = IDX1 */
    }

    g_ppr[cfg->qei_module] = cfg->ppr;
    g_max_pos[cfg->qei_module] = cfg->max_position;

    /* ---- 1. 使能GPIO和QEI外设时钟 ---- */
    SysCtlPeripheralEnable(gpio_periph);
    while (!SysCtlPeripheralReady(gpio_periph)) {}

    if (cfg->qei_module == ENCODER_QEI0) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
        while (!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)) {}
    } else {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
        while (!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1)) {}
    }

    /* ---- 2. 配置GPIO引脚为QEI复用功能 ---- */
    GPIOPinTypeQEI(gpio_base, pha_pin | phb_pin);

    /* 配置引脚复用 (PhA, PhB) */
    if (cfg->qei_module == ENCODER_QEI0) {
        GPIOPinConfigure(GPIO_PD6_PHA0);
        GPIOPinConfigure(GPIO_PD7_PHB0);
    } else {
        GPIOPinConfigure(GPIO_PC5_PHA1);
        GPIOPinConfigure(GPIO_PC4_PHB1);
    }

    /* ---- 3. 配置QEI模块 ---- */
    /* 使能正交解码, 4倍频计数 */
    config_flags = QEI_CONFIG_CAPTURE_A_B |    /* 捕获A/B两相 */
                   QEI_CONFIG_NO_RESET;         /* 索引不复位位置 */

    /* 如果配置了最大位置,则使用位置复位模式 */
    if (cfg->max_position > 0) {
        config_flags |= QEI_CONFIG_RESET_IDX;  /* 索引时复位位置 */
    }

    QEIConfigure(qei_base, config_flags, cfg->max_position);

    /* 设置速度捕获预分频 (使用系统时钟作为速度捕获时基) */
    QEIVelocityConfigure(qei_base, QEI_VELDIV_1, cfg->sys_clock_hz);

    /* ---- 4. 使能QEI和速度捕获 ---- */
    QEIEnable(qei_base);
    QEIVelocityEnable(qei_base);
}

int32_t Encoder_GetPosition(Encoder_QEI_t module)
{
    uint32_t base = (module == ENCODER_QEI0) ? QEI0_BASE : QEI1_BASE;
    int32_t pos = (int32_t)QEIPositionGet(base);

    /* 如果使用了位置范围模式, 将有符号值转换 */
    uint32_t max = g_max_pos[module];
    if (max > 0 && pos > (int32_t)(max / 2)) {
        pos -= (int32_t)max;
    }

    return pos;
}

int32_t Encoder_GetSpeed(Encoder_QEI_t module)
{
    uint32_t base = (module == ENCODER_QEI0) ? QEI0_BASE : QEI1_BASE;

    /* 获取每秒脉冲数 (硬件4倍频后) */
    int32_t pulses_per_sec = (int32_t)QEIVelocityGet(base);

    /* 判断方向: QEI方向寄存器 */
    if (QEIDirectionGet(base) < 0) {
        pulses_per_sec = -pulses_per_sec;
    }

    /* 转换为RPM: RPM = (pulses_per_sec * 60) / (ppr * 4) */
    uint32_t ppr = g_ppr[module];
    if (ppr == 0) return 0;

    int32_t rpm = (pulses_per_sec * 60) / (int32_t)(ppr * 4);
    return rpm;
}

void Encoder_ResetPosition(Encoder_QEI_t module)
{
    uint32_t base = (module == ENCODER_QEI0) ? QEI0_BASE : QEI1_BASE;
    QEIPositionSet(base, 0);
}

uint32_t Encoder_GetDirection(Encoder_QEI_t module)
{
    uint32_t base = (module == ENCODER_QEI0) ? QEI0_BASE : QEI1_BASE;
    return (QEIDirectionGet(base) >= 0) ? 1 : 0;
}
