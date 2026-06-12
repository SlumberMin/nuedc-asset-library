/**
 * @file    servo_tm4c.h
 * @brief   SG90舵机驱动 头文件 (TM4C123 PWM)
 * @details 通过PWM输出控制SG90舵机角度 (0°~180°)
 *
 * SG90参数:
 *   - PWM周期: 20ms (50Hz)
 *   - 脉宽范围: 0.5ms~2.5ms 对应 0°~180°
 *   - 分辨率: ~0.09°/step (1000级时)
 *
 * 硬件接线:
 *   SG90           TM4C123
 *   ----           --------
 *   信号 ---------->  PF1 (M1PWM5) 或其他PWM引脚
 *   VCC ----------->  5V (注意: 需外部供电, MCU 3.3V可能不够)
 *   GND ----------->  GND
 *
 * @note    使用PWM1 Module2 Gen1 (PF1) 为例
 */

#ifndef SERVO_TM4C_H
#define SERVO_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 舵机配置结构体 ========== */
typedef struct {
    uint32_t pwm_periph;        /* PWM外设时钟 SYSCTL_PERIPH_PWMx */
    uint32_t pwm_base;          /* PWM模块基地址 PWMx_BASE */
    uint32_t pwm_gen;           /* PWM发生器 PWM_GEN_x */
    uint32_t pwm_out;           /* PWM输出 PWM_OUT_x */
    uint32_t pwm_out_bit;       /* PWM输出位掩码 PWM_OUT_x_BIT */
    uint32_t pwm_gen_bit;       /* PWM发生器位掩码 PWM_GEN_x_BIT */
    uint32_t gpio_periph;       /* GPIO外设时钟 */
    uint32_t gpio_base;         /* GPIO端口基地址 */
    uint32_t gpio_pin;          /* GPIO引脚 GPIO_PIN_x */
    uint32_t pin_config;        /* 引脚复用配置 GPIO_Pxx_MxPWMn */

    uint32_t sys_clock_hz;      /* 系统时钟频率 */

    /* SG90参数 (可调整以适配其他舵机) */
    uint16_t min_pulse_us;      /* 最小脉宽 (μs), 默认500 */
    uint16_t max_pulse_us;      /* 最大脉宽 (μs), 默认2500 */
    uint16_t angle_range;       /* 角度范围 (°),  默认180 */
} Servo_Config_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化舵机PWM
 * @param  cfg  配置结构体指针
 */
void Servo_Init(const Servo_Config_t *cfg);

/**
 * @brief  设置舵机角度
 * @param  angle  目标角度 (0~180)
 * @note   超出范围会被钳位
 */
void Servo_SetAngle(uint16_t angle);

/**
 * @brief  设置舵机脉宽 (微秒)
 * @param  pulse_us  脉宽 (μs)
 */
void Servo_SetPulse(uint16_t pulse_us);

/**
 * @brief  设置舵机到中间位置 (90°)
 */
void Servo_SetCenter(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_TM4C_H */
