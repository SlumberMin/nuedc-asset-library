/**
 * @file    tb6612_tm4c.h
 * @brief   TB6612FNG 双路H桥电机驱动 驱动头文件 (TM4C123 TivaWare)
 * @details 控制两路直流电机的正转/反转/制动/调速
 *
 * 硬件接线示意:
 *   TB6612FNG         TM4C123
 *   --------          --------
 *   AIN1  ---------->  PA2  (GPIO)
 *   AIN2  ---------->  PA3  (GPIO)
 *   BIN1  ---------->  PA4  (GPIO)
 *   BIN2  ---------->  PA5  (GPIO)
 *   PWMA  ---------->  PB6  (M0PWM0) 或 PF2
 *   PWMB  ---------->  PB7  (M0PWM1) 或 PF3
 *   STBY  ---------->  PA6  (GPIO, 拉高使能)
 *   VM    ---------->  电机电源 (4.5~10V)
 *   VCC   ---------->  3.3V
 *   GND   ---------->  GND
 *
 * @note    PWMA/PWMB 可选择 PWM0 Module0 Gen0 的 PB6/PB7
 */

#ifndef TB6612_TM4C_H
#define TB6612_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 电机通道定义 ========== */
typedef enum {
    MOTOR_A = 0,    /* 电机A通道 */
    MOTOR_B = 1     /* 电机B通道 */
} TB6612_Motor_t;

/* ========== 电机方向定义 ========== */
typedef enum {
    MOTOR_STOP   = 0,   /* 停止 (制动) */
    MOTOR_FWD    = 1,   /* 正转 */
    MOTOR_REV    = 2,   /* 反转 */
    MOTOR_BRAKE  = 3    /* 制动 (两输入拉低) */
} TB6612_Dir_t;

/* ========== 配置结构体 ========== */
typedef struct {
    /* GPIO端口基地址 (使用 SYSCTL_PERIPH_GPIOx 宏) */
    uint32_t gpio_periph;       /* GPIO外设时钟 SYSCTL_PERIPH_GPIOx */
    uint32_t gpio_base;         /* GPIO端口基地址 GPIO_PORTx_BASE */
    uint32_t ain1_pin;          /* AIN1 引脚 GPIO_PIN_x */
    uint32_t ain2_pin;          /* AIN2 引脚 GPIO_PIN_x */
    uint32_t bin1_pin;          /* BIN1 引脚 GPIO_PIN_x */
    uint32_t bin2_pin;          /* BIN2 引脚 GPIO_PIN_x */
    uint32_t stby_pin;          /* STBY 引脚 GPIO_PIN_x */

    /* PWM配置 */
    uint32_t pwm_periph;        /* PWM外设时钟 SYSCTL_PERIPH_PWMx */
    uint32_t pwm_base;          /* PWM模块基地址 PWMx_BASE */
    uint32_t pwm_gen;           /* PWM发生器 PWM_GEN_x */
    uint32_t pwm_out_a;         /* PWMA输出 PWM_OUT_x */
    uint32_t pwm_out_b;         /* PWMB输出 PWM_OUT_x */
    uint32_t pwm_out_bit_a;     /* PWMA位掩码 PWM_OUT_x_BIT */
    uint32_t pwm_out_bit_b;     /* PWMB位掩码 PWM_OUT_x_BIT */
    uint32_t pwm_gen_bit;       /* PWM发生器位掩码 PWM_GEN_x_BIT */

    /* PWM引脚端口 (用于GPIOPinTypePWM) */
    uint32_t pwm_pin_port;      /* PWM引脚所在GPIO端口 */
    uint32_t pwm_pin_a;         /* PWMA引脚 GPIO_PIN_x */
    uint32_t pwm_pin_b;         /* PWMB引脚 GPIO_PIN_x */

    uint32_t pwm_freq_hz;       /* PWM频率 (默认10kHz) */
    uint32_t sys_clock_hz;      /* 系统时钟频率 */
} TB6612_Config_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化TB6612电机驱动
 * @param  cfg  配置结构体指针
 */
void TB6612_Init(const TB6612_Config_t *cfg);

/**
 * @brief  设置电机速度和方向
 * @param  motor   电机通道 (MOTOR_A / MOTOR_B)
 * @param  dir     方向
 * @param  speed   速度 (0~1000, 对应占空比0~100%)
 */
void TB6612_SetMotor(TB6612_Motor_t motor, TB6612_Dir_t dir, uint16_t speed);

/**
 * @brief  使能/禁止电机驱动 (STBY引脚)
 * @param  enable  true=使能, false=待机
 */
void TB6612_Enable(bool enable);

/**
 * @brief  紧急制动 (两路电机同时制动)
 */
void TB6612_EmergencyStop(void);

#ifdef __cplusplus
}
#endif

#endif /* TB6612_TM4C_H */
