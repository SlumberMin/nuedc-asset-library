/**
 * @file    pca9685_tm4c.h
 * @brief   PCA9685舵机驱动板驱动 头文件 (TM4C123 I2C)
 * @details 16路12位PWM输出，I2C接口，常用于舵机/LED控制
 *
 * 硬件接线:
 *   PCA9685        TM4C123
 *   --------       --------
 *   SDA  --------->  PB3 (I2C0SDA)
 *   SCL  --------->  PB2 (I2C0SCL)
 *   VCC  --------->  3.3V 或 5V
 *   V+   --------->  舵机电源 (5V~6V)
 *   GND  --------->  GND
 *
 * @note    I2C地址: 0x40 (A5~A0=000000)
 * @note    PWM频率: 50Hz (舵机), 可调范围24Hz~1526Hz
 * @note    PWM分辨率: 12位 (0~4095)
 */

#ifndef PCA9685_TM4C_H
#define PCA9685_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== PCA9685 I2C地址 ========== */
#define PCA9685_ADDR            0x40

/* ========== PCA9685寄存器 ========== */
#define PCA9685_REG_MODE1       0x00
#define PCA9685_REG_MODE2       0x01
#define PCA9685_REG_LED0_ON_L   0x06
#define PCA9685_REG_LED0_ON_H   0x07
#define PCA9685_REG_LED0_OFF_L  0x08
#define PCA9685_REG_LED0_OFF_H  0x09
#define PCA9685_REG_PRESCALE    0xFE

/* MODE1寄存器位 */
#define PCA9685_MODE1_RESTART   0x80
#define PCA9685_MODE1_EXTCLK    0x40
#define PCA9685_MODE1_AI        0x20    /* 自动递增 */
#define PCA9685_MODE1_SLEEP     0x10    /* 低功耗模式 */

/* MODE2寄存器位 */
#define PCA9685_MODE2_OUTDRV    0x04    /* 推挽输出 */
#define PCA9685_MODE2_INVRT     0x10    /* 输出反转 */

/* ========== 配置结构体 ========== */
typedef struct {
    uint32_t i2c_base;          /* I2C基地址 */
    uint32_t i2c_periph;        /* I2C外设时钟 */
    uint32_t gpio_periph;       /* GPIO外设时钟 */
    uint32_t gpio_base;         /* GPIO端口基地址 */
    uint32_t sda_pin;           /* SDA引脚 */
    uint32_t scl_pin;           /* SCL引脚 */
    uint32_t sda_config;        /* SDA引脚复用 */
    uint32_t scl_config;        /* SCL引脚复用 */
    uint32_t i2c_clock;         /* I2C时钟频率 */
    uint32_t sys_clock_hz;      /* 系统时钟 */
    float    pwm_freq;          /* PWM频率 (Hz), 默认50 */
    uint8_t  dev_addr;          /* 器件地址 (默认0x40) */
} PCA9685_Config_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化PCA9685
 * @param  cfg  配置结构体指针
 */
void PCA9685_Init(const PCA9685_Config_t *cfg);

/**
 * @brief  设置单通道PWM
 * @param  channel  通道号 (0~15)
 * @param  on       ON计数值 (0~4095)
 * @param  off      OFF计数值 (0~4095)
 */
void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off);

/**
 * @brief  设置舵机角度
 * @param  channel  通道号 (0~15)
 * @param  angle    角度 (0~180°)
 * @note   假设50Hz, 脉宽0.5ms~2.5ms对应0~180°
 */
void PCA9685_SetServoAngle(uint8_t channel, float angle);

/**
 * @brief  设置PWM占空比
 * @param  channel   通道号 (0~15)
 * @param  duty_pct  占空比 (0.0~100.0)
 */
void PCA9685_SetDuty(uint8_t channel, float duty_pct);

/**
 * @brief  设置PWM频率
 * @param  freq_hz  频率 (Hz)
 */
void PCA9685_SetFrequency(float freq_hz);

/**
 * @brief  关闭所有通道输出
 */
void PCA9685_AllOff(void);

#ifdef __cplusplus
}
#endif

#endif /* PCA9685_TM4C_H */
