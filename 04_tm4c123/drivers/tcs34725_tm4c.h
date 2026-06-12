/**
 * @file    tcs34725_tm4c.h
 * @brief   TCS34725颜色传感器驱动 头文件 (TM4C123 I2C)
 * @details RGB颜色传感器，16位通道分辨率，内置IR滤光片
 *
 * 硬件接线:
 *   TCS34725       TM4C123
 *   --------       --------
 *   SDA  --------->  PB3 (I2C0SDA)
 *   SCL  --------->  PB2 (I2C0SCL)
 *   INT  --------->  (可选) PA5 或其他GPIO
 *   VCC  --------->  3.3V
 *   GND  --------->  GND
 *
 * @note    I2C地址: 0x29 (ADDR引脚接GND时)
 * @note    需要4.7kΩ上拉电阻
 */

#ifndef TCS34725_TM4C_H
#define TCS34725_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== TCS34725 I2C地址 ========== */
#define TCS34725_ADDR           0x29

/* ========== TCS34725寄存器地址 ========== */
#define TCS34725_REG_ENABLE     0x00
#define TCS34725_REG_ATIME      0x01
#define TCS34725_REG_CONTROL    0x0F
#define TCS34725_REG_ID         0x12
#define TCS34725_REG_STATUS     0x13
#define TCS34725_REG_CDATAL     0x14
#define TCS34725_REG_CDATAH     0x15
#define TCS34725_REG_RDATAL     0x16
#define TCS34725_REG_RDATAH     0x17
#define TCS34725_REG_GDATAL     0x18
#define TCS34725_REG_GDATAH     0x19
#define TCS34725_REG_BDATAL     0x1A
#define TCS34725_REG_BDATAH     0x1B

/* 命令位: 事务类型 */
#define TCS34725_CMD_BIT        0x80
#define TCS34725_CMD_AUTO_INC   0xA0    /* 自动递增 */

/* Enable寄存器位 */
#define TCS34725_ENABLE_PON     0x01    /* 上电 */
#define TCS34725_ENABLE_AEN     0x02    /* ADC使能 */

/* 增益设置 */
typedef enum {
    TCS34725_GAIN_1X  = 0x00,
    TCS34725_GAIN_4X  = 0x01,
    TCS34725_GAIN_16X = 0x02,
    TCS34725_GAIN_60X = 0x03
} TCS34725_Gain_t;

/* ========== 颜色数据结构 ========== */
typedef struct {
    uint16_t clear;     /* 透明通道 */
    uint16_t red;       /* 红色通道 */
    uint16_t green;     /* 绿色通道 */
    uint16_t blue;      /* 蓝色通道 */
    /* 归一化百分比 (0~100) */
    float r_pct;
    float g_pct;
    float b_pct;
} TCS34725_Color_t;

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
    uint8_t  integration_time;  /* 积分时间 (0xFF=2.4ms, 0x00=700ms) */
    TCS34725_Gain_t gain;       /* 增益 */
} TCS34725_Config_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化TCS34725
 * @param  cfg  配置结构体指针
 * @return true=成功, false=器件ID校验失败
 */
bool TCS34725_Init(const TCS34725_Config_t *cfg);

/**
 * @brief  读取颜色数据
 * @param  color  输出颜色结构体指针
 */
void TCS34725_Read(TCS34725_Color_t *color);

/**
 * @brief  设置增益
 * @param  gain  增益值
 */
void TCS34725_SetGain(TCS34725_Gain_t gain);

/**
 * @brief  设置积分时间
 * @param  atime  积分时间寄存器值
 */
void TCS34725_SetIntegrationTime(uint8_t atime);

/**
 * @brief  使能/禁用传感器
 * @param  enable  true=使能, false=禁用
 */
void TCS34725_Enable(bool enable);

/**
 * @brief  获取器件ID
 * @return 器件ID字节 (TCS34725=0x44, TCS34721=0x4D)
 */
uint8_t TCS34725_GetID(void);

#ifdef __cplusplus
}
#endif

#endif /* TCS34725_TM4C_H */
