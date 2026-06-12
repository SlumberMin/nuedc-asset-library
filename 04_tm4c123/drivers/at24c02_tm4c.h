/**
 * @file    at24c02_tm4c.h
 * @brief   AT24C02 EEPROM驱动 头文件 (TM4C123 I2C)
 * @details 2Kbit(256字节)串行EEPROM，I2C接口
 *
 * 硬件接线:
 *   AT24C02        TM4C123
 *   -------        --------
 *   SDA  --------->  PB3 (I2C0SDA)
 *   SCL  --------->  PB2 (I2C0SCL)
 *   WP   --------->  GND (可写) 或 VCC (写保护)
 *   A0~A2 -------->  GND (地址0x50)
 *   VCC  --------->  3.3V 或 5V
 *   GND  --------->  GND
 *
 * @note    I2C地址: 0x50 (A2=A1=A0=0)
 * @note    页大小: 8字节, 写入周期: 最大5ms
 */

#ifndef AT24C02_TM4C_H
#define AT24C02_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== AT24C02参数 ========== */
#define AT24C02_ADDR            0x50    /* 基础地址 (A2=A1=A0=0) */
#define AT24C02_SIZE            256     /* 总容量 (字节) */
#define AT24C02_PAGE_SIZE       8       /* 页大小 (字节) */
#define AT24C02_WRITE_DELAY_MS  5       /* 写周期最大延时 */

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
    uint32_t sys_clock_hz;      /* 系统时钟 (用于延时) */
    uint8_t  dev_addr;          /* 器件地址 (默认0x50) */
} AT24C02_Config_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化AT24C02
 * @param  cfg  配置结构体指针
 */
void AT24C02_Init(const AT24C02_Config_t *cfg);

/**
 * @brief  读取单个字节
 * @param  addr  字节地址 (0~255)
 * @return 读取的字节
 */
uint8_t AT24C02_ReadByte(uint8_t addr);

/**
 * @brief  写入单个字节
 * @param  addr  字节地址 (0~255)
 * @param  data  写入数据
 */
void AT24C02_WriteByte(uint8_t addr, uint8_t data);

/**
 * @brief  连续读取多个字节
 * @param  start_addr  起始地址
 * @param  buf         数据缓冲区
 * @param  len         读取长度
 */
void AT24C02_Read(uint8_t start_addr, uint8_t *buf, uint8_t len);

/**
 * @brief  写入多个字节 (自动处理页边界)
 * @param  start_addr  起始地址
 * @param  data        数据缓冲区
 * @param  len         写入长度
 */
void AT24C02_Write(uint8_t start_addr, const uint8_t *data, uint8_t len);

/**
 * @brief  检测EEPROM是否在线
 * @return true=在线, false=无应答
 */
bool AT24C02_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* AT24C02_TM4C_H */
