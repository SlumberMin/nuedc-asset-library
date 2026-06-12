/**
 * @file    oled_tm4c.h
 * @brief   SSD1306 OLED显示驱动 头文件 (TM4C123 I2C)
 * @details 支持128x64像素 SSD1306 I2C接口OLED屏幕
 *
 * 硬件接线:
 *   SSD1306        TM4C123
 *   -------        --------
 *   SDA  --------->  PB3 (I2C0SDA)
 *   SCL  --------->  PB2 (I2C0SCL)
 *   VCC  --------->  3.3V
 *   GND  --------->  GND
 *
 * @note    I2C地址: 0x3C (SA0=0) 或 0x3D (SA0=1)
 * @note    需要外部4.7kΩ上拉电阻到VCC (部分模块已内置)
 */

#ifndef OLED_TM4C_H
#define OLED_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== OLED参数定义 ========== */
#define OLED_WIDTH      128     /* 屏幕宽度 (像素) */
#define OLED_HEIGHT     64      /* 屏幕高度 (像素) */
#define OLED_PAGES      8       /* 页数 (64/8=8) */
#define OLED_I2C_ADDR   0x3C   /* 默认I2C地址 */

/* ========== 字体大小定义 ========== */
typedef enum {
    OLED_FONT_6x8  = 0,    /* 6x8 像素字体 */
    OLED_FONT_8x16 = 1     /* 8x16 像素字体 */
} OLED_Font_t;

/* ========== 配置结构体 ========== */
typedef struct {
    uint32_t i2c_periph;        /* I2C外设时钟 SYSCTL_PERIPH_I2Cx */
    uint32_t i2c_base;          /* I2C基地址 I2Cx_BASE */
    uint32_t gpio_periph;       /* GPIO外设时钟 */
    uint32_t gpio_base;         /* GPIO端口基地址 */
    uint32_t sda_pin;           /* SDA引脚 GPIO_PIN_x */
    uint32_t scl_pin;           /* SCL引脚 GPIO_PIN_x */
    uint32_t sda_config;        /* SDA引脚复用 GPIO_Pxx_I2CxSDA */
    uint32_t scl_config;        /* SCL引脚复用 GPIO_Pxx_I2CxSCL */
    uint32_t i2c_clock;         /* I2C时钟频率 (Hz), 通常400000 */
    uint8_t  i2c_addr;          /* OLED I2C地址 (默认0x3C) */
} OLED_Config_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化OLED显示屏
 * @param  cfg  配置结构体指针
 */
void OLED_Init(const OLED_Config_t *cfg);

/**
 * @brief  清屏 (全屏填0)
 */
void OLED_Clear(void);

/**
 * @brief  更新显示 (将显存内容发送到OLED)
 * @note   修改像素后需要调用此函数刷新
 */
void OLED_Update(void);

/**
 * @brief  更新指定页 (局部刷新)
 * @param  page  页号 (0~7)
 */
void OLED_UpdatePage(uint8_t page);

/**
 * @brief  设置像素点
 * @param  x  X坐标 (0~127)
 * @param  y  Y坐标 (0~63)
 * @param  on true=亮, false=灭
 */
void OLED_SetPixel(uint8_t x, uint8_t y, bool on);

/**
 * @brief  显示字符
 * @param  x      X起始坐标 (像素列)
 * @param  y      Y起始坐标 (像素行)
 * @param  ch     ASCII字符
 * @param  font   字体大小
 */
void OLED_DrawChar(uint8_t x, uint8_t y, char ch, OLED_Font_t font);

/**
 * @brief  显示字符串
 * @param  x      X起始坐标
 * @param  y      Y起始坐标
 * @param  str    字符串
 * @param  font   字体大小
 */
void OLED_DrawString(uint8_t x, uint8_t y, const char *str, OLED_Font_t font);

/**
 * @brief  显示数字 (整数)
 * @param  x      X起始坐标
 * @param  y      Y起始坐标
 * @param  num    数字值
 * @param  len    显示位数 (不足补0)
 * @param  font   字体大小
 */
void OLED_DrawNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, OLED_Font_t font);

/**
 * @brief  显示浮点数
 * @param  x        X起始坐标
 * @param  y        Y起始坐标
 * @param  num      浮点数值
 * @param  int_len  整数位数
 * @param  dec_len  小数位数
 * @param  font     字体大小
 */
void OLED_DrawFloat(uint8_t x, uint8_t y, float num,
                    uint8_t int_len, uint8_t dec_len, OLED_Font_t font);

/**
 * @brief  绘制水平线
 * @param  x      X起始坐标
 * @param  y      Y坐标
 * @param  width  线宽
 */
void OLED_DrawHLine(uint8_t x, uint8_t y, uint8_t width);

/**
 * @brief  绘制垂直线
 * @param  x      X坐标
 * @param  y      Y起始坐标
 * @param  height 线高
 */
void OLED_DrawVLine(uint8_t x, uint8_t y, uint8_t height);

/**
 * @brief  绘制矩形
 * @param  x      左上角X
 * @param  y      左上角Y
 * @param  w      宽度
 * @param  h      高度
 */
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/**
 * @brief  填充矩形
 * @param  x      左上角X
 * @param  y      左上角Y
 * @param  w      宽度
 * @param  h      高度
 */
void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/**
 * @brief  反转显示模式
 * @param  invert  true=反显, false=正常
 */
void OLED_InvertDisplay(bool invert);

/**
 * @brief  开关显示
 * @param  on  true=开显示, false=关显示 (省电)
 */
void OLED_DisplayOn(bool on);

#ifdef __cplusplus
}
#endif

#endif /* OLED_TM4C_H */
