/**
 * @file    oled_ssd1306.h
 * @brief   SSD1306 0.96寸OLED I2C驱动 — MSPM0G3507
 *
 * 硬件连接:
 *   I2C0: PB2=SCL, PB3=SDA
 *   SSD1306 I2C地址: 0x3C
 *
 * 参数:
 *   分辨率: 128×64像素
 *   颜色: 单色(白)
 *   字库: 6×8 ASCII (32~127)
 *
 * 依赖: ti_msp_dl_config.h (SysConfig生成)
 */

#ifndef __OLED_SSD1306_H
#define __OLED_SSD1306_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ── SSD1306 I2C地址 ─────────────────────────────────────── */
#define SSD1306_ADDR            (0x3C)

/* ── SSD1306 显示参数 ─────────────────────────────────────── */
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64
#define SSD1306_PAGES           (SSD1306_HEIGHT / 8)   /* 8页 */

/* ── SSD1306 控制字节 ────────────────────────────────────── */
#define SSD1306_CMD             (0x00)   /* 后续字节为命令 */
#define SSD1306_DATA            (0x40)   /* 后续字节为数据 */

/* ── SSD1306 基本命令 ────────────────────────────────────── */
#define SSD1306_SETCONTRAST     (0x81)
#define SSD1306_DISPLAYALLON_RESUME (0xA4)
#define SSD1306_DISPLAYALLON    (0xA5)
#define SSD1306_NORMALDISPLAY   (0xA6)
#define SSD1306_INVERTDISPLAY   (0xA7)
#define SSD1306_DISPLAYOFF      (0xAE)
#define SSD1306_DISPLAYON       (0xAF)
#define SSD1306_SETDISPLAYOFFSET (0xD3)
#define SSD1306_SETCOMPINS      (0xDA)
#define SSD1306_SETVCOMDETECT   (0xDB)
#define SSD1306_SETDISPLAYCLOCKDIV (0xD5)
#define SSD1306_SETPRECHARGE    (0xD9)
#define SSD1306_SETMULTIPLEX    (0xA8)
#define SSD1306_SETLOWCOLUMN    (0x00)
#define SSD1306_SETHIGHCOLUMN   (0x10)
#define SSD1306_SETSTARTLINE    (0x40)
#define SSD1306_MEMORYMODE      (0x20)
#define SSD1306_COLUMNADDR      (0x21)
#define SSD1306_PAGEADDR        (0x22)
#define SSD1306_COMSCANINC      (0xC0)
#define SSD1306_COMSCANDEC      (0xC8)
#define SSD1306_SEGREMAP        (0xA0)
#define SSD1306_CHARGEPUMP      (0x8D)
#define SSD1306_EXTERNALVCC     (0x01)
#define SSD1306_SWITCHCAPVCC    (0x02)

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化SSD1306 OLED
 *        发送初始化序列，清屏，开启显示
 * @return true=成功, false=I2C通信失败
 */
bool OLED_Init(void);

/**
 * @brief 清屏 (显存清零并刷新)
 */
void OLED_Clear(void);

/**
 * @brief 刷新显存到OLED
 *        将内部显存缓冲区全部发送到SSD1306
 */
void OLED_Refresh(void);

/**
 * @brief 设置光标位置
 * @param x  列地址 0~127
 * @param y  页地址 0~7
 */
void OLED_SetCursor(uint8_t x, uint8_t y);

/**
 * @brief 在指定位置显示一个字符
 * @param x      列地址 0~127
 * @param y      页地址 0~7
 * @param ch     ASCII字符 (32~127)
 * @param size   字体大小: 6 (6×8)
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size);

/**
 * @brief 在指定位置显示字符串
 * @param x      列地址 0~127
 * @param y      页地址 0~7
 * @param str    字符串指针
 * @param size   字体大小: 6 (6×8)
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size);

/**
 * @brief 在指定位置显示数字
 * @param x      列地址 0~127
 * @param y      页地址 0~7
 * @param num    要显示的数字 (0~4294967295)
 * @param len    数字位数
 * @param size   字体大小: 6 (6×8)
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);

/**
 * @brief 在指定位置显示有符号数字
 * @param x      列地址 0~127
 * @param y      页地址 0~7
 * @param num    要显示的数字 (-2147483648~2147483647)
 * @param len    数字位数(不含符号)
 * @param size   字体大小: 6 (6×8)
 */
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size);

/**
 * @brief 在指定区域填充数据
 * @param x      起始列
 * @param y      起始页
 * @param width  宽度
 * @param height 高度(页数)
 * @param data   填充数据
 */
void OLED_FillArea(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t data);

/**
 * @brief 画点
 * @param x  列地址 0~127
 * @param y  行地址 0~63
 * @param on true=点亮, false=熄灭
 */
void OLED_DrawPoint(uint8_t x, uint8_t y, bool on);

/**
 * @brief 反色显示
 * @param invert true=反色, false=正常
 */
void OLED_InvertDisplay(bool invert);

/**
 * @brief 开启/关闭显示
 * @param on true=开启, false=关闭
 */
void OLED_DisplayOn(bool on);

#endif /* __OLED_SSD1306_H */
