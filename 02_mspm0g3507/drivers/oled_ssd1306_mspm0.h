/**
 * @file    oled_ssd1306_mspm0.h
 * @brief   SSD1306 OLED显示驱动 — MSPM0G3507
 * @note    适用于0.96寸I2C OLED显示屏 (128x64)
 *          使用硬件I2C通信，支持中英文显示
 *
 * 接线示例:
 *   MSPM0 PA9 → OLED SCL (I2C时钟)
 *   MSPM0 PA8 → OLED SDA (I2C数据)
 *   注意：需要4.7K上拉电阻
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#ifndef __OLED_SSD1306_MSPM0_H
#define __OLED_SSD1306_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

/* ── OLED配置 ──────────────────────────────────────────────── */
#define OLED_WIDTH      128     /* 宽度 */
#define OLED_HEIGHT     64      /* 高度 */
#define OLED_I2C_ADDR   0x3C    /* I2C地址 */

/* ── 命令定义 ──────────────────────────────────────────────── */
#define OLED_CMD        0       /* 命令模式 */
#define OLED_DATA       1       /* 数据模式 */

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化OLED
 * @param i2c  I2C实例 (如 I2C_0_INST)
 */
void OLED_Init(I2C_Regs *i2c);

/**
 * @brief 清屏
 */
void OLED_Clear(void);

/**
 * @brief 更新显存到OLED
 */
void OLED_Refresh(void);

/**
 * @brief 开启OLED显示
 */
void OLED_DisplayOn(void);

/**
 * @brief 关闭OLED显示
 */
void OLED_DisplayOff(void);

/**
 * @brief 反显模式
 * @param mode  0=正常, 1=反显
 */
void OLED_ColorTurn(uint8_t mode);

/**
 * @brief 屏幕旋转180度
 * @param mode  0=正常, 1=旋转
 */
void OLED_DisplayTurn(uint8_t mode);

/**
 * @brief 画点
 * @param x    x坐标 (0~127)
 * @param y    y坐标 (0~63)
 * @param dot  1=亮, 0=灭
 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t dot);

/**
 * @brief 画线
 * @param x1   起点x
 * @param y1   起点y
 * @param x2   终点x
 * @param y2   终点y
 * @param mode 1=亮, 0=灭
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode);

/**
 * @brief 画圆
 * @param x    圆心x
 * @param y    圆心y
 * @param r    半径
 */
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);

/**
 * @brief 显示字符
 * @param x      x坐标
 * @param y      y坐标
 * @param chr    字符
 * @param size1  字体大小 (6/8/12/16/24)
 * @param mode   1=正常, 0=反显
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode);

/**
 * @brief 显示字符串
 * @param x      x坐标
 * @param y      y坐标
 * @param str    字符串
 * @param size1  字体大小
 * @param mode   1=正常, 0=反显
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size1, uint8_t mode);

/**
 * @brief 显示数字
 * @param x      x坐标
 * @param y      y坐标
 * @param num    数字
 * @param len    位数
 * @param size1  字体大小
 * @param mode   1=正常, 0=反显
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode);

/**
 * @brief 显示浮点数
 * @param x      x坐标
 * @param y      y坐标
 * @param num    浮点数
 * @param len    整数位数
 * @param dec    小数位数
 * @param size1  字体大小
 * @param mode   1=正常, 0=反显
 */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t len, uint8_t dec, uint8_t size1, uint8_t mode);

/**
 * @brief 显示图片
 * @param x      x坐标
 * @param y      y坐标
 * @param sizex  宽度
 * @param sizey  高度
 * @param bmp    图片数据
 * @param mode   1=正常, 0=反显
 */
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, const uint8_t bmp[], uint8_t mode);

#endif /* __OLED_SSD1306_MSPM0_H */