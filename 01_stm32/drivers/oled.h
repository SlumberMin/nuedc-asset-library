/**
 * @file    oled.h
 * @brief   OLED显示模块 — STM32电赛通用代码库
 * @details 支持SSD1306 128×64 OLED显示屏，I2C接口。
 *          功能：字符显示、数字显示、清屏、画点、画线、矩形。
 * @author  电赛通用代码库
 * @version 1.0
 * @date    2026-06
 */

#ifndef __OLED_H
#define __OLED_H

#include "platform/hal_stm32.h"

/* ========================================================================== */
/*                              常量定义                                       */
/* ========================================================================== */

/** @brief OLED I2C地址（7位地址，SSD1306默认0x3C） */
#define OLED_I2C_ADDR       0x3C

/** @brief 屏幕宽度(像素) */
#define OLED_WIDTH          128

/** @brief 屏幕高度(像素) */
#define OLED_HEIGHT         64

/** @brief 每行最大字符数(6×8字体) */
#define OLED_MAX_CHAR_PER_LINE  21

/** @brief 最大行数(6×8字体) */
#define OLED_MAX_LINES          8

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/**
 * @brief OLED显示配置结构体
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;        /**< I2C句柄 */
    uint16_t           i2c_addr;    /**< I2C设备地址(7位) */
    bool               initialized; /**< 是否已初始化 */
} OLED_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化OLED显示屏
 * @param oled     OLED结构体指针
 * @param hi2c     I2C句柄(需已在CubeMX中配置好)
 * @param addr     I2C地址，通常OLED_I2C_ADDR(0x3C)
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @note   I2C速率建议400kHz(Fast Mode)
 */
ErrorCode_t OLED_Init(OLED_t *oled, I2C_HandleTypeDef *hi2c, uint16_t addr);

/**
 * @brief 清屏（全屏填充黑色）
 * @param oled  OLED结构体指针
 * @return ErrorCode_t
 */
ErrorCode_t OLED_Clear(OLED_t *oled);

/**
 * @brief 设置光标位置（字符坐标，6×8字体）
 * @param oled  OLED结构体指针
 * @param x     列坐标(0~20)
 * @param y     行坐标(0~7)
 * @return ErrorCode_t
 */
ErrorCode_t OLED_SetCursor(OLED_t *oled, uint8_t x, uint8_t y);

/**
 * @brief 显示单个字符
 * @param oled  OLED结构体指针
 * @param ch    要显示的字符(ASCII 32~126)
 * @param x     起始列(像素坐标0~122)
 * @param y     起始行(像素坐标0~56, 8的倍数)
 * @return ErrorCode_t
 */
ErrorCode_t OLED_ShowChar(OLED_t *oled, char ch, uint8_t x, uint8_t y);

/**
 * @brief 显示字符串
 * @param oled  OLED结构体指针
 * @param str   字符串指针
 * @param x     起始列(字符坐标0~20)
 * @param y     起始行(字符坐标0~7)
 * @return ErrorCode_t
 * @note   超出屏幕宽度自动截断
 */
ErrorCode_t OLED_ShowString(OLED_t *oled, const char *str, uint8_t x, uint8_t y);

/**
 * @brief 显示整数
 * @param oled  OLED结构体指针
 * @param num   要显示的整数
 * @param x     起始列(字符坐标)
 * @param y     起始行(字符坐标)
 * @return ErrorCode_t
 */
ErrorCode_t OLED_ShowInt(OLED_t *oled, int32_t num, uint8_t x, uint8_t y);

/**
 * @brief 显示浮点数
 * @param oled     OLED结构体指针
 * @param num      要显示的浮点数
 * @param decimals 小数位数(0~4)
 * @param x        起始列(字符坐标)
 * @param y        起始行(字符坐标)
 * @return ErrorCode_t
 */
ErrorCode_t OLED_ShowFloat(OLED_t *oled, float num, uint8_t decimals, uint8_t x, uint8_t y);

/**
 * @brief 画点
 * @param oled  OLED结构体指针
 * @param x     像素坐标(0~127)
 * @param y     像素坐标(0~63)
 * @return ErrorCode_t
 */
ErrorCode_t OLED_DrawPoint(OLED_t *oled, uint8_t x, uint8_t y);

/**
 * @brief 画线（Bresenham算法）
 * @param oled  OLED结构体指针
 * @param x1,y1 起点坐标
 * @param x2,y2 终点坐标
 * @return ErrorCode_t
 */
ErrorCode_t OLED_DrawLine(OLED_t *oled, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);

/**
 * @brief 画矩形
 * @param oled  OLED结构体指针
 * @param x,y   左上角坐标
 * @param w,h   宽度和高度
 * @return ErrorCode_t
 */
ErrorCode_t OLED_DrawRect(OLED_t *oled, uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/**
 * @brief 刷新显示（将缓冲区数据发送到OLED）
 * @param oled  OLED结构体指针
 * @return ErrorCode_t
 * @note   Show系列函数会自动调用此函数
 */
ErrorCode_t OLED_Refresh(OLED_t *oled);

#endif /* __OLED_H */
