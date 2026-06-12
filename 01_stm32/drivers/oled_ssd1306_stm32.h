/**
 * @file    oled_ssd1306_stm32.h
 * @brief   SSD1306 OLED显示驱动 — STM32 HAL库版本 (I2C)
 *
 * 硬件连接:
 *   I2C1: PB6=SCL, PB7=SDA
 *   SSD1306 I2C地址: 0x3C
 */

#ifndef __OLED_SSD1306_STM32_H
#define __OLED_SSD1306_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_ADDR   0x3C

/**
 * @brief 初始化OLED
 * @param hi2c I2C句柄指针 (I2C1, PB6=SCL, PB7=SDA)
 */
void OLED_Init(I2C_HandleTypeDef *hi2c);

void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t size, uint8_t mode);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t len, uint8_t dec, uint8_t size, uint8_t mode);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t dot);

#endif /* __OLED_SSD1306_STM32_H */
