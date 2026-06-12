/**
 * @file    oled.h
 * @brief   OLED显示模块头文件
 * @version 1.0
 */

#ifndef __OLED_H
#define __OLED_H

#include "stm32f1xx_hal.h"

/* OLED I2C地址 */
#define OLED_ADDRESS        0x78

/* OLED尺寸 */
#define OLED_WIDTH          128
#define OLED_HEIGHT         64

/* 函数声明 */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_ShowChar(uint8_t x, uint8_t y, char ch);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t len);
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t *bmp);

#endif /* __OLED_H */
