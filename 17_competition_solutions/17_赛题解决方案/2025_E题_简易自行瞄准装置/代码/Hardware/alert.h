/**
 * @file    alert.h
 * @brief   声光提示模块头文件
 */
#ifndef __ALERT_H
#define __ALERT_H

#include <stdint.h>

void Alert_Init(void);
void Alert_Beep(uint16_t ms);
void Alert_ShortBeep(void);
void Alert_Error(void);
void Alert_LED_On(uint8_t led);
void Alert_LED_Off(uint8_t led);

#endif /* __ALERT_H */
