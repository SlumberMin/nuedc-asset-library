/**
 * @file    key.h
 * @brief   按键模块头文件 - 磁悬浮实验装置
 * 
 * 引脚定义：PB12-PB15（4个按键）
 */
#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define KEY1_PRESS  1
#define KEY2_PRESS  2
#define KEY3_PRESS  3
#define KEY4_PRESS  4
#define KEY_NONE    0

void Key_Init(void);
uint8_t Key_Scan(void);

#endif /* __KEY_H */
