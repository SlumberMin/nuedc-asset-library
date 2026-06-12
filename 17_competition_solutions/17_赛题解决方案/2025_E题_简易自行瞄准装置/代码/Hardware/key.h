/**
 * @file    key.h
 * @brief   按键模块头文件
 */
#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

#define KEY1_PRESS  1
#define KEY2_PRESS  2
#define KEY3_PRESS  3
#define KEY4_PRESS  4
#define KEY_NONE    0

void Key_Init(void);
uint8_t Key_Scan(void);

#endif /* __KEY_H */
