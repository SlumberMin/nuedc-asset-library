/**
 * @file    key.h
 * @brief   按键模块头文件
 * @author  电赛团队
 * @date    2024
 * @note    模式选择按键(PA4) + 启动按键(PA5)
 */

#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx_hal.h"
#include "user_config.h"

/* ========================================================================== */
/*                              按键状态定义                                    */
/* ========================================================================== */

typedef enum {
    KEY_NONE = 0,       /* 无按键按下 */
    KEY_MODE,           /* 模式选择按键 */
    KEY_START           /* 启动按键 */
} KeyID_t;

/* ========================================================================== */
/*                              函数声明                                       */
/* ========================================================================== */

/**
 * @brief  按键模块初始化
 * @retval None
 */
void Key_Init(void);

/**
 * @brief  按键扫描（带消抖）
 * @note   在主循环中调用，10ms调用一次
 * @retval KeyID_t 返回按下的按键ID，无按键返回KEY_NONE
 */
KeyID_t Key_Scan(void);

/**
 * @brief  获取当前模式
 * @retval RunMode_t 当前选择的运行模式
 */
RunMode_t Key_GetMode(void);

/**
 * @brief  切换到下一个模式
 * @retval RunMode_t 切换后的模式
 */
RunMode_t Key_NextMode(void);

#endif /* __KEY_H */
