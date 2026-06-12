/**
 * @file    pid.h
 * @brief   PID控制器模块头文件
 * @version 1.0
 */

#ifndef __PID_H
#define __PID_H

#include "stm32f1xx_hal.h"

/* PID参数结构体 */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float setpoint;     // 设定值
    float integral;     // 积分累加值
    float prev_error;   // 上次误差
    float prev2_error;  // 上上次误差
    float output;       // 输出值
    float integral_max; // 积分限幅上限
    float integral_min; // 积分限幅下限
    float output_max;   // 输出限幅上限
    float output_min;   // 输出限幅下限
    float dead_zone;    // 死区范围
} PID_Controller_t;

/* 函数声明 */
void PID_Init(PID_Controller_t *pid);
void PID_Reset(PID_Controller_t *pid);
float PID_Calculate(PID_Controller_t *pid, float setpoint, float measured);
void PID_SetParams(PID_Controller_t *pid, float kp, float ki, float kd);

#endif /* __PID_H */
