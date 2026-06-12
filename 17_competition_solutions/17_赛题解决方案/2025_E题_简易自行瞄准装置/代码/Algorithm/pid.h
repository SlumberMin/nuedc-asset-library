/**
 * @file    pid.h
 * @brief   PID控制算法模块头文件
 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* PID控制器结构体 */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float integral;     // 积分累积
    float prev_error;   // 上一次误差
    float out_min;      // 输出下限
    float out_max;      // 输出上限
    float output;       // 当前输出
} PID_Controller_t;

/* 函数声明 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, 
              float out_min, float out_max);
float PID_Calculate(PID_Controller_t *pid, float target, float actual);
void PID_Reset(PID_Controller_t *pid);
void PID_SetParams(PID_Controller_t *pid, float kp, float ki, float kd);

#endif /* __PID_H */
