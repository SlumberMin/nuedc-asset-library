/*
 * 增量式PID控制器
 * 来源：飞思卡尔智能车竞赛
 * 适配平台：MSPM0G3507
 * 
 * 设计思路：
 * 1. 增量式PID只输出控制量的增量，避免积分饱和
 * 2. 计算公式：Δu = Kp×(e(k)-e(k-1)) + Ki×e(k) + Kd×(e(k)-2e(k-1)+e(k-2))
 * 3. 适用于需要快速响应的场合
 */

#ifndef INCREMENTAL_PID_H
#define INCREMENTAL_PID_H

#include <stdint.h>

// PID控制器结构体
typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    
    float error;            // 当前误差
    float error_last;       // 上次误差
    float error_prev;       // 上上次误差
    
    float output;           // 控制器输出
    float output_last;      // 上次输出
    
    float output_max;       // 输出上限
    float output_min;       // 输出下限
    
    float dead_zone;        // 死区
    
} IncrementalPID_HandleTypeDef;

// 函数声明
void IncrementalPID_Init(IncrementalPID_HandleTypeDef *hpid, float kp, float ki, float kd);
void IncrementalPID_SetOutputLimit(IncrementalPID_HandleTypeDef *hpid, float min, float max);
void IncrementalPID_SetDeadZone(IncrementalPID_HandleTypeDef *hpid, float dead_zone);
float IncrementalPID_Compute(IncrementalPID_HandleTypeDef *hpid, float target, float measured);
void IncrementalPID_Reset(IncrementalPID_HandleTypeDef *hpid);

#endif // INCREMENTAL_PID_H
