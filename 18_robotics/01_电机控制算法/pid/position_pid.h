/*
 * 位置式PID控制器
 * 来源：Robocon 竞赛
 * 适配平台：MSPM0G3507
 * 
 * 设计思路：
 * 1. 位置式PID输出控制量的绝对值
 * 2. 计算公式：u(k) = Kp×e(k) + Ki×∑e(i) + Kd×(e(k)-e(k-1))
 * 3. 适用于需要精确定位的场合
 * 4. 需要注意积分饱和问题
 */

#ifndef POSITION_PID_H
#define POSITION_PID_H

#include <stdint.h>

// PID控制器结构体
typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    
    float error;            // 当前误差
    float error_last;       // 上次误差
    float error_sum;        // 误差累积（积分项）
    
    float output;           // 控制器输出
    
    float output_max;       // 输出上限
    float output_min;       // 输出下限
    
    float integral_max;     // 积分项上限（抗积分饱和）
    float integral_min;     // 积分项下限
    
    float dead_zone;        // 死区
    
    float target;           // 目标值
    float measured;         // 测量值
    
} PositionPID_HandleTypeDef;

// 函数声明
void PositionPID_Init(PositionPID_HandleTypeDef *hpid, float kp, float ki, float kd);
void PositionPID_SetOutputLimit(PositionPID_HandleTypeDef *hpid, float min, float max);
void PositionPID_SetIntegralLimit(PositionPID_HandleTypeDef *hpid, float min, float max);
void PositionPID_SetDeadZone(PositionPID_HandleTypeDef *hpid, float dead_zone);
float PositionPID_Compute(PositionPID_HandleTypeDef *hpid, float target, float measured);
void PositionPID_Reset(PositionPID_HandleTypeDef *hpid);
void PositionPID_SetTarget(PositionPID_HandleTypeDef *hpid, float target);

#endif // POSITION_PID_H
