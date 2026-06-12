/*
 * 串级PID控制器
 * 来源：RoboMaster 步兵机器人
 * 适配平台：MSPM0G3507
 * 
 * 设计思路：
 * 1. 外环（位置环）输出作为内环（速度环）的输入
 * 2. 内环响应快，外环精度高
 * 3. 适用于需要精确位置控制的场合（如机械臂、云台）
 */

#ifndef CASCADE_PID_H
#define CASCADE_PID_H

#include <stdint.h>
#include "position_pid.h"
#include "incremental_pid.h"

// 串级PID控制器结构体
typedef struct {
    // 外环（位置环）
    PositionPID_HandleTypeDef outer_pid;
    
    // 内环（速度环）
    IncrementalPID_HandleTypeDef inner_pid;
    
    // 目标值
    float target_position;
    float target_velocity;
    
    // 测量值
    float measured_position;
    float measured_velocity;
    
    // 输出
    float output;
    
} CascadePID_HandleTypeDef;

// 函数声明
void CascadePID_Init(CascadePID_HandleTypeDef *hpid,
                     float outer_kp, float outer_ki, float outer_kd,
                     float inner_kp, float inner_ki, float inner_kd);
void CascadePID_SetTarget(CascadePID_HandleTypeDef *hpid, float position);
void CascadePID_SetMeasurement(CascadePID_HandleTypeDef *hpid, float position, float velocity);
float CascadePID_Compute(CascadePID_HandleTypeDef *hpid);
void CascadePID_Reset(CascadePID_HandleTypeDef *hpid);

#endif // CASCADE_PID_H
