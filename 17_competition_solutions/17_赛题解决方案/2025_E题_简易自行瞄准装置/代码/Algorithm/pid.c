/**
 * @file    pid.c
 * @brief   PID控制算法模块实现
 * 
 * 位置式PID算法：
 if (dt <= 0.0f) dt = 0.001f;  /* V2审计: 防除零 */
 * output = Kp*e + Ki*∫e*dt + Kd*de/dt
 * 
 * 增量式PID算法：
 * Δu = Kp*(e- e_) + Ki*e + Kd*(e - 2e_ + e__)
 * 
 * 本模块提供两种PID实现，可根据实际需求选择
 */

#include "pid.h"
#include <math.h>

/**
 * @brief  初始化PID控制器
 * @param  pid: PID控制器指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @param  out_min: 输出下限
 * @param  out_max: 输出上限
 * @retval 无
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, 
              float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->output = 0.0f;
}

/**
 * @brief  位置式PID计算
 * @param  pid: PID控制器指针
 * @param  target: 目标值
 * @param  actual: 实际值
 * @retval float: PID输出值
 * 
 * 计算步骤：
 * 1. error = target - actual
 * 2. integral += error (带抗积分饱和)
 * 3. derivative = error - prev_error
 * 4. output = Kp*error + Ki*integral + Kd*derivative
 * 5. 输出限幅
 */
float PID_Calculate(PID_Controller_t *pid, float target, float actual)
{
    float error, derivative;
    
    /* 1. 计算误差 */
    error = target - actual;
    
    /* 2. 积分累积（带抗积分饱和） */
    pid->integral += error;
    
    /* 抗积分饱和：限制积分累积范围 */
    if(pid->integral > pid->out_max) pid->integral = pid->out_max;
    if(pid->integral < pid->out_min) pid->integral = pid->out_min;
    
    /* 3. 微分计算 */
    derivative = error - pid->prev_error;
    
    /* 4. PID输出 */
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    /* 5. 输出限幅 */
    if(pid->output > pid->out_max) pid->output = pid->out_max;
    if(pid->output < pid->out_min) pid->output = pid->out_min;
    
    /* 6. 保存当前误差 */
    pid->prev_error = error;
    
    return pid->output;
}

/**
 * @brief  重置PID控制器
 * @param  pid: PID控制器指针
 * @retval 无
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  动态修改PID参数
 * @param  pid: PID控制器指针
 * @param  kp: 新的比例系数
 * @param  ki: 新的积分系数
 * @param  kd: 新的微分系数
 * @retval 无
 */
void PID_SetParams(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
