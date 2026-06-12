/**
 * @file    pid.c
 * @brief   PID控制算法实现（带微分滤波+条件积分抗饱和）
 */

#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_max = (out_max - out_min) * 0.5f;
    pid->d_filter_alpha = 0.3f;
    pid->output = 0.0f;
}

float PID_Calculate(PID_t *pid, float target, float actual)
{
    float error = target - actual;
    
    /* 条件积分抗饱和 */
    bool integrate = true;
    if(pid->output >= pid->out_max && error > 0) integrate = false;
    if(pid->output <= pid->out_min && error < 0) integrate = false;
    
    if(integrate)
    {
        pid->integral += error;
        if(pid->integral > pid->integral_max) pid->integral = pid->integral_max;
        if(pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    }
    
    /* 微分滤波 */
    float raw_d = error - pid->prev_error;
    pid->filtered_derivative = pid->d_filter_alpha * raw_d + 
                               (1.0f - pid->d_filter_alpha) * pid->filtered_derivative;
    
    /* PID输出 */
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * pid->filtered_derivative;
    
    /* 输出限幅 */
    if(pid->output > pid->out_max) pid->output = pid->out_max;
    if(pid->output < pid->out_min) pid->output = pid->out_min;
    
    pid->prev_error = error;
    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->output = 0.0f;
}

void PID_SetParams(PID_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
