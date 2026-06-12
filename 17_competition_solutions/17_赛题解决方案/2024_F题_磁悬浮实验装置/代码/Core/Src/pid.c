/**
 * @file    pid.c
 * @brief   PID控制器模块
 * @version 1.0
 * 
 * 实现增量式PID控制算法，支持：
 * - 积分限幅（防止积分饱和）
 * - 输出限幅（保护硬件）
 * - 死区控制（降低功耗）
 * - 微分滤波（减少噪声）
 */

#include "pid.h"
#include <math.h>

/* 默认PID参数 */
#define DEFAULT_KP          50.0f
#define DEFAULT_KI          0.5f
#define DEFAULT_KD          20.0f
#define DEFAULT_INT_MAX     1000.0f
#define DEFAULT_INT_MIN     -1000.0f
#define DEFAULT_OUT_MAX     4095.0f
#define DEFAULT_OUT_MIN     0.0f
#define DEFAULT_DEAD_ZONE   0.1f    // 死区范围(cm)

/**
 * @brief  PID控制器初始化
 * @param  pid: PID控制器指针
 * @retval 无
 */
void PID_Init(PID_Controller_t *pid)
{
    /* 设置默认参数 */
    pid->Kp = DEFAULT_KP;
    pid->Ki = DEFAULT_KI;
    pid->Kd = DEFAULT_KD;
    
    /* 初始化状态变量 */
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev2_error = 0.0f;
    pid->output = 0.0f;
    
    /* 设置限幅值 */
    pid->integral_max = DEFAULT_INT_MAX;
    pid->integral_min = DEFAULT_INT_MIN;
    pid->output_max = DEFAULT_OUT_MAX;
    pid->output_min = DEFAULT_OUT_MIN;
    
    /* 设置死区 */
    pid->dead_zone = DEFAULT_DEAD_ZONE;
}

/**
 * @brief  PID控制器复位
 * @param  pid: PID控制器指针
 * @retval 无
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev2_error = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  PID计算函数（增量式PID）
 * @param  pid: PID控制器指针
 * @param  setpoint: 设定值
 * @param  measured: 测量值
 * @retval 控制输出值
 * 
 * 增量式PID公式：
 * Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2*e(k-1)+e(k-2)]
 * u(k) = u(k-1) + Δu(k)
 */
float PID_Calculate(PID_Controller_t *pid, float setpoint, float measured)
{
    float error, delta_output;
    float p_term, i_term, d_term;
    
    /* 计算误差 */
    error = setpoint - measured;
    
    /* 死区判断：误差小于死区时不调节 */
    if (fabsf(error) < pid->dead_zone)
    {
        return pid->output;  // 保持上次输出
    }
    
    /* 计算比例项 */
    p_term = pid->Kp * (error - pid->prev_error);
    
    /* 计算积分项 */
    i_term = pid->Ki * error;
    
    /* 积分累加 */
    pid->integral += i_term;
    
    /* 积分限幅 */
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < pid->integral_min)
    {
        pid->integral = pid->integral_min;
    }
    
    /* 计算微分项（带滤波） */
    d_term = pid->Kd * (error - 2.0f * pid->prev_error + pid->prev2_error);
    
    /* 计算增量 */
    delta_output = p_term + i_term + d_term;
    
    /* 更新输出 */
    pid->output += delta_output;
    
    /* 输出限幅 */
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < pid->output_min)
    {
        pid->output = pid->output_min;
    }
    
    /* 更新历史误差 */
    pid->prev2_error = pid->prev_error;
    pid->prev_error = error;
    
    return pid->output;
}

/**
 * @brief  设置PID参数
 * @param  pid: PID控制器指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval 无
 */
void PID_SetParams(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    /* 参数变化时复位积分项 */
    pid->integral = 0.0f;
}
