/*
 * 位置式PID控制器实现
 * 来源：Robocon 竞赛
 * 适配平台：MSPM0G3507
 */

#include "position_pid.h"

/**
 * @brief 初始化位置式PID控制器
 * @param hpid PID句柄
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PositionPID_Init(PositionPID_HandleTypeDef *hpid, float kp, float ki, float kd)
{
    hpid->kp = kp;
    hpid->ki = ki;
    hpid->kd = kd;
    
    hpid->error = 0.0f;
    hpid->error_last = 0.0f;
    hpid->error_sum = 0.0f;
    
    hpid->output = 0.0f;
    
    hpid->output_max = 1000.0f;    // 默认输出上限
    hpid->output_min = -1000.0f;   // 默认输出下限
    
    hpid->integral_max = 500.0f;   // 默认积分项上限
    hpid->integral_min = -500.0f;  // 默认积分项下限
    
    hpid->dead_zone = 0.0f;        // 默认无死区
    
    hpid->target = 0.0f;
    hpid->measured = 0.0f;
}

/**
 * @brief 设置输出限幅
 * @param hpid PID句柄
 * @param min 输出下限
 * @param max 输出上限
 */
void PositionPID_SetOutputLimit(PositionPID_HandleTypeDef *hpid, float min, float max)
{
    hpid->output_min = min;
    hpid->output_max = max;
}

/**
 * @brief 设置积分项限幅（抗积分饱和）
 * @param hpid PID句柄
 * @param min 积分项下限
 * @param max 积分项上限
 */
void PositionPID_SetIntegralLimit(PositionPID_HandleTypeDef *hpid, float min, float max)
{
    hpid->integral_min = min;
    hpid->integral_max = max;
}

/**
 * @brief 设置死区
 * @param hpid PID句柄
 * @param dead_zone 死区大小
 */
void PositionPID_SetDeadZone(PositionPID_HandleTypeDef *hpid, float dead_zone)
{
    hpid->dead_zone = dead_zone;
}

/**
 * @brief 设置目标值
 * @param hpid PID句柄
 * @param target 目标值
 */
void PositionPID_SetTarget(PositionPID_HandleTypeDef *hpid, float target)
{
    hpid->target = target;
}

/**
 * @brief 计算位置式PID输出
 * @param hpid PID句柄
 * @param target 目标值
 * @param measured 测量值
 * @return PID输出
 */
float PositionPID_Compute(PositionPID_HandleTypeDef *hpid, float target, float measured)
{
    // 更新目标值和测量值
    hpid->target = target;
    hpid->measured = measured;
    
    // 计算误差
    hpid->error = hpid->target - hpid->measured;
    
    // 死区处理
    if (hpid->error > -hpid->dead_zone && hpid->error < hpid->dead_zone) {
        hpid->error = 0.0f;
    }
    
    // 积分项累加
    hpid->error_sum += hpid->error;
    
    // 积分限幅（抗积分饱和）
    if (hpid->error_sum > hpid->integral_max) {
        hpid->error_sum = hpid->integral_max;
    }
    if (hpid->error_sum < hpid->integral_min) {
        hpid->error_sum = hpid->integral_min;
    }
    
    // 位置式PID公式
    // u(k) = Kp×e(k) + Ki×∑e(i) + Kd×(e(k)-e(k-1))
    float p_term = hpid->kp * hpid->error;
    float i_term = hpid->ki * hpid->error_sum;
    float d_term = hpid->kd * (hpid->error - hpid->error_last);
    
    hpid->output = p_term + i_term + d_term;
    
    // 输出限幅
    if (hpid->output > hpid->output_max) {
        hpid->output = hpid->output_max;
    }
    if (hpid->output < hpid->output_min) {
        hpid->output = hpid->output_min;
    }
    
    // 更新历史误差
    hpid->error_last = hpid->error;
    
    return hpid->output;
}

/**
 * @brief 重置PID控制器
 * @param hpid PID句柄
 */
void PositionPID_Reset(PositionPID_HandleTypeDef *hpid)
{
    hpid->error = 0.0f;
    hpid->error_last = 0.0f;
    hpid->error_sum = 0.0f;
    
    hpid->output = 0.0f;
}
