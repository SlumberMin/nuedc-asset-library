/*
 * 增量式PID控制器实现
 * 来源：飞思卡尔智能车竞赛
 * 适配平台：MSPM0G3507
 */

#include "incremental_pid.h"

/**
 * @brief 初始化增量式PID控制器
 * @param hpid PID句柄
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void IncrementalPID_Init(IncrementalPID_HandleTypeDef *hpid, float kp, float ki, float kd)
{
    hpid->kp = kp;
    hpid->ki = ki;
    hpid->kd = kd;
    
    hpid->error = 0.0f;
    hpid->error_last = 0.0f;
    hpid->error_prev = 0.0f;
    
    hpid->output = 0.0f;
    hpid->output_last = 0.0f;
    
    hpid->output_max = 1000.0f;    // 默认输出上限
    hpid->output_min = -1000.0f;   // 默认输出下限
    
    hpid->dead_zone = 0.0f;        // 默认无死区
}

/**
 * @brief 设置输出限幅
 * @param hpid PID句柄
 * @param min 输出下限
 * @param max 输出上限
 */
void IncrementalPID_SetOutputLimit(IncrementalPID_HandleTypeDef *hpid, float min, float max)
{
    hpid->output_min = min;
    hpid->output_max = max;
}

/**
 * @brief 设置死区
 * @param hpid PID句柄
 * @param dead_zone 死区大小
 */
void IncrementalPID_SetDeadZone(IncrementalPID_HandleTypeDef *hpid, float dead_zone)
{
    hpid->dead_zone = dead_zone;
}

/**
 * @brief 计算增量式PID输出
 * @param hpid PID句柄
 * @param target 目标值
 * @param measured 测量值
 * @return PID输出
 */
float IncrementalPID_Compute(IncrementalPID_HandleTypeDef *hpid, float target, float measured)
{
    // 计算误差
    hpid->error = target - measured;
    
    // 死区处理
    if (hpid->error > -hpid->dead_zone && hpid->error < hpid->dead_zone) {
        hpid->error = 0.0f;
    }
    
    // 增量式PID公式
    // Δu = Kp×(e(k)-e(k-1)) + Ki×e(k) + Kd×(e(k)-2e(k-1)+e(k-2))
    float delta_output = hpid->kp * (hpid->error - hpid->error_last)
                       + hpid->ki * hpid->error
                       + hpid->kd * (hpid->error - 2.0f*hpid->error_last + hpid->error_prev);
    
    // 更新输出
    hpid->output = hpid->output_last + delta_output;
    
    // 输出限幅
    if (hpid->output > hpid->output_max) {
        hpid->output = hpid->output_max;
    }
    if (hpid->output < hpid->output_min) {
        hpid->output = hpid->output_min;
    }
    
    // 更新历史误差
    hpid->error_prev = hpid->error_last;
    hpid->error_last = hpid->error;
    hpid->output_last = hpid->output;
    
    return hpid->output;
}

/**
 * @brief 重置PID控制器
 * @param hpid PID句柄
 */
void IncrementalPID_Reset(IncrementalPID_HandleTypeDef *hpid)
{
    hpid->error = 0.0f;
    hpid->error_last = 0.0f;
    hpid->error_prev = 0.0f;
    
    hpid->output = 0.0f;
    hpid->output_last = 0.0f;
}
