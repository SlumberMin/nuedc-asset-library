/**
 * @file incremental_pid.c
 * @brief 增量式PID控制器实现（优化版）
 *
 * 增量式PID公式:
 *   Δu = Kp*(e[k]-e[k-1]) + Ki*e[k]*dt + Kd*(e[k]-2*e[k-1]+e[k-2])/dt
 *   u[k] = u[k-1] + Δu
 *
 * 优化点:
 *   1. 增量限幅防止突变
 *   2. 微分项一阶低通滤波
 *   3. 死区处理
 *   4. 输出限幅
 */
#include "incremental_pid.h"
#include <math.h>
#include <string.h>

#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

void IncrPID_Init(IncrPID_t *pid, const IncrPID_Config_t *config)
{
    if (pid == NULL || config == NULL) return;
    memset(pid, 0, sizeof(IncrPID_t));
    memcpy(&pid->config, config, sizeof(IncrPID_Config_t));
    pid->initialized = 1;
}

float IncrPID_Calc(IncrPID_t *pid, float setpoint, float feedback, float dt)
{
    return IncrPID_CalcError(pid, setpoint - feedback, dt);
}

float IncrPID_CalcError(IncrPID_t *pid, float error, float dt)
{
    if (!pid->initialized || dt <= 0.0f) {
        return pid->output;
    }

    /* 死区 */
    if (fabsf(error) < pid->config.deadband) {
        error = 0.0f;
    }

    /* 比例增量: Kp * (e[k] - e[k-1]) */
    float delta_p = pid->config.Kp * (error - pid->prev_error);

    /* 积分增量: Ki * e[k] * dt */
    float delta_i = pid->config.Ki * error * dt;

    /* 微分增量: Kd * (e[k] - 2*e[k-1] + e[k-2]) / dt */
    float d_raw = pid->config.Kd * (error - 2.0f * pid->prev_error + pid->prev_prev_error) / dt;

    /* 微分滤波 */
    float alpha = pid->config.d_filter_alpha;
    float d_filtered = alpha * pid->prev_d_filtered + (1.0f - alpha) * d_raw;
    float delta_d = d_filtered;

    /* 总增量 */
    float delta_u = delta_p + delta_i + delta_d;

    /* 增量限幅 */
    delta_u = CLAMP(delta_u, -pid->config.delta_max, pid->config.delta_max);

    /* 更新输出 */
    pid->output += delta_u;

    /* 输出限幅 */
    pid->output = CLAMP(pid->output, pid->config.output_min, pid->config.output_max);

    /* 更新历史 */
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    pid->prev_d_filtered = d_filtered;

    return pid->output;
}

void IncrPID_SetGain(IncrPID_t *pid, const IncrPID_Config_t *config)
{
    if (pid == NULL || config == NULL) return;
    memcpy(&pid->config, config, sizeof(IncrPID_Config_t));
}

void IncrPID_Reset(IncrPID_t *pid)
{
    if (pid == NULL) return;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->prev_d_filtered = 0.0f;
    pid->output = 0.0f;
}
