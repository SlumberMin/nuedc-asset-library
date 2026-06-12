/**
 * @file    pid.c
 * @brief   PID控制器模块实现
 * @details 位置式和增量式PID实现，含微分滤波、条件积分抗饱和、死区、前馈。
 */

#include "algorithm/pid.h"
#include <string.h>

/* ========================================================================== */
/*                              接口函数实现                                   */
/* ========================================================================== */

ErrorCode_t PID_Init(PID_t *pid, PID_Mode_t mode,
                     float kp, float ki, float kd, float dt_s)
{
    if (pid == NULL) return HAL_ERR_PARAM;
    if (dt_s <= 0) return HAL_ERR_PARAM;

    memset(pid, 0, sizeof(PID_t));

    pid->mode = mode;
    pid->kp   = kp;
    pid->ki   = ki;
    pid->kd   = kd;
    pid->dt_s = dt_s;

    /* 默认输出限幅 */
    pid->output_min = -1000.0f;
    pid->output_max =  1000.0f;

    /* 默认积分限幅 */
    pid->integral_max = 500.0f;

    /* 默认微分滤波 */
    pid->derivative_filter_alpha = 0.1f;

    /* 默认无条件积分 */
    pid->conditional_integral = false;
    pid->output_saturation_threshold = 0.0f;

    /* 默认无死区 */
    pid->dead_zone = 0.0f;

    pid->last_tick    = HAL_GetTick();
    pid->initialized  = true;

    DBG_PRINTF("PID init: mode=%d, Kp=%.3f, Ki=%.3f, Kd=%.3f, dt=%.3fs",
               mode, kp, ki, kd, dt_s);

    return HAL_OK_CODE;
}

ErrorCode_t PID_SetParams(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL || !pid->initialized) return HAL_ERR_NOT_INIT;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    return HAL_OK_CODE;
}

ErrorCode_t PID_SetTarget(PID_t *pid, float target)
{
    if (pid == NULL || !pid->initialized) return HAL_ERR_NOT_INIT;

    pid->target = target;

    return HAL_OK_CODE;
}

float PID_GetTarget(const PID_t *pid)
{
    if (pid == NULL) return 0.0f;
    return pid->target;
}

ErrorCode_t PID_SetOutputLimit(PID_t *pid, float min, float max)
{
    if (pid == NULL || !pid->initialized) return HAL_ERR_NOT_INIT;
    if (min >= max) return HAL_ERR_PARAM;

    pid->output_min = min;
    pid->output_max = max;

    return HAL_OK_CODE;
}

ErrorCode_t PID_SetIntegralLimit(PID_t *pid, float max_value)
{
    if (pid == NULL || !pid->initialized) return HAL_ERR_NOT_INIT;

    pid->integral_max = max_value;

    return HAL_OK_CODE;
}

ErrorCode_t PID_SetDeadZone(PID_t *pid, float dead_zone)
{
    if (pid == NULL || !pid->initialized) return HAL_ERR_NOT_INIT;

    pid->dead_zone = dead_zone;

    return HAL_OK_CODE;
}

ErrorCode_t PID_SetDerivativeFilter(PID_t *pid, float alpha)
{
    if (pid == NULL || !pid->initialized) return HAL_ERR_NOT_INIT;

    pid->derivative_filter_alpha = CLAMP(alpha, 0.0f, 1.0f);

    return HAL_OK_CODE;
}

ErrorCode_t PID_SetConditionalIntegral(PID_t *pid, bool enable, float threshold)
{
    if (pid == NULL || !pid->initialized) return HAL_ERR_NOT_INIT;

    pid->conditional_integral          = enable;
    pid->output_saturation_threshold   = threshold;

    return HAL_OK_CODE;
}

ErrorCode_t PID_SetFeedforward(PID_t *pid, float feedforward)
{
    if (pid == NULL || !pid->initialized) return HAL_ERR_NOT_INIT;

    pid->feedforward = feedforward;

    return HAL_OK_CODE;
}

float PID_Calculate(PID_t *pid, float feedback)
{
    if (pid == NULL || !pid->initialized) {
        return 0.0f;
    }

    /* 计算误差 */
    pid->error = pid->target - feedback;

    /* 死区处理 */
    if (ABS(pid->error) < pid->dead_zone) {
        pid->error = 0.0f;
    }

    float output = 0.0f;

    if (pid->mode == PID_MODE_POSITION) {
        /* ==================== 位置式PID ==================== */
        /*
         * output = Kp * e + Ki * ∫e*dt + Kd * de/dt + FF
         */

        /* 微分项（原始值） */
        float raw_derivative = (pid->error - pid->error_prev) / pid->dt_s;

        /* 微分滤波：一阶低通 */
        float alpha = pid->derivative_filter_alpha;
        pid->derivative = alpha * raw_derivative + (1.0f - alpha) * pid->derivative_prev;
        pid->derivative_prev = pid->derivative;

        /* 条件积分抗饱和 */
        bool should_integrate = true;
        if (pid->conditional_integral) {
            /* 计算不含积分的输出 */
            float p_out = pid->kp * pid->error;
            float d_out = pid->kd * pid->derivative;
            float test_output = p_out + pid->integral + d_out;

            /* 如果输出已饱和，且积分会加剧饱和，停止积分 */
            if ((test_output >= (pid->output_max - pid->output_saturation_threshold) && pid->error > 0) ||
                (test_output <= (pid->output_min + pid->output_saturation_threshold) && pid->error < 0)) {
                should_integrate = false;
            }
        }

        /* 积分累积 */
        if (should_integrate) {
            pid->integral += pid->ki * pid->error * pid->dt_s;
        }

        /* 积分限幅 */
        pid->integral = CLAMP(pid->integral, -pid->integral_max, pid->integral_max);

        /* 合成输出 */
        output = pid->kp * pid->error + pid->integral + pid->kd * pid->derivative + pid->feedforward;

    } else {
        /* ==================== 增量式PID ==================== */
        /*
         * Δu = Kp*(e(k)-e(k-1)) + Ki*e(k)*dt + Kd*(e(k)-2*e(k-1)+e(k-2)) / dt
         * output(k) = output(k-1) + Δu
         */
        float delta_error   = pid->error - pid->error_prev;
        float delta2_error  = pid->error - 2.0f * pid->error_prev + pid->error_prev2;

        /* 微分滤波 */
        float raw_derivative = delta2_error / pid->dt_s;
        float alpha = pid->derivative_filter_alpha;
        pid->derivative = alpha * raw_derivative + (1.0f - alpha) * pid->derivative_prev;
        pid->derivative_prev = pid->derivative;

        float delta_u = pid->kp * delta_error
                       + pid->ki * pid->error * pid->dt_s
                       + pid->kd * pid->derivative;

        output = pid->output + delta_u + pid->feedforward;
    }

    /* 输出限幅 */
    output = CLAMP(output, pid->output_min, pid->output_max);
    pid->output = output;

    /* 更新历史误差 */
    pid->error_prev2 = pid->error_prev;
    pid->error_prev  = pid->error;

    return output;
}

ErrorCode_t PID_Reset(PID_t *pid)
{
    if (pid == NULL) return HAL_ERR_PARAM;

    pid->error           = 0.0f;
    pid->error_prev      = 0.0f;
    pid->error_prev2     = 0.0f;
    pid->integral        = 0.0f;
    pid->derivative      = 0.0f;
    pid->derivative_prev = 0.0f;
    pid->output          = 0.0f;
    pid->last_tick       = HAL_GetTick();

    return HAL_OK_CODE;
}

float PID_GetError(const PID_t *pid)
{
    if (pid == NULL) return 0.0f;
    return pid->error;
}

float PID_GetOutput(const PID_t *pid)
{
    if (pid == NULL) return 0.0f;
    return pid->output;
}
