/**
 * @file    pid_v2.c
 * @brief   改进版PID控制算法 v2.1 - 统一权威版本
 *
 * @version 2.1
 * @date    2026-06-11
 * @sync    与nuedc-asset-library/11_控制算法库/common/pid_full.c v2.0同步
 *
 * v2.1 优化记录：
 * [OPT-1] 前馈补偿
 * [OPT-2] 条件积分抗饱和
 * [OPT-3] 死区控制
 * [OPT-4] 自适应切换
 * [OPT-5] 增量式PID微分滤波
 * [OPT-6] 回算法抗饱和
 * [OPT-7] 内联Clamp
 * [OPT-8] dt参与微分计算
 * [OPT-9] 积分项加入dt
 * [OPT-10] 微分先行模式
 */

#include "pid_v2.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline float PID_V2_Clamp(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

void PID_V2_Init(PID_V2_t *pid, float kp, float ki, float kd,
                  float out_min, float out_max, float integral_max)
{
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f; pid->prev_error2 = 0.0f;
    pid->filtered_d = 0.0f; pid->prev_output = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->out_min = out_min; pid->out_max = out_max;
    pid->integral_max = integral_max;
    pid->d_filter_alpha = 0.3f;
    pid->dead_zone = 0.0f; pid->feedforward = 0.0f;
    pid->mode = PID_MODE_POSITION;
    pid->feature = PID_FEATURE_NORMAL;
    pid->dt = 0.01f;
    pid->auto_switch_threshold = 0.0f;
    pid->back_calc_kb = 0.5f;
    pid->anti_windup = PID_AW_CLAMP;
    pid->integral_sep_threshold = 100.0f;
    pid->output = 0.0f;
}

void PID_V2_SetSampleTime(PID_V2_t *pid, float dt)
{
    pid->dt = (dt > 0.0f) ? dt : 0.001f;
}

static float PID_V2_PositionCalculate(PID_V2_t *pid, float target, float actual)
{
    float error, p_term, i_term, d_term;
    float dt = pid->dt;

    error = target - actual;

    if (pid->dead_zone > 0.0f && fabsf(error) < pid->dead_zone) {
        return pid->output;
    }

    float ki = pid->Ki;
    if (pid->feature == PID_FEATURE_INTEGRAL_SEP) {
        if (fabsf(error) > pid->integral_sep_threshold) {
            ki = 0.0f;
        }
    }

    p_term = pid->Kp * error;

    bool should_integrate = true;
    if (pid->anti_windup == PID_AW_BACK_CALC) {
        float u_unsat = p_term + pid->Ki * pid->integral + pid->Kd * (error - pid->prev_error) / dt;
        float u_sat = PID_V2_Clamp(u_unsat, pid->out_min, pid->out_max);
        float aw_correction = pid->back_calc_kb * (u_sat - u_unsat);
        pid->integral += (error + aw_correction) * dt;
    } else {
        if (pid->output >= pid->out_max && error > 0) should_integrate = false;
        if (pid->output <= pid->out_min && error < 0) should_integrate = false;
        if (should_integrate) {
            pid->integral += error * dt;
        }
    }

    pid->integral = PID_V2_Clamp(pid->integral, -pid->integral_max, pid->integral_max);
    i_term = pid->Ki * pid->integral;

    if (pid->feature == PID_FEATURE_DERIVATIVE_LPF) {
        float measurement_diff = (actual - pid->prev_measurement) / dt;
        pid->filtered_d = pid->d_filter_alpha * (-pid->Kd * measurement_diff)
                        + (1.0f - pid->d_filter_alpha) * pid->filtered_d;
        pid->prev_measurement = actual;
    } else {
        float raw_derivative = (error - pid->prev_error) / dt;
        pid->filtered_d = pid->d_filter_alpha * raw_derivative
                        + (1.0f - pid->d_filter_alpha) * pid->filtered_d;
    }
    d_term = pid->Kd * pid->filtered_d;

    pid->prev_error = error;
    pid->output = PID_V2_Clamp(p_term + i_term + d_term + pid->feedforward,
                               pid->out_min, pid->out_max);

    return pid->output;
}

static float PID_V2_IncrementCalculate(PID_V2_t *pid, float target, float actual)
{
    float error = target - actual;
    float dt = pid->dt;

    if (pid->dead_zone > 0.0f && fabsf(error) < pid->dead_zone) {
        return pid->output;
    }

    float delta_p = pid->Kp * (error - pid->prev_error);
    float delta_i = pid->Ki * error * dt;
    float delta_d = pid->Kd * (error - 2.0f * pid->prev_error + pid->prev_error2) / dt;

    float raw_delta_d = delta_p + delta_i + delta_d;
    pid->filtered_d = pid->d_filter_alpha * raw_delta_d
                    + (1.0f - pid->d_filter_alpha) * pid->filtered_d;

    pid->output += pid->filtered_d;
    pid->output = PID_V2_Clamp(pid->output, pid->out_min, pid->out_max);

    pid->prev_error2 = pid->prev_error;
    pid->prev_error = error;

    return pid->output;
}

float PID_V2_Calculate(PID_V2_t *pid, float target, float actual)
{
    if (pid->mode == PID_MODE_AUTO_SWITCH && pid->auto_switch_threshold > 0.0f) {
        float error = target - actual;
        if (fabsf(error) > pid->auto_switch_threshold) {
            return PID_V2_IncrementCalculate(pid, target, actual);
        } else {
            return PID_V2_PositionCalculate(pid, target, actual);
        }
    }

    switch (pid->mode) {
        case PID_MODE_INCREMENTAL:
            return PID_V2_IncrementCalculate(pid, target, actual);
        case PID_MODE_POSITION:
        default:
            return PID_V2_PositionCalculate(pid, target, actual);
    }
}

void PID_V2_Reset(PID_V2_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f; pid->prev_error2 = 0.0f;
    pid->filtered_d = 0.0f; pid->prev_output = 0.0f;
    pid->prev_measurement = 0.0f; pid->output = 0.0f;
}

void PID_V2_SetParams(PID_V2_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
}

void PID_V2_SetFilterAlpha(PID_V2_t *pid, float alpha)
{
    pid->d_filter_alpha = PID_V2_Clamp(alpha, 0.01f, 1.0f);
}

void PID_V2_SetDeadZone(PID_V2_t *pid, float dead_zone)
{
    pid->dead_zone = (dead_zone < 0.0f) ? 0.0f : dead_zone;
}

void PID_V2_SetFeedforward(PID_V2_t *pid, float ff)
{
    pid->feedforward = ff;
}

void PID_V2_SetMode(PID_V2_t *pid, PID_V2_Mode_t mode, PID_V2_Feature_t feature)
{
    pid->mode = mode; pid->feature = feature;
}

void PID_V2_SetAutoSwitch(PID_V2_t *pid, float threshold)
{
    pid->auto_switch_threshold = threshold;
}

void PID_V2_SetAntiWindup(PID_V2_t *pid, PID_V2_AntiWindup_t type, float kb)
{
    pid->anti_windup = type; pid->back_calc_kb = kb;
}

void PID_V2_SetIntegralSeparation(PID_V2_t *pid, float threshold)
{
    pid->integral_sep_threshold = threshold;
}

#ifdef __cplusplus
}
#endif
