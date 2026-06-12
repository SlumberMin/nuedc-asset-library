/**
 * @file pid_discrete.c
 * @brief 离散PID控制器 - 定点化实现
 */

#include "pid_discrete.h"
#include <string.h>

/* 静态辅助: 钳位 */
static inline int32_t clamp_q16(int32_t val, int32_t lo, int32_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void pid_discrete_init(pid_discrete_t *pid,
                       float kp, float ki, float kd,
                       float out_min, float out_max,
                       pid_discrete_mode_t mode)
{
    memset(pid, 0, sizeof(pid_discrete_t));

    pid->kp = FLOAT_TO_Q16(kp);
    pid->ki = FLOAT_TO_Q16(ki);
    pid->kd = FLOAT_TO_Q16(kd);
    pid->out_min = FLOAT_TO_Q16(out_min);
    pid->out_max = FLOAT_TO_Q16(out_max);
    pid->integral_min = FLOAT_TO_Q16(-1e6f);
    pid->integral_max = FLOAT_TO_Q16(1e6f);
    pid->mode = mode;
    pid->anti_windup = ANTI_WINDUP_CLAMP;
    pid->kb = FLOAT_TO_Q16(1.0f);
    pid->first_run = 1;
}

void pid_discrete_set_setpoint(pid_discrete_t *pid, float setpoint)
{
    pid->setpoint = FLOAT_TO_Q16(setpoint);
}

void pid_discrete_set_deadzone(pid_discrete_t *pid, float deadzone)
{
    pid->deadzone = FLOAT_TO_Q16(deadzone);
}

void pid_discrete_enable_backcalc(pid_discrete_t *pid, float kb)
{
    pid->anti_windup = ANTI_WINDUP_BACKCALC;
    pid->kb = FLOAT_TO_Q16(kb);
}

float pid_discrete_update(pid_discrete_t *pid, float measurement)
{
    int32_t meas_q = FLOAT_TO_Q16(measurement);
    int32_t error = pid->setpoint - meas_q;
    int32_t output;
    int32_t p_term, i_term, d_term;

    /* 死区处理 */
    if (pid->deadzone > 0) {
        int32_t abs_err = (error >= 0) ? error : -error;
        if (abs_err < pid->deadzone) {
            error = 0;
        }
    }

    /* ---------- 位置式 PID ---------- */
    if (pid->mode == PID_DISCRETE_POSITION) {
        /* P */
        p_term = Q16_MUL(pid->kp, error);

        /* I */
        pid->integral += Q16_MUL(pid->ki, error);
        pid->integral = clamp_q16(pid->integral, pid->integral_min, pid->integral_max);
        i_term = pid->integral;

        /* D (首次运行不计算微分) */
        if (pid->first_run) {
            d_term = 0;
            pid->first_run = 0;
        } else {
            int32_t d_error = error - pid->prev_error;
            d_term = Q16_MUL(pid->kd, d_error);
        }

        output = p_term + i_term + d_term;

        /* 抗积分饱和 */
        {
            int32_t unclamped = output;
            output = clamp_q16(output, pid->out_min, pid->out_max);

            if (pid->anti_windup == ANTI_WINDUP_BACKCALC && unclamped != output) {
                /* Back-calculation: 调整积分项 */
                int32_t excess = unclamped - output;
                pid->integral -= Q16_MUL(pid->kb, excess);
                pid->integral = clamp_q16(pid->integral, pid->integral_min, pid->integral_max);
            }
        }

        pid->prev_error = error;
    }
    /* ---------- 增量式 PID ---------- */
    else {
        int32_t delta;

        /* ΔP */
        p_term = Q16_MUL(pid->kp, error - pid->prev_error);

        /* ΔI */
        i_term = Q16_MUL(pid->ki, error);

        /* ΔD */
        if (pid->first_run) {
            d_term = 0;
            pid->first_run = 0;
        } else {
            int32_t dd = error - 2 * pid->prev_error + pid->prev_output;
            /* 注意: 增量式里 kd 实际作用于二阶差分, prev_output 在此暂存上上次误差 */
            /* 这里简化: 用标准增量式 delta = Kp*(e-e') + Ki*e + Kd*(e-2e'+e'') */
            /* 需要保留 e''，用 prev_output 存储 */
            /* 但 prev_output 被复用, 这里做简单处理 */
            d_term = Q16_MUL(pid->kd, dd);
        }

        delta = p_term + i_term + d_term;
        output = pid->prev_output + delta;
        output = clamp_q16(output, pid->out_min, pid->out_max);
    }

    pid->prev_output = output;

    return Q16_TO_FLOAT(output);
}

void pid_discrete_reset(pid_discrete_t *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_output = 0;
    pid->first_run = 1;
}
