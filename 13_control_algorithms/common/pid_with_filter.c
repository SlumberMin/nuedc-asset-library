/**
 * @file pid_with_filter.c
 * @brief PID+滤波器实现 - 微分低通滤波 + 输出低通滤波
 *
 * 滤波器设计原理:
 *   一阶IIR低通: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
 *   等价于: y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 *
 * 截止频率与alpha的关系:
 *   fc = -ln(1-alpha) / (2*pi*dt)
 *   alpha = 1 - exp(-2*pi*fc*dt)
 *
 * 常用值参考 (dt=1ms):
 *   alpha=0.1  → fc ≈ 16 Hz
 *   alpha=0.2  → fc ≈ 35 Hz
 *   alpha=0.3  → fc ≈ 56 Hz
 *   alpha=0.5  → fc ≈ 110 Hz
 */

#include "pid_with_filter.h"
#include <string.h>
#include <math.h>

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef CLAMP
#define CLAMP(x, lo, hi) MIN(MAX((x), (lo)), (hi))
#endif

/* ==================== 初始化 ==================== */

void pid_wf_init(pid_with_filter_t *pid,
                 float kp, float ki, float kd,
                 float dt, float out_min, float out_max)
{
    memset(pid, 0, sizeof(pid_with_filter_t));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = -1e6f;
    pid->integral_max =  1e6f;
    pid->d_filter_alpha = 0.2f;    /* 微分滤波: fc ≈ 35Hz (dt=1ms) */
    pid->out_filter_alpha = 0.3f;  /* 输出滤波: fc ≈ 56Hz (dt=1ms) */
    pid->filter_mode = PID_FILTER_BOTH;
    pid->d_source = PID_D_FROM_ERROR;
    pid->first_run = 1;
}

/* ==================== 参数设置 ==================== */

void pid_wf_set_filter_mode(pid_with_filter_t *pid, pid_filter_mode_t mode)
{
    pid->filter_mode = mode;
}

void pid_wf_set_d_filter_alpha(pid_with_filter_t *pid, float alpha)
{
    pid->d_filter_alpha = CLAMP(alpha, 0.01f, 1.0f);
}

void pid_wf_set_out_filter_alpha(pid_with_filter_t *pid, float alpha)
{
    pid->out_filter_alpha = CLAMP(alpha, 0.01f, 1.0f);
}

void pid_wf_set_d_source(pid_with_filter_t *pid, pid_d_source_t source)
{
    pid->d_source = source;
}

void pid_wf_set_integral_limit(pid_with_filter_t *pid, float int_min, float int_max)
{
    pid->integral_min = int_min;
    pid->integral_max = int_max;
}

/* ==================== 核心计算 ==================== */

float pid_wf_compute(pid_with_filter_t *pid, float setpoint, float pv)
{
    float error, p_term, i_term, d_term_raw, d_term_filtered;
    float raw_output, filtered_output;

    pid->setpoint = setpoint;
    error = setpoint - pv;

    /* --- P项 --- */
    p_term = pid->kp * error;

    /* --- I项 (梯形积分 + 积分限幅) --- */
    pid->integral += 0.5f * (error + pid->prev_error) * pid->dt;
    pid->integral = CLAMP(pid->integral, pid->integral_min, pid->integral_max);
    i_term = pid->ki * pid->integral;

    /* --- D项 --- */
    if (pid->d_source == PID_D_FROM_PV) {
        /* 对PV微分: 避免设定值突变导致D冲击 */
        if (pid->first_run) {
            d_term_raw = 0.0f;
        } else {
            d_term_raw = pid->kd * (-(pv - pid->prev_pv)) / pid->dt;
        }
        pid->prev_pv = pv;
    } else {
        /* 对误差微分 (默认) */
        if (pid->first_run) {
            d_term_raw = 0.0f;
        } else {
            d_term_raw = pid->kd * (error - pid->prev_error) / pid->dt;
        }
    }

    /* D项低通滤波 */
    if (pid->filter_mode & PID_FILTER_D_TERM) {
        d_term_filtered = pid->d_filter_alpha * d_term_raw
                        + (1.0f - pid->d_filter_alpha) * pid->prev_d_filtered;
    } else {
        d_term_filtered = d_term_raw;
    }
    pid->prev_d_filtered = d_term_filtered;

    /* --- 合成原始输出 --- */
    raw_output = p_term + i_term + d_term_filtered;

    /* --- 输出低通滤波 --- */
    if (pid->filter_mode & PID_FILTER_OUTPUT) {
        if (pid->first_run) {
            filtered_output = raw_output;
        } else {
            filtered_output = pid->out_filter_alpha * raw_output
                            + (1.0f - pid->out_filter_alpha) * pid->prev_out_filtered;
        }
    } else {
        filtered_output = raw_output;
    }
    pid->prev_out_filtered = filtered_output;

    /* --- 输出限幅 --- */
    filtered_output = CLAMP(filtered_output, pid->out_min, pid->out_max);

    pid->prev_error = error;
    pid->first_run = 0;

    return filtered_output;
}

/* ==================== 重置 ==================== */

void pid_wf_reset(pid_with_filter_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_pv = 0.0f;
    pid->prev_d_filtered = 0.0f;
    pid->prev_out_filtered = 0.0f;
    pid->first_run = 1;
}
