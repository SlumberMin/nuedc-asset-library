/**
 * @file pid_reset_windup.c
 * @brief 积分重置抗饱和PID实现
 *
 * 积分重置公式推导:
 *   PID输出: u = kp*e + ki*∫e*dt + kd*de/dt
 *   当 u > out_max 时, 令积分项恰好使 u = out_max:
 *     ki*integral = out_max - kp*e - kd*de/dt
 *     integral = (out_max - P - D) / ki
 *
 * 与传统方法对比:
 *   - 条件积分: 饱和时停止积分, 退出饱和慢
 *   - Back-calculation: 需要额外增益Kb, 调参复杂
 *   - 积分重置: 直接计算最优积分值, 退出饱和最快
 */

#include "pid_reset_windup.h"
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

void pid_rw_init(pid_rst_windup_t *pid,
                 float kp, float ki, float kd,
                 float dt, float out_min, float out_max)
{
    memset(pid, 0, sizeof(pid_rst_windup_t));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = -1e6f;
    pid->integral_max =  1e6f;
    pid->mode = RESET_WINDUP_EXACT;
    pid->deadzone = 0.0f;
    pid->d_filter_alpha = 1.0f;  /* 默认不滤波 */
    pid->first_run = 1;
}

/* ==================== 参数设置 ==================== */

void pid_rw_set_mode(pid_rst_windup_t *pid, reset_windup_mode_t mode)
{
    pid->mode = mode;
}

void pid_rw_set_deadzone(pid_rst_windup_t *pid, float deadzone)
{
    pid->deadzone = (deadzone > 0.0f) ? deadzone : 0.0f;
}

void pid_rw_set_integral_limit(pid_rst_windup_t *pid, float lo, float hi)
{
    pid->integral_min = lo;
    pid->integral_max = hi;
}

void pid_rw_set_d_filter_alpha(pid_rst_windup_t *pid, float alpha)
{
    pid->d_filter_alpha = CLAMP(alpha, 0.01f, 1.0f);
}

/* ==================== 核心计算 ==================== */

float pid_rw_compute(pid_rst_windup_t *pid, float setpoint, float pv)
{
    float error, p_term, i_term, d_term_raw, d_term_filtered;
    float raw_output, final_output;

    pid->setpoint = setpoint;
    error = setpoint - pv;

    /* --- P项 --- */
    p_term = pid->kp * error;

    /* --- I项 (梯形积分) --- */
    pid->integral += 0.5f * (error + pid->prev_error) * pid->dt;

    /* 死区处理 */
    if (pid->mode == RESET_WINDUP_DEADZONE) {
        if (fabsf(error) < pid->deadzone) {
            pid->integral = 0.0f;
        }
    }

    /* 积分限幅(基础保护) */
    pid->integral = CLAMP(pid->integral, pid->integral_min, pid->integral_max);
    i_term = pid->ki * pid->integral;

    /* --- D项 --- */
    if (pid->first_run) {
        d_term_raw = 0.0f;
    } else {
        d_term_raw = pid->kd * (error - pid->prev_error) / pid->dt;
    }

    /* D项低通滤波 */
    d_term_filtered = pid->d_filter_alpha * d_term_raw
                    + (1.0f - pid->d_filter_alpha) * pid->prev_d_term;
    pid->prev_d_term = d_term_filtered;

    /* --- 计算原始输出 --- */
    raw_output = p_term + i_term + d_term_filtered;

    /* --- 抗饱和处理 --- */
    pid->saturated = 0;
    final_output = raw_output;

    if (raw_output > pid->out_max) {
        pid->saturated = 1;
        final_output = pid->out_max;

        if (pid->mode == RESET_WINDUP_EXACT && fabsf(pid->ki) > 1e-9f) {
            /* 精确重置: 将积分值回退到使输出恰好等于out_max */
            pid->integral = (pid->out_max - p_term - d_term_filtered) / pid->ki;
            pid->integral = CLAMP(pid->integral, pid->integral_min, pid->integral_max);
        } else if (pid->mode == RESET_WINDUP_CONDITIONAL) {
            /* 条件积分: 回退本次积分增量 */
            pid->integral -= 0.5f * (error + pid->prev_error) * pid->dt;
        }
    } else if (raw_output < pid->out_min) {
        pid->saturated = 1;
        final_output = pid->out_min;

        if (pid->mode == RESET_WINDUP_EXACT && fabsf(pid->ki) > 1e-9f) {
            /* 精确重置 */
            pid->integral = (pid->out_min - p_term - d_term_filtered) / pid->ki;
            pid->integral = CLAMP(pid->integral, pid->integral_min, pid->integral_max);
        } else if (pid->mode == RESET_WINDUP_CONDITIONAL) {
            /* 条件积分 */
            pid->integral -= 0.5f * (error + pid->prev_error) * pid->dt;
        }
    }

    /* 保存调试信息 */
    pid->last_p_term = p_term;
    pid->last_d_term = d_term_filtered;

    pid->prev_error = error;
    pid->first_run = 0;

    return final_output;
}

/* ==================== 重置 ==================== */

void pid_rw_reset(pid_rst_windup_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_d_term = 0.0f;
    pid->last_p_term = 0.0f;
    pid->last_d_term = 0.0f;
    pid->saturated = 0;
    pid->first_run = 1;
}

/* ==================== 查询 ==================== */

uint8_t pid_rw_is_saturated(const pid_rst_windup_t *pid)
{
    return pid->saturated;
}
