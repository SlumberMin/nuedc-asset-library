/**
 * @file pid_gain_scheduling.c
 * @brief 增益调度PID V2 实现 - 线性插值法
 */

#include "pid_gain_scheduling.h"

/* ---------- 内部: 线性插值 ---------- */
static float lerp(float v0, float v1, float t)
{
    return v0 + t * (v1 - v0);
}

/**
 * @brief 根据调度变量插值得到有效PID参数
 */
static void interpolate(const PID_GainSched_t *pid, float sched_var,
                         float *kp_eff, float *ki_eff, float *kd_eff)
{
    if (pid->num_points == 0) {
        *kp_eff = *ki_eff = *kd_eff = 0.0f;
        return;
    }

    if (pid->num_points == 1) {
        *kp_eff = pid->kp_table[0];
        *ki_eff = pid->ki_table[0];
        *kd_eff = pid->kd_table[0];
        return;
    }

    /* 钳位到表范围 */
    if (sched_var <= pid->sched_value[0]) {
        *kp_eff = pid->kp_table[0];
        *ki_eff = pid->ki_table[0];
        *kd_eff = pid->kd_table[0];
        return;
    }
    if (sched_var >= pid->sched_value[pid->num_points - 1]) {
        uint8_t last = pid->num_points - 1;
        *kp_eff = pid->kp_table[last];
        *ki_eff = pid->ki_table[last];
        *kd_eff = pid->kd_table[last];
        return;
    }

    /* 二分查找所在区间 */
    uint8_t lo = 0, hi = pid->num_points - 1;
    while (hi - lo > 1) {
        uint8_t mid = (lo + hi) / 2;
        if (pid->sched_value[mid] <= sched_var) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    /* 插值 */
    float v0 = pid->sched_value[lo];
    float v1 = pid->sched_value[hi];
    float t = (v1 > v0) ? (sched_var - v0) / (v1 - v0) : 0.0f;

    *kp_eff = lerp(pid->kp_table[lo], pid->kp_table[hi], t);
    *ki_eff = lerp(pid->ki_table[lo], pid->ki_table[hi], t);
    *kd_eff = lerp(pid->kd_table[lo], pid->kd_table[hi], t);
}

/* ============================================================ */

void PID_GS_Init(PID_GainSched_t *pid, float dt)
{
    pid->num_points = 0;
    pid->out_min = -1.0f;
    pid->out_max = 1.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->derivative_alpha = 0.1f;  /* 轻度滤波 */
    pid->output = 0.0f;
    pid->dt = dt;
    pid->anti_windup = 0;
}

void PID_GS_SetOutputLimit(PID_GainSched_t *pid, float out_min, float out_max)
{
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PID_GS_SetDerivFilter(PID_GainSched_t *pid, float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    pid->derivative_alpha = alpha;
}

void PID_GS_EnableAntiWindup(PID_GainSched_t *pid, uint8_t enable)
{
    pid->anti_windup = enable;
}

int PID_GS_AddPoint(PID_GainSched_t *pid,
                    float sched_val, float kp, float ki, float kd)
{
    if (pid->num_points >= PID_GS_MAX_POINTS) {
        return -1;
    }

    /* 检查递增性 */
    if (pid->num_points > 0 &&
        sched_val <= pid->sched_value[pid->num_points - 1]) {
        return -1;
    }

    uint8_t idx = pid->num_points;
    pid->sched_value[idx] = sched_val;
    pid->kp_table[idx] = kp;
    pid->ki_table[idx] = ki;
    pid->kd_table[idx] = kd;
    pid->num_points++;

    return 0;
}

float PID_GS_Update(PID_GainSched_t *pid,
                    float setpoint, float feedback,
                    float sched_var)
{
    if (pid->num_points == 0) {
        return 0.0f;
    }

    /* 1. 插值得到当前有效参数 */
    float kp_eff, ki_eff, kd_eff;
    interpolate(pid, sched_var, &kp_eff, &ki_eff, &kd_eff);

    /* 2. PID计算 */
    float error = setpoint - feedback;

    /* 积分 */
    pid->integral += error * pid->dt;

    /* 微分(带一阶低通滤波) */
    float raw_derivative = (error - pid->prev_error) / pid->dt;
    float alpha = pid->derivative_alpha;
    float filtered_derivative = alpha * raw_derivative +
                                (1.0f - alpha) * pid->prev_derivative;
    pid->prev_derivative = filtered_derivative;
    pid->prev_error = error;

    /* 合成 */
    float output = kp_eff * error +
                   ki_eff * pid->integral +
                   kd_eff * filtered_derivative;

    /* 3. 输出限幅 + 抗积分饱和 */
    if (output > pid->out_max) {
        if (pid->anti_windup) {
            pid->integral -= error * pid->dt;
        }
        output = pid->out_max;
    } else if (output < pid->out_min) {
        if (pid->anti_windup) {
            pid->integral -= error * pid->dt;
        }
        output = pid->out_min;
    }

    pid->output = output;
    return output;
}

void PID_GS_GetEffectiveParams(const PID_GainSched_t *pid, float sched_var,
                               float *kp_eff, float *ki_eff, float *kd_eff)
{
    interpolate(pid, sched_var, kp_eff, ki_eff, kd_eff);
}

void PID_GS_Reset(PID_GainSched_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output = 0.0f;
}
