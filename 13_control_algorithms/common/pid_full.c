/**
 * @file pid_full.c
 * @brief PID全系列算法实现 v2.0
 *
 * v2.0优化记录:
 * [OPT-1] 前馈补偿: output = PID + feedforward
 * [OPT-2] 条件积分抗饱和: 只在输出未饱和时积分
 * [OPT-3] 死区控制: 误差在死区内保持当前输出
 * [OPT-4] 自适应切换: PID_AUTO_SWITCH模式根据误差自动切换
 * [OPT-5] 增量式PID微分滤波
 * [OPT-6] 回算法抗饱和: u_unsat - u_sat 回算修正积分
 * [OPT-7] Clamp改为内联, 减少函数调用开销
 * [OPT-8] dt参与微分计算, 更精确
 */

#include "pid_full.h"
#include <stdio.h>
#include <math.h>

/* [OPT-7] 内联Clamp函数, 减少栈帧开销 */
static inline float Clamp(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/* ========== 初始化 ========== */
void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0;
    pid->output = 0;

    pid->mode = PID_POSITION;
    pid->feature = PID_NORMAL;
    pid->anti_windup = PID_AW_CLAMP;  /* 默认使用条件积分抗饱和 */

    pid->error = 0;
    pid->error_last = 0;
    pid->error_prev = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output_last = 0;
    pid->measurement_last = 0;

    pid->output_max = 1000;
    pid->output_min = -1000;
    pid->integral_max = 500;
    pid->integral_min = -500;

    pid->integral_sep_threshold = 100;
    pid->derivative_filter_alpha = 0.3f;

    pid->segment_count = 0;

    /* [OPT-1] 前馈补偿 */
    pid->feedforward = 0.0f;
    /* [OPT-3] 死区 */
    pid->dead_zone = 0.0f;
    /* [OPT-4] 自适应切换阈值 */
    pid->auto_switch_threshold = 0.0f;  /* 0=不使用自适应 */
    /* [OPT-6] 回算法 */
    pid->back_calc_kb = 0.5f;
    pid->output_saturated = 0.0f;
    /* [OPT-8] 采样周期 */
    pid->dt = 0.01f;  /* 默认10ms */
}

void PID_SetMode(PID_t *pid, PID_Mode_t mode, PID_Feature_t feature)
{
    pid->mode = mode;
    pid->feature = feature;
}

void PID_SetOutputLimit(PID_t *pid, float min, float max)
{
    pid->output_min = min;
    pid->output_max = max;
}

void PID_SetIntegralLimit(PID_t *pid, float min, float max)
{
    pid->integral_min = min;
    pid->integral_max = max;
}

void PID_SetIntegralSeparation(PID_t *pid, float threshold)
{
    pid->integral_sep_threshold = threshold;
}

void PID_SetDerivativeFilter(PID_t *pid, float alpha)
{
    pid->derivative_filter_alpha = Clamp(alpha, 0.0f, 1.0f);
}

void PID_SetSegments(PID_t *pid, const PID_Segment_t *segments, uint8_t count)
{
    if (count > 4) count = 4;
    pid->segment_count = count;
    for (uint8_t i = 0; i < count; i++) {
        pid->segments[i] = segments[i];
    }
}

void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

/* [OPT-1] 前馈补偿 */
void PID_SetFeedforward(PID_t *pid, float ff)
{
    pid->feedforward = ff;
}

/* [OPT-3] 死区控制 */
void PID_SetDeadZone(PID_t *pid, float dead_zone)
{
    pid->dead_zone = (dead_zone < 0.0f) ? 0.0f : dead_zone;
}

/* [OPT-6] 抗饱和策略设置 */
void PID_SetAntiWindup(PID_t *pid, PID_AntiWindup_t type, float kb)
{
    pid->anti_windup = type;
    pid->back_calc_kb = kb;
}

/* [OPT-8] 采样周期 */
void PID_SetSampleTime(PID_t *pid, float dt)
{
    pid->dt = (dt > 0.0f) ? dt : 0.001f;
}

/* ========== 位置式PID ========== */
static float PID_PositionCalculate(PID_t *pid, float measurement)
{
    float kp = pid->kp, ki = pid->ki, kd = pid->kd;
    float p_term, i_term, d_term;
    float dt = pid->dt;

    pid->error = pid->target - measurement;

    /* [OPT-3] 死区控制: 误差在死区内保持当前输出 */
    if (pid->dead_zone > 0.0f && fabsf(pid->error) < pid->dead_zone) {
        return pid->output;
    }

    /* 积分分离: 误差大时关闭积分 */
    if (pid->feature == PID_INTEGRAL_SEP) {
        if (fabsf(pid->error) > pid->integral_sep_threshold) {
            ki = 0;
        }
    }

    /* P项 */
    p_term = kp * pid->error;

    /* [OPT-2] 积分项: 根据抗饱和策略选择不同方法 */
    if (pid->anti_windup == PID_AW_CLAMP) {
        /* 条件积分抗饱和: 输出饱和且积分方向会使饱和加剧时停止积分 */
        bool should_integrate = true;
        if (pid->output >= pid->output_max && pid->error > 0)
            should_integrate = false;
        if (pid->output <= pid->output_min && pid->error < 0)
            should_integrate = false;

        if (should_integrate) {
            pid->integral += pid->error * dt;
        }
    } else if (pid->anti_windup == PID_AW_BACK_CALC) {
        /* [OPT-6] 回算法: 利用饱和误差修正积分 */
        float u_unsat = p_term + ki * pid->integral + kd * (pid->error - pid->error_last) / dt;
        float u_sat = Clamp(u_unsat, pid->output_min, pid->output_max);
        float aw_correction = pid->back_calc_kb * (u_sat - u_unsat);
        pid->integral += (pid->error + aw_correction) * dt;
        pid->output_saturated = u_sat;
    } else {
        /* 简单限幅(传统方法) */
        pid->integral += pid->error * dt;
    }

    /* 积分限幅 */
    pid->integral = Clamp(pid->integral, pid->integral_min, pid->integral_max);
    i_term = ki * pid->integral;

    /* D项 */
    if (pid->feature == PID_DERIVATIVE_LPF) {
        /* 微分先行: 对测量值微分, 避免目标突变引起的微分冲击 */
        float measurement_diff = (measurement - pid->measurement_last) / dt;
        pid->derivative = pid->derivative_filter_alpha * (-kd * measurement_diff)
                        + (1.0f - pid->derivative_filter_alpha) * pid->derivative;
        pid->measurement_last = measurement;
    } else {
        /* [OPT-8] 使用dt精确计算微分 */
        float error_diff = (pid->error - pid->error_last) / dt;
        pid->derivative = pid->derivative_filter_alpha * (kd * error_diff)
                        + (1.0f - pid->derivative_filter_alpha) * pid->derivative;
    }
    d_term = pid->derivative;

    pid->error_last = pid->error;

    /* [OPT-1] 输出 = PID + 前馈 */
    pid->output = Clamp(p_term + i_term + d_term + pid->feedforward,
                        pid->output_min, pid->output_max);

    return pid->output;
}

/* ========== 增量式PID ========== */
static float PID_IncrementCalculate(PID_t *pid, float measurement)
{
    float kp = pid->kp, ki = pid->ki, kd = pid->kd;
    float increment;
    float dt = pid->dt;

    pid->error = pid->target - measurement;

    /* [OPT-3] 死区控制 */
    if (pid->dead_zone > 0.0f && fabsf(pid->error) < pid->dead_zone) {
        return pid->output;
    }

    /* 增量式: Δu = Kp*(e(k)-e(k-1)) + Ki*e(k)*dt + Kd*(e(k)-2e(k-1)+e(k-2))/dt */
    increment = kp * (pid->error - pid->error_last)
              + ki * pid->error * dt
              + kd * (pid->error - 2.0f * pid->error_last + pid->error_prev) / dt;

    /* [OPT-5] 增量式微分滤波 */
    increment = pid->derivative_filter_alpha * increment
              + (1.0f - pid->derivative_filter_alpha) * pid->derivative;
    pid->derivative = increment;

    pid->error_prev = pid->error_last;
    pid->error_last = pid->error;

    pid->output_last += increment;
    pid->output = Clamp(pid->output_last, pid->output_min, pid->output_max);
    pid->output_last = pid->output;

    return pid->output;
}

/* ========== 分段PID ========== */
static void PID_UpdateSegmentParams(PID_t *pid)
{
    float abs_error = fabsf(pid->error);

    for (uint8_t i = 0; i < pid->segment_count; i++) {
        if (abs_error >= pid->segments[i].threshold) {
            pid->kp = pid->segments[i].kp;
            pid->ki = pid->segments[i].ki;
            pid->kd = pid->segments[i].kd;
            return;
        }
    }
}

/* ========== 主计算函数 ========== */
float PID_Calculate(PID_t *pid, float measurement)
{
    /* 分段PID: 根据误差大小自动切换参数 */
    pid->error = pid->target - measurement;
    if (pid->feature == PID_SEGMENTED && pid->segment_count > 0) {
        PID_UpdateSegmentParams(pid);
    }

    switch (pid->mode) {
        case PID_INCREMENT:
            return PID_IncrementCalculate(pid, measurement);

        /* [OPT-4] 自适应切换 */
        case PID_AUTO_SWITCH:
            if (pid->auto_switch_threshold > 0.0f &&
                fabsf(pid->error) > pid->auto_switch_threshold) {
                /* 误差大时用增量式, 响应快且无积分饱和风险 */
                return PID_IncrementCalculate(pid, measurement);
            } else {
                /* 误差小时用位置式, 稳态精度高 */
                return PID_PositionCalculate(pid, measurement);
            }

        case PID_POSITION:
        default:
            return PID_PositionCalculate(pid, measurement);
    }
}

/* ========== 重置 ========== */
void PID_Reset(PID_t *pid)
{
    pid->error = 0;
    pid->error_last = 0;
    pid->error_prev = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
    pid->output_last = 0;
    pid->measurement_last = 0;
}

/* ========== 调试打印 ========== */
void PID_PrintStatus(const PID_t *pid)
{
    printf("[PID] target=%.2f error=%.2f out=%.2f | P=%.4f I=%.4f D=%.4f | int=%.2f\n",
           pid->target, pid->error, pid->output,
           pid->kp * pid->error, pid->ki * pid->integral, pid->derivative, pid->integral);
}
