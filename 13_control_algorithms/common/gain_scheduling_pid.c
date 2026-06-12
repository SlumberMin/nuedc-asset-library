/**
 * @file gain_scheduling_pid.c
 * @brief 增益调度PID控制器实现
 *
 * 支持两种调度模式：
 *   硬切换：调度变量落在某区间时直接使用该区间的PID参数
 *   软切换：在相邻工况点间线性插值，参数过渡平滑
 *
 * PID 特性：
 *   - 抗积分饱和（积分限幅 + 条件积分）
 *   - 微分先行（对反馈微分，避免设定值突变引起微分冲击）
 *   - 微分滤波（一阶低通，抑制高频噪声）
 */

#include "gain_scheduling_pid.h"
#include <string.h>

/**
 * @brief 限幅函数
 */
static float clamp(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/**
 * @brief 在工况表中查找参数（硬切换）
 *
 * 找到调度变量所在的区间，返回对应的PID参数。
 * 如果超出范围，使用最近的端点参数。
 */
static void lookup_params_hard(const GainSchedPID_t *pid, float sv,
                                float *Kp, float *Ki, float *Kd)
{
    int n = pid->point_count;
    if (n == 0) {
        *Kp = *Ki = *Kd = 0.0f;
        return;
    }

    /* 低于最小值 */
    if (sv <= pid->points[0].sv) {
        *Kp = pid->points[0].Kp;
        *Ki = pid->points[0].Ki;
        *Kd = pid->points[0].Kd;
        return;
    }

    /* 高于最大值 */
    if (sv >= pid->points[n - 1].sv) {
        *Kp = pid->points[n - 1].Kp;
        *Ki = pid->points[n - 1].Ki;
        *Kd = pid->points[n - 1].Kd;
        return;
    }

    /* 查找所在区间 */
    for (int i = 0; i < n - 1; i++) {
        if (sv >= pid->points[i].sv && sv < pid->points[i + 1].sv) {
            *Kp = pid->points[i].Kp;
            *Ki = pid->points[i].Ki;
            *Kd = pid->points[i].Kd;
            return;
        }
    }

    /* 兜底 */
    *Kp = pid->points[n - 1].Kp;
    *Ki = pid->points[n - 1].Ki;
    *Kd = pid->points[n - 1].Kd;
}

/**
 * @brief 在工况表中查找参数（软切换/线性插值）
 *
 * 在相邻工况点间线性插值，实现平滑过渡。
 */
static void lookup_params_soft(const GainSchedPID_t *pid, float sv,
                                float *Kp, float *Ki, float *Kd)
{
    int n = pid->point_count;
    if (n == 0) {
        *Kp = *Ki = *Kd = 0.0f;
        return;
    }

    /* 边界处理 */
    if (sv <= pid->points[0].sv) {
        *Kp = pid->points[0].Kp;
        *Ki = pid->points[0].Ki;
        *Kd = pid->points[0].Kd;
        return;
    }
    if (sv >= pid->points[n - 1].sv) {
        *Kp = pid->points[n - 1].Kp;
        *Ki = pid->points[n - 1].Ki;
        *Kd = pid->points[n - 1].Kd;
        return;
    }

    /* 查找区间并插值 */
    for (int i = 0; i < n - 1; i++) {
        float sv0 = pid->points[i].sv;
        float sv1 = pid->points[i + 1].sv;
        if (sv >= sv0 && sv < sv1) {
            /* 插值系数 t ∈ [0, 1) */
            float sv_range = sv1 - sv0;
            float t = (sv_range > 1e-6f) ? ((sv - sv0) / sv_range) : 0.0f;  /* 防止除零 */
            *Kp = pid->points[i].Kp + t * (pid->points[i + 1].Kp - pid->points[i].Kp);
            *Ki = pid->points[i].Ki + t * (pid->points[i + 1].Ki - pid->points[i].Ki);
            *Kd = pid->points[i].Kd + t * (pid->points[i + 1].Kd - pid->points[i].Kd);
            return;
        }
    }

    /* 兜底 */
    *Kp = pid->points[n - 1].Kp;
    *Ki = pid->points[n - 1].Ki;
    *Kd = pid->points[n - 1].Kd;
}

void GSPID_Init(GainSchedPID_t *pid, float dt, float u_max, float u_min,
                GS_SwitchMode_t mode)
{
    memset(pid, 0, sizeof(GainSchedPID_t));
    pid->dt = dt;
    pid->u_max = u_max;
    pid->u_min = u_min;
    pid->mode = mode;
    pid->integral_max = 1e6f;       /* 默认很大的积分限幅 */
    pid->d_filter_alpha = 0.1f;     /* 默认微分滤波 */
}

int GSPID_AddPoint(GainSchedPID_t *pid, float sv, float Kp, float Ki, float Kd)
{
    if (pid->point_count >= GS_PID_MAX_POINTS) return -1;

    int idx = pid->point_count;
    pid->points[idx].sv = sv;
    pid->points[idx].Kp = Kp;
    pid->points[idx].Ki = Ki;
    pid->points[idx].Kd = Kd;
    pid->point_count++;

    return 0;
}

float GSPID_Update(GainSchedPID_t *pid, float setpoint, float feedback, float sv)
{
    /* 1. 根据调度变量查找/插值 PID 参数 */
    if (pid->mode == GS_MODE_HARD) {
        lookup_params_hard(pid, sv, &pid->Kp, &pid->Ki, &pid->Kd);
    } else {
        lookup_params_soft(pid, sv, &pid->Kp, &pid->Ki, &pid->Kd);
    }

    /* 2. 计算误差 */
    float error = setpoint - feedback;

    /* 3. 比例项 */
    float P_term = pid->Kp * error;

    /* 4. 积分项（带抗饱和） */
    pid->integral += error * pid->dt;
    pid->integral = clamp(pid->integral, -pid->integral_max, pid->integral_max);
    float I_term = pid->Ki * pid->integral;

    /* 5. 微分项 */
    float d_input;
    if (pid->use_derivative_on_pv) {
        /* 微分先行：对反馈微分，避免设定值突变 */
        d_input = -(feedback - pid->last_feedback) / pid->dt;
    } else {
        /* 标准微分：对误差微分 */
        d_input = (error - pid->last_error) / pid->dt;
    }

    /* 微分滤波（一阶低通） */
    pid->d_filtered = pid->d_filter_alpha * d_input
                    + (1.0f - pid->d_filter_alpha) * pid->d_filtered;
    float D_term = pid->Kd * pid->d_filtered;

    /* 6. 输出合成与限幅 */
    float u = P_term + I_term + D_term;
    u = clamp(u, pid->u_min, pid->u_max);

    /* 7. 抗积分饱和：当输出饱和时回退积分 */
    if (u >= pid->u_max || u <= pid->u_min) {
        pid->integral -= error * pid->dt;
    }

    /* 8. 保存历史 */
    pid->last_error = error;
    pid->last_feedback = feedback;

    return u;
}

void GSPID_Reset(GainSchedPID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_feedback = 0.0f;
    pid->d_filtered = 0.0f;
}

void GSPID_SetIntegralLimit(GainSchedPID_t *pid, float limit)
{
    pid->integral_max = limit;
}

void GSPID_SetDerivativeFilter(GainSchedPID_t *pid, float alpha)
{
    pid->d_filter_alpha = alpha;
}

void GSPID_SetDerivativeOnPV(GainSchedPID_t *pid, uint8_t enable)
{
    pid->use_derivative_on_pv = enable;
}

void GSPID_GetCurrentParams(const GainSchedPID_t *pid, float *Kp, float *Ki, float *Kd)
{
    if (Kp) *Kp = pid->Kp;
    if (Ki) *Ki = pid->Ki;
    if (Kd) *Kd = pid->Kd;
}
