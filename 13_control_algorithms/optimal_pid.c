/**
 * @file optimal_pid.c
 * @brief 最优PID控制器实现
 *
 * 基于Seborg等人的经典最优PID参数表，针对不同θ/T比值
 * 提供ITAE/ISE/IAE/ISTE准则下的最优参数。
 *
 * 参考：
 * - Seborg, Edgar, Mellichamp, "Process Dynamics and Control"
 * - Åström & Hägglund, "PID Controllers: Theory, Design and Tuning"
 * - Smith & Corripio, "Principles and Practice of Automatic Process Control"
 */

#include "optimal_pid.h"
#include <math.h>

/* ======================== 最优参数查找表 ======================== */

/**
 * @brief ITAE准则下PID参数表
 *
 * 表结构：{ theta/T, A, B } 其中
 *   对于PID: Kp = A/(K*(θ/T)^B), Ti = T*C*(θ/T)^D, Td = T*E*(θ/T)^F
 * 这里简化为多项式拟合系数
 */

/* ITAE最优PID参数（基于θ/T比值的多项式拟合）
 * Kp*K = a0 + a1*(θ/T) + a2*(θ/T)^2
 * Ti/T = b0 + b1*(θ/T) + b2*(θ/T)^2
 * Td/T = c0 + c1*(θ/T) + c2*(θ/T)^2 */
typedef struct {
    float a0, a1, a2;  /* Kp*K系数 */
    float b0, b1, b2;  /* Ti/T系数 */
    float c0, c1, c2;  /* Td/T系数 */
} OptCoeffs_t;

/* ITAE设定值最优系数 */
static const OptCoeffs_t coeff_itae_sp = {
    0.586f, -0.916f, 1.161f,   /* Kp */
    1.030f, -0.165f, 0.489f,   /* Ti */
    0.000f,  0.460f, 0.000f    /* Td */
};

/* ISE设定值最优系数 */
static const OptCoeffs_t coeff_ise_sp = {
    1.042f, -0.897f, 0.977f,
    0.489f,  0.306f, 0.200f,
    0.000f,  0.560f, 0.000f
};

/* IAE设定值最优系数 */
static const OptCoeffs_t coeff_iae_sp = {
    0.758f, -0.861f, 1.020f,
    0.878f, -0.126f, 0.400f,
    0.000f,  0.490f, 0.000f
};

/* ISTE设定值最优系数 */
static const OptCoeffs_t coeff_iste_sp = {
    0.509f, -0.850f, 1.200f,
    1.100f, -0.190f, 0.520f,
    0.000f,  0.430f, 0.000f
};

/* ======================== 内部辅助函数 ======================== */

static float clampf(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief 根据最优准则获取系数
 */
static const OptCoeffs_t* get_coeffs(OptCriterion_t criterion)
{
    switch (criterion) {
        case OPT_CRITERION_ITAE: return &coeff_itae_sp;
        case OPT_CRITERION_ISE:  return &coeff_ise_sp;
        case OPT_CRITERION_IAE:  return &coeff_iae_sp;
        case OPT_CRITERION_ISTE: return &coeff_iste_sp;
        default:                 return &coeff_itae_sp;
    }
}

/**
 * @brief 计算最优PID参数
 */
static void compute_optimal_params(OptimalPID_t *pid)
{
    float K = pid->model_K;
    float T = pid->model_T;
    float theta = pid->model_theta;

    if (K == 0.0f) K = 1e-6f;
    if (T <= 0.0f) T = 1e-6f;
    if (theta < 0.0f) theta = 0.0f;

    float r = theta / T;  /* θ/T 比值 */
    pid->time_ratio = r;

    /* 限制r在有效范围内 */
    r = clampf(r, 0.05f, 3.0f);

    const OptCoeffs_t *c = get_coeffs(pid->criterion);

    /* 计算 Kp*K */
    float KpK = c->a0 + c->a1 * r + c->a2 * r * r;
    if (KpK <= 0.0f) KpK = 0.01f;  /* 保证正值 */

    /* 计算 Ti/T */
    float TiT = c->b0 + c->b1 * r + c->b2 * r * r;
    if (TiT <= 0.0f) TiT = 0.1f;

    /* 计算 Td/T */
    float TdT = c->c0 + c->c1 * r + c->c2 * r * r;
    if (TdT < 0.0f) TdT = 0.0f;

    /* 转换为标准PID参数 */
    pid->Kp = KpK / fabsf(K);
    float Ti = TiT * T;
    float Td = TdT * T;
    if (fabsf(Ti) < 1e-6f) Ti = 1e-6f;  /* V2审计: 防除零 */

    pid->Ki = pid->Kp / Ti;
    pid->Kd = pid->Kp * Td;
}

/* ======================== 公共API实现 ======================== */

void OptimalPID_Init(OptimalPID_t *pid, float dt,
                     float model_K, float model_T, float model_theta,
                     OptCriterion_t criterion)
{
    pid->dt = dt;
    pid->model_K = model_K;
    pid->model_T = model_T;
    pid->model_theta = model_theta;
    pid->criterion = criterion;

    pid->output_min = -1.0f;
    pid->output_max = 1.0f;
    pid->integral_max = 100.0f;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->itae_sum = 0.0f;
    pid->ise_sum = 0.0f;
    pid->sim_time = 0.0f;

    /* 计算最优参数 */
    compute_optimal_params(pid);
}

float OptimalPID_Compute(OptimalPID_t *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;
    float dt = pid->dt;

    /* 更新性能指标统计 */
    pid->sim_time += dt;
    pid->itae_sum += pid->sim_time * fabsf(error) * dt;
    pid->ise_sum += error * error * dt;

    /* === 比例项（on error）=== */
    float P = pid->Kp * error;

    /* === 积分项（梯形法积分 + 抗饱和）=== */
    pid->integral += 0.5f * (error + pid->prev_error) * dt;
    pid->integral = clampf(pid->integral, -pid->integral_max, pid->integral_max);
    float I = pid->Ki * pid->integral;

    /* === 微分项（on measurement，避免设定值跳变引起的冲击）=== */
    float d_measurement = (feedback - pid->prev_measurement) / dt;
    float D = -pid->Kd * d_measurement;  /* 负号因为是对measurement微分 */

    /* 合成输出 */
    float output = P + I + D;
    output = clampf(output, pid->output_min, pid->output_max);

    /* 抗饱和back-calculation */
    float unsaturated = P + I + D;
    float saturated = clampf(unsaturated, pid->output_min, pid->output_max);
    if (fabsf(unsaturated - saturated) > 1e-6f) {
        pid->integral -= 0.5f * (unsaturated - saturated) / (pid->Ki + 1e-10f);
    }

    /* 保存状态 */
    pid->prev_error = error;
    pid->prev_measurement = feedback;

    return output;
}

void OptimalPID_Reset(OptimalPID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->itae_sum = 0.0f;
    pid->ise_sum = 0.0f;
    pid->sim_time = 0.0f;
}

void OptimalPID_SetCriterion(OptimalPID_t *pid, OptCriterion_t criterion)
{
    pid->criterion = criterion;
    compute_optimal_params(pid);
}

void OptimalPID_SetOutputLimits(OptimalPID_t *pid, float min_val, float max_val)
{
    pid->output_min = min_val;
    pid->output_max = max_val;
}

float OptimalPID_GetITAE(OptimalPID_t *pid)
{
    return pid->itae_sum;
}

float OptimalPID_GetISE(OptimalPID_t *pid)
{
    return pid->ise_sum;
}
