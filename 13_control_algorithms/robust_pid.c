/**
 * @file robust_pid.c
 * @brief 鲁棒PID控制器实现（H∞优化的PID参数）
 *
 * 实现思路：
 * 1. 基于标称模型和不确定性范围，构建加权灵敏度优化问题
 * 2. 使用解析近似求解H∞约束下的最优PID参数
 * 3. 在积分项和微分项中加入鲁棒性增强措施
 *
 * 参考：
 * - Åström & Murray, "Feedback Systems", Ch.12 鲁棒性能
 * - Skogestad & Postlethwaite, "Multivariable Feedback Control"
 * - 刘金琨, "先进PID控制MATLAB仿真"
 */

#include "robust_pid.h"
#include <math.h>

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 限幅函数
 */
static float clampf(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief 基于H∞近似的PID参数自动整定
 *
 * 原理：
 * 对于一阶加纯滞后模型 G(s) = K*exp(-θs)/(Ts+1)
 *
 * 使用IMC（内模控制）方法作为H∞次优解的近似：
 *   选择闭环时间常数 λc，满足鲁棒性约束：
 *     λc > θ / (1 - ΔK)   （保证鲁棒稳定性的最小带宽）
 *
 * 然后将IMC控制器转换为标准PID参数：
 *   Kp = (2*T + θ) / (2*K*(λc + θ))
 *   Ki = Kp / (T + θ/2)
 *   Kd = Kp * T*θ / (2*T + θ)
 *
 * 最后乘以gamma因子平衡性能与鲁棒性。
 */
static void robust_tune(RobustPID_t *pid, float desired_bw)
{
    float K = pid->model_K;
    float T = pid->model_T;
    float theta = pid->model_theta;
    float dK = pid->delta_K;
    float dT = pid->delta_T;

    /* 防止除零 */
    if (K == 0.0f) K = 1e-6f;
    if (T <= 0.0f) T = 1e-6f;
    if (theta < 0.0f) theta = 0.0f;

    /* 计算鲁棒稳定性约束下的最小闭环时间常数
     * λc_min 保证在参数摄动范围内系统仍稳定 */
    float lambda_c;

    if (desired_bw > 0.0f) {
        /* 用户指定带宽，转换为闭环时间常数 */
        lambda_c = 1.0f / desired_bw;
    } else {
        /* 自动选择：取保守值确保鲁棒性
         * λc = max(θ/(1-ΔK), T*ΔT) + 安全裕度 */
        float lambda_robust = theta / (1.0f - dK + 0.01f);
        float lambda_perf = T * 0.1f;  /* 性能导向的快速响应 */
        lambda_c = (lambda_robust > lambda_perf) ? lambda_robust : lambda_perf;
        lambda_c *= pid->gamma;  /* gamma调整 */
    }

    /* 确保lambda_c不小于采样周期的2倍（避免离散化问题） */
    if (lambda_c < 2.0f * pid->dt) {
        lambda_c = 2.0f * pid->dt;
    }

    /* IMC-PID参数整定公式
     * 将IMC控制器 C_imc = (T*s+1) / (K*(λc*s+1)*(θ/2*s+1))
     * 转换为PID形式 */
    float T_eq = T + theta / 2.0f;

    pid->Kp = (2.0f * T + theta) / (2.0f * K * (lambda_c + theta));
    pid->Ki = pid->Kp / T_eq;
    pid->Kd = pid->Kp * (T * theta) / (2.0f * T + theta);

    /* 微分项低通滤波系数（截止频率约为带宽的10倍） */
    pid->alpha = pid->dt / (pid->dt + 0.1f * lambda_c);

    /* 积分分离阈值：当误差大于此值时停止积分 */
    pid->beta = 0.3f;  /* 30%的量程 */
}

/* ======================== 公共API实现 ======================== */

void RobustPID_Init(RobustPID_t *pid, float dt,
                    float model_K, float model_T, float model_theta,
                    float delta_K, float delta_T, float desired_bw)
{
    /* 保存模型参数 */
    pid->dt = dt;
    pid->model_K = model_K;
    pid->model_T = model_T;
    pid->model_theta = model_theta;
    pid->delta_K = delta_K;
    pid->delta_T = delta_T;
    pid->gamma = 1.0f;

    /* 默认输出限幅 */
    pid->output_min = -1.0f;
    pid->output_max = 1.0f;

    /* 清零状态 */
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
    pid->filtered_D = 0.0f;

    /* 自动整定PID参数 */
    robust_tune(pid, desired_bw);
}

float RobustPID_Compute(RobustPID_t *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;

    /* === 比例项 === */
    float P = pid->Kp * error;

    /* === 积分项（带积分分离） ===
     * 当误差过大时停止积分，防止windup */
    float abs_error = fabsf(error);
    float I;

    if (abs_error < pid->beta) {
        /* 误差在阈值内，正常积分 */
        pid->integral += error * pid->dt;
    } else if (error * pid->integral < 0) {
        /* 误差过大但方向与积分相反，允许积分（用于退饱和） */
        pid->integral += error * pid->dt;
    }
    /* 否则冻结积分 */

    I = pid->Ki * pid->integral;

    /* === 微分项（带一阶低通滤波） ===
     * 使用带滤波的微分，避免测量噪声放大
     * D = Kd * α * (error - prev_error)/dt + (1-α) * prev_D */
    float raw_d = (error - pid->prev_error) / pid->dt;
    float D = pid->Kd * raw_d;

    /* 一阶低通滤波微分项 */
    pid->filtered_D = pid->alpha * D + (1.0f - pid->alpha) * pid->filtered_D;
    D = pid->filtered_D;

    /* === 合成输出 === */
    float output_unclamped = P + I + D;

    /* 输出限幅 */
    float output = clampf(output_unclamped, pid->output_min, pid->output_max);

    /* 积分抗饱和（back-calculation）
     * 如果输出饱和，回退积分项以防止windup */
    if (output != output_unclamped) {
        float excess = output_unclamped - output;
        pid->integral -= excess / (pid->Ki + 1e-10f) * 0.5f;
    }

    /* 保存状态 */
    pid->prev_error = error;
    pid->prev_output = output;

    return output;
}

void RobustPID_Reset(RobustPID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
    pid->filtered_D = 0.0f;
}

void RobustPID_SetOutputLimits(RobustPID_t *pid, float min_val, float max_val)
{
    pid->output_min = min_val;
    pid->output_max = max_val;
}

void RobustPID_SetGains(RobustPID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void RobustPID_SetGamma(RobustPID_t *pid, float gamma)
{
    pid->gamma = gamma;
    /* 重新整定 */
    robust_tune(pid, 0.0f);
}
