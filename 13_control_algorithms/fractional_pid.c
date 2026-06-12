/**
 * @file fractional_pid.c
 * @brief 分数阶PID控制器实现（Grünwald-Letnikov离散化）
 *
 * 实现核心：
 * 1. 使用GL定义离散化分数阶积分和微分
 * 2. GL权重通过递推公式计算：w_k = (1-(α+1)/k)*w_{k-1}, w_0=1
 * 3. 使用循环缓冲区存储历史误差，节省内存
 *
 * 参考：
 * - Podlubny, "Fractional Differential Equations" (1999)
 * - Monje et al., "Fractional-order Systems and Controls" (2010)
 * - 薛定宇, "分数阶微积分学与分数阶控制" 
 * - GitHub: CRHeather/fractional_pid (MATLAB参考实现)
 */

#include "fractional_pid.h"
#include <math.h>

/* ======================== 内部辅助函数 ======================== */

static float clampf(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief 计算Grünwald-Letnikov权重系数
 *
 * GL系数递推公式：
 *   w[0] = 1
 *   w[k] = (1 - (α+1)/k) * w[k-1],  k = 1, 2, ..., L-1
 *
 * 这个递推关系来自二项式系数的性质：
 *   C(α, k) = C(α, k-1) * (α-k+1)/k
 *   w[k] = (-1)^k * C(α, k) = (1 - (α+1)/k) * w[k-1]
 *
 * @param weights   输出权重数组
 * @param alpha     分数阶次（正数=微分，负数=积分）
 * @param length    权重数组长度
 */
static void compute_gl_weights(float *weights, float alpha, uint16_t length)
{
    weights[0] = 1.0f;

    for (uint16_t k = 1; k < length; k++) {
        weights[k] = weights[k - 1] * (1.0f - (alpha + 1.0f) / (float)k);
    }
}

/**
 * @brief 使用GL权重计算分数阶差分/积分
 *
 * D^α f[n] ≈ h^(-α) * Σ_{k=0}^{L-1} w[k] * f[n-k]
 * 其中 h = dt（采样周期）
 *
 * @param pid       控制器结构体
 * @param weights   GL权重系数
 * @param alpha     分数阶次
 * @return 分数阶差分/积分值
 */
static float gl_compute(FracPID_t *pid, float *weights, float alpha)
{
    float sum = 0.0f;
    uint16_t count = pid->buf_count;

    if (count > pid->mem_len) {
        count = pid->mem_len;
    }

    /* 从最新到最旧遍历历史误差，与GL权重卷积 */
    for (uint16_t k = 0; k < count; k++) {
        /* 计算环形缓冲区中的实际索引 */
        uint16_t idx = (pid->buf_idx + pid->mem_len - k) % pid->mem_len;
        sum += weights[k] * pid->error_buf[idx];
    }

    /* 乘以 h^(-α) = dt^(-α) */
    float h_neg_alpha = powf(pid->dt, -alpha);
    sum *= h_neg_alpha;

    return sum;
}

/* ======================== 公共API实现 ======================== */

void FracPID_Init(FracPID_t *pid, float dt,
                  float Kp, float Ki, float Kd,
                  float lambda_, float mu, uint16_t mem_len)
{
    pid->dt = dt;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->lambda = lambda_;
    pid->mu = mu;

    /* 限制记忆长度 */
    if (mem_len > FPID_MEMORY_LENGTH) {
        mem_len = FPID_MEMORY_LENGTH;
    }
    if (mem_len < 5) {
        mem_len = 5;
    }
    pid->mem_len = mem_len;

    /* 默认输出限幅 */
    pid->output_min = -1.0f;
    pid->output_max = 1.0f;
    pid->integral_max = 100.0f;

    /* 清零缓冲区 */
    pid->buf_idx = 0;
    pid->buf_count = 0;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    for (uint16_t i = 0; i < FPID_MEMORY_LENGTH; i++) {
        pid->error_buf[i] = 0.0f;
        pid->gl_weights_i[i] = 0.0f;
        pid->gl_weights_d[i] = 0.0f;
    }

    /* 计算GL权重系数
     * 积分：D^(-λ) 对应 α = -lambda（负阶次=积分）
     * 微分：D^(μ)  对应 α = mu（正阶次=微分） */
    compute_gl_weights(pid->gl_weights_i, -lambda_, mem_len);
    compute_gl_weights(pid->gl_weights_d, mu, mem_len);
}

float FracPID_Compute(FracPID_t *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;

    /* 将新误差写入环形缓冲区 */
    pid->buf_idx = (pid->buf_idx + 1) % pid->mem_len;
    pid->error_buf[pid->buf_idx] = error;
    if (pid->buf_count < pid->mem_len) {
        pid->buf_count++;
    }

    /* === 比例项 === */
    float P = pid->Kp * error;

    /* === 分数阶积分项 ===
     * I = Ki * D^(-λ) * e(t)
     * 使用GL离散化：I ≈ Ki * h^λ * Σ w_i[k] * e[n-k] */
    float frac_integral = gl_compute(pid, pid->gl_weights_i, -pid->lambda);
    float I = pid->Ki * frac_integral;
    I = clampf(I, -pid->integral_max, pid->integral_max);

    /* === 分数阶微分项 ===
     * D = Kd * D^(μ) * e(t)
     * 使用GL离散化：D ≈ Kd * h^(-μ) * Σ w_d[k] * e[n-k] */
    float frac_derivative = gl_compute(pid, pid->gl_weights_d, pid->mu);
    float D = pid->Kd * frac_derivative;

    /* 合成输出 */
    float output = P + I + D;
    output = clampf(output, pid->output_min, pid->output_max);

    pid->prev_error = error;

    return output;
}

void FracPID_Reset(FracPID_t *pid)
{
    pid->buf_idx = 0;
    pid->buf_count = 0;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    for (uint16_t i = 0; i < FPID_MEMORY_LENGTH; i++) {
        pid->error_buf[i] = 0.0f;
    }
}

void FracPID_SetOutputLimits(FracPID_t *pid, float min_val, float max_val)
{
    pid->output_min = min_val;
    pid->output_max = max_val;
}

void FracPID_SetOrders(FracPID_t *pid, float lambda_, float mu)
{
    pid->lambda = lambda_;
    pid->mu = mu;

    /* 重新计算GL权重 */
    compute_gl_weights(pid->gl_weights_i, -lambda_, pid->mem_len);
    compute_gl_weights(pid->gl_weights_d, mu, pid->mem_len);
}

void FracPID_SetGains(FracPID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}
