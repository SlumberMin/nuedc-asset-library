/**
 * @file super_twisting_smc.c
 * @brief 超螺旋滑模控制器实现 v1.0
 * @date 2026-06-11
 *
 * Super-Twisting Algorithm (Levant, 1993)
 *
 * 算法公式:
 *   u = u1 + u2
 *   du1/dt = -lambda * |s|^{1/2} * sign(s)
 *   du2/dt = -alpha * sign(s)
 *
 * 离散化:
 *   u1(k+1) = u1(k) - lambda * |s(k)|^{1/2} * sign(s(k)) * dt
 *   u2(k+1) = u2(k) - alpha * sign(s(k)) * dt
 *   u(k) = u1(k) + u2(k)
 *
 * 参数要求:
 *   lambda > 0
 *   alpha > lambda (稳定性必要条件)
 *
 * 收敛时间:
 *   T ≤ 2 * sqrt(|s(0)|) / lambda (有限时间收敛)
 *
 * 性能优化记录:
 *   [OPT-1] 预计算sqrt(lambda), 避免重复pow运算
 *   [OPT-2] 快速sign函数, 适合分支预测
 *   [OPT-3] 边界层平滑, 进一步抑制抖振 (可选)
 *   [OPT-4] 输出限幅, 防止饱和
 */

#include "super_twisting_smc.h"
#include <math.h>

/* 嵌入式环境: 移除所有printf，用空宏替代 */
#ifndef EMBEDDED_DEBUG
#define printf(...) ((void)0)
#endif

/* 快速符号函数 */
static inline float Sign(float x)
{
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

/**
 * @brief 限幅函数
 */
static inline float Clamp(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/* ========== 初始化接口 ========== */

void SuperTwisting_Init(SuperTwistingSMC_t *smc, float c, float lambda, float alpha)
{
    /* 参数检查 */
    if (lambda <= 0.0f) {
        printf("[SuperTwisting] 警告: lambda(%.2f) <= 0, 强制设为1.0\n", lambda);
        lambda = 1.0f;
    }
    if (alpha <= lambda) {
        printf("[SuperTwisting] 警告: alpha(%.2f) <= lambda(%.2f), 强制设为%.2f\n",
               alpha, lambda, 2.0f * lambda);
        alpha = 2.0f * lambda;
    }

    /* 滑模面参数 */
    smc->c = c;
    smc->s = 0.0f;

    /* 超螺旋算法参数 */
    smc->lambda = lambda;
    smc->alpha = alpha;

    /* 预计算 */
    smc->sqrt_lambda = sqrtf(lambda);
    smc->inv_lambda = 1.0f / lambda;

    /* 内部积分状态 */
    smc->u1 = 0.0f;
    smc->u2 = 0.0f;

    /* 误差状态 */
    smc->error = 0.0f;
    smc->error_dot = 0.0f;

    /* 控制输出 */
    smc->output = 0.0f;

    /* 输出限幅 */
    smc->output_max = 100.0f;
    smc->output_min = -100.0f;

    printf("[SuperTwisting] 初始化完成: c=%.2f, λ=%.2f, α=%.2f\n",
           c, lambda, alpha);
    printf("[SuperTwisting] 参数验证: α/λ=%.2f (>1 OK)\n", alpha / lambda);
}

void SuperTwisting_SetSlidingSurface(SuperTwistingSMC_t *smc, float c)
{
    smc->c = c;
    printf("[SuperTwisting] 滑模面更新: c=%.2f\n", c);
}

void SuperTwisting_SetParameters(SuperTwistingSMC_t *smc, float lambda, float alpha)
{
    if (lambda <= 0.0f) {
        printf("[SuperTwisting] 错误: lambda必须>0\n");
        return;
    }
    if (alpha <= lambda) {
        printf("[SuperTwisting] 错误: alpha(%.2f)必须>lambda(%.2f)\n", alpha, lambda);
        return;
    }

    smc->lambda = lambda;
    smc->alpha = alpha;
    smc->sqrt_lambda = sqrtf(lambda);
    smc->inv_lambda = 1.0f / lambda;

    printf("[SuperTwisting] 参数更新: λ=%.2f, α=%.2f, α/λ=%.2f\n",
           lambda, alpha, alpha / lambda);
}

void SuperTwisting_SetOutputLimit(SuperTwistingSMC_t *smc, float min, float max)
{
    smc->output_min = min;
    smc->output_max = max;
    printf("[SuperTwisting] 输出限幅: [%.2f, %.2f]\n", min, max);
}

/* ========== 核心计算 ========== */

float SuperTwisting_Calculate(SuperTwistingSMC_t *smc, float r, float y,
                              float y_dot, float dt)
{
    float sign_s, sqrt_abs_s;

    /* 1. 计算误差 */
    smc->error = r - y;
    smc->error_dot = 0.0f - y_dot;  /* 假设参考速度为0 */

    /* 2. 计算滑模面 s = e_dot + c*e */
    smc->s = smc->error_dot + smc->c * smc->error;

    /* 3. 超螺旋算法核心:
     *    du1/dt = -lambda * |s|^{1/2} * sign(s)
     *    du2/dt = -alpha * sign(s)
     */
    sign_s = Sign(smc->s);
    sqrt_abs_s = sqrtf(fabsf(smc->s));

    /* 离散化积分 */
    smc->u1 += -smc->lambda * sqrt_abs_s * sign_s * dt;
    smc->u2 += -smc->alpha * sign_s * dt;

    /* 4. 输出 u = u1 + u2 */
    smc->output = smc->u1 + smc->u2;

    /* 5. 输出限幅 */
    smc->output = Clamp(smc->output, smc->output_min, smc->output_max);

    return smc->output;
}

float SuperTwisting_CalculateSimple(SuperTwistingSMC_t *smc, float r, float y, float dt)
{
    /* 使用差分近似计算速度 (仅使用位置信息)
     * 警告: 此函数使用static变量, 仅支持单实例调用!
     *       多实例请使用SuperTwisting_Calculate并手动传入y_dot
     */
    static float y_last = 0.0f;
    float y_dot_approx;

    if (dt <= 1e-6f) dt = 0.001f;  /* 防止除零 */

    y_dot_approx = (y - y_last) / dt;
    y_last = y;

    return SuperTwisting_Calculate(smc, r, y, y_dot_approx, dt);
}

void SuperTwisting_Reset(SuperTwistingSMC_t *smc)
{
    smc->s = 0.0f;
    smc->u1 = 0.0f;
    smc->u2 = 0.0f;
    smc->error = 0.0f;
    smc->error_dot = 0.0f;
    smc->output = 0.0f;
    printf("[SuperTwisting] 状态重置\n");
}
