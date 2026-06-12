/**
 * @file model_free_adaptive.c
 * @brief 无模型自适应控制（MFAC）实现
 *
 * 基于紧格式动态线性化（CFDL-MFAC）方法实现。
 *
 * 参考：
 * - 侯忠生, "无模型自适应控制——理论与应用" (2013)
 * - Hou & Jin, "Model Free Adaptive Control" (CRC Press, 2013)
 * - Hou & Wang, "Closed-loop dynamic linearization and MFAC" (2013)
 * - GitHub: zhaozhongch/mfac_control
 */

#include "model_free_adaptive.h"
#include <math.h>

/* ======================== 内部辅助函数 ======================== */

static float clampf(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief sign函数
 */
static float signf(float x)
{
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

/* ======================== 公共API实现 ======================== */

void MFAC_Init(MFAC_t *mfac, float dt)
{
    mfac->dt = dt;

    /* 默认控制参数 */
    mfac->eta = 0.3f;       /* 学习率 */
    mfac->mu = 0.1f;        /* 控制量权重 */
    mfac->rho = 0.9f;       /* 学习率衰减 */
    mfac->lambda_f = 0.98f; /* 遗忘因子 */

    /* PPD参数 */
    mfac->phi_init = 1.0f;
    mfac->phi = mfac->phi_init;
    mfac->phi_min = 0.01f;   /* PPD下限，防止符号翻转 */
    mfac->phi_max = 100.0f;  /* PPD上限 */
    mfac->P = 1.0f;          /* 初始协方差 */

    /* 状态清零 */
    mfac->prev_u = 0.0f;
    mfac->prev_y = 0.0f;
    mfac->prev_dy = 0.0f;
    mfac->prev_du = 0.0f;
    mfac->prev2_u = 0.0f;
    mfac->prev2_y = 0.0f;
    mfac->init_flag = 0;

    /* 输出限幅 */
    mfac->output_min = -1.0f;
    mfac->output_max = 1.0f;
    mfac->delta_u_max = 0.1f;  /* 单步最大控制增量 */

    /* 调试信息 */
    mfac->debug_phi = mfac->phi;
}

float MFAC_Compute(MFAC_t *mfac, float setpoint, float feedback)
{
    float y = feedback;
    float y_ref = setpoint;

    /* === 首次调用，仅保存状态 === */
    if (!mfac->init_flag) {
        mfac->prev_y = y;
        /* prev_u 保持MFAC_Init设定的初始值（0.0f） */
        mfac->init_flag = 1;
        return mfac->prev_u;
    }

    /* === 计算输出增量 Δy(k) = y(k) - y(k-1) === */
    float dy = y - mfac->prev_y;

    /* === 更新伪偏导数（PPD）φ(k) ===
     * 使用带遗忘因子的投影算法估计PPD：
     *
     * φ̂(k) = φ̂(k-1) + (η * Δu(k-1)) / (μ + |Δu(k-1)|²)
     *         * (Δy(k) - φ̂(k-1)*Δu(k-1))
     *
     * 这是简化的梯度自适应律，不需要矩阵运算。
     * μ是防止除零的正则化参数。
     */
    float du_prev = mfac->prev_du;

    float denom = mfac->mu + du_prev * du_prev;
    if (fabsf(denom) < 1e-10f) {
        denom = 1e-10f;
    }

    /* PPD估计误差 */
    float phi_error = dy - mfac->phi * du_prev;

    /* PPD更新（带遗忘因子） */
    mfac->phi += mfac->eta * du_prev / denom * phi_error;

    /* PPD限幅（重要：保持符号一致性） */
    if (fabsf(mfac->phi) < mfac->phi_min) {
        /* 使用phi_error的符号推断PPD应有符号，而非用phi自身+epsilon
         * phi_error = dy - phi*du_prev，若phi_error与du_prev同号，
         * 说明phi偏小（应为正），否则phi偏大（应为负） */
        float inferred_sign = signf(phi_error * du_prev);
        if (inferred_sign == 0.0f) inferred_sign = 1.0f; /* 默认正号 */
        mfac->phi = mfac->phi_init * inferred_sign;
    }
    mfac->phi = clampf(mfac->phi, mfac->phi_min, mfac->phi_max);

    /* 保存调试信息 */
    mfac->debug_phi = mfac->phi;

    /* === 计算控制增量 Δu(k) ===
     * 基于动态线性化模型 Δy(k+1) = φ(k)*Δu(k)
     * 最优控制增量使性能指标 J 最小：
     *
     * J = |y_ref - y(k+1)|² + μ*|Δu(k)|²
     * 代入 Δy(k+1) = φ(k)*Δu(k)，y(k+1) = y(k) + Δy(k+1)：
     *
     * Δu(k) = ρ * φ(k) / (μ + φ(k)²) * (y_ref - y(k))
     *
     * ρ是学习率衰减因子。
     */
    float phi_sq = mfac->phi * mfac->phi;
    float denom2 = mfac->mu + phi_sq;
    if (fabsf(denom2) < 1e-10f) {
        denom2 = 1e-10f;
    }

    float error = y_ref - y;
    float delta_u = mfac->rho * mfac->phi / denom2 * error;

    /* 控制增量限幅 */
    delta_u = clampf(delta_u, -mfac->delta_u_max, mfac->delta_u_max);

    /* === 计算控制量 === */
    float u = mfac->prev_u + delta_u;

    /* 输出限幅 */
    u = clampf(u, mfac->output_min, mfac->output_max);

    /* === 保存状态 === */
    mfac->prev2_y = mfac->prev_y;
    mfac->prev_y = y;
    mfac->prev_dy = dy;
    mfac->prev2_u = mfac->prev_u;
    mfac->prev_u = u;
    mfac->prev_du = delta_u;

    return u;
}

void MFAC_Reset(MFAC_t *mfac)
{
    mfac->phi = mfac->phi_init;
    mfac->P = 1.0f;
    mfac->prev_u = 0.0f;
    mfac->prev_y = 0.0f;
    mfac->prev_dy = 0.0f;
    mfac->prev_du = 0.0f;
    mfac->prev2_u = 0.0f;
    mfac->prev2_y = 0.0f;
    mfac->init_flag = 0;
}

void MFAC_SetParams(MFAC_t *mfac, float eta, float mu, float rho)
{
    mfac->eta = clampf(eta, 0.001f, 1.0f);
    mfac->mu = clampf(mu, 1e-6f, 10.0f);
    mfac->rho = clampf(rho, 0.01f, 1.0f);
}

void MFAC_SetOutputLimits(MFAC_t *mfac, float min_val, float max_val)
{
    mfac->output_min = min_val;
    mfac->output_max = max_val;
}

float MFAC_GetPPD(MFAC_t *mfac)
{
    return mfac->debug_phi;
}
