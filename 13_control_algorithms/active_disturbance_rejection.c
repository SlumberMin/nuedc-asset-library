/**
 * @file active_disturbance_rejection.c
 * @brief ADRC V2自抗扰控制器实现
 *
 * 改进点：
 * 1. 使用fhan()实现最速离散跟踪微分器（TD）
 * 2. 使用fal()非线性函数改进ESO
 * 3. 非线性状态误差反馈（NLSEF）+ 扰动补偿
 *
 * 参考：
 * - 韩京清, "自抗扰控制技术——估计补偿不确定因素的控制技术"
 * - Gao, "Scaling and Parameterization Based Controller Tuning" (2006)
 * - Han, "From PID to Active Disturbance Rejection Control" (2009)
 * - GitHub: ADRC-cppn/adrc (开源ADRC实现)
 * - GitHub: xiaomao996688/ADRC (C语言ADRC参考)
 */

#include "active_disturbance_rejection.h"
#include <math.h>

/* ======================== 内部辅助函数 ======================== */

static float clampf(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief fhan函数 — 最速离散系统函数（韩京清）
 *
 * 用于跟踪微分器（TD）和非线性组合，是ADRC的核心函数之一。
 * 实现了离散最速综合函数，可以在有限时间内无超调地跟踪目标。
 *
 * @param x1    状态1（位置/信号）
 * @param x2    状态2（速度/微分）
 * @param r     速度因子（越大跟踪越快）
 * @param h     滤波步长（越大滤波越强）
 * @return      最速控制量
 */
static float fhan(float x1, float x2, float r, float h)
{
    float d = r * h * h;
    if (d < 1e-10f) d = 1e-10f; /* 防止d=0导致除零 */
    float a0 = h * x2;
    float y = x1 + a0;
    float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    float a2 = a0 + (y > 0 ? 1.0f : -1.0f) * (a1 - d) * 0.5f;
    float sy = (y > 0 ? 1.0f : (y < 0 ? -1.0f : 0.0f));
    float sa2 = (a2 > 0 ? 1.0f : (a2 < 0 ? -1.0f : 0.0f));
    float result;

    if (fabsf(y) >= d) {
        result = -r * sa2;
    } else {
        result = -r * y / d;
    }

    /* 处理a和y符号不同的情况 */
    float a = (fabsf(y) >= d) ? a2 : a0 + y;
    if (sy * sa2 < 0) {
        result = -r * a / d;
    }

    return result;
}

/**
 * @brief fal函数 — 非线性映射函数
 *
 * fal(e, α, δ) 的特性：
 * - |e| > δ 时: sign(e) * |e|^α（大误差时非线性放大）
 * - |e| ≤ δ 时: e / δ^(1-α)（小误差时线性，避免抖振）
 *
 * 这个函数实现了"大误差小增益，小误差大增益"的效果，
 * 类似于变结构控制的思想。
 *
 * @param e     误差输入
 * @param alpha 指数 (0<α<1时具有大误差小增益特性)
 * @param delta 线性区宽度
 * @return      fal函数值
 */
static float fal(float e, float alpha, float delta)
{
    if (fabsf(e) > delta) {
        float sign = (e > 0) ? 1.0f : -1.0f;
        return sign * powf(fabsf(e), alpha);
    } else {
        return e / powf(delta, 1.0f - alpha);
    }
}

/**
 * @brief 更新ESO增益（基于带宽配置化方法）
 *
 * 使用Gao的带宽法：
 *   β1 = 3*ωo
 *   β2 = 3*ωo²
 *   β3 = ωo³
 *
 * 这保证ESO的特征多项式为 (s+ωo)³，三重极点在-ωo
 */
static void update_eso_gains(ADRC_t *adrc)
{
    float wo = adrc->eso_omega_o;
    adrc->eso_beta1 = 3.0f * wo;
    adrc->eso_beta2 = 3.0f * wo * wo;
    adrc->eso_beta3 = wo * wo * wo;
}

/**
 * @brief 更新NLSEF增益
 *
 * 使用带宽法：
 *   k1 = ωc²
 *   k2 = 2*ωc
 */
static void update_nlsef_gains(ADRC_t *adrc)
{
    float wc = adrc->nl_omega_c;
    adrc->nl_k1 = wc * wc;
    adrc->nl_k2 = 2.0f * wc;
}

/* ======================== 公共API实现 ======================== */

void ADRC_Init(ADRC_t *adrc, float dt, float b0,
               float omega_c, float omega_o)
{
    adrc->dt = dt;

    /* 控制增益估计 */
    adrc->eso_b0 = (b0 == 0.0f) ? 1.0f : b0;

    /* TD初始化 */
    adrc->td_x1 = 0.0f;
    adrc->td_x2 = 0.0f;
    adrc->td_r = 100.0f;  /* 跟踪速度因子 */
    adrc->td_h = dt;       /* 滤波因子=采样周期 */

    /* ESO带宽设置 */
    if (omega_o <= 0.0f) {
        if (dt <= 0.0f) dt = 0.001f;  /* 防除零 */
        omega_o = 10.0f / dt;  /* 默认：10倍采样频率的带宽 */
    }
    adrc->eso_omega_o = omega_o;
    adrc->eso_z1 = 0.0f;
    adrc->eso_z2 = 0.0f;
    adrc->eso_z3 = 0.0f;
    update_eso_gains(adrc);

    /* NLSEF设置 */
    if (omega_c <= 0.0f) {
        omega_c = omega_o / 3.0f;  /* 默认：ESO带宽的1/3 */
    }
    adrc->nl_omega_c = omega_c;
    adrc->nl_alpha1 = 0.5f;   /* fal指数 */
    adrc->nl_alpha2 = 0.25f;
    adrc->nl_delta = 0.01f;   /* fal线性区宽度 */
    update_nlsef_gains(adrc);

    /* 输出限幅 */
    adrc->output_min = -1.0f;
    adrc->output_max = 1.0f;
    adrc->u0_prev = 0.0f;
}

float ADRC_Compute(ADRC_t *adrc, float setpoint, float feedback)
{
    float dt = adrc->dt;
    float v0 = setpoint;  /* 设定值 */

    /* ========== Step 1: 跟踪微分器（TD） ==========
     * 对设定值进行平滑跟踪，生成过渡信号v1和微分v2
     * 使用fhan实现最速离散跟踪：
     *   fh = fhan(v1-v0, v2, r, h)
     *   v1 = v1 + h*v2
     *   v2 = v2 + h*fh
     */
    float e_td = adrc->td_x1 - v0;
    float fh = fhan(e_td, adrc->td_x2, adrc->td_r, adrc->td_h);
    adrc->td_x1 += dt * adrc->td_x2;
    adrc->td_x2 += dt * fh;

    float v1 = adrc->td_x1;  /* 跟踪信号 */
    float v2 = adrc->td_x2;  /* 跟踪信号微分 */

    /* ========== Step 2: 扩张状态观测器（ESO） ==========
     * 三阶ESO估计系统状态z1, z2和总扰动z3
     *
     * 系统模型：x1' = x2, x2' = f(x1,x2,w,t) + b0*u
     * 其中f为总扰动（含未知动态+外部扰动）
     *
     * ESO更新方程（使用fal非线性函数）：
     *   e = z1 - y
     *   z1' = z2 - β1*e
     *   z2' = z3 - β2*fal(e,α1,δ) + b0*u
     *   z3' = -β3*fal(e,α2,δ)
     */
    float y = feedback;
    float eso_e = adrc->eso_z1 - y;

    /* 使用fal函数的改进ESO */
    float fal_e1 = fal(eso_e, adrc->nl_alpha1, adrc->nl_delta);
    float fal_e2 = fal(eso_e, adrc->nl_alpha2, adrc->nl_delta);

    float z1_dot = adrc->eso_z2 - adrc->eso_beta1 * eso_e;
    float z2_dot = adrc->eso_z3 - adrc->eso_beta2 * fal_e1
                   + adrc->eso_b0 * adrc->u0_prev;
    float z3_dot = -adrc->eso_beta3 * fal_e2;

    /* 欧拉法更新ESO状态 */
    adrc->eso_z1 += dt * z1_dot;
    adrc->eso_z2 += dt * z2_dot;
    adrc->eso_z3 += dt * z3_dot;

    /* ESO状态限幅（防止发散） */
    adrc->eso_z3 = clampf(adrc->eso_z3,
                          -fabsf(adrc->output_max) * 10.0f,
                           fabsf(adrc->output_max) * 10.0f);

    /* ========== Step 3: 非线性状态误差反馈（NLSEF） ==========
     * 状态误差：e1 = v1 - z1, e2 = v2 - z2
     * 非线性组合：u0 = k1*fal(e1,α1,δ) + k2*fal(e2,α2,δ)
     * 扰动补偿：u = (u0 - z3) / b0
     */
    float e1 = v1 - adrc->eso_z1;
    float e2 = v2 - adrc->eso_z2;

    float u0 = adrc->nl_k1 * fal(e1, adrc->nl_alpha1, adrc->nl_delta)
             + adrc->nl_k2 * fal(e2, adrc->nl_alpha2, adrc->nl_delta);

    /* 扰动补偿：减去ESO估计的总扰动 */
    float u = (u0 - adrc->eso_z3) / adrc->eso_b0;

    /* 输出限幅 */
    u = clampf(u, adrc->output_min, adrc->output_max);

    adrc->u0_prev = u;

    return u;
}

void ADRC_Reset(ADRC_t *adrc)
{
    adrc->td_x1 = 0.0f;
    adrc->td_x2 = 0.0f;
    adrc->eso_z1 = 0.0f;
    adrc->eso_z2 = 0.0f;
    adrc->eso_z3 = 0.0f;
    adrc->u0_prev = 0.0f;
}

void ADRC_SetOutputLimits(ADRC_t *adrc, float min_val, float max_val)
{
    adrc->output_min = min_val;
    adrc->output_max = max_val;
}

void ADRC_SetEsoBandwidth(ADRC_t *adrc, float omega_o)
{
    adrc->eso_omega_o = omega_o;
    update_eso_gains(adrc);
}

void ADRC_SetControlBandwidth(ADRC_t *adrc, float omega_c)
{
    adrc->nl_omega_c = omega_c;
    update_nlsef_gains(adrc);
}

void ADRC_SetB0(ADRC_t *adrc, float b0)
{
    adrc->eso_b0 = (b0 == 0.0f) ? 1e-6f : b0;
}

float ADRC_GetDisturbanceEstimate(ADRC_t *adrc)
{
    return adrc->eso_z3;
}
