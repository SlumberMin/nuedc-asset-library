/**
 * @file predictive_functional.c
 * @brief 预测函数控制（PFC）实现
 *
 * 实现基于一阶加纯滞后模型的PFC控制器。
 * 使用解析求解，计算量极小，适合嵌入式。
 *
 * 参考：
 * - Richalet, "Predictive Functional Control" (2009)
 * - 钱积惠, "预测控制" (化工出版社)
 * - 席裕庚, "预测控制" (国防工业出版社)
 * - GitHub: pfc-controller (MATLAB参考实现)
 */

#include "predictive_functional.h"
#include <math.h>

/* ======================== 内部辅助函数 ======================== */

static float clampf(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief 更新模型离散化系数
 *
 * 一阶模型 G(s) = K/(Ts+1) 的零阶保持离散化：
 *   y(k+1) = a*y(k) + b*u(k)
 *   a = exp(-dt/T)
 *   b = K*(1 - exp(-dt/T))
 */
static void update_model_coeffs(PFC_t *pfc)
{
    if (pfc->model_T > 0.0f) {
        pfc->model_a = expf(-pfc->dt / pfc->model_T);
    } else {
        pfc->model_a = 0.0f;
    }
    pfc->model_b = pfc->model_K * (1.0f - pfc->model_a);
}

/**
 * @brief 计算一阶模型的阶跃响应系数
 *
 * 一阶系统在恒定输入u0下的响应：
 *   y(k+j) = a^j * y(k) + b * (1-a^j)/(1-a) * u0
 *         = a^j * y(k) + K*(1-a^j) * u0
 *
 * @param pfc   控制器
 * @param step  预测步数j
 * @return 阶跃响应系数 g(j) = K*(1-a^j)
 */
static float step_response_coeff(PFC_t *pfc, uint16_t step)
{
    float a_pow = powf(pfc->model_a, (float)step);
    return pfc->model_K * (1.0f - a_pow);
}

/* ======================== 公共API实现 ======================== */

void PFC_Init(PFC_t *pfc, float dt,
              float model_K, float model_T, float model_theta,
              float Tr, uint16_t P)
{
    pfc->dt = dt;
    pfc->model_K = model_K;
    pfc->model_T = model_T;
    pfc->model_theta = model_theta;

    /* 计算纯滞后步数 */
    if (dt <= 0.0f) dt = 0.001f;  /* 防止除零 */
    pfc->dead_steps = (uint16_t)(model_theta / dt + 0.5f);

    /* 自动设置参考轨迹时间常数 */
    if (Tr <= 0.0f) {
        Tr = model_T * 0.5f;  /* 默认取0.5倍时间常数 */
    }
    pfc->Tr = Tr;
    pfc->alpha = expf(-dt / Tr);

    /* 自动设置预测时域 */
    if (P == 0) {
        if (dt <= 0.0f) dt = 0.001f;  /* 防止除零 */
        P = (uint16_t)(3.0f * model_T / dt);  /* 约3倍时间常数 */
        if (P > PFC_PREDICTION_HORIZON) P = PFC_PREDICTION_HORIZON;
        if (P < 5) P = 5;
    }
    pfc->P = P;

    /* 基函数类型 */
    pfc->basis_type = PFC_BASIS_STEP;

    /* 模型离散化 */
    update_model_coeffs(pfc);
    pfc->model_state = 0.0f;

    /* 初始状态 */
    pfc->prev_u = 0.0f;
    pfc->prev_y = 0.0f;
    pfc->y_sp = 0.0f;

    /* 输出限幅 */
    pfc->output_min = -1.0f;
    pfc->output_max = 1.0f;

    /* 清零缓存 */
    for (uint16_t i = 0; i < PFC_PREDICTION_HORIZON; i++) {
        pfc->y_free[i] = 0.0f;
        pfc->y_ref[i] = 0.0f;
    }
}

float PFC_Compute(PFC_t *pfc, float setpoint, float feedback)
{
    float dt = pfc->dt;
    float y = feedback;
    float y_sp = setpoint;

    pfc->y_sp = y_sp;

    /* ========== Step 1: 更新内部模型 ==========
     * 模型状态更新：x(k+1) = a*x(k) + b*u(k)
     * 模型输出：ŷ_model(k) = x(k)
     */
    pfc->model_state = pfc->model_a * pfc->model_state
                     + pfc->model_b * pfc->prev_u;
    float y_model = pfc->model_state;

    /* ========== Step 2: 反馈校正 ==========
     * 用当前时刻的实测值与模型输出的偏差进行校正
     * ε(k) = y(k) - y_model(k)
     * 校正后的预测 = 自由响应 + ε(k)
     */
    float epsilon = y - y_model;

    /* ========== Step 3: 计算参考轨迹 ==========
     * y_r(k+j) = y_sp - (y_sp - y(k)) * α^j
     * 指数趋近参考轨迹
     */
    float error_0 = y_sp - y;
    for (uint16_t j = 1; j <= pfc->P; j++) {
        pfc->y_ref[j - 1] = y_sp - error_0 * powf(pfc->alpha, (float)j);
    }

    /* ========== Step 4: 计算自由响应（无新控制增量时的预测） ==========
     * 自由响应 = 模型在当前控制量持续作用下的输出 + 反馈校正
     * y_free(k+j) = a^j * y_model(k) + K*(1-a^j) * u(k-1) + ε(k)
     *
     * 对于带纯滞后的系统，延迟dead_steps步后才有效果
     */
    for (uint16_t j = 1; j <= pfc->P; j++) {
        float a_pow = powf(pfc->model_a, (float)j);
        /* 模型自由响应 */
        float y_model_j = a_pow * y_model
                        + step_response_coeff(pfc, j) * pfc->prev_u;
        /* 反馈校正（假设误差在预测时域内保持不变） */
        pfc->y_free[j - 1] = y_model_j + epsilon;
    }

    /* ========== Step 5: 计算控制增量 ==========
     *
     * PFC的解析解（对于阶跃基函数）：
     * Δu(k) = [Σ(y_r(k+j) - y_free(k+j))] / [Σ g(j)]
     *   j从dead_steps+1到P
     *
     * 这里g(j)是模型的阶跃响应系数
     */
    float numerator = 0.0f;
    float denominator = 0.0f;

    uint16_t start_j = pfc->dead_steps + 1;  /* 从纯滞后之后开始 */
    if (start_j < 1) start_j = 1;

    for (uint16_t j = start_j; j <= pfc->P; j++) {
        float y_ref_j = pfc->y_ref[j - 1];
        float y_free_j = pfc->y_free[j - 1];
        float g_j = step_response_coeff(pfc, j);

        numerator += (y_ref_j - y_free_j);
        denominator += g_j;
    }

    /* 防止除零 */
    if (fabsf(denominator) < 1e-10f) {
        denominator = 1e-10f;
    }

    float delta_u = numerator / denominator;

    /* ========== Step 6: 计算最终控制量 ========== */
    float u = pfc->prev_u + delta_u;

    /* 输出限幅 */
    u = clampf(u, pfc->output_min, pfc->output_max);

    /* 保存状态 */
    pfc->prev_u = u;
    pfc->prev_y = y;

    return u;
}

void PFC_Reset(PFC_t *pfc)
{
    pfc->model_state = 0.0f;
    pfc->prev_u = 0.0f;
    pfc->prev_y = 0.0f;

    for (uint16_t i = 0; i < PFC_PREDICTION_HORIZON; i++) {
        pfc->y_free[i] = 0.0f;
        pfc->y_ref[i] = 0.0f;
    }
}

void PFC_SetOutputLimits(PFC_t *pfc, float min_val, float max_val)
{
    pfc->output_min = min_val;
    pfc->output_max = max_val;
}

void PFC_SetTr(PFC_t *pfc, float Tr)
{
    pfc->Tr = Tr;
    pfc->alpha = expf(-pfc->dt / Tr);
}

void PFC_SetBasisType(PFC_t *pfc, PFC_BasisType_t type)
{
    pfc->basis_type = type;
}
