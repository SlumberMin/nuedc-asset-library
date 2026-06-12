/**
 * @file disturbance_observer.c
 * @brief 扰动观测器 (DOB) 实现
 *
 * 一阶DOB离散化:
 *   Q(z) = alpha / (1 - (1-alpha)*z^-1)    alpha = dt/(tau+dt)
 *   d_hat = Q * (y_model - y_meas)
 *
 * 二阶DOB使用双线性变换离散化状态空间。
 */
#include "disturbance_observer.h"
#include <string.h>

#define CLAMP(val, lo, hi) ((val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val)))

/* ================================================================
 *  一阶系统 DOB
 * ================================================================ */

void DOB_FirstOrder_Init(DOB_FirstOrder_t *dob, float Kn, float an,
                         float q_tau, float dt)
{
    memset(dob, 0, sizeof(DOB_FirstOrder_t));
    dob->Kn    = Kn;
    dob->an    = an;
    dob->q_tau = q_tau;
    dob->dt    = dt;
    dob->q_alpha = dt / (q_tau + dt);
    dob->initialized = 1;
}

float DOB_FirstOrder_Update(DOB_FirstOrder_t *dob, float u, float y)
{
    if (!dob->initialized) return 0.0f;

    /*
     * 标称模型输出预测:
     *   y_model(k) = (1 - an*dt)*y(k-1) + Kn*dt*u(k-1)
     */
    float y_model = (1.0f - dob->an * dob->dt) * dob->prev_y
                  + dob->Kn * dob->dt * dob->prev_u;

    /* 模型与实际输出的误差 */
    float e = y_model - y;

    /* Q滤波器: 一阶低通 */
    dob->q_state = (1.0f - dob->q_alpha) * dob->q_state
                 + dob->q_alpha * e;

    dob->disturbance_hat = dob->q_state;
    dob->prev_u = u;
    dob->prev_y = y;

    return dob->disturbance_hat;
}

float DOB_FirstOrder_GetDisturbance(DOB_FirstOrder_t *dob)
{
    return dob->disturbance_hat;
}

void DOB_FirstOrder_Reset(DOB_FirstOrder_t *dob)
{
    float Kn = dob->Kn, an = dob->an, tau = dob->q_tau, dt = dob->dt;
    DOB_FirstOrder_Init(dob, Kn, an, tau, dt);
}

/* ================================================================
 *  二阶系统 DOB (状态空间形式)
 *
 *  标称模型:
 *    x1_dot = x2
 *    x2_dot = -a0*x1 - a1*x2 + Kn*u + d
 *    y = x1
 *
 *  使用双线性变换离散化, 观测器结构:
 *    x_hat_dot = A*x_hat + B*u + L*(y - C*x_hat)
 * ================================================================ */

void DOB_SecondOrder_Init(DOB_SecondOrder_t *dob, float Kn, float a1, float a0,
                          float q_wn, float dt)
{
    memset(dob, 0, sizeof(DOB_SecondOrder_t));
    dob->Kn   = Kn;
    dob->a1   = a1;
    dob->a0   = a0;
    dob->q_wn = q_wn;
    dob->dt   = dt;
    dob->q_zeta = 0.707f;  /* Butterworth */
    dob->initialized = 1;
}

float DOB_SecondOrder_Update(DOB_SecondOrder_t *dob, float u, float y)
{
    if (!dob->initialized) return 0.0f;

    float dt = dob->dt;
    float L1 = 2.0f * dob->q_zeta * dob->q_wn;
    float L2 = dob->q_wn * dob->q_wn;

    /* 观测器状态更新 (前向Euler) */
    float e = y - dob->x1;
    float dx1 = dob->x2 + L1 * e;
    float dx2 = -dob->a0 * dob->x1 - dob->a1 * dob->x2 + dob->Kn * u + L2 * e;

    dob->x1 += dx1 * dt;
    dob->x2 += dx2 * dt;

    /* 扰动估计: d_hat = x2_dot + a0*x1 + a1*x2 - Kn*u */
    dob->disturbance_hat = dx2 + dob->a0 * dob->x1 + dob->a1 * dob->x2 - dob->Kn * u;
    dob->prev_u = u;

    return dob->disturbance_hat;
}

float DOB_SecondOrder_GetDisturbance(DOB_SecondOrder_t *dob)
{
    return dob->disturbance_hat;
}

void DOB_SecondOrder_Reset(DOB_SecondOrder_t *dob)
{
    float Kn = dob->Kn, a1 = dob->a1, a0 = dob->a0, wn = dob->q_wn, dt = dob->dt;
    DOB_SecondOrder_Init(dob, Kn, a1, a0, wn, dt);
}

/* ================================================================
 *  速度扰动观测器
 *
 *  简化的一阶DOB, 用差分估计速度, 通过滤波提取扰动分量
 * ================================================================ */

void DOB_Velocity_Init(DOB_Velocity_t *dob, float model_K, float model_tau,
                       float alpha, float dt)
{
    memset(dob, 0, sizeof(DOB_Velocity_t));
    dob->model_K   = model_K;
    dob->model_tau = model_tau;
    dob->alpha     = alpha;
    dob->dt        = dt;
    dob->initialized = 1;
}

float DOB_Velocity_Update(DOB_Velocity_t *dob, float u, float y)
{
    if (!dob->initialized) return 0.0f;

    float dt = dob->dt;

    /* 标称模型预测: y_pred = (1-dt/tau)*y + (K*dt/tau)*u */
    float alpha_model = dt / dob->model_tau;
    float y_pred = (1.0f - alpha_model) * dob->prev_y
                 + dob->model_K * alpha_model * dob->prev_u;

    /* 误差 */
    float e = y_pred - y;

    /* 低通滤波提取扰动 */
    dob->d_hat = (1.0f - dob->alpha) * dob->d_hat + dob->alpha * e;

    dob->prev_u = u;
    dob->prev_y = y;

    return dob->d_hat;
}

float DOB_Velocity_GetDisturbance(DOB_Velocity_t *dob)
{
    return dob->d_hat;
}

void DOB_Velocity_Reset(DOB_Velocity_t *dob)
{
    float mk = dob->model_K, mt = dob->model_tau, a = dob->alpha, dt = dob->dt;
    DOB_Velocity_Init(dob, mk, mt, a, dt);
}

/* ================================================================
 *  互补滤波器工具
 * ================================================================ */
float DOB_ComplementFilter(float raw, float filtered, float alpha)
{
    return alpha * raw + (1.0f - alpha) * filtered;
}
