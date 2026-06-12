/**
 * @file observer.c
 * @brief 龙伯格状态观测器实现
 */

#include "observer.h"
#include <string.h>

/* ========== 通用观测器 ========== */

void Observer_Init(LuenbergerObserver_t *obs, uint8_t n, uint8_t p, uint8_t m, float dt)
{
    if (obs == NULL) return;
    obs->n = n;
    obs->p = p;
    obs->m = m;
    obs->dt = dt;
    memset(obs->A, 0, sizeof(obs->A));
    memset(obs->B, 0, sizeof(obs->B));
    memset(obs->C, 0, sizeof(obs->C));
    memset(obs->L, 0, sizeof(obs->L));
    memset(obs->x_hat, 0, sizeof(obs->x_hat));
}

void Observer_SetA(LuenbergerObserver_t *obs, const float *A_data)
{
    for (uint8_t i = 0; i < obs->n; i++)
        for (uint8_t j = 0; j < obs->n; j++)
            obs->A[i][j] = A_data[i * obs->n + j];
}

void Observer_SetB(LuenbergerObserver_t *obs, const float *B_data)
{
    for (uint8_t i = 0; i < obs->n; i++)
        for (uint8_t j = 0; j < obs->m; j++)
            obs->B[i][j] = B_data[i * obs->m + j];
}

void Observer_SetC(LuenbergerObserver_t *obs, const float *C_data)
{
    for (uint8_t i = 0; i < obs->p; i++)
        for (uint8_t j = 0; j < obs->n; j++)
            obs->C[i][j] = C_data[i * obs->n + j];
}

void Observer_SetL(LuenbergerObserver_t *obs, const float *L_data)
{
    for (uint8_t i = 0; i < obs->n; i++)
        for (uint8_t j = 0; j < obs->p; j++)
            obs->L[i][j] = L_data[i * obs->p + j];
}

void Observer_SetInitialState(LuenbergerObserver_t *obs, const float *x0)
{
    for (uint8_t i = 0; i < obs->n; i++)
        obs->x_hat[i] = x0[i];
}

void Observer_Update(LuenbergerObserver_t *obs, const float *u, const float *y)
{
    if (obs == NULL || u == NULL || y == NULL) return;
    uint8_t n = obs->n, p = obs->p, m = obs->m;
    float x_new[OBS_MAX_STATES] = {0};

    /* 计算 y_hat = C * x_hat */
    float y_hat[OBS_MAX_OUTPUTS] = {0};
    for (uint8_t i = 0; i < p; i++)
        for (uint8_t j = 0; j < n; j++)
            y_hat[i] += obs->C[i][j] * obs->x_hat[j];

    /* 计算创新(残差): y - y_hat */
    float innovation[OBS_MAX_OUTPUTS];
    for (uint8_t i = 0; i < p; i++)
        innovation[i] = y[i] - y_hat[i];

    /* x_new = A*x_hat + B*u + L*innovation */
    for (uint8_t i = 0; i < n; i++) {
        float sum = 0.0f;
        for (uint8_t j = 0; j < n; j++)
            sum += obs->A[i][j] * obs->x_hat[j];
        for (uint8_t j = 0; j < m; j++)
            sum += obs->B[i][j] * u[j];
        for (uint8_t j = 0; j < p; j++)
            sum += obs->L[i][j] * innovation[j];
        x_new[i] = sum;
    }

    /* 前向欧拉离散化: x_hat[k+1] = x_hat[k] + dt * x_new */
    for (uint8_t i = 0; i < n; i++)
        obs->x_hat[i] += obs->dt * x_new[i];
}

void Observer_GetState(const LuenbergerObserver_t *obs, float *x_hat)
{
    for (uint8_t i = 0; i < obs->n; i++)
        x_hat[i] = obs->x_hat[i];
}

/* ========== 一阶速度观测器(简化) ========== */

void VelocityObserver_Init(VelocityObserver_t *vo, float L1, float L2, float dt)
{
    vo->x1_hat = 0.0f;
    vo->x2_hat = 0.0f;
    vo->L1 = L1;
    vo->L2 = L2;
    vo->dt = dt;
}

float VelocityObserver_Update(VelocityObserver_t *vo, float position_meas)
{
    /* 观测器模型:
     * x1_dot = x2
     * x2_dot = 0  (假设匀速)
     *
     * 带反馈:
     * x1_hat_dot = x2_hat + L1*(y - x1_hat)
     * x2_hat_dot =            L2*(y - x1_hat)
     */
    float innovation = position_meas - vo->x1_hat;
    vo->x1_hat += (vo->x2_hat + vo->L1 * innovation) * vo->dt;
    vo->x2_hat += (vo->L2 * innovation) * vo->dt;

    return vo->x2_hat;
}

float VelocityObserver_GetVelocity(const VelocityObserver_t *vo)
{
    return vo->x2_hat;
}

/* ========== 扭矩/负载观测器(简化) ========== */

void TorqueObserver_Init(TorqueObserver_t *to, float J, float L1, float L2, float dt)
{
    to->omega_hat = 0.0f;
    to->tau_hat = 0.0f;
    to->J = J;
    to->L1 = L1;
    to->L2 = L2;
    to->dt = dt;
}

void TorqueObserver_Update(TorqueObserver_t *to, float omega_meas, float u_voltage)
{
    /* 电机模型: J * dω/dt = Kt*u - τ_load
     * 观测器:
     *   dω_hat/dt = (Kt/J)*u - τ_hat/J + L1*(ω - ω_hat)
     *   dτ_hat/dt =                          L2*(ω - ω_hat)
     *
     * 简化: Kt/J 合并到 u (这里假设 u 已经是归一化力矩输入)
     */
    float innovation = omega_meas - to->omega_hat;
    to->omega_hat += ((u_voltage - to->tau_hat / to->J) + to->L1 * innovation) * to->dt;
    to->tau_hat += (to->L2 * innovation) * to->dt;
}

float TorqueObserver_GetTorque(const TorqueObserver_t *to)
{
    return to->tau_hat;
}
