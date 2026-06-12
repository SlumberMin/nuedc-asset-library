/**
 * @file luenberger_observer.c
 * @brief Luenberger观测器实现
 * 
 * 离散化实现: x_hat(k+1) = A_d*x_hat(k) + B_d*u(k) + L*(y(k) - C*x_hat(k))
 * 采用前向欧拉离散化: A_d = I + A*Ts, B_d = B*Ts
 */

#include "luenberger_observer.h"
#include <string.h>

/* 矩阵索引宏(行优先) */
#define MAT(A, row, col, cols)  ((A)[(row)*(cols)+(col)])

int LO_Init(LuenbergerObs_t *obs, int32_t n, int32_t m, int32_t p, float Ts)
{
    if (!obs || n < 1 || n > LO_MAX_N || m < 1 || m > LO_MAX_M || p < 1 || Ts <= 0)
        return -1;

    obs->n  = n;
    obs->m  = m;
    obs->p  = p;
    obs->Ts = Ts;

    memset(obs->A, 0, sizeof(obs->A));
    memset(obs->B, 0, sizeof(obs->B));
    memset(obs->C, 0, sizeof(obs->C));
    memset(obs->L, 0, sizeof(obs->L));

    /* 默认单位矩阵A */
    for (int i = 0; i < n; i++)
        MAT(obs->A, i, i, n) = 1.0f;

    LO_Reset(obs);
    return 0;
}

void LO_SetSystemMatrices(LuenbergerObs_t *obs, const float *A, const float *B, const float *C)
{
    if (!obs) return;
    int n = obs->n, m = obs->m, p = obs->p;
    if (A) memcpy(obs->A, A, n * n * sizeof(float));
    if (B) memcpy(obs->B, B, n * p * sizeof(float));
    if (C) memcpy(obs->C, C, m * n * sizeof(float));
}

void LO_SetGain(LuenbergerObs_t *obs, const float *L)
{
    if (!obs || !L) return;
    memcpy(obs->L, L, obs->n * obs->m * sizeof(float));
}

float* LO_Compute(LuenbergerObs_t *obs, const float *u, const float *y)
{
    if (!obs) return NULL;

    int n = obs->n, m = obs->m, p = obs->p;
    float Ts = obs->Ts;
    float x_new[LO_MAX_N];
    float y_hat[LO_MAX_M];

    /* y_hat = C * x_hat */
    for (int i = 0; i < m; i++) {
        y_hat[i] = 0.0f;
        for (int j = 0; j < n; j++)
            y_hat[i] += MAT(obs->C, i, j, n) * obs->x_hat[j];
    }

    /* 残差: y - y_hat */
    float innovation[LO_MAX_M];
    for (int i = 0; i < m; i++)
        innovation[i] = (y ? y[i] : 0.0f) - y_hat[i];

    /* x_hat_new = (I + A*Ts) * x_hat + B*Ts * u + L * innovation
     * 即: x_hat_new = x_hat + Ts*(A*x_hat + B*u) + L*innovation
     */
    for (int i = 0; i < n; i++) {
        /* A*x_hat */
        float Ax = 0.0f;
        for (int j = 0; j < n; j++)
            Ax += MAT(obs->A, i, j, n) * obs->x_hat[j];

        /* B*u */
        float Bu = 0.0f;
        for (int j = 0; j < p; j++)
            Bu += MAT(obs->B, i, j, p) * u[j];

        /* L*innovation */
        float Linn = 0.0f;
        for (int j = 0; j < m; j++)
            Linn += MAT(obs->L, i, j, m) * innovation[j];

        /* 前向欧拉离散化 */
        x_new[i] = obs->x_hat[i] + Ts * (Ax + Bu) + Linn;
    }

    /* 更新状态 */
    memcpy(obs->x_hat, x_new, n * sizeof(float));
    memcpy(obs->y_hat, y_hat, m * sizeof(float));

    return obs->x_hat;
}

const float* LO_GetStateEstimate(const LuenbergerObs_t *obs)
{
    return obs ? obs->x_hat : NULL;
}

void LO_SetInitialState(LuenbergerObs_t *obs, const float *x0)
{
    if (!obs || !x0) return;
    memcpy(obs->x_hat, x0, obs->n * sizeof(float));
}

void LO_Reset(LuenbergerObs_t *obs)
{
    if (!obs) return;
    memset(obs->x_hat, 0, sizeof(obs->x_hat));
    memset(obs->y_hat, 0, sizeof(obs->y_hat));
}
