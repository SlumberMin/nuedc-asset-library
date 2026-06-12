/**
 * @file    kalman_filter.c
 * @brief   2D卡尔曼滤波器实现
 *
 * 算法核心（矩阵运算用展开式避免通用矩阵库依赖）:
 *
 * 预测步骤:
 *   x_pred = A * x + B * u
 *   P_pred = A * P * A^T + Q
 *
 * 更新步骤（以1D观测为例，H=[h0, h1]）:
 *   S  = H * P_pred * H^T + R           (新息协方差, 标量)
 *   K  = P_pred * H^T / S                (卡尔曼增益, 2x1)
 *   y  = z - H * x_pred                  (新息/残差, 标量)
 *   x  = x_pred + K * y                  (状态更新)
 *   P  = (I - K*H) * P_pred              (协方差更新)
 */
#include "drivers/kalman_filter.h"
#include <math.h>

/* ================================================================
 * 恒速模型初始化（最常用）
 * ================================================================ */
void Kalman_Init(KalmanFilter_t *kf, float dt, float proc_noise, float meas_noise)
{
    int i, j;

    kf->dt = dt;
    kf->obs_dim = 1;
    kf->initialized = true;

    /* 状态转移矩阵 A（恒速模型）
     *   A = [1,  dt]    位置 = 旧位置 + 速度*dt
     *       [0,   1]    速度保持不变 */
    kf->A[0][0] = 1.0f;
    kf->A[0][1] = dt;
    kf->A[1][0] = 0.0f;
    kf->A[1][1] = 1.0f;

    /* 控制输入矩阵 B
     *   B = [0.5*dt^2]   加速度对位置的影响
     *       [dt      ]   加速度对速度的影响 */
    kf->B[0] = 0.5f * dt * dt;
    kf->B[1] = dt;

    /* 观测矩阵 H（默认: 仅观测位置）
     *   H = [1, 0] */
    kf->H[0][0] = 1.0f;
    kf->H[0][1] = 0.0f;
    kf->H[1][0] = 0.0f;
    kf->H[1][1] = 0.0f;

    /* 过程噪声协方差 Q（连续白噪声加速度模型离散化）
     *   Q = proc_noise * [dt^4/4, dt^3/2]
     *                    [dt^3/2, dt^2  ] */
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    kf->Q[0][0] = proc_noise * dt4 * 0.25f;
    kf->Q[0][1] = proc_noise * dt3 * 0.5f;
    kf->Q[1][0] = proc_noise * dt3 * 0.5f;
    kf->Q[1][1] = proc_noise * dt2;

    /* 观测噪声协方差 R */
    kf->R[0][0] = meas_noise;
    kf->R[0][1] = 0.0f;
    kf->R[1][0] = 0.0f;
    kf->R[1][1] = meas_noise;

    /* 初始化状态为零 */
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;

    /* 初始误差协方差 P（较大值表示初始不确定性高） */
    for (i = 0; i < 2; i++)
        for (j = 0; j < 2; j++)
            kf->P[i][j] = (i == j) ? 1.0f : 0.0f;
}

/* ================================================================
 * 自定义初始化
 * ================================================================ */
void Kalman_InitCustom(KalmanFilter_t *kf, float dt,
                       const float A[2][2], const float B[2],
                       const float Q[2][2], const float R[2][2])
{
    int i, j;

    kf->dt = dt;
    kf->obs_dim = 1;
    kf->initialized = true;

    for (i = 0; i < 2; i++) {
        for (j = 0; j < 2; j++) {
            kf->A[i][j] = A[i][j];
            kf->Q[i][j] = Q[i][j];
            kf->R[i][j] = R[i][j];
        }
        kf->B[i] = B[i];
    }

    kf->H[0][0] = 1.0f;  kf->H[0][1] = 0.0f;
    kf->H[1][0] = 0.0f;  kf->H[1][1] = 0.0f;

    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;

    for (i = 0; i < 2; i++)
        for (j = 0; j < 2; j++)
            kf->P[i][j] = (i == j) ? 1.0f : 0.0f;
}

/* ================================================================
 * 设置观测矩阵H
 * ================================================================ */
void Kalman_SetH(KalmanFilter_t *kf, const float H[2][2])
{
    int i, j;
    for (i = 0; i < 2; i++)
        for (j = 0; j < 2; j++)
            kf->H[i][j] = H[i][j];
}

/* ================================================================
 * 预测步骤（无控制输入）
 *
 * x_pred = A * x
 * P_pred = A * P * A^T + Q
 * ================================================================ */
void Kalman_Predict(KalmanFilter_t *kf)
{
    float x0 = kf->x[0], x1 = kf->x[1];

    /* x_pred = A * x */
    kf->x[0] = kf->A[0][0] * x0 + kf->A[0][1] * x1;
    kf->x[1] = kf->A[1][0] * x0 + kf->A[1][1] * x1;

    /* P_pred = A * P * A^T + Q
     * 先计算 AP = A * P */
    float ap00 = kf->A[0][0] * kf->P[0][0] + kf->A[0][1] * kf->P[1][0];
    float ap01 = kf->A[0][0] * kf->P[0][1] + kf->A[0][1] * kf->P[1][1];
    float ap10 = kf->A[1][0] * kf->P[0][0] + kf->A[1][1] * kf->P[1][0];
    float ap11 = kf->A[1][0] * kf->P[0][1] + kf->A[1][1] * kf->P[1][1];

    /* P_pred = AP * A^T + Q */
    kf->P[0][0] = ap00 * kf->A[0][0] + ap01 * kf->A[0][1] + kf->Q[0][0];
    kf->P[0][1] = ap00 * kf->A[1][0] + ap01 * kf->A[1][1] + kf->Q[0][1];
    kf->P[1][0] = ap10 * kf->A[0][0] + ap11 * kf->A[0][1] + kf->Q[1][0];
    kf->P[1][1] = ap10 * kf->A[1][0] + ap11 * kf->A[1][1] + kf->Q[1][1];
}

/* ================================================================
 * 带控制输入的预测
 *   x_pred = A * x + B * u
 * ================================================================ */
void Kalman_PredictWithInput(KalmanFilter_t *kf, float u)
{
    Kalman_Predict(kf);
    kf->x[0] += kf->B[0] * u;
    kf->x[1] += kf->B[1] * u;
}

/* ================================================================
 * 1D观测更新（最常用场景）
 *
 * 观测方程: z = H * x + v
 *   其中 H = [h0, h1]，通常为 [1, 0]
 *
 * S  = H * P * H^T + R        (新息协方差, 标量)
 * K  = P * H^T / S             (卡尔曼增益, 2x1)
 * y  = z - H * x               (新息/残差)
 * x  = x + K * y               (状态更新)
 * P  = (I - K*H) * P           (协方差更新)
 * ================================================================ */
void Kalman_Update1D(KalmanFilter_t *kf, float z)
{
    float h0 = kf->H[0][0], h1 = kf->H[0][1];

    /* 新息: y = z - H * x */
    float y = z - (h0 * kf->x[0] + h1 * kf->x[1]);

    /* 新息协方差: S = H * P * H^T + R */
    float S = h0 * (h0 * kf->P[0][0] + h1 * kf->P[1][0])
            + h1 * (h0 * kf->P[0][1] + h1 * kf->P[1][1])
            + kf->R[0][0];

    if (S < 1e-10f) S = 1e-10f;  /* 除零保护 */

    /* 卡尔曼增益: K = P * H^T / S */
    float K0 = (kf->P[0][0] * h0 + kf->P[0][1] * h1) / S;
    float K1 = (kf->P[1][0] * h0 + kf->P[1][1] * h1) / S;

    /* 状态更新: x = x + K * y */
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;

    /* 协方差更新: P = (I - K*H) * P */
    float P00 = kf->P[0][0] - K0 * (h0 * kf->P[0][0] + h1 * kf->P[1][0]);
    float P01 = kf->P[0][1] - K0 * (h0 * kf->P[0][1] + h1 * kf->P[1][1]);
    float P10 = kf->P[1][0] - K1 * (h0 * kf->P[0][0] + h1 * kf->P[1][0]);
    float P11 = kf->P[1][1] - K1 * (h0 * kf->P[0][1] + h1 * kf->P[1][1]);

    kf->P[0][0] = P00;
    kf->P[0][1] = P01;
    kf->P[1][0] = P10;
    kf->P[1][1] = P11;
}

/* ================================================================
 * 2D观测更新
 *
 * 观测方程: z = H * x + v (H为2x2)
 * 需要计算2x2矩阵的逆
 * ================================================================ */
void Kalman_Update2D(KalmanFilter_t *kf, float z[2])
{
    float h00 = kf->H[0][0], h01 = kf->H[0][1];
    float h10 = kf->H[1][0], h11 = kf->H[1][1];

    /* 新息: y = z - H * x */
    float y0 = z[0] - (h00 * kf->x[0] + h01 * kf->x[1]);
    float y1 = z[1] - (h10 * kf->x[0] + h11 * kf->x[1]);

    /* HP = H * P */
    float hp00 = h00 * kf->P[0][0] + h01 * kf->P[1][0];
    float hp01 = h00 * kf->P[0][1] + h01 * kf->P[1][1];
    float hp10 = h10 * kf->P[0][0] + h11 * kf->P[1][0];
    float hp11 = h10 * kf->P[0][1] + h11 * kf->P[1][1];

    /* S = HP * H^T + R (2x2) */
    float S00 = hp00 * h00 + hp01 * h01 + kf->R[0][0];
    float S01 = hp00 * h10 + hp01 * h11 + kf->R[0][1];
    float S10 = hp10 * h00 + hp11 * h01 + kf->R[1][0];
    float S11 = hp10 * h10 + hp11 * h11 + kf->R[1][1];

    /* S的行列式 */
    float det = S00 * S11 - S01 * S10;
    if (fabsf(det) < 1e-10f) det = 1e-10f;

    /* S^(-1) */
    float inv_S00 =  S11 / det;
    float inv_S01 = -S01 / det;
    float inv_S10 = -S10 / det;
    float inv_S11 =  S00 / det;

    /* PH^T = P * H^T */
    float pht00 = kf->P[0][0] * h00 + kf->P[0][1] * h01;
    float pht01 = kf->P[0][0] * h10 + kf->P[0][1] * h11;
    float pht10 = kf->P[1][0] * h00 + kf->P[1][1] * h01;
    float pht11 = kf->P[1][0] * h10 + kf->P[1][1] * h11;

    /* K = PH^T * S^(-1) */
    float K00 = pht00 * inv_S00 + pht01 * inv_S10;
    float K01 = pht00 * inv_S01 + pht01 * inv_S11;
    float K10 = pht10 * inv_S00 + pht11 * inv_S10;
    float K11 = pht10 * inv_S01 + pht11 * inv_S11;

    /* 状态更新: x = x + K * y */
    kf->x[0] += K00 * y0 + K01 * y1;
    kf->x[1] += K10 * y0 + K11 * y1;

    /* 协方差更新: P = (I - K*H) * P */
    float ikh00 = 1.0f - K00 * h00 - K01 * h10;
    float ikh01 =       - K00 * h01 - K01 * h11;
    float ikh10 =       - K10 * h00 - K11 * h10;
    float ikh11 = 1.0f - K10 * h01 - K11 * h11;

    kf->P[0][0] = ikh00 * kf->P[0][0] + ikh01 * kf->P[1][0];
    kf->P[0][1] = ikh00 * kf->P[0][1] + ikh01 * kf->P[1][1];
    kf->P[1][0] = ikh10 * kf->P[0][0] + ikh11 * kf->P[1][0];
    kf->P[1][1] = ikh10 * kf->P[0][1] + ikh11 * kf->P[1][1];
}

/* ================================================================
 * 获取估计不确定性（P矩阵的迹）
 * ================================================================ */
float Kalman_GetUncertainty(const KalmanFilter_t *kf)
{
    return kf->P[0][0] + kf->P[1][1];
}

/* ================================================================
 * 重置滤波器
 * ================================================================ */
void Kalman_Reset(KalmanFilter_t *kf)
{
    int i, j;
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;
    for (i = 0; i < 2; i++)
        for (j = 0; j < 2; j++)
            kf->P[i][j] = (i == j) ? 1.0f : 0.0f;
}
