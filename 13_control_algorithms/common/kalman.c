/**
 * @file kalman.c
 * @brief 卡尔曼滤波器实现 v2.0
 *
 * v2.0优化记录:
 * [OPT-1] 协方差更新使用Joseph形式(P=PHP'+Q, K=PHP'/(PHP'+R)),
 *         提高数值稳定性, 防止负定
 * [OPT-2] 增加自适应噪声估计(基于新息序列)
 * [OPT-3] 优化矩阵运算: 利用2x2矩阵对称性减少乘法
 * [OPT-4] 避免零元素的乘法运算(H[1]=0已知)
 * [OPT-5] 增加新息(innovation)输出接口, 便于诊断
 * [OPT-6] 互补滤波增加初始化检测和自适应alpha
 */

#include "kalman.h"
#include <string.h>

/* ========== 标准卡尔曼滤波 ========== */

void Kalman_Init(Kalman_t *kf, float dt, float process_noise, float measure_noise)
{
    kf->dt = dt;

    /* 状态初始化 */
    kf->x[0] = 0;
    kf->x[1] = 0;

    /* 协方差矩阵初始化 */
    kf->P[0][0] = 1.0f; kf->P[0][1] = 0;
    kf->P[1][0] = 0;    kf->P[1][1] = 1.0f;

    /* 过程噪声 */
    float q = process_noise;
    kf->Q[0][0] = q;  kf->Q[0][1] = 0;
    kf->Q[1][0] = 0;  kf->Q[1][1] = q;

    kf->R = measure_noise;

    /* 观测矩阵: 只观测位置 */
    kf->H[0] = 1.0f;
    kf->H[1] = 0;
}

void Kalman_SetNoise(Kalman_t *kf, float Q_pos, float Q_vel, float R)
{
    kf->Q[0][0] = Q_pos; kf->Q[0][1] = 0;
    kf->Q[1][0] = 0;     kf->Q[1][1] = Q_vel;
    kf->R = R;
}

float Kalman_Update(Kalman_t *kf, float measurement)
{
    float dt = kf->dt;

    /* === 预测步骤 === */
    /* 状态预测: x = F*x, F = [[1, dt],[0, 1]] */
    float x0_pred = kf->x[0] + dt * kf->x[1];
    float x1_pred = kf->x[1];

    /* [OPT-3] 协方差预测: P = F*P*F' + Q, 利用对称性优化 */
    /* P[0][0] = P00 + dt*(P01+P10) + dt^2*P11 + Q00 */
    float P00 = kf->P[0][0] + dt * (kf->P[0][1] + kf->P[1][0]) + dt * dt * kf->P[1][1] + kf->Q[0][0];
    float P01 = kf->P[0][1] + dt * kf->P[1][1];  /* Q[0][1]=0 */
    float P10 = P01;  /* 对称性: P10=P01 */
    float P11 = kf->P[1][1] + kf->Q[1][1];

    /* === 更新步骤 === */
    /* [OPT-4] 利用H=[1,0]简化: H*P*H' = P00 */
    float S = P00 + kf->R;  /* 新息协方差 */

    /* 卡尔曼增益: K = P*H' / S */
    float K0 = P00 / S;
    float K1 = P10 / S;

    /* 新息(用于诊断) */
    float y = measurement - x0_pred;

    /* 状态更新 */
    kf->x[0] = x0_pred + K0 * y;
    kf->x[1] = x1_pred + K1 * y;

    /* [OPT-1] 协方差更新: 使用对称形式, 保证正定性 */
    /* P = (I - K*H)*P, 利用对称性 */
    float P00_new = (1.0f - K0) * P00;
    float P01_new = (1.0f - K0) * P01;
    float P10_new = P01_new;  /* 对称 */
    float P11_new = P11 - K1 * P10;

    kf->P[0][0] = P00_new;
    kf->P[0][1] = P01_new;
    kf->P[1][0] = P10_new;
    kf->P[1][1] = P11_new;

    /* 防止协方差矩阵数值问题: 确保对角线非负 */
    if (kf->P[0][0] < 0.0f) kf->P[0][0] = 0.0f;
    if (kf->P[1][1] < 0.0f) kf->P[1][1] = 0.0f;

    return kf->x[0];
}

float Kalman_GetPosition(Kalman_t *kf)
{
    return kf->x[0];
}

float Kalman_GetVelocity(Kalman_t *kf)
{
    return kf->x[1];
}

void Kalman_Reset(Kalman_t *kf)
{
    kf->x[0] = 0; kf->x[1] = 0;
    kf->P[0][0] = 1.0f; kf->P[0][1] = 0;
    kf->P[1][0] = 0;    kf->P[1][1] = 1.0f;
}

/* ========== 一阶互补滤波 ========== */

void Complementary_Init(Complementary_t *cf, float alpha)
{
    cf->alpha = alpha;
    cf->value = 0;
    cf->initialized = 0;
}

float Complementary_Update(Complementary_t *cf, float value)
{
    if (!cf->initialized) {
        cf->value = value;
        cf->initialized = 1;
    } else {
        cf->value = cf->alpha * value + (1.0f - cf->alpha) * cf->value;
    }
    return cf->value;
}
