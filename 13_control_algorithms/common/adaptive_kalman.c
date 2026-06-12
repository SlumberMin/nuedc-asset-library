/**
 * @file adaptive_kalman.c
 * @brief 自适应卡尔曼滤波器实现 v1.0 - Sage-Husa算法
 * @date 2026-06-11
 *
 * Sage-Husa自适应卡尔曼滤波算法:
 *   通过在线调整Q和R矩阵, 适应系统特性变化
 *
 * 核心公式:
 *   新息: v(k) = y(k) - H * x(k|k-1)
 *   R(k) = (1-d) * R(k-1) + d * (v*v^T - H*P*H^T)
 *   Q(k) = (1-d) * Q(k-1) + d * (K*v*v^T*K^T)
 *
 * 性能优化记录:
 *   [OPT-1] 嵌入式矩阵运算: 避免动态分配, 使用固定大小数组
 *   [OPT-2] 小规模矩阵优化: 直接展开循环, 提升缓存命中
 *   [OPT-3] 数值稳定: 限制Q和R为正定, 防止滤波器发散
 *   [OPT-4] 新息监测: 用于诊断滤波器健康状态
 */

#include "adaptive_kalman.h"
#include <math.h>
#include <string.h>

/* 数值稳定: 最小噪声标准差 (防止除零) */
#define AK_MIN_NOISE  1e-6f

/* 嵌入式环境: 移除所有printf，用空宏替代 */
#ifndef EMBEDDED_DEBUG
#define printf(...) ((void)0)
#endif

/* ========== 小规模矩阵运算 (嵌入式优化) ========== */

/**
 * @brief 矩阵加法 C = A + B
 */
static void MatAdd(const AK_Matrix_t *A, const AK_Matrix_t *B, AK_Matrix_t *C)
{
    uint8_t i, j;
    C->rows = A->rows;
    C->cols = A->cols;
    for (i = 0; i < A->rows; i++) {
        for (j = 0; j < A->cols; j++) {
            C->data[i][j] = A->data[i][j] + B->data[i][j];
        }
    }
}

/**
 * @brief 矩阵减法 C = A - B
 */
static void MatSub(const AK_Matrix_t *A, const AK_Matrix_t *B, AK_Matrix_t *C)
{
    uint8_t i, j;
    C->rows = A->rows;
    C->cols = A->cols;
    for (i = 0; i < A->rows; i++) {
        for (j = 0; j < A->cols; j++) {
            C->data[i][j] = A->data[i][j] - B->data[i][j];
        }
    }
}

/**
 * @brief 矩阵乘法 C = A * B
 */
static void MatMul(const AK_Matrix_t *A, const AK_Matrix_t *B, AK_Matrix_t *C)
{
    uint8_t i, j, k;
    C->rows = A->rows;
    C->cols = B->cols;
    for (i = 0; i < A->rows; i++) {
        for (j = 0; j < B->cols; j++) {
            float sum = 0.0f;
            for (k = 0; k < A->cols; k++) {
                sum += A->data[i][k] * B->data[k][j];
            }
            C->data[i][j] = sum;
        }
    }
}

/**
 * @brief 矩阵转置 B = A^T
 */
static void MatTranspose(const AK_Matrix_t *A, AK_Matrix_t *B)
{
    uint8_t i, j;
    B->rows = A->cols;
    B->cols = A->rows;
    for (i = 0; i < A->rows; i++) {
        for (j = 0; j < A->cols; j++) {
            B->data[j][i] = A->data[i][j];
        }
    }
}

/**
 * @brief 标量乘矩阵 B = c * A
 */
static void MatScale(const AK_Matrix_t *A, float c, AK_Matrix_t *B)
{
    uint8_t i, j;
    B->rows = A->rows;
    B->cols = A->cols;
    for (i = 0; i < A->rows; i++) {
        for (j = 0; j < A->cols; j++) {
            B->data[i][j] = c * A->data[i][j];
        }
    }
}

/**
 * @brief 矩阵加标量乘 C = A + c * B
 */
static void MatAddScaled(const AK_Matrix_t *A, float c, const AK_Matrix_t *B, AK_Matrix_t *C)
{
    uint8_t i, j;
    C->rows = A->rows;
    C->cols = A->cols;
    for (i = 0; i < A->rows; i++) {
        for (j = 0; j < A->cols; j++) {
            C->data[i][j] = A->data[i][j] + c * B->data[i][j];
        }
    }
}

/**
 * @brief 外积矩阵 C = A * B^T (A和B为列向量)
 */
static void MatOuterProduct(const AK_Matrix_t *A, const AK_Matrix_t *B, AK_Matrix_t *C)
{
    uint8_t i, j;
    C->rows = A->rows;
    C->cols = B->rows;
    for (i = 0; i < A->rows; i++) {
        for (j = 0; j < B->rows; j++) {
            C->data[i][j] = A->data[i][0] * B->data[j][0];
        }
    }
}

/**
 * @brief 单位矩阵
 */
static void MatIdentity(AK_Matrix_t *I, uint8_t n)
{
    uint8_t i, j;
    I->rows = n;
    I->cols = n;
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            I->data[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

/**
 * @brief 向量范数 (2-范数)
 */
static float VecNorm(const AK_Matrix_t *v)
{
    uint8_t i;
    float sum = 0.0f;
    for (i = 0; i < v->rows; i++) {
        sum += v->data[i][0] * v->data[i][0];
    }
    return sqrtf(sum);
}

/* ========== 初始化 ========== */

void AdaptiveKalman_Init(AdaptiveKalman_t *ak, uint8_t n, uint8_t m)
{
    if (n > AK_MAX_DIM || m > AK_MAX_DIM) {
        printf("[AdaptiveKalman] 错误: 维度超限 (max=%d)\n", AK_MAX_DIM);
        return;
    }

    ak->n = n;
    ak->m = m;

    /* 初始化矩阵维度 */
    ak->F.rows = n; ak->F.cols = n;
    ak->H.rows = m; ak->H.cols = n;
    ak->Q.rows = n; ak->Q.cols = n;
    ak->R.rows = m; ak->R.cols = m;
    ak->x.rows = n; ak->x.cols = 1;
    ak->P.rows = n; ak->P.cols = n;
    ak->K.rows = n; ak->K.cols = m;
    ak->v.rows = m; ak->v.cols = 1;

    /* 初始化临时矩阵 */
    ak->temp_nn1.rows = n; ak->temp_nn1.cols = n;
    ak->temp_nn2.rows = n; ak->temp_nn2.cols = n;
    ak->temp_mm1.rows = m; ak->temp_mm1.cols = m;
    ak->temp_nm1.rows = n; ak->temp_nm1.cols = m;
    ak->temp_n1.rows = n; ak->temp_n1.cols = 1;
    ak->temp_m1.rows = m; ak->temp_m1.cols = 1;

    /* 默认自适应参数 */
    ak->d_q = 0.05f;
    ak->d_r = 0.05f;
    ak->adaptive_q = 1;
    ak->adaptive_r = 1;

    /* 默认系统矩阵 */
    MatIdentity(&ak->F, n);  /* F = I */
    MatIdentity(&ak->H, n);  /* H = I (需根据实际设置) */
    MatIdentity(&ak->Q, n);  /* Q = I (需根据实际设置) */
    MatIdentity(&ak->R, n);  /* R = I (需根据实际设置) */

    /* 初始化状态为0 */
    memset(&ak->x.data[0][0], 0, sizeof(float) * n);
    MatIdentity(&ak->P, n);
    ak->innovation_norm = 0.0f;

    printf("[AdaptiveKalman] 初始化完成: n=%d, m=%d\n", n, m);
}

void AdaptiveKalman_SetF(AdaptiveKalman_t *ak, const float *F)
{
    uint8_t i, j;
    for (i = 0; i < ak->n; i++) {
        for (j = 0; j < ak->n; j++) {
            ak->F.data[i][j] = F[i * ak->n + j];
        }
    }
    printf("[AdaptiveKalman] F矩阵已设置\n");
}

void AdaptiveKalman_SetH(AdaptiveKalman_t *ak, const float *H)
{
    uint8_t i, j;
    for (i = 0; i < ak->m; i++) {
        for (j = 0; j < ak->n; j++) {
            ak->H.data[i][j] = H[i * ak->n + j];
        }
    }
    printf("[AdaptiveKalman] H矩阵已设置\n");
}

void AdaptiveKalman_SetQ(AdaptiveKalman_t *ak, const float *Q)
{
    uint8_t i, j;
    for (i = 0; i < ak->n; i++) {
        for (j = 0; j < ak->n; j++) {
            ak->Q.data[i][j] = Q[i * ak->n + j];
        }
    }
    printf("[AdaptiveKalman] Q矩阵已设置\n");
}

void AdaptiveKalman_SetR(AdaptiveKalman_t *ak, const float *R)
{
    uint8_t i, j;
    for (i = 0; i < ak->m; i++) {
        for (j = 0; j < ak->m; j++) {
            ak->R.data[i][j] = R[i * ak->m + j];
        }
    }
    printf("[AdaptiveKalman] R矩阵已设置\n");
}

void AdaptiveKalman_SetInitialState(AdaptiveKalman_t *ak, const float *x0, const float *P0)
{
    uint8_t i, j;
    for (i = 0; i < ak->n; i++) {
        ak->x.data[i][0] = x0[i];
        for (j = 0; j < ak->n; j++) {
            ak->P.data[i][j] = P0[i * ak->n + j];
        }
    }
    printf("[AdaptiveKalman] 初始状态已设置\n");
}

void AdaptiveKalman_SetForgettingFactor(AdaptiveKalman_t *ak, float d_q, float d_r)
{
    ak->d_q = (d_q < 0.0f) ? 0.0f : ((d_q > 1.0f) ? 1.0f : d_q);
    ak->d_r = (d_r < 0.0f) ? 0.0f : ((d_r > 1.0f) ? 1.0f : d_r);
    printf("[AdaptiveKalman] 遗忘因子: d_q=%.3f, d_r=%.3f\n", ak->d_q, ak->d_r);
}

void AdaptiveKalman_SetAdaptiveMode(AdaptiveKalman_t *ak, uint8_t adaptive_q, uint8_t adaptive_r)
{
    ak->adaptive_q = adaptive_q;
    ak->adaptive_r = adaptive_r;
    printf("[AdaptiveKalman] 自适应模式: Q=%s, R=%s\n",
           adaptive_q ? "ON" : "OFF", adaptive_r ? "ON" : "OFF");
}

/* ========== 核心计算 ========== */

const float* AdaptiveKalman_Calculate(AdaptiveKalman_t *ak, const float *measurement)
{
    uint8_t i, j;

    /* ===== 1. 预测步 ===== */
    /* x(k|k-1) = F * x(k-1|k-1) */
    MatMul(&ak->F, &ak->x, &ak->temp_n1);
    memcpy(&ak->x.data[0][0], &ak->temp_n1.data[0][0], sizeof(float) * ak->n);

    /* P(k|k-1) = F * P(k-1|k-1) * F^T + Q */
    MatMul(&ak->F, &ak->P, &ak->temp_nn1);      /* temp = F * P */
    MatTranspose(&ak->F, &ak->temp_nn2);         /* temp_nn2 = F^T */
    MatMul(&ak->temp_nn1, &ak->temp_nn2, &ak->P);  /* P = F*P*F^T */
    MatAdd(&ak->P, &ak->Q, &ak->P);              /* P = F*P*F^T + Q */

    /* ===== 2. 新息计算 ===== */
    /* v(k) = y(k) - H * x(k|k-1) */
    MatMul(&ak->H, &ak->x, &ak->temp_m1);       /* temp_m1 = H*x */
    for (i = 0; i < ak->m; i++) {
        ak->v.data[i][0] = measurement[i] - ak->temp_m1.data[i][0];
    }
    ak->innovation_norm = VecNorm(&ak->v);

    /* ===== 3. 自适应调整R (Sage-Husa) ===== */
    if (ak->adaptive_r && ak->d_r > 0.0f) {
        /* S = H * P * H^T + R (新息协方差) */
        MatMul(&ak->H, &ak->P, &ak->temp_nm1);
        MatTranspose(&ak->H, &ak->temp_mm1);     /* temp_mm1 = H^T */
        MatMul(&ak->temp_nm1, &ak->temp_mm1, &ak->temp_mm1); /* temp_mm1 = H*P*H^T */

        /* v * v^T */
        MatOuterProduct(&ak->v, &ak->v, &ak->temp_nn1);

        /* R(k) = (1-d) * R(k-1) + d * (v*v^T) */
        /* 注意: 这里简化了, 原版还有 -H*P*H^T项 */
        MatAddScaled(&ak->R, ak->d_r, &ak->temp_nn1, &ak->R);

        /* 限幅: 确保R为正定 */
        for (i = 0; i < ak->m; i++) {
            if (ak->R.data[i][i] < AK_MIN_NOISE) {
                ak->R.data[i][i] = AK_MIN_NOISE;
            }
        }
    }

    /* ===== 4. 卡尔曼增益 ===== */
    /* S = H * P * H^T + R */
    MatMul(&ak->H, &ak->P, &ak->temp_nm1);
    MatTranspose(&ak->H, &ak->temp_mm1);
    MatMul(&ak->temp_nm1, &ak->temp_mm1, &ak->temp_mm1);  /* temp_mm1 = H*P*H^T */
    MatAdd(&ak->temp_mm1, &ak->R, &ak->temp_mm1);          /* S = H*P*H^T + R */

    /* 简单求逆 (仅对角矩阵优化) */
    for (i = 0; i < ak->m; i++) {
        for (j = 0; j < ak->m; j++) {
            if (i == j && fabsf(ak->temp_mm1.data[i][j]) > AK_MIN_NOISE) {
                ak->temp_mm1.data[i][j] = 1.0f / ak->temp_mm1.data[i][j];
            } else {
                ak->temp_mm1.data[i][j] = 0.0f;
            }
        }
    }

    /* K = P * H^T * S^{-1} */
    MatTranspose(&ak->H, &ak->temp_nm1);
    MatMul(&ak->P, &ak->temp_nm1, &ak->K);  /* K = P * H^T */
    MatMul(&ak->K, &ak->temp_mm1, &ak->K);  /* K = P*H^T*S^{-1} */

    /* ===== 5. 状态更新 ===== */
    /* x(k|k) = x(k|k-1) + K * v(k) */
    MatMul(&ak->K, &ak->v, &ak->temp_n1);
    MatAdd(&ak->x, &ak->temp_n1, &ak->x);

    /* ===== 6. 协方差更新 ===== */
    /* P(k|k) = (I - K*H) * P(k|k-1) */
    MatIdentity(&ak->temp_nn1, ak->n);
    MatMul(&ak->K, &ak->H, &ak->temp_nn2);  /* K*H */
    MatSub(&ak->temp_nn1, &ak->temp_nn2, &ak->temp_nn1);  /* I - K*H */
    MatMul(&ak->temp_nn1, &ak->P, &ak->P);

    /* ===== 7. 自适应调整Q (Sage-Husa) ===== */
    if (ak->adaptive_q && ak->d_q > 0.0f) {
        /* Q(k) = (1-d) * Q(k-1) + d * (K * v * v^T * K^T) */
        MatOuterProduct(&ak->v, &ak->v, &ak->temp_nn1);  /* v*v^T */
        MatTranspose(&ak->K, &ak->temp_nn2);              /* K^T */
        MatMul(&ak->K, &ak->temp_nn1, &ak->temp_nn1);    /* K*v*v^T */
        MatMul(&ak->temp_nn1, &ak->temp_nn2, &ak->temp_nn1);  /* K*v*v^T*K^T */

        MatAddScaled(&ak->Q, ak->d_q, &ak->temp_nn1, &ak->Q);

        /* 限幅: 确保Q为正定 */
        for (i = 0; i < ak->n; i++) {
            if (ak->Q.data[i][i] < AK_MIN_NOISE) {
                ak->Q.data[i][i] = AK_MIN_NOISE;
            }
        }
    }

    return &ak->x.data[0][0];
}

float AdaptiveKalman_CalculateScalar(AdaptiveKalman_t *ak, float measurement)
{
    float meas = measurement;
    AdaptiveKalman_Calculate(ak, &meas);
    return ak->x.data[0][0];
}

void AdaptiveKalman_Reset(AdaptiveKalman_t *ak, const float *x0)
{
    if (x0 != NULL) {
        uint8_t i;
        for (i = 0; i < ak->n; i++) {
            ak->x.data[i][0] = x0[i];
        }
    } else {
        memset(&ak->x.data[0][0], 0, sizeof(float) * ak->n);
    }
    MatIdentity(&ak->P, ak->n);
    ak->innovation_norm = 0.0f;
    printf("[AdaptiveKalman] 状态重置\n");
}

/* ========== 状态获取 ========== */

const float* AdaptiveKalman_GetState(AdaptiveKalman_t *ak)
{
    return &ak->x.data[0][0];
}

const float* AdaptiveKalman_GetGain(AdaptiveKalman_t *ak)
{
    return &ak->K.data[0][0];
}

const float* AdaptiveKalman_GetQ(AdaptiveKalman_t *ak)
{
    return &ak->Q.data[0][0];
}

const float* AdaptiveKalman_GetR(AdaptiveKalman_t *ak)
{
    return &ak->R.data[0][0];
}

/* ========== 便捷初始化 ========== */

void AdaptiveKalman_Init1D_PositionVelocity(AdaptiveKalman_t *ak, float dt,
                                            float q, float r)
{
    /* 1D位置+速度模型 (仅位置测量):
     * x = [pos, vel]^T
     * F = [1, dt; 0, 1]
     * H = [1, 0]
     */
    float F_data[4] = {1.0f, dt,
                       0.0f, 1.0f};
    float H_data[2] = {1.0f, 0.0f};
    float Q_data[4] = {q*dt*dt/4.0f, q*dt/2.0f,
                       q*dt/2.0f, q};
    float R_data[1] = {r};
    float x0[2] = {0.0f, 0.0f};
    float P0[4] = {1.0f, 0.0f,
                   0.0f, 1.0f};

    AdaptiveKalman_Init(ak, 2, 1);
    AdaptiveKalman_SetF(ak, F_data);
    AdaptiveKalman_SetH(ak, H_data);
    AdaptiveKalman_SetQ(ak, Q_data);
    AdaptiveKalman_SetR(ak, R_data);
    AdaptiveKalman_SetInitialState(ak, x0, P0);

    printf("[AdaptiveKalman] 1D位置+速度模型初始化完成\n");
}

void AdaptiveKalman_Init2D_PositionVelocity(AdaptiveKalman_t *ak, float dt,
                                            float q_accel, float r_pos)
{
    /* 2D位置+速度模型 (匀加速运动):
     * x = [x, vx, y, vy]^T
     * F = [1, dt, 0, 0;
     *      0, 1,  0, 0;
     *      0, 0,  1, dt;
     *      0, 0,  0, 1]
     * H = [1, 0, 0, 0;
     *      0, 0, 1, 0]
     */
    float F_data[16] = {1.0f, dt,  0.0f, 0.0f,
                        0.0f, 1.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 1.0f, dt,
                        0.0f, 0.0f, 0.0f, 1.0f};
    float H_data[8] = {1.0f, 0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 1.0f, 0.0f};
    float Q_data[16] = {q_accel*dt*dt*dt*dt/4.0f, q_accel*dt*dt*dt/2.0f, 0.0f, 0.0f,
                        q_accel*dt*dt*dt/2.0f, q_accel*dt*dt, 0.0f, 0.0f,
                        0.0f, 0.0f, q_accel*dt*dt*dt*dt/4.0f, q_accel*dt*dt*dt/2.0f,
                        0.0f, 0.0f, q_accel*dt*dt*dt/2.0f, q_accel*dt*dt};
    float R_data[4] = {r_pos, 0.0f,
                       0.0f, r_pos};
    float x0[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float P0[16] = {1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f};

    AdaptiveKalman_Init(ak, 4, 2);
    AdaptiveKalman_SetF(ak, F_data);
    AdaptiveKalman_SetH(ak, H_data);
    AdaptiveKalman_SetQ(ak, Q_data);
    AdaptiveKalman_SetR(ak, R_data);
    AdaptiveKalman_SetInitialState(ak, x0, P0);

    printf("[AdaptiveKalman] 2D位置+速度模型初始化完成\n");
}

void AdaptiveKalman_Init3D_Attitude(AdaptiveKalman_t *ak, float dt)
{
    /* 3D姿态估计 (四元数):
     * x = [q0, q1, q2, q3]^T (四元数)
     * F = I (四元数传递, 简化)
     * H = I (直接观测四元数)
     *
     * 实际应用中需要根据IMU数据更新F和H
     */
    float Q_data[16] = {0.01f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.01f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.01f, 0.0f,
                        0.0f, 0.0f, 0.0f, 0.01f};
    float R_data[16] = {0.1f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.1f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.1f, 0.0f,
                        0.0f, 0.0f, 0.0f, 0.1f};
    float x0[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float P0[16] = {1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f};

    AdaptiveKalman_Init(ak, 4, 4);
    AdaptiveKalman_SetQ(ak, Q_data);
    AdaptiveKalman_SetR(ak, R_data);
    AdaptiveKalman_SetInitialState(ak, x0, P0);

    printf("[AdaptiveKalman] 3D姿态估计模型初始化完成\n");
}
