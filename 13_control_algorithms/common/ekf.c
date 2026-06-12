/**
 * @file ekf.c
 * @brief 扩展卡尔曼滤波器 (EKF) 实现
 *
 * 实现通用的 EKF 框架，用户只需提供：
 *   - 状态转移函数 f(x, u)
 *   - 量测函数 h(x)
 *   - 对应的雅可比矩阵
 */

#include "ekf.h"
#include <string.h>
#include <math.h>

/* ========== 矩阵运算辅助函数 ========== */

/**
 * @brief 矩阵加法 C = A + B
 */
static void mat_add(const float *A, const float *B, float *C, int rows, int cols)
{
    int size = rows * cols;
    for (int i = 0; i < size; i++) {
        C[i] = A[i] + B[i];
    }
}

/**
 * @brief 矩阵减法 C = A - B
 */
static void mat_sub(const float *A, const float *B, float *C, int rows, int cols)
{
    int size = rows * cols;
    for (int i = 0; i < size; i++) {
        C[i] = A[i] - B[i];
    }
}

/**
 * @brief 矩阵乘法 C = A * B
 * @param A [r×m], B [m×c], C [r×c]
 */
static void mat_mul(const float *A, const float *B, float *C,
                    int r, int m, int c)
{
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            float sum = 0.0f;
            for (int k = 0; k < m; k++) {
                sum += A[i * m + k] * B[k * c + j];
            }
            C[i * c + j] = sum;
        }
    }
}

/**
 * @brief 矩阵转置 B = A^T
 */
static void mat_trans(const float *A, float *B, int rows, int cols)
{
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            B[j * rows + i] = A[i * cols + j];
        }
    }
}

/**
 * @brief 小矩阵求逆（Gauss-Jordan 消元法）
 * @param A 输入矩阵 [n×n]（会被修改）
 * @param A_inv 输出逆矩阵 [n×n]
 * @param n 矩阵维度
 * @return 0=成功, -1=奇异矩阵
 */
static int mat_inv(const float *A, float *A_inv, int n)
{
    /* [OPT] 快速2×2矩阵求逆（解析公式，省去Gauss-Jordan开销） */
    if (n == 2) {
        float det = A[0]*A[3] - A[1]*A[2];
        if (fabsf(det) < 1e-10f) return -1;
        float inv_det = 1.0f / det;
        A_inv[0] =  A[3] * inv_det;
        A_inv[1] = -A[1] * inv_det;
        A_inv[2] = -A[2] * inv_det;
        A_inv[3] =  A[0] * inv_det;
        return 0;
    }

    /* 构建增广矩阵 [A | I] */
    float aug[EKF_MAX_MEASURES * 2 * EKF_MAX_MEASURES];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i * 2 * n + j] = A[i * n + j];
            aug[i * 2 * n + n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    /* 前向消元 */
    for (int col = 0; col < n; col++) {
        /* 选主元 */
        float max_val = fabsf(aug[col * 2 * n + col]);
        int max_row = col;
        for (int row = col + 1; row < n; row++) {
            float val = fabsf(aug[row * 2 * n + col]);
            if (val > max_val) {
                max_val = val;
                max_row = row;
            }
        }

        if (max_val < 1e-10f) return -1;  /* 奇异矩阵 */

        /* 交换行 */
        if (max_row != col) {
            for (int j = 0; j < 2 * n; j++) {
                float tmp = aug[col * 2 * n + j];
                aug[col * 2 * n + j] = aug[max_row * 2 * n + j];
                aug[max_row * 2 * n + j] = tmp;
            }
        }

        /* 归一化主行 */
        float pivot = aug[col * 2 * n + col];
        for (int j = 0; j < 2 * n; j++) {
            aug[col * 2 * n + j] /= pivot;
        }

        /* 消元 */
        for (int row = 0; row < n; row++) {
            if (row == col) continue;
            float factor = aug[row * 2 * n + col];
            for (int j = 0; j < 2 * n; j++) {
                aug[row * 2 * n + j] -= factor * aug[col * 2 * n + j];
            }
        }
    }

    /* 提取逆矩阵 */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            A_inv[i * n + j] = aug[i * 2 * n + n + j];
        }
    }

    return 0;
}

/* ========== EKF 公共接口实现 ========== */

void EKF_Init(EKF_t *ekf, int n, int p, float dt)
{
    memset(ekf, 0, sizeof(EKF_t));
    ekf->n = n;
    ekf->p = p;
    ekf->dt = dt;

    /* 初始化 P 为单位阵 */
    for (int i = 0; i < n; i++) {
        ekf->P[i * n + i] = 1.0f;
    }
}

void EKF_SetFunctions(EKF_t *ekf,
                       EKF_StateFunc_t f_func,
                       EKF_MeasFunc_t h_func,
                       EKF_JacobianFunc_t F_jac,
                       EKF_JacobianFunc_t H_jac)
{
    ekf->f_func = f_func;
    ekf->h_func = h_func;
    ekf->F_jacobian = F_jac;
    ekf->H_jacobian = H_jac;
}

void EKF_SetInitialState(EKF_t *ekf, const float *x0)
{
    memcpy(ekf->x, x0, ekf->n * sizeof(float));
}

void EKF_SetInitialCovariance(EKF_t *ekf, const float *P0)
{
    memcpy(ekf->P, P0, ekf->n * ekf->n * sizeof(float));
}

void EKF_SetProcessNoise(EKF_t *ekf, const float *Q)
{
    memcpy(ekf->Q, Q, ekf->n * ekf->n * sizeof(float));
}

void EKF_SetMeasurementNoise(EKF_t *ekf, const float *R)
{
    memcpy(ekf->R, R, ekf->p * ekf->p * sizeof(float));
}

/**
 * @brief EKF 预测步
 *
 * 1. 状态预测：x̂⁻ = f(x̂, u)
 * 2. 计算雅可比 F
 * 3. 协方差预测：P⁻ = F * P * Fᵀ + Q
 */
void EKF_Predict(EKF_t *ekf, const float *u)
{
    int n = ekf->n;

    /* 状态预测 */
    if (ekf->f_func) {
        ekf->f_func(ekf->x, u, ekf->x_pred, n, 0);
        memcpy(ekf->x, ekf->x_pred, n * sizeof(float));
    }

    /* 计算雅可比 F */
    if (ekf->F_jacobian) {
        ekf->F_jacobian(ekf->x, u, ekf->F, n, 0);
    }

    /* P⁻ = F * P * Fᵀ + Q */
    float FT[EKF_MAX_STATES * EKF_MAX_STATES];
    float temp[EKF_MAX_STATES * EKF_MAX_STATES];
    mat_trans(ekf->F, FT, n, n);
    mat_mul(ekf->F, ekf->P, temp, n, n, n);
    mat_mul(temp, FT, ekf->P_pred, n, n, n);
    mat_add(ekf->P_pred, ekf->Q, ekf->P_pred, n, n);
    memcpy(ekf->P, ekf->P_pred, n * n * sizeof(float));
}

/**
 * @brief EKF 更新步
 *
 * 1. 计算量测预测：ẑ = h(x̂⁻)
 * 2. 计算量测雅可比 H
 * 3. 卡尔曼增益：K = P⁻*Hᵀ*(H*P⁻*Hᵀ+R)⁻¹
 * 4. 状态更新：x̂ = x̂⁻ + K*(z - ẑ)
 * 5. 协方差更新：P = (I - K*H)*P⁻
 */
void EKF_Update(EKF_t *ekf, const float *z)
{
    int n = ekf->n;
    int p = ekf->p;

    /* 量测预测 */
    float z_pred[EKF_MAX_MEASURES];
    if (ekf->h_func) {
        ekf->h_func(ekf->x, z_pred, n, p);
    }

    /* 计算量测雅可比 H */
    if (ekf->H_jacobian) {
        ekf->H_jacobian(ekf->x, NULL, ekf->H, n, p);
    }

    /* S = H * P⁻ * Hᵀ + R */
    float HT[EKF_MAX_STATES * EKF_MAX_MEASURES];
    float S[EKF_MAX_MEASURES * EKF_MAX_MEASURES];
    float temp[EKF_MAX_MEASURES * EKF_MAX_STATES];

    mat_trans(ekf->H, HT, p, n);
    mat_mul(ekf->H, ekf->P, temp, p, n, n);
    mat_mul(temp, HT, S, p, n, p);
    mat_add(S, ekf->R, S, p, p);

    /* K = P⁻ * Hᵀ * S⁻¹ */
    float S_inv[EKF_MAX_MEASURES * EKF_MAX_MEASURES];
    if (mat_inv(S, S_inv, p) < 0) return;

    float temp2[EKF_MAX_STATES * EKF_MAX_MEASURES];
    mat_mul(ekf->P, HT, temp2, n, n, p);
    mat_mul(temp2, S_inv, ekf->K, n, p, p);

    /* 状态更新：x̂ = x̂⁻ + K * (z - ẑ) */
    float innovation[EKF_MAX_MEASURES];
    for (int i = 0; i < p; i++) {
        innovation[i] = z[i] - z_pred[i];
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < p; j++) {
            ekf->x[i] += ekf->K[i * p + j] * innovation[j];
        }
    }

    /* 协方差更新：P = (I - K*H) * P⁻ */
    float KH[EKF_MAX_STATES * EKF_MAX_STATES];
    float I_KH[EKF_MAX_STATES * EKF_MAX_STATES];
    mat_mul(ekf->K, ekf->H, KH, n, p, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            I_KH[i * n + j] = (i == j ? 1.0f : 0.0f) - KH[i * n + j];
        }
    }
    float P_new[EKF_MAX_STATES * EKF_MAX_STATES];
    mat_mul(I_KH, ekf->P, P_new, n, n, n);
    memcpy(ekf->P, P_new, n * n * sizeof(float));
}

void EKF_Step(EKF_t *ekf, const float *u, const float *z)
{
    EKF_Predict(ekf, u);
    EKF_Update(ekf, z);
}

float EKF_GetState(const EKF_t *ekf, int index)
{
    if (index >= 0 && index < ekf->n) {
        return ekf->x[index];
    }
    return 0.0f;
}
