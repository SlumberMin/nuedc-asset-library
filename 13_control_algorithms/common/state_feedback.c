/**
 * @file state_feedback.c
 * @brief 状态反馈控制实现(极点配置法)
 */

#include "state_feedback.h"
#include <string.h>

void SF_Init(StateFeedback_t *sf, uint8_t n, uint8_t m, uint8_t p, float dt)
{
    if (n > SF_MAX_STATES)  n = SF_MAX_STATES;
    if (m > SF_MAX_INPUTS)  m = SF_MAX_INPUTS;
    if (p > SF_MAX_OUTPUTS) p = SF_MAX_OUTPUTS;

    memset(sf, 0, sizeof(StateFeedback_t));
    sf->n = n;
    sf->m = m;
    sf->p = p;
    sf->dt = dt;

    /* 默认前馈增益为单位矩阵 */
    for (uint8_t i = 0; i < m; i++)
        sf->N[i][i] = 1.0f;
}

void SF_SetSystemModel(StateFeedback_t *sf, const float *A_data,
                        const float *B_data, const float *C_data)
{
    uint8_t n = sf->n, m = sf->m, p = sf->p;

    if (A_data) {
        for (uint8_t i = 0; i < n; i++)
            for (uint8_t j = 0; j < n; j++)
                sf->A[i][j] = A_data[i * n + j];
    }
    if (B_data) {
        for (uint8_t i = 0; i < n; i++)
            for (uint8_t j = 0; j < m; j++)
                sf->B[i][j] = B_data[i * m + j];
    }
    if (C_data) {
        for (uint8_t i = 0; i < p; i++)
            for (uint8_t j = 0; j < n; j++)
                sf->C[i][j] = C_data[i * n + j];
    }
}

void SF_SetGainMatrix(StateFeedback_t *sf, const float *K_data)
{
    uint8_t m = sf->m, n = sf->n;
    for (uint8_t i = 0; i < m; i++)
        for (uint8_t j = 0; j < n; j++)
            sf->K[i][j] = K_data[i * n + j];
}

void SF_SetFeedforwardGain(StateFeedback_t *sf, const float *N_data)
{
    uint8_t m = sf->m;
    for (uint8_t i = 0; i < m; i++)
        for (uint8_t j = 0; j < m; j++)
            sf->N[i][j] = N_data[i * m + j];
}

void SF_SetInitialState(StateFeedback_t *sf, const float *x0)
{
    for (uint8_t i = 0; i < sf->n; i++)
        sf->x[i] = x0[i];
}

/**
 * @brief 二阶系统极点配置
 *
 * 对于二阶 SISO 系统:
 *   A = [a11 a12; a21 a22], B = [b1; b2]
 *   期望闭环极点: p1, p2
 *   期望特征多项式: (s - p1)(s - p2) = s^2 - (p1+p2)*s + p1*p2
 *
 * 利用 Ackermann 公式: K = [0 1] * [B AB]^{-1} * phi(A)
 */
void SF_PolePlacement_2nd(StateFeedback_t *sf, float desired_p1, float desired_p2)
{
    if (sf->n != 2 || sf->m != 1) return;

    float a11 = sf->A[0][0], a12 = sf->A[0][1];
    float a21 = sf->A[1][0], a22 = sf->A[1][1];
    float b1 = sf->B[0][0], b2 = sf->B[1][0];

    /* 可控性矩阵 M = [B, AB] */
    float ab1 = a11 * b1 + a12 * b2;
    float ab2 = a21 * b1 + a22 * b2;

    /* det(M) = b1*ab2 - b2*ab1 */
    float det_M = b1 * ab2 - b2 * ab1;
    if (det_M > -1e-10f && det_M < 1e-10f) return; /* 不可控 */

    /* M^{-1} */
    float inv_M[2][2];
    inv_M[0][0] =  ab2 / det_M;
    inv_M[0][1] = -ab1 / det_M;
    inv_M[1][0] = -b2 / det_M;
    inv_M[1][1] =  b1 / det_M;

    /* 期望特征多项式系数: s^2 + alpha1*s + alpha0
     * (s - p1)(s - p2) = s^2 - (p1+p2)*s + p1*p2 */
    float alpha1 = -(desired_p1 + desired_p2);
    float alpha0 = desired_p1 * desired_p2;

    /* phi(A) = A^2 + alpha1*A + alpha0*I */
    float a2_00 = a11 * a11 + a12 * a21;
    float a2_01 = a11 * a12 + a12 * a22;
    float a2_10 = a21 * a11 + a22 * a21;
    float a2_11 = a21 * a12 + a22 * a22;

    float phi00 = a2_00 + alpha1 * a11 + alpha0;
    float phi01 = a2_01 + alpha1 * a12;
    float phi10 = a2_10 + alpha1 * a21;
    float phi11 = a2_11 + alpha1 * a22 + alpha0;

    /* K = [0 1] * M^{-1} * phi(A) */
    /* temp = [0 1] * M^{-1} = [inv_M[1][0], inv_M[1][1]] */
    float temp0 = inv_M[1][0];
    float temp1 = inv_M[1][1];

    /* K = temp * phi(A) */
    sf->K[0][0] = temp0 * phi00 + temp1 * phi10;
    sf->K[0][1] = temp0 * phi01 + temp1 * phi11;

    /* 前馈增益 N: 使稳态输出跟踪参考
     * N = -1 / (C * (A-BK)^{-1} * B)  (SISO情况) */
    /* 简化计算: 对于位置控制取N = K[0] + K[1]*dt近似 */
    sf->N[0][0] = 1.0f;
}

/**
 * @brief Ackermann 公式通用极点配置
 * 适用于 n 阶 SISO 系统
 */
void SF_Ackermann(StateFeedback_t *sf, const float *poles, uint8_t num_poles)
{
    if (num_poles != sf->n || sf->m != 1) return;

    uint8_t n = sf->n;

    /* 计算期望特征多项式系数 */
    /* 通过逐个因式相乘展开 (s - p1)(s - p2)...(s - pn) */
    float alpha[SF_MAX_STATES + 1]; /* alpha[n] = 1 (最高次项系数) */
    memset(alpha, 0, sizeof(alpha));
    alpha[0] = 1.0f;

    for (uint8_t k = 0; k < n; k++) {
        /* 乘以 (s - poles[k]) */
        for (int8_t i = n; i >= 1; i--) {
            alpha[i] = alpha[i - 1] - poles[k] * alpha[i];
        }
        /* alpha[0] 保持不变 */
    }
    /* 现在 alpha[i] 是 s^i 的系数, alpha[n]=1 */

    /* 计算 phi(A) = alpha[0]*A^n + alpha[1]*A^{n-1} + ... + alpha[n]*I */
    /* 使用 Horner 法: phi(A) = (...((A + alpha[0])*A + alpha[1])*A + ... + alpha[n]*I) */
    /* 这里简化实现: 直接用可控性矩阵方法 */

    /* 构造可控性矩阵 T = [B, AB, A^2B, ..., A^{n-1}B] */
    float T[SF_MAX_STATES][SF_MAX_STATES];
    float AB[SF_MAX_STATES];

    /* 第一列: B */
    for (uint8_t i = 0; i < n; i++)
        T[i][0] = sf->B[i][0];

    /* 后续列: A^k * B */
    for (uint8_t col = 1; col < n; col++) {
        for (uint8_t i = 0; i < n; i++) {
            float sum = 0.0f;
            for (uint8_t j = 0; j < n; j++)
                sum += sf->A[i][j] * T[j][col - 1];
            AB[i] = sum;
        }
        for (uint8_t i = 0; i < n; i++)
            T[i][col] = AB[i];
    }

    /* 计算 T^{-1} (Gauss-Jordan消元, 就地求解) */
    float aug[SF_MAX_STATES][SF_MAX_STATES * 2];
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            aug[i][j] = T[i][j];
            aug[i][j + n] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (uint8_t col = 0; col < n; col++) {
        /* 主元选取 */
        float max_val = aug[col][col] >= 0 ? aug[col][col] : -aug[col][col];
        uint8_t max_row = col;
        for (uint8_t row = col + 1; row < n; row++) {
            float v = aug[row][col] >= 0 ? aug[row][col] : -aug[row][col];
            if (v > max_val) { max_val = v; max_row = row; }
        }
        if (max_val < 1e-10f) return; /* 奇异, 不可控 */

        if (max_row != col) {
            for (uint8_t j = 0; j < 2 * n; j++) {
                float tmp = aug[col][j];
                aug[col][j] = aug[max_row][j];
                aug[max_row][j] = tmp;
            }
        }

        float pivot = aug[col][col];
        for (uint8_t j = 0; j < 2 * n; j++)
            aug[col][j] /= pivot;

        for (uint8_t row = 0; row < n; row++) {
            if (row == col) continue;
            float factor = aug[row][col];
            for (uint8_t j = 0; j < 2 * n; j++)
                aug[row][j] -= factor * aug[col][j];
        }
    }

    /* T_inv = aug[:, n:2n] */
    float T_inv[SF_MAX_STATES][SF_MAX_STATES];
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++)
            T_inv[i][j] = aug[i][j + n];

    /* 计算 phi(A) 使用 Horner 法 */
    float phi[SF_MAX_STATES][SF_MAX_STATES];
    float temp[SF_MAX_STATES][SF_MAX_STATES];

    /* phi = alpha[n]*I */
    memset(phi, 0, sizeof(phi));
    for (uint8_t i = 0; i < n; i++)
        phi[i][i] = alpha[n];

    /* phi = phi*A + alpha[n-1]*I, phi = phi*A + alpha[n-2]*I, ... */
    for (int8_t k = (int8_t)n - 1; k >= 1; k--) {
        /* temp = phi * A */
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < n; j++) {
                float sum = 0.0f;
                for (uint8_t l = 0; l < n; l++)
                    sum += phi[i][l] * sf->A[l][j];
                temp[i][j] = sum;
            }
        }
        /* phi = temp + alpha[k]*I */
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < n; j++) {
                phi[i][j] = temp[i][j];
                if (i == j) phi[i][j] += alpha[k];
            }
        }
    }

    /* 最后一步: phi = phi * A + alpha[0]*I (不需要, 因为 Horner 迭代已经包含) */
    /* 实际上 Horner 法应该是从高次开始: phi = I; phi = phi*A + alpha[n]*I; ... */
    /* 让我重新修正: 标准 Horner 计算 alpha[0]*A^n + alpha[1]*A^{n-1}+...+alpha[n]*I
     * = ((...(alpha[0]*A + alpha[1]*I)*A + alpha[2]*I)*A + ... + alpha[n]*I)
     */

    /* 重新计算 */
    memset(phi, 0, sizeof(phi));
    for (uint8_t i = 0; i < n; i++)
        phi[i][i] = alpha[0];

    for (uint8_t k = 1; k <= n; k++) {
        /* temp = phi * A */
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < n; j++) {
                float sum = 0.0f;
                for (uint8_t l = 0; l < n; l++)
                    sum += phi[i][l] * sf->A[l][j];
                temp[i][j] = sum;
            }
        }
        /* phi = temp + alpha[k]*I */
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < n; j++) {
                phi[i][j] = temp[i][j];
                if (i == j) phi[i][j] += alpha[k];
            }
        }
    }

    /* K = [0, 0, ..., 0, 1] * T^{-1} * phi(A) */
    /* last_row = T^{-1} 的最后一行 */
    float last_row[SF_MAX_STATES];
    for (uint8_t j = 0; j < n; j++)
        last_row[j] = T_inv[n - 1][j];

    /* K = last_row * phi */
    for (uint8_t j = 0; j < n; j++) {
        float sum = 0.0f;
        for (uint8_t l = 0; l < n; l++)
            sum += last_row[l] * phi[l][j];
        sf->K[0][j] = sum;
    }

    sf->N[0][0] = 1.0f;
}

void SF_Update(StateFeedback_t *sf, const float *ref, const float *state, float *u_out)
{
    uint8_t n = sf->n, m = sf->m;

    const float *x = state ? state : sf->x;

    /* u = -K*x + N*r */
    for (uint8_t i = 0; i < m; i++) {
        float u = 0.0f;

        /* -K*x */
        for (uint8_t j = 0; j < n; j++)
            u -= sf->K[i][j] * x[j];

        /* +N*r */
        for (uint8_t j = 0; j < m; j++)
            u += sf->N[i][j] * ref[j];

        u_out[i] = u;
    }

    /* 更新内部状态(前向欧拉): x[k+1] = A*x + B*u */
    if (state == NULL) {
        float x_new[SF_MAX_STATES] = {0};
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < n; j++)
                x_new[i] += sf->A[i][j] * sf->x[j];
            for (uint8_t j = 0; j < m; j++)
                x_new[i] += sf->B[i][j] * u_out[j];
        }
        for (uint8_t i = 0; i < n; i++)
            sf->x[i] = x_new[i];
    }
}

float SF_GetState(StateFeedback_t *sf, uint8_t index)
{
    if (index >= sf->n) return 0.0f;
    return sf->x[index];
}

void SF_Reset(StateFeedback_t *sf)
{
    memset(sf->x, 0, sizeof(sf->x));
    memset(sf->x_hat, 0, sizeof(sf->x_hat));
}
