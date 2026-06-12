/**
 * @file h_infinity.c
 * @brief H∞鲁棒控制器实现（简化版）
 *
 * 算法流程：
 * 1. 设置系统模型 A,B,C 和加权 Q,R
 * 2. γ迭代：从gamma_max开始，二分法搜索满足条件的最小γ
 * 3. 每次γ迭代中求解代数Riccati方程（ARE）：
 *    A^T*P + P*A + Q + (1/γ²)*P*B2*B2^T*P - P*B*R^{-1}*B^T*P = 0
 * 4. 计算状态反馈增益：K = R^{-1} * B^T * P
 * 5. 控制律：u = -K * (x - x_ref)
 */

#include "h_infinity.h"
#include <math.h>
#include <string.h>

/* 内部矩阵运算（小矩阵，嵌入式友好） */
static void _mat_mult(const float *A, const float *B, float *C,
                       int m, int n, int p)
{
    /* C(m×p) = A(m×n) * B(n×p) */
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++) {
                sum += A[i * n + k] * B[k * p + j];
            }
            C[i * p + j] = sum;
        }
    }
}

static void _mat_transpose(const float *A, float *AT, int m, int n)
{
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            AT[j * m + i] = A[i * n + j];
}

static void _mat_add(const float *A, const float *B, float *C, int rows, int cols)
{
    int total = rows * cols;
    for (int i = 0; i < total; i++)
        C[i] = A[i] + B[i];
}

static void _mat_sub(const float *A, const float *B, float *C, int rows, int cols)
{
    int total = rows * cols;
    for (int i = 0; i < total; i++)
        C[i] = A[i] - B[i];
}

static void _mat_scale(const float *A, float *B, int rows, int cols, float s)
{
    int total = rows * cols;
    for (int i = 0; i < total; i++)
        B[i] = A[i] * s;
}

static void _mat_identity(float *A, int n)
{
    memset(A, 0, n * n * sizeof(float));
    for (int i = 0; i < n; i++)
        A[i * n + i] = 1.0f;
}

static void _mat_copy(float *dst, const float *src, int rows, int cols)
{
    memcpy(dst, src, rows * cols * sizeof(float));
}

/* 简化矩阵求逆（Gauss-Jordan，适用于小矩阵） */
static int _mat_inverse(const float *A, float *Ainv, int n)
{
    float aug[HINF_MAX_STATES * 2 * HINF_MAX_STATES];
    memset(aug, 0, sizeof(aug));

    /* 构造增广矩阵 [A | I] */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i * 2 * n + j] = A[i * n + j];
        }
        aug[i * 2 * n + n + i] = 1.0f;
    }

    /* Gauss-Jordan消元 */
    for (int col = 0; col < n; col++) {
        /* 选主元 */
        int max_row = col;
        float max_val = fabsf(aug[col * 2 * n + col]);
        for (int row = col + 1; row < n; row++) {
            float v = fabsf(aug[row * 2 * n + col]);
            if (v > max_val) { max_val = v; max_row = row; }
        }
        if (max_val < 1e-12f) return -1; /* 奇异 */

        /* 交换行 */
        if (max_row != col) {
            for (int j = 0; j < 2 * n; j++) {
                float tmp = aug[col * 2 * n + j];
                aug[col * 2 * n + j] = aug[max_row * 2 * n + j];
                aug[max_row * 2 * n + j] = tmp;
            }
        }

        /* 归一化 */
        float pivot = aug[col * 2 * n + col];
        for (int j = 0; j < 2 * n; j++)
            aug[col * 2 * n + j] /= pivot;

        /* 消元 */
        for (int row = 0; row < n; row++) {
            if (row == col) continue;
            float factor = aug[row * 2 * n + col];
            for (int j = 0; j < 2 * n; j++)
                aug[row * 2 * n + j] -= factor * aug[col * 2 * n + j];
        }
    }

    /* 提取逆矩阵 */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            Ainv[i * n + j] = aug[i * 2 * n + n + j];

    return 0;
}

void HInf_Init(HInf_t *ctrl, uint8_t n, uint8_t m, uint8_t p)
{
    memset(ctrl, 0, sizeof(HInf_t));
    ctrl->n = n;
    ctrl->m = m;
    ctrl->p = p;
    ctrl->m_d = n;  /* 默认干扰维数等于状态维数 */

    ctrl->gamma = 1.0f;
    ctrl->gamma_min = 0.1f;
    ctrl->gamma_max = 100.0f;
    ctrl->gamma_tol = 0.01f;
    ctrl->riccati_max_iter = 200;
    ctrl->riccati_tol = 1e-6f;
    ctrl->mode = HINF_MODE_STATIC_GAIN;

    /* 默认Q = I, R = I */
    _mat_identity(ctrl->Q, n);
    _mat_identity(ctrl->R, m);
}

void HInf_SetPlant(HInf_t *ctrl,
                    const float *A, const float *B, const float *C)
{
    int n = ctrl->n, m = ctrl->m, p = ctrl->p;
    _mat_copy(ctrl->A, A, n, n);
    _mat_copy(ctrl->B, B, n, m);
    _mat_copy(ctrl->C, C, p, n);
}

void HInf_SetWeight(HInf_t *ctrl, const float *Q, const float *R)
{
    _mat_copy(ctrl->Q, Q, ctrl->n, ctrl->n);
    _mat_copy(ctrl->R, R, ctrl->m, ctrl->m);
}

void HInf_SetDisturbance(HInf_t *ctrl, const float *B2, uint8_t m_d)
{
    ctrl->m_d = m_d;
    _mat_copy(ctrl->B2, B2, ctrl->n, m_d);
}

/*
 * 求解代数Riccati方程（迭代法）：
 * A^T*P + P*A + Q + (1/γ²)*P*B2*B2^T*P - P*B*R^{-1}*B^T*P = 0
 *
 * 迭代格式：离散Lyapunov方程反复迭代
 */
static int _SolveRiccati(HInf_t *ctrl, float gamma, float *P)
{
    int n = ctrl->n, m = ctrl->m;
    float temp_n_n[HINF_MAX_STATES * HINF_MAX_STATES];
    float temp2_n_n[HINF_MAX_STATES * HINF_MAX_STATES];
    float temp_n_m[HINF_MAX_STATES * HINF_MAX_STATES];
    float BT[HINF_MAX_STATES * HINF_MAX_STATES];
    float Rinv[HINF_MAX_STATES * HINF_MAX_STATES];
    float B2T[HINF_MAX_STATES * HINF_MAX_STATES];
    float AT[HINF_MAX_STATES * HINF_MAX_STATES];
    float gamma2_inv;

    _mat_transpose(ctrl->A, AT, n, n);
    _mat_transpose(ctrl->B, BT, n, m);
    _mat_inverse(ctrl->R, Rinv, m);

    if (gamma > 1e-6f)
        gamma2_inv = 1.0f / (gamma * gamma);
    else
        gamma2_inv = 1e6f;

    /* 初始化P = Q */
    _mat_copy(P, ctrl->Q, n, n);

    for (int iter = 0; iter < ctrl->riccati_max_iter; iter++) {
        float P_old[HINF_MAX_STATES * HINF_MAX_STATES];
        _mat_copy(P_old, P, n, n);

        /* 计算 B*R^{-1}*B^T*P */
        _mat_mult(BT, P, temp_n_m, m, n, n);    /* BT*P: m×n */
        _mat_mult(Rinv, temp_n_m, temp_n_m, m, m, n); /* Rinv*BT*P: m×n */
        _mat_mult(ctrl->B, temp_n_m, temp_n_n, n, m, n); /* B*Rinv*BT*P: n×n */

        /* 计算 (1/γ²)*P*B2*B2^T*P */
        _mat_transpose(ctrl->B2, B2T, n, ctrl->m_d);
        _mat_mult(B2T, P, temp2_n_n, ctrl->m_d, n, n);
        _mat_mult(ctrl->B2, temp2_n_n, temp2_n_n, n, ctrl->m_d, n);
        _mat_scale(temp2_n_n, temp2_n_n, n, n, gamma2_inv);

        /* 更新：P_new = P + dt*(A^T*P + P*A + Q + gamma_term - BRinvBtP) */
        /* 简化：直接用不动点迭代 */
        float lhs[HINF_MAX_STATES * HINF_MAX_STATES];
        float rhs[HINF_MAX_STATES * HINF_MAX_STATES];

        /* rhs = Q + (1/γ²)*P*B2*B2^T*P */
        _mat_add(ctrl->Q, temp2_n_n, rhs, n, n);

        /* lhs = A^T*P + P*A */
        _mat_mult(AT, P, temp_n_n, n, n, n);
        _mat_add(temp_n_n, rhs, lhs, n, n);
        _mat_mult(P, ctrl->A, temp_n_n, n, n, n);
        _mat_add(lhs, temp_n_n, lhs, n, n);

        /* P += (lhs - B*Rinv*Bt*P) * dt_step */
        _mat_sub(lhs, rhs, temp_n_n, n, n);

        /* 使用小步长迭代 */
        float step = 0.01f;
        _mat_scale(temp_n_n, temp_n_n, n, n, step);
        _mat_add(P, temp_n_n, P, n, n);

        /* 检查收敛 */
        float diff = 0.0f;
        for (int i = 0; i < n * n; i++) {
            float d = P[i] - P_old[i];
            diff += d * d;
        }
        diff = sqrtf(diff / (n * n));

        if (diff < ctrl->riccati_tol) return 0;
    }
    return -1; /* 不收敛 */
}

int HInf_Solve(HInf_t *ctrl, float gamma)
{
    int n = ctrl->n, m = ctrl->m;
    float BT[HINF_MAX_STATES * HINF_MAX_STATES];
    float Rinv[HINF_MAX_STATES * HINF_MAX_STATES];
    float KT[HINF_MAX_STATES * HINF_MAX_STATES];

    _mat_transpose(ctrl->B, BT, n, m);
    _mat_inverse(ctrl->R, Rinv, m);

    if (gamma > 0.0f) {
        /* 指定γ求解 */
        ctrl->gamma = gamma;
        int ret = _SolveRiccati(ctrl, gamma, ctrl->P);
        if (ret != 0) return ret;
    } else {
        /* γ搜索：二分法找最小可行γ */
        float g_lo = ctrl->gamma_min;
        float g_hi = ctrl->gamma_max;

        while ((g_hi - g_lo) > ctrl->gamma_tol) {
            float g_mid = (g_lo + g_hi) * 0.5f;
            float P_try[HINF_MAX_STATES * HINF_MAX_STATES];
            int ret = _SolveRiccati(ctrl, g_mid, P_try);
            if (ret == 0) {
                g_hi = g_mid;
                _mat_copy(ctrl->P, P_try, n, n);
            } else {
                g_lo = g_mid;
            }
        }
        ctrl->gamma = g_hi;
    }

    /* 计算增益 K = R^{-1} * B^T * P */
    _mat_mult(BT, ctrl->P, KT, m, n, n);  /* BT*P: m×n */
    _mat_mult(Rinv, KT, ctrl->K, m, m, n); /* Rinv*BT*P: m×n */

    return 0;
}

float* HInf_Compute(HInf_t *ctrl, const float *x_ref, const float *x_meas)
{
    int n = ctrl->n, m = ctrl->m;

    /* 计算状态误差 e = x_ref - x_meas */
    float e[HINF_MAX_STATES];
    if (x_ref) {
        for (int i = 0; i < n; i++)
            e[i] = x_ref[i] - x_meas[i];
    } else {
        for (int i = 0; i < n; i++)
            e[i] = -x_meas[i]; /* 调节问题：x_ref = 0 */
    }

    /* u = -K * e */
    for (int i = 0; i < m; i++) {
        float sum = 0.0f;
        for (int j = 0; j < n; j++) {
            sum += ctrl->K[i * n + j] * e[j];
        }
        ctrl->u[i] = -sum;
    }

    /* 更新内部状态 */
    _mat_copy(ctrl->x, x_meas, n, 1);

    return ctrl->u;
}

void HInf_GetGain(HInf_t *ctrl, float *K_out)
{
    _mat_copy(K_out, ctrl->K, ctrl->m, ctrl->n);
}

void HInf_Reset(HInf_t *ctrl)
{
    memset(ctrl->x, 0, sizeof(ctrl->x));
    memset(ctrl->u, 0, sizeof(ctrl->u));
}
