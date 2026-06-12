/**
 * @file lqr.c
 * @brief LQR最优控制器实现 - 离线迭代求解离散Riccati方程
 * @details 线性二次调节器(LQR)通过最小化二次代价函数J = Σ(x'Qx + u'Ru)
 *          求解最优状态反馈增益K。
 *          本实现使用迭代法求解离散代数Riccati方程(DARE):
 *          P = A'PA - A'PB(R + B'PB)^{-1}B'PA + Q
 *          K = (R + B'PB)^{-1} * B'PA
 *          适用于线性时不变系统的最优控制。
 */

#include "lqr.h"
#include <string.h>
#include <math.h>

/**
 * @brief 浮点数限幅辅助函数
 * @param v 输入值
 * @param lo 下限
 * @param hi 上限
 * @return 限幅后的值
 */
static float ClampF(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/**
 * @brief 初始化LQR控制器
 * @param lqr LQR结构体指针
 * @param n 状态维度(最大LQR_MAX_STATES)
 */
void LQR_Init(LQR_t *lqr, uint8_t n)
{
    if (lqr == NULL) return;
    lqr->n = (n > LQR_MAX_STATES) ? LQR_MAX_STATES : n;
    memset(lqr->A, 0, sizeof(lqr->A));
    memset(lqr->B, 0, sizeof(lqr->B));
    memset(lqr->Q, 0, sizeof(lqr->Q));
    memset(lqr->K, 0, sizeof(lqr->K));
    memset(lqr->x, 0, sizeof(lqr->x));
    lqr->R = 1.0f;
    lqr->output = 0;
    lqr->output_max = 1000;
    lqr->output_min = -1000;
}

/**
 * @brief 设置系统状态空间模型 A, B
 * @param lqr LQR结构体指针
 * @param A 状态矩阵数据(行优先 n×n)
 * @param B 输入矩阵数据(长度n)
 */
void LQR_SetSystem(LQR_t *lqr, const float *A, const float *B)
{
    if (lqr == NULL || A == NULL || B == NULL) return;
    uint8_t n = lqr->n;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            lqr->A[i][j] = A[i * n + j];
        }
        lqr->B[i] = B[i];
    }
}

/**
 * @brief 设置LQR权重矩阵Q和R
 * @param lqr LQR结构体指针
 * @param Q 状态权重矩阵数据(行优先 n×n)
 * @param R 控制量权重标量
 */
void LQR_SetWeight(LQR_t *lqr, const float *Q, float R)
{
    if (lqr == NULL || Q == NULL) return;
    uint8_t n = lqr->n;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            lqr->Q[i][j] = Q[i * n + j];
        }
    }
    lqr->R = R;
}

/**
 * @brief 设置输出限幅
 * @param lqr LQR结构体指针
 * @param min 输出下限
 * @param max 输出上限
 */
void LQR_SetOutputLimit(LQR_t *lqr, float min, float max)
{
    if (lqr == NULL) return;
    lqr->output_min = min;
    lqr->output_max = max;
}

/**
 * @brief 离线求解Riccati方程并计算最优增益K
 * @param lqr LQR结构体指针
 *
 * @details 使用不动点迭代法求解DARE:
 *          P = A'PA - A'PB(R + B'PB)^{-1}B'PA + Q
 *          迭代直到收敛(差异<1e-6)或达到200次迭代
 *          最终计算 K = (R + B'PB)^{-1} * B'PA
 */
void LQR_ComputeGain(LQR_t *lqr)
{
    if (lqr == NULL) return;

    uint8_t n = lqr->n;
    float P[LQR_MAX_STATES][LQR_MAX_STATES];
    float P_new[LQR_MAX_STATES][LQR_MAX_STATES];
    float temp_nn[LQR_MAX_STATES][LQR_MAX_STATES];
    float BtP[LQR_MAX_STATES];
    float BtPB, inv_term;
    
    /* 初始化P = Q (初始猜测) */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            P[i][j] = lqr->Q[i][j];
    
    /* 迭代求解DARE */
    for (int iter = 0; iter < 200; iter++) {
        /* 计算 A'PA: 先算 temp = P*A */
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                temp_nn[i][j] = 0;
                for (int k = 0; k < n; k++)
                    temp_nn[i][j] += P[i][k] * lqr->A[k][j];
            }
        }
        /* AtPA = A' * (P*A) */
        float AtPA[LQR_MAX_STATES][LQR_MAX_STATES];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                AtPA[i][j] = 0;
                for (int k = 0; k < n; k++)
                    AtPA[i][j] += lqr->A[k][i] * temp_nn[k][j];
            }
        }
        
        /* 计算 BtP = B'*P 和 BtPB = B'*P*B */
        for (int i = 0; i < n; i++) {
            BtP[i] = 0;
            for (int k = 0; k < n; k++)
                BtP[i] += lqr->B[k] * P[k][i];
        }
        BtPB = 0;
        for (int i = 0; i < n; i++)
            BtPB += BtP[i] * lqr->B[i];
        
        /* (R + B'PB)^{-1} */
        if ((lqr->R + BtPB) < 1e-10f) {
            inv_term = 1e10f;  /* 防除零 */
        } else {
            inv_term = 1.0f / (lqr->R + BtPB);
        }
        
        /* 计算 AtPB = A'*B */
        float AtPB[LQR_MAX_STATES];
        for (int i = 0; i < n; i++) {
            AtPB[i] = 0;
            for (int k = 0; k < n; k++)
                AtPB[i] += lqr->A[k][i] * lqr->B[k];
        }
        
        /* P_new = AtPA - inv_term * AtPB * BtP' + Q */
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                P_new[i][j] = AtPA[i][j] - inv_term * AtPB[i] * BtP[j] + lqr->Q[i][j];
            }
        }
        
        /* 收敛检查 */
        float diff = 0;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                diff += fabsf(P_new[i][j] - P[i][j]);
        
        /* 更新P */
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                P[i][j] = P_new[i][j];
        
        if (diff < 1e-6f) break;  /* 已收敛 */
    }
    
    /* 计算最优增益 K = (R + B'PB)^{-1} * BtPA */
    for (int i = 0; i < n; i++) {
        BtP[i] = 0;
        for (int k = 0; k < n; k++)
            BtP[i] += lqr->B[k] * P[k][i];
    }
    BtPB = 0;
    for (int i = 0; i < n; i++)
        BtPB += BtP[i] * lqr->B[i];
    if ((lqr->R + BtPB) < 1e-10f) {
        inv_term = 1e10f;
    } else {
        inv_term = 1.0f / (lqr->R + BtPB);
    }
    
    /* K[i] = sum_j(inv_term * BtP[j] * A[j][i]) */
    for (int i = 0; i < n; i++) {
        lqr->K[i] = 0;
        for (int j = 0; j < n; j++)
            lqr->K[i] += inv_term * BtP[j] * lqr->A[j][i];
    }
}

/**
 * @brief 执行一步LQR控制计算
 * @param lqr LQR结构体指针
 * @param state 当前状态数组(长度n)
 * @param target 目标状态数组(长度n), 可为NULL(默认为零)
 * @return LQR控制器输出
 */
float LQR_Calculate(LQR_t *lqr, const float *state, const float *target)
{
    if (lqr == NULL || state == NULL) return 0.0f;

    uint8_t n = lqr->n;
    float u = 0;
    
    /* 计算状态误差并求控制量 u = -K*(state-target) */
    for (int i = 0; i < n; i++) {
        lqr->x[i] = state[i] - (target ? target[i] : 0);
        u -= lqr->K[i] * lqr->x[i];
    }
    
    /* 输出限幅 */
    lqr->output = ClampF(u, lqr->output_min, lqr->output_max);
    return lqr->output;
}

/**
 * @brief 重置LQR控制器状态
 * @param lqr LQR结构体指针
 */
void LQR_Reset(LQR_t *lqr)
{
    if (lqr == NULL) return;
    memset(lqr->x, 0, sizeof(lqr->x));
    lqr->output = 0;
}
