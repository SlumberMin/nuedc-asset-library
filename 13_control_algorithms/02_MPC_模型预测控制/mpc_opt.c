/**
 * @file mpc_opt.c
 * @brief MPC 嵌入式简化实现 -- 性能优化版
 *
 * 优化策略:
 * 1. 减少梯度下降迭代次数: 20 -> 5 (配合更好的初始值)
 * 2. 使用动量梯度下降: 收敛更快，允许更少迭代
 * 3. 内联状态预测: 消除函数调用开销
 * 4. 预计算参考轨迹: 避免循环内重复访问 ref
 * 5. 减少 memcpy/memset: 使用直接赋值
 * 6. 展开内层循环 (2状态系统)
 *
 * 预期性能提升:
 * - MPC_Update: ~3x 加速 (迭代20->5 + 内联优化)
 */

#include "mpc.h"
#include <string.h>

void MPC_Init(MPC_t *mpc, float dt)
{
    memset(mpc, 0, sizeof(MPC_t));
    mpc->dt = dt;
    mpc->Q = 1.0f;
    mpc->R = 0.1f;
    mpc->u_min = -100; mpc->u_max = 100;
    mpc->du_min = -10;  mpc->du_max = 10;
}

void MPC_SetModel(MPC_t *mpc, float A[2][2], float B[2], float C[2])
{
    for (int i = 0; i < MPC_NX; i++) {
        for (int j = 0; j < MPC_NX; j++)
            mpc->A[i][j] = A[i][j];
        mpc->B[i] = B[i];
        mpc->C[i] = C[i];
    }
}

void MPC_SetWeight(MPC_t *mpc, float Q, float R) { mpc->Q = Q; mpc->R = R; }

void MPC_SetConstraint(MPC_t *mpc, float u_min, float u_max, float du_min, float du_max)
{
    mpc->u_min = u_min; mpc->u_max = u_max;
    mpc->du_min = du_min; mpc->du_max = du_max;
}

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float MPC_Update(MPC_t *mpc, float ref, float y_meas)
{
    /* 用观测值修正状态 */
    mpc->x[0] = y_meas;

    /* 优化: 使用动量梯度下降，减少迭代次数 */
    static float du_prev[MPC_NC] = {0};  /* 动量 */
    float alpha = 0.15f;   /* 梯度步长 (比原来略大) */
    float beta = 0.5f;     /* 动量系数 */

    /* 保留上一次的 du 作为初始值 (warm start) */
    /* 不清零 du，使用上次结果作为起点 */

    /* 预计算 A, B, C 的局部引用 (编译器优化友好) */
    float a00 = mpc->A[0][0], a01 = mpc->A[0][1];
    float a10 = mpc->A[1][0], a11 = mpc->A[1][1];
    float b0 = mpc->B[0], b1 = mpc->B[1];
    float c0 = mpc->C[0], c1 = mpc->C[1];
    float Q2 = 2.0f * mpc->Q;
    float R2 = 2.0f * mpc->R;

    /* 优化: 5次迭代 (原来20次), 使用动量加速收敛 */
    for (int iter = 0; iter < 5; iter++) {
        float grad[MPC_NC] = {0};

        /* 前向预测 -- 内联状态更新，展开2状态循环 */
        float xp0 = mpc->x[0], xp1 = mpc->x[1];
        float y_pred_arr[MPC_NP];
        float u_curr = 0;

        for (int k = 0; k < MPC_NP; k++) {
            int ci = (k < MPC_NC) ? k : MPC_NC - 1;
            if (k == 0) {
                u_curr = mpc->u_total + mpc->du[0];
            } else {
                u_curr = u_curr + mpc->du[ci];
            }

            /* 内联状态预测 (展开2x2矩阵乘法) */
            float xn0 = a00 * xp0 + a01 * xp1 + b0 * u_curr;
            float xn1 = a10 * xp0 + a11 * xp1 + b1 * u_curr;
            xp0 = xn0;
            xp1 = xn1;

            /* 内联输出计算 */
            y_pred_arr[k] = c0 * xn0 + c1 * xn1;
        }

        /* 计算梯度 */
        for (int j = 0; j < MPC_NC; j++) {
            float g = 0.0f;
            for (int k = j; k < MPC_NP; k++) {
                g += y_pred_arr[k] - ref;
            }
            grad[j] = -Q2 * g + R2 * mpc->du[j];
        }

        /* 动量梯度下降更新 */
        for (int j = 0; j < MPC_NC; j++) {
            float update = alpha * grad[j] + beta * du_prev[j];
            mpc->du[j] -= update;
            du_prev[j] = update;
        }
    }

    /* 应用第一个控制增量 */
    mpc->u_total += mpc->du[0];
    mpc->u_total = clampf(mpc->u_total, mpc->u_min, mpc->u_max);

    return mpc->u_total;
}
