#include "mpc.h"
#include <string.h>

/*
 * MPC嵌入式简化实现
 * 
 * 核心思想：基于模型预测未来Np步输出，通过最小化代价函数
 * J = Σ[Q*(y_ref - y_pred)^2 + R*(Δu)^2] 求解最优控制增量序列
 * 
 * 简化求解：使用梯度下降法代替QP求解器，适合嵌入式
 */

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

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 状态预测 */
static void predict_state(MPC_t *mpc, float *x, float u, float *x_next)
{
    x_next[0] = mpc->A[0][0]*x[0] + mpc->A[0][1]*x[1] + mpc->B[0]*u;
    x_next[1] = mpc->A[1][0]*x[0] + mpc->A[1][1]*x[1] + mpc->B[1]*u;
}

static float output(MPC_t *mpc, float *x)
{
    return mpc->C[0]*x[0] + mpc->C[1]*x[1];
}

float MPC_Update(MPC_t *mpc, float ref, float y_meas)
{
    /* 用观测值修正状态（简化：直接替换第一个状态） */
    mpc->x[0] = y_meas;
    
    /* 控制增量序列 du[0..Nc-1]，用梯度下降迭代求解 */
    memset(mpc->du, 0, sizeof(mpc->du));
    
    float alpha = 0.1f; /* 梯度步长 */
    for (int iter = 0; iter < 20; iter++) {
        float grad[MPC_NC];
        memset(grad, 0, sizeof(grad));
        
        /* 前向预测 */
        float xp[MPC_NX] = {mpc->x[0], mpc->x[1]};
        float u_curr = 0; /* 当前累积控制量 */
        
        float xp_all[MPC_NP+1][MPC_NX];
        memcpy(xp_all[0], xp, sizeof(xp));
        float y_pred[MPC_NP], u_pred_arr[MPC_NP];
        
        for (int k = 0; k < MPC_NP; k++) {
            int ci = (k < MPC_NC) ? k : MPC_NC - 1;
            if (k == 0) u_curr = mpc->u_total + mpc->du[0]; /* 基于上一步控制量 */
            else u_curr = u_pred_arr[k-1] + mpc->du[ci];
            u_pred_arr[k] = u_curr;
            
            float xn[MPC_NX];
            predict_state(mpc, xp_all[k], u_curr, xn);
            memcpy(xp_all[k+1], xn, sizeof(xn));
            y_pred[k] = output(mpc, xn);
        }
        
        /* 计算梯度 */
        for (int j = 0; j < MPC_NC; j++) {
            for (int k = j; k < MPC_NP; k++) {
                float err = y_pred[k] - ref;
                grad[j] += -2.0f * mpc->Q * err;
            }
            grad[j] += 2.0f * mpc->R * mpc->du[j];
        }
        
        /* 梯度下降更新 */
        for (int j = 0; j < MPC_NC; j++)
            mpc->du[j] -= alpha * grad[j];
    }
    
    /* 应用第一个控制增量 */
    mpc->u_total += mpc->du[0];
    mpc->u_total = clampf(mpc->u_total, mpc->u_min, mpc->u_max);
    
    return mpc->u_total;
}
