/**
 * @file    mpc.c
 * @brief   简化MPC模型预测控制器实现
 * 
 * 使用简化QP求解（梯度法），适合STM32实时计算
 * 
 * 算法步骤：
 * 1. 预测未来N步状态
 * 2. 计算最优控制序列
 * 3. 只应用第一步控制量
 * 4. 下一周期重复
 */

#include "mpc.h"
#include <math.h>

void MPC_Init(MPC_t *mpc, float A, float B, float Q, float R,
              float u_min, float u_max, float du_max)
{
    mpc->A = A;
    mpc->B = B;
    mpc->Q = Q;
    mpc->R = R;
    mpc->u_min = u_min;
    mpc->u_max = u_max;
    mpc->du_max = du_max;
    mpc->x = 0;
    mpc->u = 0;
    mpc->u_prev = 0;
    mpc->output = 0;
}

float MPC_Calculate(MPC_t *mpc, float ref, float measured)
{
    mpc->x = measured;
    
    /* 简化MPC：只预测一步，计算最优控制增量 */
    float error = ref - mpc->x;
    
    /* 预测下一步状态 */
    float x_pred = mpc->A * mpc->x + mpc->B * mpc->u;
    float error_pred = ref - x_pred;
    
    /* 计算最优控制增量（梯度法） */
    float du = -mpc->Q * error - mpc->R * (mpc->u - mpc->u_prev);
    
    /* 控制增量限幅 */
    if(du > mpc->du_max) du = mpc->du_max;
    if(du < -mpc->du_max) du = -mpc->du_max;
    
    /* 更新控制量 */
    mpc->u += du;
    
    /* 控制量限幅 */
    if(mpc->u > mpc->u_max) mpc->u = mpc->u_max;
    if(mpc->u < mpc->u_min) mpc->u = mpc->u_min;
    
    mpc->u_prev = mpc->u;
    mpc->output = mpc->u;
    
    return mpc->output;
}

void MPC_Reset(MPC_t *mpc)
{
    mpc->x = 0;
    mpc->u = 0;
    mpc->u_prev = 0;
    mpc->output = 0;
}
