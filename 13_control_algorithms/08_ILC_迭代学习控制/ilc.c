/**
 * @file ilc.c
 * @brief 迭代学习控制（ILC）实现
 *
 * 支持P/D/PD/PID四种学习律：
 *   P型:  u_{k+1}(t) = Q * u_k(t) + Lp * e_k(t)
 *   D型:  u_{k+1}(t) = Q * u_k(t) + Ld * Δe_k(t)
 *   PD型: u_{k+1}(t) = Q * u_k(t) + Lp * e_k(t) + Ld * Δe_k(t)
 *   PID型:u_{k+1}(t) = Q * u_k(t) + Lp * e_k(t) + Li * Σe_k(t) + Ld * Δe_k(t)
 *
 * 参考：
 * - Bristow et al., "A survey of iterative learning control" (IEEE CSM, 2006)
 * - Ahn et al., "Iterative learning control: brief survey and categorization" (2007)
 *
 * @note 所有学习律均基于上一次迭代的误差 e_k(t) 更新控制量。
 *       首次迭代无先验信息，使用P型反馈控制。
 *       调用者需在每步ILC_Update()之后调用ILC_RecordError()记录当前误差。
 */

#include "ilc.h"
#include <string.h>

void ILC_Init(ILC_t *ilc, ILC_Type type, float Lp, float Li, float Ld, float Q, int N)
{
    memset(ilc, 0, sizeof(ILC_t));
    ilc->type = type;
    ilc->Lp = Lp; ilc->Li = Li; ilc->Ld = Ld;
    ilc->Q = Q;
    ilc->N = (N > 0 && N < ILC_N) ? N : ILC_N;
    ilc->u_max = 100.0f;
    ilc->iter = 0;
}

void ILC_SetOutputLimit(ILC_t *ilc, float max) { ilc->u_max = max; }

void ILC_StartIteration(ILC_t *ilc)
{
    /* 新迭代开始时清零误差累积（PID型用） */
    if (ilc->type == ILC_PID)
        memset(ilc->e_sum, 0, sizeof(float) * ilc->N);
}

float ILC_Update(ILC_t *ilc, int step, float ref, float y)
{
    if (step >= ilc->N || step < 0) return 0;
    
    float e = ref - y;
    float u;
    
    if (ilc->iter == 0) {
        /* 第一次迭代：无先验信息，仅用P控制 */
        u = ilc->Lp * e;
    } else {
        /* 基础学习律: u_{k+1} = Q*u_k + correction */
        float u_base = ilc->Q * ilc->u_prev[step];
        float correction = 0;
        
        float e_prev = ilc->e_prev[step];
        
        switch (ilc->type) {
        case ILC_P:
            /* P型: u = Q*u_prev + Lp*e_prev */
            correction = ilc->Lp * e_prev;
            break;
            
        case ILC_D:
            /* D型: u = Q*u_prev + Ld*(e_prev[k] - e_prev[k-1]) */
            if (step > 0)
                correction = ilc->Ld * (e_prev - ilc->e_prev[step-1]);
            break;
            
        case ILC_PD:
            /* PD型: u = Q*u_prev + Lp*e_prev + Ld*Δe_prev */
            correction = ilc->Lp * e_prev;
            if (step > 0)
                correction += ilc->Ld * (e_prev - ilc->e_prev[step-1]);
            break;
            
        case ILC_PID:
            /* PID型: u = Q*u_prev + Lp*e_prev + Li*Σe_prev + Ld*Δe_prev */
            ilc->e_sum[step] += e_prev;
            correction = ilc->Lp * e_prev 
                       + ilc->Li * ilc->e_sum[step];
            if (step > 0)
                correction += ilc->Ld * (e_prev - ilc->e_prev[step-1]);
            break;
        }
        
        u = u_base + correction;
    }
    
    /* 输出限幅 */
    if (u > ilc->u_max) u = ilc->u_max;
    if (u < -ilc->u_max) u = -ilc->u_max;
    
    ilc->u_out[step] = u;
    return u;
}

void ILC_EndIteration(ILC_t *ilc)
{
    /* 保存本次迭代数据供下次使用 */
    memcpy(ilc->u_prev, ilc->u_out, sizeof(float) * ilc->N);
    /* e_prev在Update中已经记录 */
    ilc->iter++;
}

/* 外部调用：记录当前步误差 */
void ILC_RecordError(ILC_t *ilc, int step, float e)
{
    if (step >= 0 && step < ilc->N)
        ilc->e_prev[step] = e;
}

int ILC_GetIteration(ILC_t *ilc) { return ilc->iter; }
