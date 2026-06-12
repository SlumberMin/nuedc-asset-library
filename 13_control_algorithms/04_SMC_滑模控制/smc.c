#include "smc.h"
#include <math.h>

/*
 * SMC滑模控制实现
 * 支持4种趋近律，使用边界层法减小抖振
 */

void SMC_Init(SMC_t *smc, SMC_LawType law, float c, float eps, float k, float phi)
{
    smc->law = law;
    smc->c = c;
    smc->eps = eps;
    smc->k = k;
    smc->alpha = 0.5f;  /* 默认幂次指数 */
    smc->phi = phi;
    smc->u_max = 100.0f;
}

void SMC_SetOutputLimit(SMC_t *smc, float max) { smc->u_max = max; }

/* 带边界层的符号函数，减小抖振 */
static float sat(float s, float phi)
{
    if (phi <= 0) return (s > 0) - (s < 0);
    if (s > phi)  return 1.0f;
    if (s < -phi) return -1.0f;
    return s / phi;
}

float SMC_Update(SMC_t *smc, float e, float e_dot, float u_eq)
{
    /* 计算滑模面 s = e_dot + c * e */
    float s = e_dot + smc->c * e;
    
    float u_sw = 0;
    
    switch (smc->law) {
    case SMC_LAW_CONSTANT:
        /* 等速趋近律: u_sw = eps * sat(s/phi) */
        u_sw = smc->eps * sat(s, smc->phi);
        break;
        
    case SMC_LAW_EXPONENTIAL:
        /* 指数趋近律: u_sw = eps*sat(s/phi) + k*s */
        u_sw = smc->eps * sat(s, smc->phi) + smc->k * s;
        break;
        
    case SMC_LAW_POWER:
        /* 幂次趋近律: u_sw = k*|s|^alpha * sat(s/phi) */
        {
            float abs_s = fabsf(s);
            float s_alpha = (abs_s < 1e-10f) ? 0.0f : powf(abs_s, smc->alpha);
            u_sw = smc->k * s_alpha * sat(s, smc->phi);
        }
        break;
        
    case SMC_LAW_COMBINED:
        /* 组合趋近律: k*|s|^alpha*sat(s/phi) + eps*sat(s/phi) + lambda*s */
        {
            float abs_s = fabsf(s);
            float s_alpha = (abs_s < 1e-10f) ? 0.0f : powf(abs_s, smc->alpha);
            u_sw = smc->k * s_alpha * sat(s, smc->phi) 
                 + smc->eps * sat(s, smc->phi)
                 + 2.0f * smc->k * s;  /* lambda=2*k */
        }
        break;
    }
    
    float u = u_eq + u_sw;
    
    /* 输出限幅 */
    if (u > smc->u_max) u = smc->u_max;
    if (u < -smc->u_max) u = -smc->u_max;
    
    return u;
}
