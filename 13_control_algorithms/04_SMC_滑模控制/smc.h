#ifndef SMC_H
#define SMC_H

/*
 * 滑模控制 SMC (Sliding Mode Control) - 多种趋近律实现
 * 
 * 原理：设计切换面s(x)=0，使系统状态在有限时间到达并沿滑模面运动
 * 控制律由等效控制+切换控制组成: u = u_eq + u_sw
 * 
 * 趋近律类型：
 * 1. 等速趋近律: ds = -eps*sign(s)
 * 2. 指数趋近律: ds = -eps*sign(s) - k*s
 * 3. 幂次趋近律: ds = -k*|s|^alpha*sign(s)
 * 4. 组合趋近律: 指数+幂次
 * 
 * 适用场景：电机控制、机械臂、飞行器、强扰动系统
 * 参数整定：c越大收敛越快，eps越大抖振越大
 */

typedef enum {
    SMC_LAW_CONSTANT = 0,   /* 等速趋近律 */
    SMC_LAW_EXPONENTIAL,    /* 指数趋近律 */
    SMC_LAW_POWER,          /* 幂次趋近律 */
    SMC_LAW_COMBINED        /* 组合趋近律 */
} SMC_LawType;

typedef struct {
    float c;         /* 滑模面参数 s = e_dot + c*e */
    float eps;       /* 切换增益 */
    float k;         /* 指数/幂次增益 */
    float alpha;     /* 幂次指数 (0.5~1) */
    float phi;       /* 边界层厚度（用于sigmoid代替sign，减小抖振） */
    float u_max;     /* 输出限幅 */
    SMC_LawType law;
} SMC_t;

void SMC_Init(SMC_t *smc, SMC_LawType law, float c, float eps, float k, float phi);
float SMC_Update(SMC_t *smc, float e, float e_dot, float u_eq);
void SMC_SetOutputLimit(SMC_t *smc, float max);

#endif
