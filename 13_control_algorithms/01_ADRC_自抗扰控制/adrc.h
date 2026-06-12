#ifndef ADRC_H
#define ADRC_H

typedef struct {
    /* TD参数 */
    float r0;          /* 速度因子 */
    float h0;          /* 滤波因子 */
    float v1, v2;      /* TD输出: 跟踪值, 微分值 */
    
    /* ESO参数 */
    float beta01, beta02, beta03;  /* ESO增益 */
    float z1, z2, z3;  /* ESO状态: 位置估计, 速度估计, 扰动估计 */
    
    /* NLSEF参数 */
    float kp, kd;      /* 比例、微分增益 */
    float b0;          /* 系统增益估计 */
    float delta;       /* 线性区间 */
    
    /* 系统参数 */
    float dt;          /* 采样周期 */
    float u;           /* 控制输出 */
    float u_max;       /* 输出限幅 */
} ADRC_t;

void ADRC_Init(ADRC_t *adrc, float r0, float h0, float b0, 
               float omega_c, float omega_o, float delta, float dt);
float ADRC_Update(ADRC_t *adrc, float ref, float y);
void ADRC_SetOutputLimit(ADRC_t *adrc, float max);
void ADRC_Reset(ADRC_t *adrc);

#endif
