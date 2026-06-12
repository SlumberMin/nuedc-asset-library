#include "adrc.h"
#include <math.h>

/*
 * 自抗扰控制 ADRC (Active Disturbance Rejection Control)
 * 
 * 原理说明：
 * ADRC由韩京清教授提出，核心思想是将外部扰动和内部模型不确定性
 * 统一视为"总扰动"，通过扩张状态观测器(ESO)实时估计并补偿。
 * 
 * 组成部分：
 * 1. 跟踪微分器(TD) - 安排过渡过程，提取微分信号
 * 2. 扩张状态观测器(ESO) - 估计系统状态和总扰动
 * 3. 非线性状态误差反馈(NLSEF) - 产生控制量
 * 
 * 适用场景：
 * - 电机转速/位置控制
 * - 姿态控制（无人机、平衡车）
 * - 温度控制、液位控制
 * - 任何存在不确定性和扰动的系统
 * 
 * 参数整定指南：
 * r0: TD速度因子，越大跟踪越快，但噪声敏感
 * h0: TD滤波因子，越大滤波效果越好
 * beta01,beta02,beta03: ESO增益，通常用带宽法整定(omega_o)
 *   beta01=3*omega_o, beta02=3*omega_o^2, beta03=omega_o^3
 * b0: 系统增益估计值，需要较准确估计
 * omega_c: 控制器带宽，决定响应速度
 * delta: NLSEF线性区间，过小会振荡
 */

void ADRC_Init(ADRC_t *adrc, float r0, float h0, float b0, 
               float omega_c, float omega_o, float delta, float dt)
{
    /* 跟踪微分器参数 */
    adrc->r0 = r0;
    adrc->h0 = h0;
    
    /* ESO参数 - 使用带宽法整定 */
    adrc->beta01 = 3.0f * omega_o;
    adrc->beta02 = 3.0f * omega_o * omega_o;
    adrc->beta03 = omega_o * omega_o * omega_o;
    
    /* 控制器参数 */
    adrc->b0 = b0;
    adrc->delta = delta;
    adrc->dt = dt;
    
    /* NLSEF增益 */
    adrc->kp = omega_c * omega_c;
    adrc->kd = 2.0f * omega_c;
    
    /* 状态清零 */
    adrc->v1 = 0.0f;
    adrc->v2 = 0.0f;
    adrc->z1 = 0.0f;
    adrc->z2 = 0.0f;
    adrc->z3 = 0.0f;
    adrc->u  = 0.0f;
}

/*
 * 跟踪微分器 TD (Tracking Differentiator)
 * 实现离散最速跟踪微分器
 * 输入: v0 - 目标信号
 * 输出: v1 - 跟踪信号, v2 - 微分信号
 */
static void TD_Update(ADRC_t *adrc, float v0)
{
    float fh = adrc->r0 * adrc->h0 * adrc->h0;
    float e = adrc->v1 - v0;
    
    /* 最速综合函数 fhan */
    float d = adrc->r0 * adrc->h0;
    float d0 = d * adrc->h0;
    float y = e + adrc->h0 * adrc->v2;
    float a0 = sqrtf(d * d + 8.0f * adrc->r0 * fabsf(y));
    float a;
    
    if (fabsf(y) > d0) {
        a = adrc->v2 + (a0 - d) * 0.5f * ((y > 0) ? 1.0f : -1.0f);
    } else {
        a = adrc->v2 + y / adrc->h0;
    }
    
    float fhan;
    if (fabsf(a) > d) {
        fhan = -adrc->r0 * ((a > 0) ? 1.0f : -1.0f);
    } else {
        fhan = -adrc->r0 * a / d;
    }
    
    adrc->v1 += adrc->h0 * adrc->v2;
    adrc->v2 += fh * fhan;
}

/*
 * 扩张状态观测器 ESO (Extended State Observer)
 * 
 * 三阶ESO估计系统状态z1,z2和总扰动z3
 * z1 -> y的估计
 * z2 -> y'的估计  
 * z3 -> 总扰动f的估计
 * 
 * 使用非线性ESO: fal函数
 */
static float fal(float e, float alpha, float delta)
{
    if (fabsf(e) > delta) {
        return powf(fabsf(e), alpha) * ((e > 0) ? 1.0f : -1.0f);
    } else {
        return e / powf(delta, 1.0f - alpha);
    }
}

static void ESO_Update(ADRC_t *adrc, float y, float u)
{
    float e = adrc->z1 - y;
    
    /* fal函数用于非线性ESO */
    float fe1 = fal(e, 0.5f, adrc->delta);
    float fe2 = fal(e, 0.25f, adrc->delta);
    
    adrc->z1 += adrc->dt * (adrc->z2 - adrc->beta01 * e);
    adrc->z2 += adrc->dt * (adrc->z3 - adrc->beta02 * fe1 + adrc->b0 * u);
    adrc->z3 += adrc->dt * (-adrc->beta03 * fe2);
}

/*
 * 非线性状态误差反馈 NLSEF
 * 使用fal函数实现非线性PID
 */
static float NLSEF(ADRC_t *adrc, float e1, float e2)
{
    float u0 = adrc->kp * fal(e1, 0.5f, adrc->delta) 
              + adrc->kd * fal(e2, 0.25f, adrc->delta);
    return u0;
}

float ADRC_Update(ADRC_t *adrc, float ref, float y)
{
    /* 1. 跟踪微分器: 安排过渡过程 */
    TD_Update(adrc, ref);
    
    /* 2. 扩张状态观测器: 估计状态和扰动 */
    ESO_Update(adrc, y, adrc->u);
    
    /* 3. 计算误差 */
    float e1 = adrc->v1 - adrc->z1;  /* 跟踪误差 */
    float e2 = adrc->v2 - adrc->z2;  /* 微分误差 */
    
    /* 4. 非线性误差反馈 */
    float u0 = NLSEF(adrc, e1, e2);
    
    /* 5. 扰动补偿 */
    adrc->u = (u0 - adrc->z3) / adrc->b0;
    
    /* 输出限幅 */
    if (adrc->u > adrc->u_max) adrc->u = adrc->u_max;
    if (adrc->u < -adrc->u_max) adrc->u = -adrc->u_max;
    
    return adrc->u;
}

void ADRC_SetOutputLimit(ADRC_t *adrc, float max)
{
    adrc->u_max = max;
}

void ADRC_Reset(ADRC_t *adrc)
{
    adrc->v1 = 0; adrc->v2 = 0;
    adrc->z1 = 0; adrc->z2 = 0; adrc->z3 = 0;
    adrc->u = 0;
}
