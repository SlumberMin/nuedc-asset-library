/**
 * @file    adrc.c
 * @brief   ADRC自抗扰控制器实现
 * 
 * 参数整定方法（带宽法）：
 * 1. b0：系统增益的粗略估计
 *    - 电机系统：b0 ≈ K_motor（电机增益）
 *    - 磁悬浮系统：b0 ≈ dF/dI（电磁力对电流的导数）
 * 
 * 2. ESO带宽ωo：
 *    β01 = 3ωo
 *    β02 = 3ωo²
 *    β03 = ωo³
 *    ωo越大，ESO跟踪越快，但对噪声越敏感
 *    推荐：ωo = 5~20（根据采样频率调整）
 * 
 * 3. 控制器带宽ωc：
 *    kp = ωc²
 *    kd = 2ωc
 *    ωc决定闭环响应速度
 *    推荐：ωc = ωo / 3~5
 * 
 * 4. fal函数参数：
 *    α1 = 0.5（大误差快速响应）
 *    α2 = 0.25（扰动估计更精确）
 *    δ = 5~10倍噪声水平
 */

#include "adrc.h"
#include <math.h>

/**
 * @brief  fal非线性函数
 * @param  e: 误差
 * @param  alpha: 指数(0<alpha<1)
 * @param  delta: 线性区间
 * @retval float: fal输出
 * 
 * fal函数特性：
 * |e| > δ 时：|e|^α * sign(e)，对大误差快速响应
 * |e| ≤ δ 时：e/δ^(1-α)，对小误差线性处理
 */
static float fal(float e, float alpha, float delta)
{
    float abs_e = fabsf(e);
    if(abs_e > delta)
    {
        return powf(abs_e, alpha) * ((e > 0) ? 1.0f : -1.0f);
    }
    else
    {
        return e / powf(delta, 1.0f - alpha);
    }
}

/**
 * @brief  TD跟踪微分器
 * @param  adrc: ADRC控制器指针
 * @param  r: 参考输入
 * 
 * TD作用：对参考信号安排过渡过程
 * 避免阶跃输入导致的大超调
 * 
 * 最速离散系统：
 * x1(k+1) = x1(k) + h * x2(k)
 * x2(k+1) = x2(k) + h * fst(x1(k)-r, x2(k), r, h0)
 */
static void TD_Update(ADRC_t *adrc, float r)
{
    float d = adrc->r * adrc->h0 * adrc->h0;
    float a0 = adrc->h0 * adrc->x2;
    float y = adrc->x1 - r + a0;
    float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    float a2 = a0 + ((y > 0) ? 1.0f : -1.0f) * (a1 - d) * 0.5f;
    float sy = ((y > 0) ? 1.0f : -1.0f);
    float sa = ((a2 > 0) ? 1.0f : -1.0f);
    float fst;
    
    if(fabsf(y) >= d)
        fst = -adrc->r * sa;
    else
        fst = -adrc->r * a2 / d;
    
    adrc->x1 += adrc->h * adrc->x2;
    adrc->x2 += adrc->h * fst;
}

/**
 * @brief  ESO扩张状态观测器更新
 * @param  adrc: ADRC控制器指针
 * @param  y: 系统输出（传感器测量值）
 * @param  u: 上一次控制输出
 * 
 * ESO核心：估计系统状态(z1,z2)和总扰动(z3)
 * 
 * z1(k+1) = z1(k) + h*(z2 - β01*e)
 * z2(k+1) = z2(k) + h*(z3 - β02*fal(e,α1,δ) + b0*u)
 * z3(k+1) = z3(k) + h*(-β03*fal(e,α2,δ))
 */
static void ESO_Update(ADRC_t *adrc, float y, float u)
{
    float e = adrc->z1 - y;
    
    adrc->z1 += adrc->h * (adrc->z2 - adrc->beta01 * e);
    adrc->z2 += adrc->h * (adrc->z3 - adrc->beta02 * fal(e, adrc->alpha1, adrc->delta) + adrc->b0 * u);
    adrc->z3 += adrc->h * (-adrc->beta03 * fal(e, adrc->alpha2, adrc->delta));
}

/**
 * @brief  NLSEF非线性状态误差反馈
 * @param  adrc: ADRC控制器指针
 * @param  v1: TD输出1
 * @param  v2: TD输出2
 * @retval float: 控制量
 * 
 * u0 = kp*fal(v1-z1, α1, δ) + kd*fal(v2-z2, α2, δ)
 * u = (u0 - z3) / b0
 */
static float NLSEF_Calculate(ADRC_t *adrc, float v1, float v2)
{
    float e1 = v1 - adrc->z1;
    float e2 = v2 - adrc->z2;
    
    float u0 = adrc->kp * fal(e1, adrc->alpha1, adrc->delta) + 
               adrc->kd * fal(e2, adrc->alpha2, adrc->delta);
    
    float u = (u0 - adrc->z3) / adrc->b0;
    
    /* 输出限幅 */
    if(u > adrc->u_max) u = adrc->u_max;
    if(u < adrc->u_min) u = adrc->u_min;
    
    return u;
}

/**
 * @brief  初始化ADRC控制器
 * @param  adrc: ADRC控制器指针
 * @param  h: 采样周期(s)
 * @param  b0: 系统增益估计
 * @param  kp: 比例增益
 * @param  kd: 微分增益
 * @param  u_min: 输出下限
 * @param  u_max: 输出上限
 */
void ADRC_Init(ADRC_t *adrc, float h, float b0, float kp, float kd,
               float u_min, float u_max)
{
    adrc->h = h;
    adrc->b0 = b0;
    adrc->kp = kp;
    adrc->kd = kd;
    adrc->u_min = u_min;
    adrc->u_max = u_max;
    
    /* 默认参数 */
    adrc->r = 100.0f;       // 速度因子
    adrc->h0 = h;           // 滤波因子=采样周期
    adrc->alpha1 = 0.5f;
    adrc->alpha2 = 0.25f;
    adrc->delta = 5.0f * h; // 线性区间=5倍采样周期
    
    /* 默认ESO增益（带宽法，ωo=10） */
    float omega_o = 10.0f;
    adrc->beta01 = 3.0f * omega_o;
    adrc->beta02 = 3.0f * omega_o * omega_o;
    adrc->beta03 = omega_o * omega_o * omega_o;
    
    /* 初始化状态 */
    adrc->x1 = 0; adrc->x2 = 0;
    adrc->z1 = 0; adrc->z2 = 0; adrc->z3 = 0;
    adrc->u = 0;
}

/**
 * @brief  通过带宽法设置ADRC参数（推荐方法）
 * @param  adrc: ADRC控制器指针
 * @param  omega_o: ESO带宽（越大跟踪越快，但对噪声敏感）
 * @param  omega_c: 控制器带宽（决定闭环响应速度）
 * 
 * 推荐关系：omega_c = omega_o / 3 ~ omega_o / 5
 */
void ADRC_SetBandwidth(ADRC_t *adrc, float omega_o, float omega_c)
{
    adrc->beta01 = 3.0f * omega_o;
    adrc->beta02 = 3.0f * omega_o * omega_o;
    adrc->beta03 = omega_o * omega_o * omega_o;
    
    adrc->kp = omega_c * omega_c;
    adrc->kd = 2.0f * omega_c;
}

/**
 * @brief  ADRC控制器更新（主调用函数）
 * @param  adrc: ADRC控制器指针
 * @param  y: 系统当前输出（传感器测量值）
 * @param  r: 参考输入（目标值）
 * @retval float: 控制输出
 */
float ADRC_Update(ADRC_t *adrc, float y, float r)
{
    /* 1. TD：安排过渡过程 */
    TD_Update(adrc, r);
    
    /* 2. ESO：估计状态和扰动 */
    ESO_Update(adrc, y, adrc->u);
    
    /* 3. NLSEF：生成控制量 */
    adrc->u = NLSEF_Calculate(adrc, adrc->x1, adrc->x2);
    
    return adrc->u;
}

/**
 * @brief  重置ADRC控制器
 */
void ADRC_Reset(ADRC_t *adrc)
{
    adrc->x1 = 0; adrc->x2 = 0;
    adrc->z1 = 0; adrc->z2 = 0; adrc->z3 = 0;
    adrc->u = 0;
}
