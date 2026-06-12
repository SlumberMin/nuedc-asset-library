/**
 * @file ladrc.c
 * @brief 线性自抗扰控制器 (LADRC) 实现
 *
 * 二阶系统的 LADRC 结构：
 *   被控对象：y'' = f(y, y', w, t) + b*u
 *   其中 f 为总扰动（内扰+外扰），b 为增益
 *
 * LESO 观测器方程（离散化）：
 *   e = z1 - y
 *   z1 += (z2 - β1*e) * dt
 *   z2 += (z3 - β2*e + b0*u) * dt
 *   z3 += (-β3*e) * dt
 *
 * LSEF 控制律：
 *   u0 = Kp*(v1 - z1) + Kd*(v2 - z2)
 *   u = (u0 - z3) / b0
 */

#include "ladrc.h"

/**
 * @brief 限幅函数
 */
static float saturate(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/**
 * @brief 初始化 LADRC 控制器
 *
 * 使用带宽法自动计算 LESO 和 LSEF 增益：
 *   LESO 特征多项式：(s + ωo)³ → β1=3ωo, β2=3ωo², β3=ωo³
 *   LSEF 特征多项式：(s + ωc)² → Kp=ωc², Kd=2ωc
 */
void LADRC_Init(LADRC_t *ladrc, float wc, float wo, float b0,
                float dt, float u_max, float u_min)
{
    /* 保存基本参数 */
    ladrc->wc = wc;
    ladrc->wo = wo;
    ladrc->b0 = (fabsf(b0) < 1e-6f) ? 1e-6f : b0;  /* 防止除零 */
    ladrc->dt = dt;

    /* 输出限幅 */
    ladrc->u_max = u_max;
    ladrc->u_min = u_min;

    /* 计算 LESO 增益（带宽法） */
    ladrc->beta1 = 3.0f * wo;
    ladrc->beta2 = 3.0f * wo * wo;
    ladrc->beta3 = wo * wo * wo;

    /* 计算 LSEF 增益（带宽法） */
    ladrc->Kp = wc * wc;
    ladrc->Kd = 2.0f * wc;

    /* 初始化 TD 默认参数 */
    ladrc->r0 = 100.0f;   /* 默认速度因子 */
    ladrc->h0 = dt;        /* 默认滤波因子等于采样周期 */

    /* 清零所有状态 */
    LADRC_Reset(ladrc);
}

/**
 * @brief 设置 TD 参数
 */
void LADRC_SetTD(LADRC_t *ladrc, float r0, float h0)
{
    ladrc->r0 = r0;
    ladrc->h0 = h0;
}

/**
 * @brief 重置控制器状态
 */
void LADRC_Reset(LADRC_t *ladrc)
{
    ladrc->z1 = 0.0f;
    ladrc->z2 = 0.0f;
    ladrc->z3 = 0.0f;
    ladrc->v1 = 0.0f;
    ladrc->v2 = 0.0f;
    ladrc->u0 = 0.0f;
    ladrc->u = 0.0f;
    ladrc->last_output = 0.0f;
}

/**
 * @brief 跟踪微分器 (TD)
 *
 * 实现离散最速跟踪微分器，对设定值安排过渡过程。
 * 使用 fhan（最速综合函数）实现无超调跟踪。
 *
 * 离散形式：
 *   fh = fhan(v1 - sp, v2, r0, h0)
 *   v1 += h0 * v2
 *   v2 += h0 * fh
 */
static void LADRC_TD_Update(LADRC_t *ladrc, float setpoint)
{
    float h = ladrc->h0;
    float r = ladrc->r0;
    float x1 = ladrc->v1 - setpoint;
    float x2 = ladrc->v2;

    /* fhan 函数计算 */
    float d = r * h;
    float d0 = d * h;
    float y = x1 + h * x2;
    float a0 = sqrtf(d * d + 8.0f * r * fabsf(y));

    float a;
    if (fabsf(y) > d0) {
        a = x2 + (a0 - d) * 0.5f * ((y > 0) ? 1.0f : -1.0f);
    } else {
        a = x2 + y / h;
    }

    float fh;
    if (fabsf(a) > d) {
        fh = -r * ((a > 0) ? 1.0f : -1.0f);
    } else {
        fh = -r * a / d;
    }

    /* 更新 TD 状态 */
    ladrc->v1 += h * x2;
    ladrc->v2 += h * fh;
}

/**
 * @brief 线性扩张状态观测器 (LESO)
 *
 * 三阶 LESO，估计系统状态 z1（位置）、z2（速度）和 z3（总扰动）。
 *
 * 离散化方程（前向欧拉）：
 *   e = z1 - y
 *   z1 += (z2 - β1 * e) * dt
 *   z2 += (z3 - β2 * e + b0 * u) * dt
 *   z3 += (-β3 * e) * dt
 */
static void LADRC_LESO_Update(LADRC_t *ladrc, float y, float u)
{
    float dt = ladrc->dt;
    float e = ladrc->z1 - y;

    ladrc->z1 += (ladrc->z2 - ladrc->beta1 * e) * dt;
    ladrc->z2 += (ladrc->z3 - ladrc->beta2 * e + ladrc->b0 * u) * dt;
    ladrc->z3 += (-ladrc->beta3 * e) * dt;
}

/**
 * @brief 线性状态误差反馈 (LSEF) + 扰动补偿
 *
 * LSEF: u0 = Kp*(v1 - z1) + Kd*(v2 - z2)
 * 扰动补偿: u = (u0 - z3) / b0
 */
static float LADRC_LSEF(LADRC_t *ladrc)
{
    /* PD 控制律 */
    ladrc->u0 = ladrc->Kp * (ladrc->v1 - ladrc->z1)
              + ladrc->Kd * (ladrc->v2 - ladrc->z2);

    /* 扰动补偿 */
    float u = (ladrc->u0 - ladrc->z3) / ladrc->b0;

    /* 输出限幅 */
    u = saturate(u, ladrc->u_min, ladrc->u_max);

    return u;
}

/**
 * @brief LADRC 主计算函数
 *
 * 执行顺序：TD → LESO → LSEF
 */
float LADRC_Update(LADRC_t *ladrc, float setpoint, float feedback)
{
    /* 1. 跟踪微分器：安排过渡过程 */
    LADRC_TD_Update(ladrc, setpoint);

    /* 2. LESO：估计状态和总扰动 */
    LADRC_LESO_Update(ladrc, feedback, ladrc->u);

    /* 3. LSEF：计算控制量 */
    ladrc->u = LADRC_LSEF(ladrc);

    /* 保存当前输出 */
    ladrc->last_output = feedback;

    return ladrc->u;
}
