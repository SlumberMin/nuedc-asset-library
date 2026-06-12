/**
 * @file mrac.c
 * @brief 模型参考自适应控制实现
 *
 * 核心思想：
 *   1. 设计一个理想的参考模型（期望的系统行为）
 *   2. 自适应律根据跟踪误差实时调整控制器参数
 *   3. 使实际系统的响应尽可能接近参考模型
 *
 * MIT规则:
 *   参考模型: dxm/dt = -am*xm + am*r (一阶)
 *   控制律:   u = θ * r
 *   自适应律: dθ/dt = -γ * e * xm
 *   其中 e = xm - y (参考输出 - 实际输出)
 *
 * 收敛条件：
 *   - γ > 0 (自适应增益为正)
 *   - 参考模型稳定 (am > 0)
 *   - 系统满足正实性条件
 */

#include "mrac.h"
#include <math.h>

#define CLAMP(val, min_v, max_v) \
    do { if ((val) < (min_v)) (val) = (min_v); \
         if ((val) > (max_v)) (val) = (max_v); } while(0)

#define SIGN(x) ((x > 0) ? 1.0f : ((x < 0) ? -1.0f : 0.0f))

/* ==================== 一阶系统参考模型 ==================== */
/*
* 参考模型: dxm/dt = -am * xm + am * r
 * 离散化:   xm(k+1) = xm(k) + dt * (-am*xm(k) + am*r(k))
 *           xm(k+1) = xm(k) * (1 - am*dt) + am*dt * r(k)
 *
 * 自适应律:
 *   θ(k+1) = θ(k) - γ * e(k) * xm(k) * dt
 *   其中 e(k) = xm(k) - y(k)
 *
 * 控制律:
 *   u(k) = θ(k) * r(k)
 */

static void _UpdateFirstOrder(MRAC_t *mrac, float ref, float y_actual)
{
    float e, dtheta, u;

    /* 计算跟踪误差 */
    e = mrac->xm1 - y_actual;
    mrac->error = e;

    /* 参考模型更新 (欧拉前向离散化) */
    float xm_new = mrac->xm1 + mrac->dt * (-mrac->ref_am1 * mrac->xm1 + mrac->ref_am1 * ref);
    mrac->xm_prev1 = mrac->xm1;
    mrac->xm1 = xm_new;

    /* 自适应律更新 */
    switch (mrac->rule) {
    case MRAC_MIT_RULE:
        /* MIT规则: dθ/dt = -γ * e * xm */
        dtheta = -mrac->gamma * e * mrac->xm_prev1;
        break;

    case MRAC_STRONG_RULE:
        /* 强调规则: dθ/dt = -γ * e * sign(xm) */
        dtheta = -mrac->gamma * e * SIGN(mrac->xm_prev1);
        break;

    default:
        dtheta = -mrac->gamma * e * mrac->xm_prev1;
        break;
    }

    /* 离散化自适应律 */
    mrac->theta += dtheta * mrac->dt;

    /* θ 防止为负（增益不能为负） */
    if (mrac->theta < 0.0f) mrac->theta = 0.0f;

    /* 计算控制输出 */
    u = mrac->theta * ref;

    /* 输出限幅 */
    CLAMP(u, mrac->u_min, mrac->u_max);

    mrac->u_prev = u;
}

/* ==================== 二阶系统参考模型 ==================== */
/*
* 参考模型 (标准形式):
*   d²xm/dt² + 2*ζ*ωn*dxm/dt + ωn²*xm = ωn²*r
*   或写成状态空间:
*   dx1/dt = x2
*   dx2/dt = -ωn²*x1 - 2*ζ*ωn*x2 + ωn²*r
 *
 * 离散化 (欧拉前向):
 *   x1(k+1) = x1(k) + dt * x2(k)
 *   x2(k+1) = x2(k) + dt * (-ωn²*x1(k) - 2*ζ*ωn*x2(k) + ωn²*r(k))
 *
 * 自适应律:
 *   θ(k+1) = θ(k) - γ * e(k) * xm1(k) * dt
 *   其中 e(k) = xm1(k) - y(k) (只使用状态1计算误差)
 *
 * 控制律:
 *   u(k) = θ(k) * r(k)
 */

static void _UpdateSecondOrder(MRAC_t *mrac, float ref, float y_actual)
{
    float e, dtheta, u;
    float xn1_new, xn2_new;

    /* 计算跟踪误差 (使用位置误差) */
    e = mrac->xm1 - y_actual;
    mrac->error = e;

    /* 参考模型更新 (欧拉前向离散化) */
    xn1_new = mrac->xm1 + mrac->dt * mrac->xm2;
    xn2_new = mrac->xm2 + mrac->dt * (-mrac->ref_wn * mrac->ref_wn * mrac->xm1
                                     - 2.0f * mrac->ref_zeta * mrac->ref_wn * mrac->xm2
                                     + mrac->ref_wn * mrac->ref_wn * ref);

    mrac->xm_prev1 = mrac->xm1;
    mrac->xm1 = xn1_new;
    mrac->xm2 = xn2_new;

    /* 自适应律更新 */
    switch (mrac->rule) {
    case MRAC_MIT_RULE:
        /* MIT规则: dθ/dt = -γ * e * xm1 */
        dtheta = -mrac->gamma * e * mrac->xm_prev1;
        break;

    case MRAC_STRONG_RULE:
        /* 强调规则: dθ/dt = -γ * e * sign(xm1) */
        dtheta = -mrac->gamma * e * SIGN(mrac->xm_prev1);
        break;

    default:
        dtheta = -mrac->gamma * e * mrac->xm_prev1;
        break;
    }

    /* 离散化自适应律 */
    mrac->theta += dtheta * mrac->dt;

    /* θ 防止为负 */
    if (mrac->theta < 0.0f) mrac->theta = 0.0f;

    /* 计算控制输出 */
    u = mrac->theta * ref;

    /* 输出限幅 */
    CLAMP(u, mrac->u_min, mrac->u_max);

    mrac->u_prev = u;
}

/* ==================== 接口函数实现 ==================== */

void MRAC_InitFirstOrder(MRAC_t *mrac, float am, float gamma, float dt)
{
    /* 清零结构体 */
    mrac->order = MRAC_ORDER_1ST;
    mrac->rule = MRAC_MIT_RULE;

    /* 参考模型参数 */
    mrac->ref_am1 = am;
    mrac->ref_am2 = 0.0f;
    mrac->ref_zeta = 0.0f;
    mrac->ref_wn = 0.0f;

    /* 自适应参数 */
    mrac->theta = 1.0f;  /* 初始增益设为1 */
    mrac->gamma = gamma;

    /* 状态清零 */
    mrac->xm1 = 0.0f;
    mrac->xm2 = 0.0f;
    mrac->xm_prev1 = 0.0f;
    mrac->u_prev = 0.0f;

    /* 误差历史 */
    mrac->error = 0.0f;
    mrac->error_integral = 0.0f;

    /* 输出限幅 */
    mrac->u_max = 100.0f;
    mrac->u_min = -100.0f;

    /* 时间 */
    mrac->dt = dt;

    /* 标记初始化完成 */
    mrac->is_initialized = 1;
}

void MRAC_InitSecondOrder(MRAC_t *mrac, float wn, float zeta, float gamma, float dt)
{
    /* 清零结构体 */
    mrac->order = MRAC_ORDER_2ND;
    mrac->rule = MRAC_MIT_RULE;

    /* 参考模型参数 */
    mrac->ref_am1 = 0.0f;
    mrac->ref_am2 = 0.0f;
    mrac->ref_zeta = zeta;
    mrac->ref_wn = wn;

    /* 自适应参数 */
    mrac->theta = 1.0f;
    mrac->gamma = gamma;

    /* 状态清零 */
    mrac->xm1 = 0.0f;
    mrac->xm2 = 0.0f;
    mrac->xm_prev1 = 0.0f;
    mrac->u_prev = 0.0f;

    /* 误差历史 */
    mrac->error = 0.0f;
    mrac->error_integral = 0.0f;

    /* 输出限幅 */
    mrac->u_max = 100.0f;
    mrac->u_min = -100.0f;

    /* 时间 */
    mrac->dt = dt;

    /* 标记初始化完成 */
    mrac->is_initialized = 1;
}

void MRAC_SetGamma(MRAC_t *mrac, float gamma)
{
    if (gamma > 0.0f) {
        mrac->gamma = gamma;
    }
}

void MRAC_SetRule(MRAC_t *mrac, MRAC_Rule_e rule)
{
    mrac->rule = rule;
}

float MRAC_Compute(MRAC_t *mrac, float ref, float y_actual)
{
    float u;

    /* 检查初始化状态 */
    if (!mrac->is_initialized) {
        return 0.0f;
    }

    /* 根据阶数选择更新方式 */
    switch (mrac->order) {
    case MRAC_ORDER_1ST:
        _UpdateFirstOrder(mrac, ref, y_actual);
        break;

    case MRAC_ORDER_2ND:
        _UpdateSecondOrder(mrac, ref, y_actual);
        break;

    default:
        return 0.0f;
    }

    return mrac->u_prev;
}

void MRAC_Reset(MRAC_t *mrac)
{
    mrac->xm1 = 0.0f;
    mrac->xm2 = 0.0f;
    mrac->xm_prev1 = 0.0f;
    mrac->u_prev = 0.0f;
    mrac->error = 0.0f;
    mrac->error_integral = 0.0f;

    /* θ 重置为初始值 */
    mrac->theta = 1.0f;
}

float MRAC_GetError(MRAC_t *mrac)
{
    return mrac->error;
}

float MRAC_GetTheta(MRAC_t *mrac)
{
    return mrac->theta;
}

float MRAC_GetRefModel(MRAC_t *mrac)
{
    return mrac->xm1;
}
