/**
 * @file super_twisting_smc.h
 * @brief 超螺旋滑模控制器 - 二阶滑模, 消除抖振
 * @version 1.0
 * @date 2026-06-11
 *
 * Super-Twisting Algorithm (STA) 是二阶滑模控制的代表算法:
 *   - 在有限时间内收敛到滑模面
 *   - 控制量连续, 消除传统SMC的抖振问题
 *   - 不需要导数信息 (仅使用滑模面s)
 *
 * 标准形式:
 *   u = u1 + u2
 *   du1/dt = -lambda * |s|^0.5 * sign(s)
 *   du2/dt = -alpha * sign(s)
 *
 * 参数要求:
 *   lambda > 0 (影响收敛速度)
 *   alpha > lambda (保证稳定性)
 *
 * 收敛时间估计:
 *   T ≤ 2 * sqrt(|s(0)|) / lambda
 *
 * 推荐应用:
 *   - 需要滑模鲁棒性但不能容忍抖振的场合
 *   - 电机控制、位置伺服
 *   - 机器人关节控制
 */

#ifndef __SUPER_TWISTING_SMC_H
#define __SUPER_TWISTING_SMC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 超螺旋算法参数 ========== */
typedef struct {
    /* 滑模面参数 */
    float c;                   /* 滑模面斜率: s = e_dot + c*e */
    float s;                   /* 当前滑模面值 */

    /* 超螺旋算法参数 */
    float lambda;              /* 收敛速度参数 (lambda > 0) */
    float alpha;               /* 稳定性参数 (alpha > lambda) */

    /* 内部积分状态 */
    float u1;                  /* 积分项1: du1/dt = -lambda * |s|^0.5 * sign(s) */
    float u2;                  /* 积分项2: du2/dt = -alpha * sign(s) */

    /* 误差状态 */
    float error;               /* 位置误差: e = r - y */
    float error_dot;           /* 速度误差: e_dot = 0 - y_dot */

    /* 控制输出 */
    float output;              /* 总控制输出: u = u1 + u2 */

    /* 输出限幅 */
    float output_max;
    float output_min;

    /* 内部辅助参数 */
    float sqrt_lambda;         /* sqrt(lambda), 预计算 */
    float inv_lambda;          /* 1/lambda, 预计算 */
} SuperTwistingSMC_t;

/* ========== 初始化接口 ========== */

/**
 * @brief 初始化超螺旋滑模控制器
 * @param smc 超螺旋SMC结构体指针
 * @param c 滑模面斜率 (推荐: 5~20)
 * @param lambda 收敛速度参数 (推荐: 1~10)
 * @param alpha 稳定性参数 (推荐: 2~20, 必须>lambda)
 */
void SuperTwisting_Init(SuperTwistingSMC_t *smc, float c, float lambda, float alpha);

/**
 * @brief 设置滑模面参数
 * @param smc 超螺旋SMC结构体指针
 * @param c 滑模面斜率
 */
void SuperTwisting_SetSlidingSurface(SuperTwistingSMC_t *smc, float c);

/**
 * @brief 设置超螺旋算法参数
 * @param smc 超螺旋SMC结构体指针
 * @param lambda 收敛速度参数 (推荐: 1~10)
 * @param alpha 稳定性参数 (推荐: 2~20, 必须>lambda)
 *
 * 参数整定原则:
 *   1. lambda 越大 → 收敛越快, 但控制量越大
 *   2. alpha 越大 → 越稳定, 但响应变慢
 *   3. alpha > lambda 是稳定性必要条件
 *   4. 推荐 alpha ≈ 2~3 * lambda
 */
void SuperTwisting_SetParameters(SuperTwistingSMC_t *smc, float lambda, float alpha);

/**
 * @brief 设置输出限幅
 * @param smc 超螺旋SMC结构体指针
 * @param min 输出下限
 * @param max 输出上限
 */
void SuperTwisting_SetOutputLimit(SuperTwistingSMC_t *smc, float min, float max);

/* ========== 计算接口 ========== */

/**
 * @brief 超螺旋SMC计算 (核心)
 * @param smc 超螺旋SMC结构体指针
 * @param r 参考输入 (设定值)
 * @param y 测量输出 (反馈值)
 * @param y_dot 测量速度 (导数, 可选)
 * @param dt 采样步长 (s)
 * @return 控制输出u
 *
 * 算法步骤:
 *   1. 计算误差 e = r - y, e_dot = 0 - y_dot
 *   2. 计算滑模面 s = e_dot + c*e
 *   3. 更新积分:
 *      u1 += -lambda * |s|^0.5 * sign(s) * dt
 *      u2 += -alpha * sign(s) * dt
 *   4. 输出 u = u1 + u2
 */
float SuperTwisting_Calculate(SuperTwistingSMC_t *smc, float r, float y,
                              float y_dot, float dt);

/**
 * @brief 超螺旋SMC计算 (简化版, 仅用位置信息)
 * @param smc 超螺旋SMC结构体指针
 * @param r 参考输入
 * @param y 测量输出
 * @param dt 采样步长 (s)
 * @return 控制输出u
 *
 * 说明: 速度通过差分近似计算
 */
float SuperTwisting_CalculateSimple(SuperTwistingSMC_t *smc, float r, float y, float dt);

/**
 * @brief 重置超螺旋SMC状态
 * @param smc 超螺旋SMC结构体指针
 */
void SuperTwisting_Reset(SuperTwistingSMC_t *smc);

/* ========== 状态获取 ========== */

/**
 * @brief 获取当前滑模面值s
 */
static inline float SuperTwisting_GetSlidingSurface(SuperTwistingSMC_t *smc) { return smc->s; }

/**
 * @brief 获取控制误差e
 */
static inline float SuperTwisting_GetError(SuperTwistingSMC_t *smc) { return smc->error; }

/**
 * @brief 获取收敛时间估计
 * @param smc 超螺旋SMC结构体指针
 * @return 预估收敛时间 (s)
 *
 * 公式: T ≤ 2 * sqrt(|s(0)|) / lambda
 */
static inline float SuperTwisting_EstimateConvergeTime(SuperTwistingSMC_t *smc)
{
    return 2.0f * sqrtf(fabsf(smc->s)) / smc->lambda;
}

/* ========== 与传统SMC兼容接口 ========== */

/**
 * @brief 获取传统SMC格式的控制律 (用于对比)
 * @param smc 超螺旋SMC结构体指针
 * @return 传统SMC控制量 (不推荐使用, 仅用于对比)
 */
static inline float SuperTwisting_GetLegacySMC(SuperTwistingSMC_t *smc)
{
    /* 传统SMC: u = -k*sign(s) */
    float sign_s = (smc->s > 0.0f) ? 1.0f : ((smc->s < 0.0f) ? -1.0f : 0.0f);
    return -smc->alpha * sign_s;
}

#ifdef __cplusplus
}
#endif

#endif /* __SUPER_TWISTING_SMC_H */
