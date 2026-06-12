/**
 * @file mrac.h
 * @brief 模型参考自适应控制（Model Reference Adaptive Control）
 * @version 1.0
 * @date 2026-06-11
 *
 * 使用MIT规则实现自适应控制：
 *   参考模型: dxm/dt = -am*xm + am*r (一阶) 或 二阶
 *   自适应律: dθ/dt = -γ*e*xm
 *   控制律:   u = θ*r
 *
 * 特点：
 *   - 无需精确系统模型
 *   - 实时自适应调整控制器参数
 *   - 稳定性和收敛性有理论保证
 *   - 嵌入式友好：无动态内存，纯浮点运算
 *
 * 适用于：
 *   - 参数不确定系统
 *   - 模型不确定系统
 *   - 时变系统控制
 */

#ifndef __MRAC_H
#define __MRAC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MRAC阶数选择 */
typedef enum {
    MRAC_ORDER_1ST = 1,    /* 一阶系统 */
    MRAC_ORDER_2ND = 2     /* 二阶系统 */
} MRAC_Order_e;

/* MRAC控制模式 */
typedef enum {
    MRAC_MIT_RULE = 0,     /* MIT规则: dθ/dt = -γ*e*xm */
    MRAC_STRONG_RULE       /* 强调规则: dθ/dt = -γ*e*sign(xm) */
} MRAC_Rule_e;

/* MRAC结构体 */
typedef struct {
    MRAC_Order_e order;           /* 系统阶数 */
    MRAC_Rule_e rule;             /* 自适应规则 */

    /* 参考模型参数 */
    float ref_am1;                /* 参考模型极点（一阶）*/
    float ref_am2;                /* 参考模型二阶极点（二阶）*/
    float ref_zeta;               /* 参考模型阻尼比（二阶）*/
    float ref_wn;                 /* 参考模型自然频率（二阶）*/

    /* 自适应参数 */
    float theta;                  /* 自适应参数（可调增益）*/
    float gamma;                  /* 自适应增益 */

    /* 参考模型状态 */
    float xm1;                    /* 参考模型状态1 */
    float xm2;                    /* 参考模型状态2 */
    float xm_prev1;               /* 参考模型前一状态1 */

    /* 控制器状态 */
    float u_prev;                 /* 前一时刻输出 */
    float u_max;                  /* 输出上限 */
    float u_min;                  /* 输出下限 */

    /* 误差和历史 */
    float error;                  /* 跟踪误差 */
    float error_integral;         /* 误差积分 */

    /* 时间相关 */
    float dt;                     /* 采样时间 */

    /* 初始化标记 */
    uint8_t is_initialized;       /* 初始化状态 */
} MRAC_t;

/**
 * @brief 初始化MRAC控制器（一阶）
 *
 * @param mrac     控制器结构体
 * @param am       参考模型时间常数 (am > 0)
 * @param gamma    自适应增益 (γ > 0)
 * @param dt       采样时间 (s)
 */
void MRAC_InitFirstOrder(MRAC_t *mrac, float am, float gamma, float dt);

/**
 * @brief 初始化MRAC控制器（二阶）
 *
 * @param mrac     控制器结构体
 * @param wn       参考模型自然频率 (rad/s)
 * @param zeta     参考模型阻尼比 (0 < ζ < 1)
 * @param gamma    自适应增益 (γ > 0)
 * @param dt       采样时间 (s)
 */
void MRAC_InitSecondOrder(MRAC_t *mrac, float wn, float zeta, float gamma, float dt);

/**
 * @brief 设置自适应增益γ
 *
 * @param mrac  控制器结构体
 * @param gamma 新的自适应增益
 */
void MRAC_SetGamma(MRAC_t *mrac, float gamma);

/**
 * @brief 设置自适应规则
 *
 * @param mrac  控制器结构体
 * @param rule  新的自适应规则
 */
void MRAC_SetRule(MRAC_t *mrac, MRAC_Rule_e rule);

/**
 * @brief MRAC控制计算
 *
 * @param mrac     控制器结构体
 * @param ref      参考输入（期望轨迹）
 * @param y_actual 实际系统输出
 * @return 控制输出 u = θ * ref
 */
float MRAC_Compute(MRAC_t *mrac, float ref, float y_actual);

/**
 * @brief 复位MRAC控制器状态
 *
 * @param mrac 控制器结构体
 */
void MRAC_Reset(MRAC_t *mrac);

/**
 * @brief 获取当前跟踪误差
 *
 * @param mrac 控制器结构体
 * @return 当前误差值
 */
float MRAC_GetError(MRAC_t *mrac);

/**
 * @brief 获取自适应参数θ（即等效增益）
 *
 * @param mrac 控制器结构体
 * @return 当前自适应参数
 */
float MRAC_GetTheta(MRAC_t *mrac);

/**
 * @brief 获取参考模型输出
 *
 * @param mrac 控制器结构体
 * @return 当前参考模型状态
 */
float MRAC_GetRefModel(MRAC_t *mrac);

#ifdef __cplusplus
}
#endif

#endif /* __MRAC_H */
