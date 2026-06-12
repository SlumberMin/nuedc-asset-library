/**
 * @file active_disturbance_rejection.h
 * @brief ADRC V2（自抗扰控制器 V2：改进ESO + 非线性组合）
 *
 * 【算法原理】
 * 自抗扰控制器（Active Disturbance Rejection Controller）由韩京清提出，
 * 核心思想是将系统内外所有扰动（包括模型不确定性）视为"总扰动"，
 * 通过扩张状态观测器（ESO）实时估计并补偿。
 *
 * ADRC V2改进点（相比经典ADRC）：
 * 1. 改进ESO：使用变增益/自适应带宽ESO，提升观测精度
 * 2. 非线性组合：使用fhan()（最速离散系统）替代传统非线性组合
 * 3. 跟踪微分器（TD）：对设定值进行平滑过渡，避免阶跃冲击
 * 4. 增加了参数自整定功能
 *
 * 结构：
 *   TD（跟踪微分器）→ 安排过渡过程
 *   ESO（扩张状态观测器）→ 估计状态和总扰动
 *   NLSEF（非线性状态误差反馈）→ 生成控制量并补偿扰动
 *
 * 【适用场景】
 * - 模型完全未知或高度不确定的系统
 * - 强外部扰动环境（风扰、负载突变）
 * - 非线性、时变系统
 * - 电机控制（速度环、位置环）
 * - 倒立摆、平衡车
 * - 四旋翼飞行器姿态控制
 * - 电赛中万能控制器（不知道用什么时选ADRC）
 *
 * 【参数整定指南】
 * 1. ESO参数（最关键）：
 *    - omega_o: ESO带宽，通常取控制器带宽的3~5倍
 *    - 初始值：omega_o = 10 * 采样频率 / (2π)
 *    - 增大omega_o → 观测更快但噪声敏感
 *    - 减小omega_o → 更平滑但响应慢
 *
 * 2. 控制器参数：
 *    - omega_c: 控制器带宽，决定响应速度
 *    - 初始值：omega_c = 采样频率 / (2π)
 *    - 增大omega_c → 响应更快但可能振荡
 *
 * 3. TD参数：
 *    - r: 跟踪速度因子，越大过渡越快
 *    - h: 滤波因子，越大滤波效果越好
 *
 * 4. 快速整定法（推荐）：
 *    a. 设 omega_c = 1/b0 (b0为控制增益估计)
 *    b. 设 omega_o = 3~5 * omega_c
 *    c. 观察效果，逐步调整
 */

#ifndef ACTIVE_DISTURBANCE_REJECTION_H
#define ACTIVE_DISTURBANCE_REJECTION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ADRC V2控制器结构体
 */
typedef struct {
    /* --- 跟踪微分器（TD）状态 --- */
    float td_x1;        /**< TD输出：跟踪信号 */
    float td_x2;        /**< TD输出：跟踪信号的微分 */
    float td_r;         /**< 跟踪速度因子 */
    float td_h;         /**< 滤波因子(通常=采样周期) */

    /* --- 扩张状态观测器（ESO）状态 --- */
    float eso_z1;       /**< ESO估计：系统状态x1 */
    float eso_z2;       /**< ESO估计：系统状态x2（微分） */
    float eso_z3;       /**< ESO估计：总扰动 */
    float eso_omega_o;  /**< ESO带宽(rad/s) */
    float eso_beta1;    /**< ESO增益1（由omega_o计算） */
    float eso_beta2;    /**< ESO增益2 */
    float eso_beta3;    /**< ESO增益3 */
    float eso_b0;       /**< 控制增益估计（最重要的参数） */

    /* --- 非线性组合（NLSEF）参数 --- */
    float nl_omega_c;   /**< 控制器带宽(rad/s) */
    float nl_k1;        /**< 非线性反馈增益1（由omega_c计算） */
    float nl_k2;        /**< 非线性反馈增益2 */
    float nl_alpha1;    /**< fal函数指数1 (0.25~0.5) */
    float nl_alpha2;    /**< fal函数指数2 (0.5~1.0) */
    float nl_delta;     /**< fal函数线性区宽度 */

    /* --- 通用参数 --- */
    float dt;           /**< 采样周期(s) */
    float output_min;   /**< 输出下限 */
    float output_max;   /**< 输出上限 */
    float u0_prev;      /**< 上一次控制输出 */

    /* --- 预计算优化参数 (性能优化版使用) --- */
    float inv_delta_1malpha1; /**< 1.0 / delta^(1-alpha1), fal线性区预计算 */
    float inv_delta_1malpha2; /**< 1.0 / delta^(1-alpha2), fal线性区预计算 */
    float inv_eso_b0;         /**< 1.0 / eso_b0, 用乘法替代除法 */
} ADRC_t;

/**
 * @brief 初始化ADRC控制器
 *
 * @param pid       控制器结构体
 * @param dt        采样周期(s)
 * @param b0        控制增益估计（最关键的参数）
 *                  - 电机：b0 ≈ Kt/J 或 1/(L*s)
 *                  - 不确定时设为1.0，靠ESO补偿
 * @param omega_c   控制器带宽(rad/s)，0则自动选择
 * @param omega_o   ESO带宽(rad/s)，0则自动选择
 */
void ADRC_Init(ADRC_t *adrc, float dt, float b0,
               float omega_c, float omega_o);

/**
 * @brief 计算ADRC输出
 * @param setpoint  目标值
 * @param feedback  当前反馈值
 * @return 控制输出
 */
float ADRC_Compute(ADRC_t *adrc, float setpoint, float feedback);

/**
 * @brief 重置ADRC状态
 */
void ADRC_Reset(ADRC_t *adrc);

/**
 * @brief 设置输出限幅
 */
void ADRC_SetOutputLimits(ADRC_t *adrc, float min_val, float max_val);

/**
 * @brief 设置ESO带宽
 */
void ADRC_SetEsoBandwidth(ADRC_t *adrc, float omega_o);

/**
 * @brief 设置控制器带宽
 */
void ADRC_SetControlBandwidth(ADRC_t *adrc, float omega_c);

/**
 * @brief 设置控制增益估计
 */
void ADRC_SetB0(ADRC_t *adrc, float b0);

/**
 * @brief 获取ESO估计的总扰动（用于监控/调试）
 */
float ADRC_GetDisturbanceEstimate(ADRC_t *adrc);

#ifdef __cplusplus
}
#endif

#endif /* ACTIVE_DISTURBANCE_REJECTION_H */
