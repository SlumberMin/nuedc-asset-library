/**
 * @file model_free_adaptive.h
 * @brief 无模型自适应控制（MFAC: Model-Free Adaptive Control）
 *
 * 【算法原理】
 * 无模型自适应控制（MFAC）由侯忠生教授提出，核心是"动态线性化"
 * 方法。它不需要被控对象的数学模型，仅利用系统的I/O数据，
 * 通过在线估计"伪偏导数（PPD）"实现自适应控制。
 *
 * 基本思想：
 * 1. 将非线性系统在每个工作点等价为一个时变的动态线性系统
 * 2. 伪偏导数（PPD）φ(k) 代替传统模型参数
 * 3. 使用紧格式动态线性化（CFDL）：
 *    Δy(k+1) = φ(k) * Δu(k)
 * 4. 通过带遗忘因子的最小二乘法在线估计φ(k)
 * 5. 基于φ(k)的估计值计算控制增量
 *
 * MFAC的优势：
 * - 完全不需要数学模型（黑箱控制）
 * - 仅需一个可调参数（学习率λ）
 * - 对非线性、时变系统自适应
 * - 计算量小，适合嵌入式
 *
 * 【适用场景】
 * - 完全未知模型的系统
 * - 强非线性、时变系统
 * - 电赛中快速开发（不需要建模）
 * - 传感器数据驱动的控制场景
 * - 系统特性会随时间/环境变化的场合
 *
 * 【参数整定指南】
 * 1. η (学习率): 最关键参数
 *    - 推荐初始值：0.1~0.5
 *    - 增大→响应快但可能振荡
 *    - 减小→更稳定但响应慢
 *
 * 2. μ (权重因子): 控制量变化的惩罚
 *    - 推荐初始值：0.01~1.0
 *    - 增大→控制量变化更平滑
 *
 * 3. ρ (学习率衰减): 防止估计发散
 *    - 推荐值：0.5~1.0
 *
 * 4. λ_f (遗忘因子): 影响估计的跟踪能力
 *    - 推荐值：0.95~0.999
 *    - 越小跟踪越快但越不稳定
 *
 * 5. φ(0) (PPD初始值): 一般设为1.0
 */

#ifndef MODEL_FREE_ADAPTIVE_H
#define MODEL_FREE_ADAPTIVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 无模型自适应控制器结构体
 */
typedef struct {
    /* 控制参数 */
    float eta;          /**< 学习率/步长因子 (0.1~1.0) */
    float mu;           /**< 控制量权重因子 (0~1) */
    float rho;          /**< 学习率衰减因子 (0~1) */
    float lambda_f;     /**< 遗忘因子 (0.95~0.999) */
    float phi;          /**< 伪偏导数（PPD）估计值 */

    /* PPD估计相关 */
    float phi_init;     /**< PPD初始值 */
    float phi_min;      /**< PPD下限（防止符号反转） */
    float phi_max;      /**< PPD上限 */
    float P;            /**< 最小二乘法协方差 */

    /* 状态缓存 */
    float prev_u;       /**< 上一次控制量 u(k-1) */
    float prev_y;       /**< 上一次输出 y(k-1) */
    float prev_dy;      /**< 上一次输出增量 Δy(k-1) */
    float prev_du;      /**< 上一次控制增量 Δu(k-1) */
    float prev2_u;      /**< u(k-2) */
    float prev2_y;      /**< y(k-2) */
    uint8_t init_flag;  /**< 初始化标志 */

    /* 输出限幅 */
    float output_min;
    float output_max;
    float delta_u_max;  /**< 控制增量限幅 */

    /* 采样参数 */
    float dt;

    /* 调试信息 */
    float debug_phi;    /**< 当前PPD估计值（只读） */
} MFAC_t;

/**
 * @brief 初始化无模型自适应控制器
 *
 * @param pid       控制器结构体
 * @param dt        采样周期(s)
 */
void MFAC_Init(MFAC_t *mfac, float dt);

/**
 * @brief 计算MFAC输出
 * @param setpoint  目标值
 * @param feedback  反馈值
 * @return 控制输出
 */
float MFAC_Compute(MFAC_t *mfac, float setpoint, float feedback);

/**
 * @brief 重置控制器
 */
void MFAC_Reset(MFAC_t *mfac);

/**
 * @brief 设置控制参数
 */
void MFAC_SetParams(MFAC_t *mfac, float eta, float mu, float rho);

/**
 * @brief 设置输出限幅
 */
void MFAC_SetOutputLimits(MFAC_t *mfac, float min_val, float max_val);

/**
 * @brief 获取当前PPD估计值
 */
float MFAC_GetPPD(MFAC_t *mfac);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_FREE_ADAPTIVE_H */
