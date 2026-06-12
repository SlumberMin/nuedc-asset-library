/**
 * @file fractional_pid.h
 * @brief 分数阶PID控制器（Grünwald-Letnikov离散化）
 *
 * 【算法原理】
 * 分数阶PID（PI^λ D^μ）是经典PID的推广，积分阶次λ和微分阶次μ
 * 不再局限于整数1，可以取任意实数（通常0<λ,μ<2）。
 *
 * 控制律：u(t) = Kp*e(t) + Ki*D^(-λ)*e(t) + Kd*D^(μ)*e(t)
 *
 * 使用Grünwald-Letnikov（GL）定义进行离散化：
 *   D^α f(t) ≈ (1/T)^α * Σ_{k=0}^{n} w_k^(α) * f(t-kT)
 *   其中 w_k^(α) = (-1)^k * C(α,k) = (1-(α+1)/k)*w_{k-1}
 *
 * 优势：
 * - 比整数阶PID多2个可调参数（λ, μ），控制更灵活
 * - 对分数阶系统（如扩散、粘弹性）天然匹配
 * - 等阻尼特性：在宽频率范围内保持一致的相位裕度
 *
 * 【适用场景】
 * - 温度控制（热扩散过程本身是分数阶的）
 * - 液位控制（非整数阶动态特性）
 * - 电池充放电控制（电化学过程分数阶特性）
 * - 需要比传统PID更好性能但不想用复杂MPC的场合
 * - 电赛中追求控制精度的场合
 *
 * 【参数整定指南】
 * 1. 先用传统方法整定Kp, Ki, Kd的初始值
 * 2. 设定λ=1, μ=1，验证基本功能
 * 3. 逐步调整λ（0.5~1.5）：
 *    - λ<1: 积分效果减弱，减少超调
 *    - λ>1: 积分效果增强，消除稳态误差更快
 * 4. 逐步调整μ（0.5~1.5）：
 *    - μ<1: 微分效果减弱，更平滑
 *    - μ>1: 微分效果增强，响应更快但更敏感
 * 5. 推荐起点：λ=0.8, μ=0.9（文献中常用值）
 * 6. GL离散化记忆长度L一般取20~50
 */

#ifndef FRACTIONAL_PID_H
#define FRACTIONAL_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief GL离散化最大记忆长度 */
#define FPID_MEMORY_LENGTH  50

/**
 * @brief 分数阶PID控制器结构体
 */
typedef struct {
    /* PID增益 */
    float Kp;           /**< 比例增益 */
    float Ki;           /**< 积分增益 */
    float Kd;           /**< 微分增益 */

    /* 分数阶次 */
    float lambda;       /**< 积分阶次 (0.0~2.0, 通常0.5~1.5) */
    float mu;           /**< 微分阶次 (0.0~2.0, 通常0.5~1.5) */

    /* GL离散化记忆缓存 */
    float error_buf[FPID_MEMORY_LENGTH];  /**< 误差历史缓存 */
    float gl_weights_i[FPID_MEMORY_LENGTH]; /**< 积分GL系数 */
    float gl_weights_d[FPID_MEMORY_LENGTH]; /**< 微分GL系数 */
    uint16_t buf_idx;   /**< 循环缓冲区索引 */
    uint16_t buf_count; /**< 已填充的数量 */
    uint16_t mem_len;   /**< 实际使用的记忆长度 */

    /* 内部状态 */
    float integral;     /**< 整数积分项（用于混合模式） */
    float prev_error;
    float dt;

    /* 输出限幅 */
    float output_min;
    float output_max;
    float integral_max;
} FracPID_t;

/**
 * @brief 初始化分数阶PID控制器
 *
 * @param pid       控制器结构体
 * @param dt        采样周期(s)
 * @param Kp        比例增益
 * @param Ki        积分增益
 * @param Kd        微分增益
 * @param lambda_   积分阶次 (推荐0.5~1.5)
 * @param mu        微分阶次 (推荐0.5~1.5)
 * @param mem_len   GL记忆长度 (推荐20~50)
 */
void FracPID_Init(FracPID_t *pid, float dt,
                  float Kp, float Ki, float Kd,
                  float lambda_, float mu, uint16_t mem_len);

/**
 * @brief 计算分数阶PID输出
 */
float FracPID_Compute(FracPID_t *pid, float setpoint, float feedback);

/**
 * @brief 重置控制器
 */
void FracPID_Reset(FracPID_t *pid);

/**
 * @brief 设置输出限幅
 */
void FracPID_SetOutputLimits(FracPID_t *pid, float min_val, float max_val);

/**
 * @brief 在线调整分数阶次
 */
void FracPID_SetOrders(FracPID_t *pid, float lambda_, float mu);

/**
 * @brief 设置PID增益
 */
void FracPID_SetGains(FracPID_t *pid, float Kp, float Ki, float Kd);

#ifdef __cplusplus
}
#endif

#endif /* FRACTIONAL_PID_H */
