/**
 * @file predictive_functional.h
 * @brief 预测函数控制（PFC: Predictive Functional Control）
 *
 * 【算法原理】
 * 预测函数控制（PFC）是模型预测控制（MPC）家族中最简单实用的
 * 成员，由Richalet等人在1980年代提出。其核心特点是：
 *
 * 1. 基函数假设：控制量被限制为若干预定义基函数的加权组合
 *    u(k+i) = Σ μ_n * B_n(i)
 *    常用基函数：阶跃函数、斜坡函数、指数函数
 *
 * 2. 模型预测：使用被控对象模型预测未来输出
 *    ŷ(k+i) = y_model(k+i) + y(k) - y_model(k)
 *    （反馈校正：用当前误差修正预测）
 *
 * 3. 滚动优化：在预测时域内最小化参考轨迹与预测输出的偏差
 *    min J = Σ [y_r(k+i) - ŷ(k+i)]²
 *
 * 4. 参考轨迹：目标值沿一阶指数轨迹趋近
 *    y_r(k+i) = y_sp - (y_sp - y(k)) * α^i
 *    其中 α = exp(-Ts/Tr)，Tr为期望响应时间
 *
 * PFC相比传统MPC的优势：
 * - 计算量极小（解析解，不需要QP求解器）
 * - 只需少量参数
 * - 天然抗积分饱和
 * - 适合嵌入式实时控制
 *
 * 【适用场景】
 * - 一阶/二阶加纯滞后过程（温度、液位、流量控制）
 * - 需要预测控制但计算资源有限（MCU/单片机）
 * - 电赛中需要展示先进控制算法
 * - 工业过程控制（化工、食品、制药）
 * - 对超调量有严格限制的场合
 *
 * 【参数整定指南】
 * 1. 模型参数（必须）：
 *    - model_K: 稳态增益（可通过阶跃响应辨识）
 *    - model_T: 时间常数（阶跃响应63.2%处的时间）
 *    - model_theta: 纯滞后时间
 *
 * 2. 参考轨迹时间常数 Tr：
 *    - 决定响应速度
 *    - Tr小 → 响应快但可能超调
 *    - Tr大 → 响应慢但平滑
 *    - 推荐：Tr = (0.3~1.0) * T
 *
 * 3. 预测时域 P：
 *    - 应覆盖主要动态响应
 if (dt <= 0.0f) dt = 0.001f;  /* 防止除零 */
 *    - 推荐：P ≈ T/dt（即一个时间常数内的采样数）
 *
 * 4. 基函数选择：
 *    - 一阶过程：阶跃基函数（最简单）
 *    - 二阶过程：阶跃+斜坡基函数
 */

#ifndef PREDICTIVE_FUNCTIONAL_H
#define PREDICTIVE_FUNCTIONAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 最大预测时域长度 */
#define PFC_PREDICTION_HORIZON  50

/** @brief 基函数类型 */
typedef enum {
    PFC_BASIS_STEP = 0,    /**< 阶跃基函数（最简单，推荐起步） */
    PFC_BASIS_RAMP,        /**< 阶跃+斜坡基函数 */
    PFC_BASIS_EXPONENTIAL  /**< 阶跃+指数基函数 */
} PFC_BasisType_t;

/**
 * @brief PFC控制器结构体
 */
typedef struct {
    /* 模型参数 */
    float model_K;      /**< 模型增益 */
    float model_T;      /**< 时间常数(s) */
    float model_theta;  /**< 纯滞后(s) */

    /* 模型状态（一阶离散化） */
    float model_state;  /**< 模型内部状态 */
    float model_a;      /**< 离散化系数 a = exp(-dt/T) */
    float model_b;      /**< 离散化系数 b = K*(1-exp(-dt/T)) */

    /* PFC参数 */
    float alpha;        /**< 参考轨迹衰减系数 α = exp(-dt/Tr) */
    float Tr;           /**< 参考轨迹时间常数(s) */
    uint16_t P;         /**< 预测时域 */
    uint16_t dead_steps; /**< 纯滞后对应的采样步数 */
    PFC_BasisType_t basis_type; /**< 基函数类型 */

    /* 内部状态 */
    float prev_u;       /**< 上一次控制量 */
    float prev_y;       /**< 上一次测量值 */
    float y_sp;         /**< 当前设定值 */
    float y_ref[PFC_PREDICTION_HORIZON]; /**< 参考轨迹 */
    float dt;           /**< 采样周期(s) */

    /* 输出限幅 */
    float output_min;
    float output_max;

    /* 模型预测缓存 */
    float y_free[PFC_PREDICTION_HORIZON]; /**< 自由响应预测 */
} PFC_t;

/**
 * @brief 初始化PFC控制器
 *
 * @param pfc       控制器结构体
 * @param dt        采样周期(s)
 * @param model_K   模型增益
 * @param model_T   时间常数(s)
 * @param model_theta 纯滞后(s)
 * @param Tr        参考轨迹时间常数(s)，0则自动选择
 * @param P         预测时域，0则自动选择
 */
void PFC_Init(PFC_t *pfc, float dt,
              float model_K, float model_T, float model_theta,
              float Tr, uint16_t P);

/**
 * @brief 计算PFC输出
 * @param setpoint  目标值
 * @param feedback  反馈值
 * @return 控制输出
 */
float PFC_Compute(PFC_t *pfc, float setpoint, float feedback);

/**
 * @brief 重置控制器
 */
void PFC_Reset(PFC_t *pfc);

/**
 * @brief 设置输出限幅
 */
void PFC_SetOutputLimits(PFC_t *pfc, float min_val, float max_val);

/**
 * @brief 设置参考轨迹时间常数
 */
void PFC_SetTr(PFC_t *pfc, float Tr);

/**
 * @brief 设置基函数类型
 */
void PFC_SetBasisType(PFC_t *pfc, PFC_BasisType_t type);

#ifdef __cplusplus
}
#endif

#endif /* PREDICTIVE_FUNCTIONAL_H */
