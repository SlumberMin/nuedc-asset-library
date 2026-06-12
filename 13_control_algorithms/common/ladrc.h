/**
 * @file ladrc.h
 * @brief 线性自抗扰控制器 (Linear Active Disturbance Rejection Control, LADRC)
 *
 * LADRC 是韩京清研究员提出的 ADRC 的简化线性版本，由高志强教授推广。
 * 核心思想：将内部扰动和外部扰动统一估计并补偿，实现"自抗扰"。
 *
 * 三个核心组件：
 * 1. 跟踪微分器 (TD) - 安排过渡过程，避免阶跃突变
 * 2. 线性扩张状态观测器 (LESO) - 估计系统状态和总扰动
 * 3. 线性状态误差反馈 (LSEF) - PD控制器 + 扰动补偿
 *
 * 参数整定指南（带宽法）：
 * ==========================
 * LADRC 只需整定 3 个参数：
 *   ωc - 控制器带宽，决定响应速度，一般取系统期望闭环带宽
 *   ωo - 观测器带宽，一般取 ωo = (2~5) × ωc
 *   b0 - 系统增益估计值，可通过系统辨识或试凑获得
 *
 * 整定步骤：
 * 1. 先确定 b0：令 u 产生阶跃，观察输出变化率，b0 ≈ Δy/Δu
 * 2. 设定 ωc：从较小值开始，逐步增大直到满意响应速度
 * 3. 设定 ωo：通常取 ωo = 3 × ωc，观测器越快估计越准
 * 4. 若振荡则减小 ωc，若响应慢则增大 ωc
 *
 * 适用场景：
 * - 模型不确定的系统
 * - 存在外部扰动的场合
 * - 替代传统PID的高性能场合
 */

#ifndef LADRC_H
#define LADRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 二阶 LADRC 结构体（适用于大多数工程系统） */
typedef struct {
    /* ========== 控制器参数 ========== */
    float wc;           /* 控制器带宽 ωc (rad/s)，决定响应速度 */
    float wo;           /* 观测器带宽 ωo (rad/s)，决定估计精度 */
    float b0;           /* 系统增益估计值（输入到输出的增益） */

    /* ========== LESO 状态变量（三阶观测器） ========== */
    float z1;           /* 状态估计：位置/输出 */
    float z2;           /* 状态估计：速度/输出微分 */
    float z3;           /* 扩张状态估计：总扰动 */

    /* ========== TD 跟踪微分器 ========== */
    float v1;           /* 跟踪信号 */
    float v2;           /* 跟踪信号微分 */
    float h0;           /* TD 滤波因子 */
    float r0;           /* TD 速度因子 */

    /* ========== 输出限幅 ========== */
    float u_max;        /* 输出上限 */
    float u_min;        /* 输出下限 */

    /* ========== 内部参数（自动计算） ========== */
    float dt;           /* 采样周期 (s) */
    float Kp;           /* 比例增益 = ωc² */
    float Kd;           /* 微分增益 = 2ωc */
    float beta1;        /* LESO 增益 1 = 3ωo */
    float beta2;        /* LESO 增益 2 = 3ωo² */
    float beta3;        /* LESO 增益 3 = ωo³ */

    /* ========== 内部变量 ========== */
    float u0;           /* LSEF 输出（补偿前） */
    float u;            /* 最终控制输出 */
    float last_output;  /* 上一次输出（用于数值微分） */
} LADRC_t;

/**
 * @brief 初始化 LADRC 控制器
 * @param ladrc    控制器指针
 * @param wc       控制器带宽 (rad/s)
 * @param wo       观测器带宽 (rad/s)
 * @param b0       系统增益估计值
 * @param dt       采样周期 (s)
 * @param u_max    输出上限
 * @param u_min    输出下限
 */
void LADRC_Init(LADRC_t *ladrc, float wc, float wo, float b0,
                float dt, float u_max, float u_min);

/**
 * @brief 设置 TD 跟踪微分器参数
 * @param ladrc    控制器指针
 * @param r0       速度因子（越大跟踪越快，但噪声越大）
 * @param h0       滤波因子（越大滤波效果越好）
 */
void LADRC_SetTD(LADRC_t *ladrc, float r0, float h0);

/**
 * @brief LADRC 控制器计算（一个采样周期调用一次）
 * @param ladrc    控制器指针
 * @param setpoint  设定值
 * @param feedback  反馈值（传感器测量值）
 * @return 控制输出
 */
float LADRC_Update(LADRC_t *ladrc, float setpoint, float feedback);

/**
 * @brief 重置控制器状态
 */
void LADRC_Reset(LADRC_t *ladrc);

/**
 * @brief 跟踪微分器（离散最速系统）
 */
static void LADRC_TD_Update(LADRC_t *ladrc, float setpoint);

/**
 * @brief 线性扩张状态观测器 (LESO)
 */
static void LADRC_LESO_Update(LADRC_t *ladrc, float y, float u);

/**
 * @brief 线性状态误差反馈 + 扰动补偿
 */
static float LADRC_LSEF(LADRC_t *ladrc);

#ifdef __cplusplus
}
#endif

#endif /* LADRC_H */
