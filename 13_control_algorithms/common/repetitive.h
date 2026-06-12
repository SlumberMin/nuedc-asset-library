#ifndef __REPETITIVE_H
#define __REPETITIVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 重复控制器 (Repetitive Control)
 * @note  基于内模原理, 在反馈回路中嵌入周期延迟环节 e^(-sT),
 *        可消除周期性扰动。适用于逆变器、UPS、电机谐波抑制等。
 *
 *        Q(z) * z^(-N) 是核心, Q(z)通常取0.95~0.99的低通滤波器
 *        控制律: u[k] = u_f[k] + u_r[k]
 *        u_r[k] = Q * u_r[k-N] + K_r * e[k-N+d]
 *        d为超前补偿步数
 */

#define REP_MAX_PERIOD_SAMPLES  512  /* 最大周期缓冲长度 */

typedef struct {
    /* 周期延迟缓冲 */
    float buffer[REP_MAX_PERIOD_SAMPLES];
    uint16_t period_samples;   /* 一个周期的采样点数 N */
    uint16_t index;

    /* 重复控制增益 */
    float Kr;

    /* Q滤波器系数 (0~1), 越接近1稳态精度越高但鲁棒性越差 */
    float Q;

    /* 超前补偿步数 */
    uint16_t lead_steps;

    /* 基础控制器(如PI)输出, 作为前馈 */
    float base_output;

    /* 输出限幅 */
    float out_min;
    float out_max;

    float output;
} RepetitiveCtrl_t;

/**
 * @brief 初始化重复控制器
 * @param period_samples 一个基波周期对应的采样点数
 * @param Kr  重复控制增益
 * @param Q   Q滤波器系数 (推荐0.95~0.99)
 * @param lead_steps 超前补偿步数 (一般为1~3)
 */
void Repetitive_Init(RepetitiveCtrl_t *ctrl,
                     uint16_t period_samples,
                     float Kr, float Q,
                     uint16_t lead_steps);

/**
 * @brief 设置输出限幅
 */
void Repetitive_SetOutputLimit(RepetitiveCtrl_t *ctrl, float min, float max);

/**
 * @brief 设置基础控制器输出(如PI的输出)
 */
void Repetitive_SetBaseOutput(RepetitiveCtrl_t *ctrl, float base);

/**
 * @brief 计算重复控制输出
 * @param error 误差信号 (setpoint - feedback)
 * @return 总控制量 = 基础输出 + 重复控制补偿
 */
float Repetitive_Compute(RepetitiveCtrl_t *ctrl, float error);

/**
 * @brief 重置
 */
void Repetitive_Reset(RepetitiveCtrl_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* __REPETITIVE_H */
