/**
 * @file repetitive_control.h
 * @brief 重复控制器 - 周期性扰动抑制
 * 
 * 重复控制(Repetitive Control)基于内模原理，将周期信号发生器嵌入
 * 控制回路中，可实现对周期性参考信号的零稳态误差跟踪或周期性
 * 扰动的完全抑制。
 * 
 * 典型应用：逆变器输出波形控制、电机转矩脉动抑制、
 *           周期性振动抑制等。
 * 
 * 算法原理：
 *   u(k) = Q(z) * u(k-N) + K_r * e(k-N)
 * 
 *   N  = 每周期采样点数 = fs / f_disturbance
 *   Q(z) = 低通滤波器（增强鲁棒性，通常取0.95~0.98）
 *   K_r = 重复控制增益
 * 
 * 参数整定指南：
 *   - N：由采样频率和扰动基频决定，fs/f0
 *   - K_r：从0.1开始逐步增大，观察收敛速度与稳定性
 *   - Q值：0.95~0.98，越大跟踪精度越高但鲁棒性越差
 *   - 可串联超前补偿器改善相位滞后
 */

#ifndef REPETITIVE_CONTROL_H
#define REPETITIVE_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大支持每周期512个采样点 */
#define RC_MAX_PERIOD_SAMPLES  512

typedef struct {
    /* --- 配置参数 --- */
    float Ts;               /* 采样周期(s) */
    float f0;               /* 扰动/参考基频(Hz) */
    float Kr;               /* 重复控制增益 */
    float Q;                /* 滤波器系数(0~1)，通常0.95~0.98 */
    
    /* --- 运行时变量 --- */
    int32_t N;              /* 每周期采样点数 */
    int32_t idx;            /* 当前环形缓冲区索引 */
    float delay_buf[RC_MAX_PERIOD_SAMPLES];  /* 控制量延迟缓冲区 u(k-N) */
    float err_buf[RC_MAX_PERIOD_SAMPLES];   /* 误差延迟缓冲区 e(k-N) */
    float u_prev;           /* 上一时刻控制量 */
    float err_prev;         /* 上一时刻误差 */
} RepetitiveCtrl_t;

/**
 * @brief 初始化重复控制器
 * @param rc       控制器句柄
 * @param Ts       采样周期(s)
 * @param f0       扰动基频(Hz)
 * @param Kr       重复增益
 * @param Q        滤波系数
 * @return 0=成功, -1=参数错误
 */
int RC_Init(RepetitiveCtrl_t *rc, float Ts, float f0, float Kr, float Q);

/**
 * @brief 重复控制计算
 * @param rc   控制器句柄
 * @param ref  参考值
 * @param fbk  反馈值
 * @return 控制输出量
 */
float RC_Compute(RepetitiveCtrl_t *rc, float ref, float fbk);

/**
 * @brief 重置控制器状态
 */
void RC_Reset(RepetitiveCtrl_t *rc);

#ifdef __cplusplus
}
#endif

#endif /* REPETITIVE_CONTROL_H */
