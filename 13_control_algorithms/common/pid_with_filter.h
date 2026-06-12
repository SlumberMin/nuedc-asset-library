/**
 * @file pid_with_filter.h
 * @brief PID+滤波器 - 微分低通滤波 + 输出低通滤波
 *
 * 解决问题:
 *   1. 微分项对高频噪声敏感 → 一阶低通滤波微分信号
 *   2. 输出抖动/毛刺 → 输出端一阶低通平滑
 *   3. 设定值微分突变 → 可选"误差微分"或"PV微分(仅对测量值微分)"
 *
 * 滤波器结构:
 *   D项:  raw_d = kd * (error - prev_error) / dt
 *         filtered_d = alpha_d * raw_d + (1-alpha_d) * prev_filtered_d
 *   输出: raw_out = P + filtered_D + I
 *         filtered_out = alpha_out * raw_out + (1-alpha_out) * prev_filtered_out
 *
 * 适用场景:
 *   - 传感器噪声大的场合(编码器、陀螺仪、ADC)
 *   - 需要平滑输出的舵机/PWM控制
 *   - 电流环、速度环等对微分噪声敏感的内环
 */

#ifndef PID_WITH_FILTER_H
#define PID_WITH_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 滤波模式选择
 */
typedef enum {
    PID_FILTER_D_TERM    = (1 << 0),    /* 仅微分项滤波 */
    PID_FILTER_OUTPUT    = (1 << 1),    /* 仅输出滤波 */
    PID_FILTER_BOTH      = (1 << 0) | (1 << 1)  /* 两者都启用 */
} pid_filter_mode_t;

/**
 * @brief 微分信号来源
 */
typedef enum {
    PID_D_FROM_ERROR = 0,    /* 对误差微分 (默认, 设定值变化时D项有冲击) */
    PID_D_FROM_PV            /* 对PV(测量值)微分 (推荐, 设定值变化更平滑) */
} pid_d_source_t;

/**
 * @brief PID+滤波器控制器句柄
 */
typedef struct {
    /* PID增益 */
    float kp;
    float ki;
    float kd;

    /* 设定值 */
    float setpoint;

    /* 内部状态 */
    float integral;
    float prev_error;
    float prev_pv;            /* 上一次PV值(PV微分模式用) */
    float prev_d_filtered;    /* 微分项滤波后值 */
    float prev_out_filtered;  /* 输出滤波后值 */
    uint8_t first_run;

    /* 输出限幅 */
    float out_min;
    float out_max;

    /* 积分限幅 */
    float integral_min;
    float integral_max;

    /* 滤波参数 (0~1, 越大越信任新值, 1=不滤波) */
    float d_filter_alpha;     /* 微分项低通滤波系数 */
    float out_filter_alpha;   /* 输出低通滤波系数 */

    /* 采样时间 (秒) */
    float dt;

    /* 配置 */
    pid_filter_mode_t filter_mode;
    pid_d_source_t d_source;
} pid_with_filter_t;

/**
 * @brief 初始化PID+滤波器
 * @param pid       控制器句柄
 * @param kp        比例增益
 * @param ki        积分增益
 * @param kd        微分增益
 * @param dt        采样周期(秒)
 * @param out_min   输出下限
 * @param out_max   输出上限
 */
void pid_wf_init(pid_with_filter_t *pid,
                 float kp, float ki, float kd,
                 float dt, float out_min, float out_max);

/**
 * @brief 设置滤波模式
 * @param pid    控制器句柄
 * @param mode   滤波模式 (PID_FILTER_D_TERM / PID_FILTER_OUTPUT / PID_FILTER_BOTH)
 */
void pid_wf_set_filter_mode(pid_with_filter_t *pid, pid_filter_mode_t mode);

/**
 * @brief 设置微分滤波系数
 * @param pid    控制器句柄
 * @param alpha  滤波系数 (0~1, 越大越不滤波, 推荐0.1~0.3)
 */
void pid_wf_set_d_filter_alpha(pid_with_filter_t *pid, float alpha);

/**
 * @brief 设置输出滤波系数
 * @param pid    控制器句柄
 * @param alpha  滤波系数 (0~1, 越大越不滤波, 推荐0.2~0.5)
 */
void pid_wf_set_out_filter_alpha(pid_with_filter_t *pid, float alpha);

/**
 * @brief 设置微分信号来源
 * @param pid      控制器句柄
 * @param source   PID_D_FROM_ERROR 或 PID_D_FROM_PV
 */
void pid_wf_set_d_source(pid_with_filter_t *pid, pid_d_source_t source);

/**
 * @brief 设置积分限幅
 * @param pid       控制器句柄
 * @param int_min   积分下限
 * @param int_max   积分上限
 */
void pid_wf_set_integral_limit(pid_with_filter_t *pid, float int_min, float int_max);

/**
 * @brief 计算滤波后的PID输出(位置式)
 * @param pid    控制器句柄
 * @param setpoint  设定值
 * @param pv        测量值(过程变量)
 * @return 滤波后的控制输出
 */
float pid_wf_compute(pid_with_filter_t *pid, float setpoint, float pv);

/**
 * @brief 重置PID内部状态
 * @param pid  控制器句柄
 */
void pid_wf_reset(pid_with_filter_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_WITH_FILTER_H */
