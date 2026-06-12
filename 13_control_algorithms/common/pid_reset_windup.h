/**
 * @file pid_reset_windup.h
 * @brief 积分重置抗饱和PID - 积分值回退(Bumpless)抗饱和策略
 *
 * 核心思想:
 *   当输出饱和时, 不是简单停止积分, 而是将积分值"重置"到恰好
 *   使输出等于饱和边界的值。这比条件积分更精确, 比back-calculation
 *   更直观。
 *
 * 重置策略:
 *   当 output > out_max:
 *     integral = (out_max - P_term - D_term) / ki
 *   当 output < out_min:
 *     integral = (out_min - P_term - D_term) / ki
 *
 * 附加特性:
 *   1. 积分死区: |error| < threshold 时清零积分(消除稳态微振荡)
 *   2. 积分限幅: 双重保护
 *   3. Bumpless切换: 重置时保证输出无跳变
 *
 * 适用场景:
 *   - 电机控制(电流环、速度环)
 *   - 温度控制(执行器有硬限幅)
 *   - 任何执行器存在饱和的场合
 */

#ifndef PID_RESET_WINDUP_H
#define PID_RESET_WINDUP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 积分重置抗饱和模式
 */
typedef enum {
    RESET_WINDUP_NONE       = 0,    /* 不使用重置 */
    RESET_WINDUP_EXACT      = 1,    /* 精确重置: 积分=使输出等于边界的值 */
    RESET_WINDUP_CONDITIONAL = 2,   /* 条件积分: 饱和时停止积分 */
    RESET_WINDUP_DEADZONE   = 3     /* 死区重置: |error|<阈值时清零积分 */
} reset_windup_mode_t;

/**
 * @brief 积分重置抗饱和PID控制器句柄
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
    float prev_d_term;        /* 微分项滤波后值 */
    uint8_t first_run;

    /* 输出限幅 */
    float out_min;
    float out_max;

    /* 积分限幅(二次保护) */
    float integral_min;
    float integral_max;

    /* 重置抗饱和配置 */
    reset_windup_mode_t mode;

    /* 死区阈值 */
    float deadzone;

    /* 微分滤波系数 */
    float d_filter_alpha;

    /* 采样时间 */
    float dt;

    /* 调试信息 */
    float last_p_term;        /* 上一次P项(供调试) */
    float last_d_term;        /* 上一次D项(供调试) */
    uint8_t saturated;        /* 当前是否饱和 */
} pid_rst_windup_t;

/**
 * @brief 初始化积分重置抗饱和PID
 * @param pid       控制器句柄
 * @param kp        比例增益
 * @param ki        积分增益
 * @param kd        微分增益
 * @param dt        采样周期(秒)
 * @param out_min   输出下限
 * @param out_max   输出上限
 */
void pid_rw_init(pid_rst_windup_t *pid,
                 float kp, float ki, float kd,
                 float dt, float out_min, float out_max);

/**
 * @brief 设置抗饱和模式
 * @param pid   控制器句柄
 * @param mode  抗饱和模式
 */
void pid_rw_set_mode(pid_rst_windup_t *pid, reset_windup_mode_t mode);

/**
 * @brief 设置死区阈值(仅DEADZONE模式)
 * @param pid       控制器句柄
 * @param deadzone  死区阈值
 */
void pid_rw_set_deadzone(pid_rst_windup_t *pid, float deadzone);

/**
 * @brief 设置积分限幅
 * @param pid   控制器句柄
 * @param lo    积分下限
 * @param hi    积分上限
 */
void pid_rw_set_integral_limit(pid_rst_windup_t *pid, float lo, float hi);

/**
 * @brief 设置微分滤波系数
 * @param pid     控制器句柄
 * @param alpha   滤波系数 (0~1)
 */
void pid_rw_set_d_filter_alpha(pid_rst_windup_t *pid, float alpha);

/**
 * @brief 计算PID输出
 * @param pid       控制器句柄
 * @param setpoint  设定值
 * @param pv        测量值
 * @return 控制输出
 */
float pid_rw_compute(pid_rst_windup_t *pid, float setpoint, float pv);

/**
 * @brief 重置PID内部状态
 * @param pid  控制器句柄
 */
void pid_rw_reset(pid_rst_windup_t *pid);

/**
 * @brief 获取当前是否饱和
 * @param pid  控制器句柄
 * @return 1=饱和, 0=未饱和
 */
uint8_t pid_rw_is_saturated(const pid_rst_windup_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_RESET_WINDUP_H */
