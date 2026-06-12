/**
 * @file    foc_simple.h
 * @brief   简化FOC算法（Clark/Park变换+SVPWM，适合无刷电机）
 * @note    基于机器人竞赛优秀方案FOC实现，适配MSPM0G3507
 */

#ifndef FOC_SIMPLE_H
#define FOC_SIMPLE_H

#include <stdint.h>
#include <math.h>
#include <string.h>

/* 电机参数 */
#define FOC_POLE_PAIRS       7       /* 极对数 */
#define FOC_PWM_FREQ         20000   /* PWM频率 (Hz) */
#define FOC_MAX_DUTY         0.95f   /* 最大占空比 */
#define FOC_VBUS_DEFAULT     12.0f   /* 默认母线电压 (V) */

/* PI控制器子结构 */
typedef struct {
    float kp;
    float ki;
    float integral;
    float output_limit;
} foc_pi_t;

/* FOC句柄 */
typedef struct {
    /* 传感器 */
    float angle;            /* 电角度 (rad) */
    float speed;            /* 机械转速 (rpm) */

    /* 三相电流 */
    float i_a, i_b, i_c;

    /* Clark变换结果 */
    float i_alpha, i_beta;

    /* Park变换结果 */
    float i_d, i_q;

    /* 目标值 */
    float i_d_ref;          /* d轴目标（通常为0） */
    float i_q_ref;          /* q轴目标（控制转矩） */
    float speed_ref;        /* 速度目标 */

    /* PI控制器输出 */
    float v_d, v_q;

    /* 反Park变换结果 */
    float v_alpha, v_beta;

    /* SVPWM输出 */
    float duty_a, duty_b, duty_c;

    /* PI控制器 */
    foc_pi_t pi_d;
    foc_pi_t pi_q;
    foc_pi_t pi_speed;

    /* 母线电压 */
    float v_bus;
} FOC_HandleTypeDef;

/* 初始化 */
void FOC_Init(FOC_HandleTypeDef *hfoc);

/* Clark变换: 三相静止 → 两相静止 (α-β) */
void FOC_ClarkeTransform(float i_a, float i_b, float i_c,
                         float *i_alpha, float *i_beta);

/* Park变换: 两相静止 → 两相旋转 (d-q) */
void FOC_ParkTransform(float i_alpha, float i_beta, float angle,
                       float *i_d, float *i_q);

/* 反Park变换: 两相旋转 → 两相静止 */
void FOC_InverseParkTransform(float v_d, float v_q, float angle,
                              float *v_alpha, float *v_beta);

/* PI控制器 */
float FOC_PIController(float ref, float meas, float kp, float ki,
                       float limit, float *integral);

/* SVPWM: 空间矢量脉宽调制 */
void FOC_SVPWM(float v_alpha, float v_beta, float v_bus,
               float *duty_a, float *duty_b, float *duty_c);

/* FOC主控制循环（应在PWM中断中调用） */
void FOC_Update(FOC_HandleTypeDef *hfoc,
                float i_a, float i_b, float i_c, float angle);

/* 设置PI参数 */
void FOC_SetPI(FOC_HandleTypeDef *hfoc, float d_kp, float d_ki,
               float q_kp, float q_ki, float spd_kp, float spd_ki);

/* 设置速度目标 */
void FOC_SetSpeedRef(FOC_HandleTypeDef *hfoc, float rpm);

#endif /* FOC_SIMPLE_H */
