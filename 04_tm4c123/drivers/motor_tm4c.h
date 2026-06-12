/**
 * @file motor_tm4c.h
 * @brief TM4C123 电机驱动模块 (PWM)
 *
 * 支持直流电机正反转、制动、PWM调速。
 * 使用 PWM0 模块的 Generator 0/1 (PB6/PB7) 和 Generator 2/3 (PB4/PB5)。
 * 配合 H桥驱动芯片(TB6612/BTS7960等)使用。
 */
#ifndef __MOTOR_TM4C_H
#define __MOTOR_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 配置宏 ======================== */
#define MOTOR_PWM_FREQ_HZ      20000   /* 20kHz PWM频率，静音 */
#define MOTOR_PWM_BASE          PWM0_BASE

/* 电机通道定义 */
typedef enum {
    MOTOR_CH_A = 0,     /* 左电机 - PWM0_OUT0 (PB6) */
    MOTOR_CH_B = 1,     /* 右电机 - PWM0_OUT1 (PB7) */
    MOTOR_CH_MAX
} motor_ch_t;

/* 电机方向 */
typedef enum {
    MOTOR_DIR_FWD = 0,  /* 正转 */
    MOTOR_DIR_REV = 1,  /* 反转 */
    MOTOR_DIR_BRAKE = 2 /* 制动(两线短接) */
} motor_dir_t;

/* 电机配置结构体 */
typedef struct {
    uint32_t pwm_gen;       /* PWM发生器 PWM_OUT_0 / PWM_OUT_1 */
    uint32_t in1_port;      /* 方向引脚1 GPIO端口 */
    uint8_t  in1_pin;       /* 方向引脚1 GPIO引脚 */
    uint32_t in2_port;      /* 方向引脚2 GPIO端口 */
    uint8_t  in2_pin;       /* 方向引脚2 GPIO引脚 */
    motor_dir_t dir;        /* 当前方向 */
    float    duty;          /* 当前占空比 0~1.0 */
} motor_handle_t;

/* ======================== API ======================== */

/**
 * @brief 初始化电机PWM和方向控制GPIO
 */
void motor_init(void);

/**
 * @brief 设置电机速度和方向
 * @param ch    电机通道 MOTOR_CH_A / MOTOR_CH_B
 * @param speed 速度值 -1000 ~ +1000 (负值=反转)
 *        绝对值映射到 0~100% 占空比
 */
void motor_set(motor_ch_t ch, int16_t speed);

/**
 * @brief 电机刹车(两线接地)
 */
void motor_brake(motor_ch_t ch);

/**
 * @brief 停止所有电机
 */
void motor_stop_all(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_TM4C_H */
