/**
 * @file    user_config.h
 * @brief   用户配置文件 - 2024电赛H题自动行驶小车
 * @author  电赛团队
 * @date    2024
 * @note    包含所有可调参数和硬件引脚定义
 */

#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

/* ========================================================================== */
/*                              硬件引脚定义                                   */
/* ========================================================================== */

/* ---- 循迹传感器引脚 (7路) ---- */
#define SENSOR_S1_PIN           GPIO_PIN_2      /* PA2 */
#define SENSOR_S1_PORT          GPIOA
#define SENSOR_S2_PIN           GPIO_PIN_3      /* PA3 */
#define SENSOR_S2_PORT          GPIOA
#define SENSOR_S3_PIN           GPIO_PIN_0      /* PB0 */
#define SENSOR_S3_PORT          GPIOB
#define SENSOR_S4_PIN           GPIO_PIN_1      /* PB1 */
#define SENSOR_S4_PORT          GPIOB
#define SENSOR_S5_PIN           GPIO_PIN_10     /* PB10 */
#define SENSOR_S5_PORT          GPIOB
#define SENSOR_S6_PIN           GPIO_PIN_11     /* PB11 */
#define SENSOR_S6_PORT          GPIOB
#define SENSOR_S7_PIN           GPIO_PIN_12     /* PB12 */
#define SENSOR_S7_PORT          GPIOB

/* ---- 电机驱动引脚 (TB6612FNG) ---- */
/* 左电机 (通道A) */
#define MOTOR_L_IN1_PIN         GPIO_PIN_13     /* PB13 */
#define MOTOR_L_IN1_PORT        GPIOB
#define MOTOR_L_IN2_PIN         GPIO_PIN_14     /* PB14 */
#define MOTOR_L_IN2_PORT        GPIOB
#define MOTOR_L_PWM_TIM         TIM4
#define MOTOR_L_PWM_CH          TIM_CHANNEL_1   /* PB6 */

/* 右电机 (通道B) */
#define MOTOR_R_IN1_PIN         GPIO_PIN_15     /* PB15 */
#define MOTOR_R_IN1_PORT        GPIOB
#define MOTOR_R_IN2_PIN         GPIO_PIN_8      /* PA8 */
#define MOTOR_R_IN2_PORT        GPIOA
#define MOTOR_R_PWM_TIM         TIM4
#define MOTOR_R_PWM_CH          TIM_CHANNEL_2   /* PB7 */

/* TB6612 STBY引脚 */
#define MOTOR_STBY_PIN          GPIO_PIN_9      /* PA9 */
#define MOTOR_STBY_PORT         GPIOA

/* ---- 编码器引脚 ---- */
/* 左编码器: TIM2 CH1/CH2 (PA0/PA1) */
#define ENCODER_L_TIM           TIM2
/* 右编码器: TIM3 CH1/CH2 (PA6/PA7) */
#define ENCODER_R_TIM           TIM3

/* ---- 声光提示引脚 ---- */
#define BUZZER_PIN              GPIO_PIN_8      /* PB8 */
#define BUZZER_PORT             GPIOB
#define LED_PIN                 GPIO_PIN_9      /* PB9 */
#define LED_PORT                GPIOB

/* ---- 按键引脚 ---- */
#define KEY_MODE_PIN            GPIO_PIN_4      /* PA4 */
#define KEY_MODE_PORT           GPIOA
#define KEY_START_PIN           GPIO_PIN_5      /* PA5 */
#define KEY_START_PORT          GPIOA

/* ========================================================================== */
/*                             系统参数配置                                     */
/* ========================================================================== */

/* ---- PWM参数 ---- */
#define PWM_MAX                 999             /* PWM最大值 (1000-1) */
#define PWM_MIN                 300             /* PWM最小值（电机启动最低值） */
#define PWM_FREQUENCY           10000           /* PWM频率 10kHz */

/* ---- 编码器参数 ---- */
#define ENCODER_PPR             200             /* 编码器每转脉冲数 */
#define ENCODER_MULTIPLIER      4               /* 四倍频 */
#define GEAR_RATIO              34              /* 减速比 34:1 */
#define WHEEL_DIAMETER_MM       65              /* 车轮直径(mm) */
#define WHEEL_TRACK_MM          100             /* 轮距(mm) */

/* 每个编码器脉冲对应的距离(mm) */
/* = π × 轮径 / (编码器线数 × 四倍频 × 减速比) */
/* = 3.14159 × 65 / (200 × 4 × 34) ≈ 0.075mm */
#define DISTANCE_PER_PULSE_MM   0.075f

/* 每厘米对应的脉冲数 */
/* = 10 / 0.075 ≈ 133.3 */
#define PULSES_PER_CM           133.3f

/* ---- 路径距离参数(cm) ---- */
#define DIST_AB_CM              100.0f          /* AB段直线距离 */
#define DIST_BC_CM              125.7f          /* BC段半圆弧距离 (π×40) */
#define DIST_CD_CM              100.0f          /* CD段直线距离 */
#define DIST_DA_CM              125.7f          /* DA段半圆弧距离 (π×40) */
#define DIST_ONE_LAP_CM         451.4f          /* 一圈总距离 */

#define DISTANCE_TOLERANCE_CM   5.0f            /* 距离判断容差(cm) */

/* ---- 循迹传感器权重 ---- */
/* 7路传感器权重，从左到右: -3, -2, -1, 0, +1, +2, +3 */
#define SENSOR_WEIGHT_LEFT      -3
#define SENSOR_WEIGHT_ML        -2
#define SENSOR_WEIGHT_SL        -1
#define SENSOR_WEIGHT_CENTER     0
#define SENSOR_WEIGHT_SR        +1
#define SENSOR_WEIGHT_MR        +2
#define SENSOR_WEIGHT_RIGHT     +3

/* ---- 直线段PID参数 (×100存储，使用时除以100) ---- */
#define LINE_KP                 25.0f
#define LINE_KI                 0.1f
#define LINE_KD                 15.0f
#define LINE_BASE_SPEED         200             /* 直线段基础速度(PWM值) */

/* ---- 弧线段PID参数 ---- */
#define ARC_KP                  35.0f
#define ARC_KI                  0.2f
#define ARC_KD                  20.0f
#define ARC_BASE_SPEED          150             /* 弧线段基础速度(PWM值) */

/* ---- 速度限制 ---- */
#define SPEED_MAX               600             /* 最大速度(PWM值) */
#define SPEED_MIN               250             /* 最小速度(PWM值) */

/* ---- 状态机相关 ---- */
#define MAX_LAPS                4               /* 最大圈数 */
#define START_DELAY_MS          1000            /* 启动延迟(ms) */
#define POINT_ALERT_MS          500             /* 关键点提示持续时间(ms) */

/* ========================================================================== */
/*                             工作模式定义                                     */
/* ========================================================================== */

/* 运行模式 */
typedef enum {
    MODE_IDLE = 0,          /* 空闲模式 */
    MODE_1_AB_LINE,         /* 模式1: A→B直线行驶 */
    MODE_2_FORWARD_LAP,     /* 模式2: A→B→C→D→A 正向一圈 */
    MODE_3_REVERSE_LAP,     /* 模式3: A→C→B→D→A 反向一圈 */
    MODE_4_MULTI_LAPS       /* 模式4: 连续行驶4圈 */
} RunMode_t;

/* 路径段状态 */
typedef enum {
    SEGMENT_IDLE = 0,       /* 空闲 */
    SEGMENT_AB_LINE,        /* AB直线段 */
    SEGMENT_BC_ARC,         /* BC右弧线段 */
    SEGMENT_CD_LINE,        /* CD直线段 */
    SEGMENT_DA_ARC,         /* DA左弧线段 */
    SEGMENT_AC_ARC,         /* AC弧线段(反向) */
    SEGMENT_CB_LINE,        /* CB直线段(反向) */
    SEGMENT_BD_ARC,         /* BD弧线段(反向) */
    SEGMENT_DA_LINE,        /* DA直线段(反向) */
    SEGMENT_COMPLETE        /* 完成 */
} PathSegment_t;

/* 小车运行状态 */
typedef enum {
    CAR_STOP = 0,           /* 停止 */
    CAR_RUNNING,            /* 行驶中 */
    CAR_ALERTING,           /* 声光提示中 */
    CAR_FINISHED            /* 任务完成 */
} CarState_t;

#endif /* __USER_CONFIG_H */
