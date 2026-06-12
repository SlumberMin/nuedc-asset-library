/**
 * @file    line_following_pid.c
 * @brief   PID循迹小车完整示例 — MSPM0G3507
 *
 * 功能概述:
 *   1. 8路灰度传感器检测黑线位置
 *   2. 增量式PID控制转向
 *   3. 变速策略: 直道加速、弯道减速、十字停车
 *   4. 脱线保护: 自动停车或原地搜索
 *   5. OLED显示偏差值和PID参数
 *   6. 蓝牙在线调参
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │                    硬件接线说明                           │
 * ├──────────────────────────────────────────────────────────┤
 * │ TB6612 电机驱动:                                         │
 * │   MSPM0 PA0 → AIN1    PA1 → AIN2   PA12(PWM) → PWMA   │
 * │   MSPM0 PA2 → BIN1    PA3 → BIN2   PA13(PWM) → PWMB   │
 * │                                                          │
 * │ 感为8路灰度传感器 (数字模式):                             │
 * │   MSPM0 PB0~PB7 → G0~G7 (数字输出)                      │
 * │   注意: 灰度与编码器互斥(PB0~PB5共用)                     │
 * │                                                          │
 * │ N20 编码器电机 (使用IR红外版本传感器时):                   │
 * │   编码器引脚需另选，或使用不带编码器的开环控制              │
 * │                                                          │
 * │ OLED (I2C0):                                             │
 * │   MSPM0 PB2(SCL) → OLED SCL                             │
 * │   MSPM0 PB3(SDA) → OLED SDA                             │
 * │                                                          │
 * │ HC-05 蓝牙 (可选，用于调参):                              │
 * │   MSPM0 PA17(U1TX) → HC-05 RX                          │
 * │   MSPM0 PA18(U1RX) ← HC-05 TX                          │
 * └──────────────────────────────────────────────────────────┘
 *
 * 增量式PID公式:
 *   Δu = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2*e[k-1]+e[k-2])
 *   u[k] = u[k-1] + Δu
 *
 * 依赖驱动: motor_mspm0, grayscale_mspm0, oled_ssd1306_mspm0,
 *           bluetooth_hc05_mspm0, pin_config
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/motor_mspm0.h"
#include "drivers/grayscale_mspm0.h"
#include "drivers/oled_ssd1306_mspm0.h"
#include "drivers/bluetooth_hc05_mspm0.h"
#include "drivers/pin_config.h"

/* ══════════════════════════════════════════════════════════════
 *  配置参数
 * ══════════════════════════════════════════════════════════════ */

/* PWM参数 */
#define PWM_PERIOD          1000

/* 基础速度 */
#define BASE_SPEED_FAST     600     /* 直道速度 */
#define BASE_SPEED_NORMAL   400     /* 普通速度 */
#define BASE_SPEED_SLOW     250     /* 弯道速度 */
#define BASE_SPEED_CROSS    200     /* 十字路口速度 */

/* PID参数初始值 (可通过蓝牙在线调整) */
#define PID_KP              0.8f    /* 比例系数 */
#define PID_KI              0.01f   /* 积分系数 */
#define PID_KD              0.3f    /* 微分系数 */

/* PID输出限幅 */
#define PID_OUTPUT_MAX      500.0f
#define PID_OUTPUT_MIN     -500.0f

/* 积分限幅 (防止积分饱和) — 错误经验#7 */
#define INTEGRAL_MAX        300.0f
#define INTEGRAL_MIN       -300.0f

/* 偏差阈值 (用于变速策略) */
#define ERROR_STRAIGHT      100     /* |偏差| < 此值认为是直道 */
#define ERROR_CURVE         400     /* |偏差| > 此值认为是急弯 */

/* 脱线判定 */
#define OFF_TRACK_TIMEOUT   500     /* 脱线后搜索时间 (ms) */

/* OLED刷新间隔 (ms) */
#define OLED_REFRESH_MS     150

/* ══════════════════════════════════════════════════════════════
 *  增量式PID控制器
 * ══════════════════════════════════════════════════════════════ */

typedef struct {
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float kd;           /* 微分系数 */
    float error;        /* 当前误差 e[k] */
    float error_last;   /* 上次误差 e[k-1] */
    float error_prev;   /* 上上次误差 e[k-2] */
    float output;       /* 当前输出 */
    float out_max;      /* 输出上限 */
    float out_min;      /* 输出下限 */
    float integral;     /* 积分累加 (用于限幅) */
} IncPID_t;

/**
 * @brief 初始化增量式PID
 */
static void IncPID_Init(IncPID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error = 0.0f;
    pid->error_last = 0.0f;
    pid->error_prev = 0.0f;
    pid->output = 0.0f;
    pid->out_max = PID_OUTPUT_MAX;
    pid->out_min = PID_OUTPUT_MIN;
    pid->integral = 0.0f;
}

/**
 * @brief 增量式PID计算
 * @param pid    PID控制器指针
 * @param error  当前误差
 * @return PID输出值
 *
 * 增量式PID优势:
 *   1. 不需要积分限幅（增量本身有限）
 *   2. 切换模式时无冲击
 *   3. 计算量小，适合嵌入式
 */
static float IncPID_Calc(IncPID_t *pid, float error)
{
    pid->error = error;

    /* 增量计算 */
    float delta_kp = pid->kp * (pid->error - pid->error_last);
    float delta_ki = pid->ki * pid->error;
    float delta_kd = pid->kd * (pid->error - 2.0f * pid->error_last + pid->error_prev);

    float delta_u = delta_kp + delta_ki + delta_kd;

    /* 累加输出 */
    pid->output += delta_u;

    /* 输出限幅 */
    if (pid->output > pid->out_max) {
        pid->output = pid->out_max;
    } else if (pid->output < pid->out_min) {
        pid->output = pid->out_min;
    }

    /* 更新历史误差 */
    pid->error_prev = pid->error_last;
    pid->error_last = pid->error;

    return pid->output;
}

/**
 * @brief 重置PID控制器
 */
static void IncPID_Reset(IncPID_t *pid)
{
    pid->error = 0.0f;
    pid->error_last = 0.0f;
    pid->error_prev = 0.0f;
    pid->output = 0.0f;
    pid->integral = 0.0f;
}

/* ══════════════════════════════════════════════════════════════
 *  全局变量
 * ══════════════════════════════════════════════════════════════ */

/* PID控制器实例 */
static IncPID_t g_line_pid;

/* 运行状态 */
static volatile int16_t g_base_speed = BASE_SPEED_NORMAL;
static volatile uint8_t g_running = 1;          /* 1=运行, 0=停止 */
static volatile uint8_t g_off_track = 0;        /* 脱线标志 */
static volatile uint32_t g_off_track_tick = 0;  /* 脱线计时 */
static volatile uint32_t g_sys_tick = 0;        /* 系统计时 */

/* OLED刷新计时 */
static uint32_t g_last_oled_tick = 0;

/* 当前偏差值 (用于显示) */
static volatile int16_t g_current_error = 0;

/* ══════════════════════════════════════════════════════════════
 *  硬件配置
 * ══════════════════════════════════════════════════════════════ */

/* TB6612电机配置 */
static const MotorConfig motor_cfg[MOTOR_MAX] = {
    [MOTOR_A] = {
        .port_in1    = PIN_TB6612_PORT,
        .pin_in1     = PIN_TB6612_AIN1,
        .port_in2    = PIN_TB6612_PORT,
        .pin_in2     = PIN_TB6612_AIN2,
        .pwm_timer   = PIN_TB6612_PWM_TIM,
        .pwm_channel = PIN_TB6612_PWM_C0_IDX,
        .pwm_period  = PWM_PERIOD
    },
    [MOTOR_B] = {
        .port_in1    = PIN_TB6612_PORT,
        .pin_in1     = PIN_TB6612_BIN1,
        .port_in2    = PIN_TB6612_PORT,
        .pin_in2     = PIN_TB6612_BIN2,
        .pwm_timer   = PIN_TB6612_PWM_TIM,
        .pwm_channel = PIN_TB6612_PWM_C3_IDX,
        .pwm_period  = PWM_PERIOD
    }
};

/* 灰度传感器配置 (数字模式, PB0~PB7) */
static const GrayscaleConfig gray_cfg = {
    .addr_port   = NULL,        /* 数字模式不需要地址线 */
    .addr_pin_0  = 0,
    .addr_pin_1  = 0,
    .addr_pin_2  = 0,
    .adc         = NULL,
    .adc_channel = 0,
    .direction   = 0
};

/* ══════════════════════════════════════════════════════════════
 *  循迹核心算法
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 根据偏差值选择基础速度 (变速策略)
 *
 * 策略:
 *   |偏差| < 100  → 直道，高速
 *   |偏差| < 400  → 缓弯，中速
 *   |偏差| >= 400 → 急弯，低速
 *   十字路口    → 特殊低速
 */
static int16_t Select_BaseSpeed(int16_t error)
{
    uint16_t abs_error = (uint16_t)abs(error);

    /* 检测十字路口 */
    if (Grayscale_DetectCross()) {
        return BASE_SPEED_CROSS;
    }

    /* 根据偏差选择速度 */
    if (abs_error < ERROR_STRAIGHT) {
        return BASE_SPEED_FAST;
    } else if (abs_error < ERROR_CURVE) {
        return BASE_SPEED_NORMAL;
    } else {
        return BASE_SPEED_SLOW;
    }
}

/**
 * @brief 循迹主控制函数
 *
 * 工作流程:
 *   1. 读取灰度传感器偏差
 *   2. 判断是否脱线
 *   3. PID计算转向修正量
 *   4. 差速驱动
 */
static void LineFollow_Control(void)
{
    /* 1. 读取循迹偏差 (-1000 ~ +1000) */
    int16_t error = Grayscale_GetTrackError();
    g_current_error = error;

    /* 2. 脱线检测 */
    if (Grayscale_IsOffTrack()) {
        if (!g_off_track) {
            g_off_track = 1;
            g_off_track_tick = g_sys_tick;
        }
        /* 脱线超过阈值时间，停车 */
        /* 错误经验#1: 减法不会溢出（无符号回绕在计时场景下可接受） */
        if ((g_sys_tick - g_off_track_tick) > OFF_TRACK_TIMEOUT) {
            Motor_Brake(MOTOR_A);
            Motor_Brake(MOTOR_B);
            IncPID_Reset(&g_line_pid);
            return;
        }
        /* 短暂脱线: 保持上次转向继续搜索 */
    } else {
        g_off_track = 0;
    }

    /* 3. 选择基础速度 (变速策略) */
    g_base_speed = Select_BaseSpeed(error);

    /* 4. PID计算转向修正 */
    float pid_out = IncPID_Calc(&g_line_pid, (float)error);

    /* 5. 差速驱动 */
    int16_t left_speed  = g_base_speed + (int16_t)pid_out;
    int16_t right_speed = g_base_speed - (int16_t)pid_out;

    /* 速度限幅 — 错误经验#7 */
    if (left_speed > PWM_PERIOD)  left_speed = PWM_PERIOD;
    if (left_speed < -PWM_PERIOD) left_speed = -PWM_PERIOD;
    if (right_speed > PWM_PERIOD)  right_speed = PWM_PERIOD;
    if (right_speed < -PWM_PERIOD) right_speed = -PWM_PERIOD;

    Motor_SetSpeed(MOTOR_A, left_speed);
    Motor_SetSpeed(MOTOR_B, right_speed);
}

/* ══════════════════════════════════════════════════════════════
 *  OLED显示
 * ══════════════════════════════════════════════════════════════ */

static void OLED_UpdateDisplay(void)
{
    OLED_Clear();

    /* 第1行: 标题 */
    OLED_ShowString(0, 0, (char *)"Line Follow PID", 12, 1);

    /* 第2行: 偏差值 */
    OLED_ShowString(0, 16, (char *)"Err:", 12, 1);
    /* 显示有符号数 */
    if (g_current_error >= 0) {
        OLED_ShowChar(36, 16, '+', 12, 1);
        OLED_ShowNum(48, 16, (uint32_t)g_current_error, 4, 12, 1);
    } else {
        OLED_ShowChar(36, 16, '-', 12, 1);
        OLED_ShowNum(48, 16, (uint32_t)(-g_current_error), 4, 12, 1);
    }

    /* 第3行: 速度 */
    OLED_ShowString(0, 32, (char *)"Spd:", 12, 1);
    OLED_ShowNum(36, 32, (uint32_t)g_base_speed, 4, 12, 1);

    /* 第4行: PID参数 */
    OLED_ShowString(0, 48, (char *)"Kp:", 12, 1);
    OLED_ShowFloat(30, 48, g_line_pid.kp, 1, 2, 12, 1);
    OLED_ShowString(72, 48, (char *)"Kd:", 12, 1);
    OLED_ShowFloat(96, 48, g_line_pid.kd, 1, 2, 12, 1);

    OLED_Refresh();
}

/* ══════════════════════════════════════════════════════════════
 *  蓝牙调参
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 蓝牙在线调参 (简易协议)
 *
 * 协议: 单字符指令
 *   'p'/'P' = Kp + 0.1    'q'/'Q' = Kp - 0.1
 *   'i'/'I' = Ki + 0.005  'o'/'O' = Ki - 0.005
 *   'd'/'D' = Kd + 0.05   'c'/'C' = Kd - 0.05
 *   'S'     = 停止         'G'     = 启动
 *   '1'~'5' = 速度档位
 */
static void BT_TuneProcess(void)
{
    if (!BT_HC05_IsDataReceived()) return;

    uint8_t buf[32];
    uint16_t len = BT_HC05_GetReceivedData(buf, sizeof(buf));
    /* 错误经验#7: 长度已在函数内校验 */

    for (uint16_t i = 0; i < len; i++) {
        switch (buf[i]) {
        case 'p': g_line_pid.kp += 0.1f; break;
        case 'q': g_line_pid.kp -= 0.1f;
                  if (g_line_pid.kp < 0.0f) g_line_pid.kp = 0.0f;
                  break;
        case 'i': g_line_pid.ki += 0.005f; break;
        case 'o': g_line_pid.ki -= 0.005f;
                  if (g_line_pid.ki < 0.0f) g_line_pid.ki = 0.0f;
                  break;
        case 'd': g_line_pid.kd += 0.05f; break;
        case 'c': g_line_pid.kd -= 0.05f;
                  if (g_line_pid.kd < 0.0f) g_line_pid.kd = 0.0f;
                  break;
        case 'S': g_running = 0;
                  Motor_Brake(MOTOR_A);
                  Motor_Brake(MOTOR_B);
                  break;
        case 'G': g_running = 1;
                  IncPID_Reset(&g_line_pid);
                  break;
        case '1': g_base_speed = 200; break;
        case '2': g_base_speed = 350; break;
        case '3': g_base_speed = 500; break;
        case '4': g_base_speed = 650; break;
        case '5': g_base_speed = 800; break;
        default: break;
        }
    }
    BT_HC05_ClearRxBuffer();

    /* 回传当前PID参数 */
    char msg[64];
    snprintf(msg, sizeof(msg), "Kp:%.2f Ki:%.3f Kd:%.2f\r\n",
             g_line_pid.kp, g_line_pid.ki, g_line_pid.kd);
    BT_HC05_SendString(msg);
}

/* ══════════════════════════════════════════════════════════════
 *  系统初始化
 * ══════════════════════════════════════════════════════════════ */

static void System_Init(void)
{
    SYSCFG_DL_init();

    /* 初始化电机 */
    Motor_Init(motor_cfg);

    /* 初始化灰度传感器 */
    Grayscale_Init(&gray_cfg);

    /* 初始化OLED */
    OLED_Init(I2C_0_INST);
    OLED_Clear();
    OLED_ShowString(10, 24, (char *)"Line Follower", 16, 1);
    OLED_Refresh();

    /* 初始化蓝牙 (可选) */
    BT_HC05_Init(&(BT_HC05_Config){
        .uart = UART_1_INST,
        .state_port = PIN_BT_EN_PORT,
        .state_pin  = PIN_BT_EN_PIN,
        .baudrate   = 9600
    });

    /* 初始化PID */
    IncPID_Init(&g_line_pid, PID_KP, PID_KI, PID_KD);

    /* 校准灰度传感器 (可选: 读取白/黑校准值) */
    /* Grayscale_Calibrate(white_cal, black_cal); */

    delay_cycles(16000000);  /* 等待传感器稳定 (~500ms) */
}

/* ══════════════════════════════════════════════════════════════
 *  主函数
 * ══════════════════════════════════════════════════════════════ */

int main(void)
{
    System_Init();

    while (1) {
        /* 1. 读取灰度传感器 */
        Grayscale_Read();

        /* 2. 蓝牙调参处理 */
        BT_TuneProcess();

        /* 3. 循迹控制 */
        if (g_running) {
            LineFollow_Control();
        }

        /* 4. OLED显示更新 */
        if ((g_sys_tick - g_last_oled_tick) >= OLED_REFRESH_MS) {
            g_last_oled_tick = g_sys_tick;
            OLED_UpdateDisplay();
        }

        /* 5. 控制周期 ~5ms */
        delay_cycles(160000);   /* ~5ms @32MHz */
        g_sys_tick += 5;
    }
}
