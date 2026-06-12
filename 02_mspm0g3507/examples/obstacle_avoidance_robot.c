/**
 * @file    obstacle_avoidance_robot.c
 * @brief   避障机器人完整示例 — MSPM0G3507
 *
 * 功能概述:
 *   1. 超声波传感器测距 (SR04/US-016)
 *   2. 舵机扫描左右方向 (SG90)
 *   3. 有限状态机控制行为决策
 *   4. 三路距离检测: 左/中/右
 *   5. OLED显示距离和状态
 *
 * 状态机:
 *   IDLE → FORWARD → OBSTACLE_DETECT → SCAN → TURN → FORWARD
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │                    硬件接线说明                           │
 * ├──────────────────────────────────────────────────────────┤
 * │ TB6612 电机驱动:                                         │
 * │   MSPM0 PA0 → AIN1    PA1 → AIN2   PA12(PWM) → PWMA   │
 * │   MSPM0 PA2 → BIN1    PA3 → BIN2   PA13(PWM) → PWMB   │
 * │                                                          │
 * │ SR04 超声波:                                             │
 * │   MSPM0 PB6 → Trig     PB7 → Echo                       │
 * │   注意: 超声波与灰度互斥(PB6/PB7共用)                     │
 * │                                                          │
 * │ SG90 舵机 (搭载超声波):                                   │
 * │   MSPM0 PA8(TIMA0 CH0) → 舵机信号线                      │
 * │   注意: PA8与L298N ENA互斥                                │
 * │                                                          │
 * │ OLED (I2C0):                                             │
 * │   MSPM0 PB2(SCL) → OLED SCL                             │
 * │   MSPM0 PB3(SDA) → OLED SDA                             │
 * │                                                          │
 * │ LED指示:                                                 │
 * │   MSPM0 PA22 → 状态LED                                   │
 * └──────────────────────────────────────────────────────────┘
 *
 * 依赖驱动: motor_mspm0, ultrasonic_mspm0, servo_mspm0,
 *           oled_ssd1306_mspm0, state_machine, pin_config
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
#include "drivers/ultrasonic_mspm0.h"
#include "drivers/servo_mspm0.h"
#include "drivers/oled_ssd1306_mspm0.h"
#include "drivers/state_machine.h"
#include "drivers/pin_config.h"

/* ══════════════════════════════════════════════════════════════
 *  配置参数
 * ══════════════════════════════════════════════════════════════ */

/* PWM参数 */
#define PWM_PERIOD          1000

/* 运动速度 */
#define SPEED_FORWARD       400     /* 前进速度 */
#define SPEED_TURN          350     /* 转弯速度 */
#define SPEED_SLOW          200     /* 慢速 */

/* 避障距离阈值 (cm) */
#define DIST_DANGER         20.0f   /* 危险距离: 立即停车 */
#define DIST_WARNING        35.0f   /* 警告距离: 减速 */
#define DIST_SAFE           50.0f   /* 安全距离: 正常行驶 */

/* 舵机角度 */
#define SERVO_CENTER        90      /* 正前方 */
#define SERVO_LEFT          150     /* 左转45° */
#define SERVO_RIGHT         30      /* 右转45° */
#define SERVO_FAR_LEFT      170     /* 极左 */
#define SERVO_FAR_RIGHT     10      /* 极右 */

/* 扫描延时 (ms) — 舵机到位等待 */
#define SERVO_SETTLE_MS     300

/* 转弯持续时间 (ms) */
#define TURN_DURATION_MS    400

/* 倒车时间 (ms) */
#define REVERSE_DURATION_MS 300

/* OLED刷新间隔 (ms) */
#define OLED_REFRESH_MS     200

/* ══════════════════════════════════════════════════════════════
 *  状态机定义
 * ══════════════════════════════════════════════════════════════ */

/* 避障状态枚举 */
typedef enum {
    STATE_IDLE = 0,         /* 空闲/初始化 */
    STATE_FORWARD,          /* 前进 */
    STATE_OBSTACLE_DETECT,  /* 检测到障碍 */
    STATE_SCAN,             /* 扫描左右 */
    STATE_REVERSE,          /* 后退 */
    STATE_TURN_LEFT,        /* 左转 */
    STATE_TURN_RIGHT,       /* 右转 */
    STATE_TURN_AROUND,      /* 掉头 */
    STATE_COUNT
} AvoidState;

/* 扫描结果 */
typedef struct {
    float dist_left;        /* 左方距离 */
    float dist_center;      /* 正前方距离 */
    float dist_right;       /* 右方距离 */
} ScanResult;

/* ══════════════════════════════════════════════════════════════
 *  全局变量
 * ══════════════════════════════════════════════════════════════ */

/* 当前状态 */
static volatile AvoidState g_state = STATE_IDLE;
static volatile AvoidState g_prev_state = STATE_IDLE;

/* 状态计时 */
static volatile uint32_t g_sys_tick = 0;
static uint32_t g_state_enter_tick = 0;

/* 扫描结果 */
static ScanResult g_scan;

/* 超声波距离 */
static volatile float g_front_dist = 100.0f;

/* OLED刷新计时 */
static uint32_t g_last_oled_tick = 0;

/* 状态名称 */
static const char *state_names[] = {
    "IDLE", "FWD", "DETECT", "SCAN",
    "REVS", "TRNL", "TRNR", "TRNA"
};

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

/* 超声波配置 (PB6=Trig, PB7=Echo) */
static const UltrasonicConfig ultra_cfg = {
    .port       = PIN_ULTRA_PORT,
    .trig_pin   = PIN_ULTRA_TRIG,
    .echo_pin   = PIN_ULTRA_ECHO,
    .type       = ULTRASONIC_SR04,
    .filter_size = 5
};

/* ══════════════════════════════════════════════════════════════
 *  运动控制辅助函数
 * ══════════════════════════════════════════════════════════════ */

static void Car_Forward(int16_t speed)
{
    Motor_SetSpeed(MOTOR_A, speed);
    Motor_SetSpeed(MOTOR_B, speed);
}

static void Car_Backward(int16_t speed)
{
    Motor_SetSpeed(MOTOR_A, -speed);
    Motor_SetSpeed(MOTOR_B, -speed);
}

static void Car_TurnLeft(int16_t speed)
{
    Motor_SetSpeed(MOTOR_A, -speed);    /* 左轮反转 */
    Motor_SetSpeed(MOTOR_B,  speed);    /* 右轮正转 */
}

static void Car_TurnRight(int16_t speed)
{
    Motor_SetSpeed(MOTOR_A,  speed);    /* 左轮正转 */
    Motor_SetSpeed(MOTOR_B, -speed);    /* 右轮反转 */
}

static void Car_Stop(void)
{
    Motor_Brake(MOTOR_A);
    Motor_Brake(MOTOR_B);
}

/* ══════════════════════════════════════════════════════════════
 *  超声波扫描
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 三方向扫描: 左→中→右
 *
 * 舵机转到指定方向后等待稳定，再读取距离。
 * 错误经验#34: Servo_SetAngle内部已做角度限幅和脉宽范围保护。
 */
static void Scan_ThreeDirections(void)
{
    /* 扫描正前方 */
    Servo_SetAngle(SERVO_CENTER);
    delay_cycles(SERVO_SETTLE_MS * 32000);  /* 等待舵机到位 */
    g_scan.dist_center = Ultrasonic_GetFilteredDistance();

    /* 扫描左方 */
    Servo_SetAngle(SERVO_LEFT);
    delay_cycles(SERVO_SETTLE_MS * 32000);
    g_scan.dist_left = Ultrasonic_GetFilteredDistance();

    /* 扫描右方 */
    Servo_SetAngle(SERVO_RIGHT);
    delay_cycles(SERVO_SETTLE_MS * 32000);
    g_scan.dist_right = Ultrasonic_GetFilteredDistance();

    /* 回正 */
    Servo_SetAngle(SERVO_CENTER);
}

/**
 * @brief 判断最佳转向方向
 * @return 0=左转, 1=右转, 2=掉头(两边都不通)
 */
static uint8_t Find_BestDirection(void)
{
    /* 错误经验#1: 距离值不会为除数，无除零风险 */
    if (g_scan.dist_left > g_scan.dist_right) {
        if (g_scan.dist_left > DIST_SAFE) {
            return 0;   /* 左方空间足够 */
        }
    } else {
        if (g_scan.dist_right > DIST_SAFE) {
            return 1;   /* 右方空间足够 */
        }
    }

    /* 两边都不够宽 */
    if (g_scan.dist_left > g_scan.dist_right) {
        return 0;   /* 选相对宽的左边 */
    } else {
        return 1;
    }
}

/* ══════════════════════════════════════════════════════════════
 *  状态机处理函数
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 状态转换
 */
static void State_Transition(AvoidState new_state)
{
    g_prev_state = g_state;
    g_state = new_state;
    g_state_enter_tick = g_sys_tick;
}

/**
 * @brief 获取当前状态持续时间 (ms)
 */
static uint32_t State_Elapsed(void)
{
    return g_sys_tick - g_state_enter_tick;
}

/**
 * @brief 状态机主循环 (每个控制周期调用)
 */
static void StateMachine_Update(void)
{
    /* 读取正前方距离 */
    g_front_dist = Ultrasonic_GetFilteredDistance();

    switch (g_state) {

    case STATE_IDLE:
        /* 初始化完成，直接进入前进状态 */
        State_Transition(STATE_FORWARD);
        break;

    case STATE_FORWARD:
        Car_Forward(SPEED_FORWARD);

        /* 检测前方障碍 */
        if (g_front_dist < DIST_DANGER) {
            Car_Stop();
            State_Transition(STATE_OBSTACLE_DETECT);
        } else if (g_front_dist < DIST_WARNING) {
            /* 减速行驶 */
            Car_Forward(SPEED_SLOW);
        }
        break;

    case STATE_OBSTACLE_DETECT:
        /* 确认障碍 (二次测量防误判) */
        Car_Stop();
        delay_cycles(160000);   /* 等待5ms */
        g_front_dist = Ultrasonic_GetFilteredDistance();

        if (g_front_dist < DIST_WARNING) {
            State_Transition(STATE_SCAN);
        } else {
            /* 误判，继续前进 */
            State_Transition(STATE_FORWARD);
        }
        break;

    case STATE_SCAN:
        Car_Stop();
        Scan_ThreeDirections();

        /* 根据扫描结果决策 */
        {
            uint8_t best = Find_BestDirection();
            if (best == 0) {
                State_Transition(STATE_TURN_LEFT);
            } else if (best == 1) {
                State_Transition(STATE_TURN_RIGHT);
            } else {
                State_Transition(STATE_TURN_AROUND);
            }
        }
        break;

    case STATE_REVERSE:
        Car_Backward(SPEED_SLOW);
        if (State_Elapsed() >= REVERSE_DURATION_MS) {
            Car_Stop();
            State_Transition(STATE_SCAN);
        }
        break;

    case STATE_TURN_LEFT:
        Car_TurnLeft(SPEED_TURN);
        if (State_Elapsed() >= TURN_DURATION_MS) {
            Car_Stop();
            State_Transition(STATE_FORWARD);
        }
        break;

    case STATE_TURN_RIGHT:
        Car_TurnRight(SPEED_TURN);
        if (State_Elapsed() >= TURN_DURATION_MS) {
            Car_Stop();
            State_Transition(STATE_FORWARD);
        }
        break;

    case STATE_TURN_AROUND:
        /* 掉头: 转更长时间 */
        Car_TurnRight(SPEED_TURN);
        if (State_Elapsed() >= (TURN_DURATION_MS * 2)) {
            Car_Stop();
            State_Transition(STATE_FORWARD);
        }
        break;

    default:
        /* 错误经验: switch添加default分支 */
        Car_Stop();
        State_Transition(STATE_IDLE);
        break;
    }
}

/* ══════════════════════════════════════════════════════════════
 *  OLED显示
 * ══════════════════════════════════════════════════════════════ */

static void OLED_UpdateDisplay(void)
{
    OLED_Clear();

    /* 第1行: 标题 */
    OLED_ShowString(0, 0, (char *)"Avoid Robot", 16, 1);

    /* 第2行: 状态 */
    OLED_ShowString(0, 16, (char *)"St:", 12, 1);
    OLED_ShowString(24, 16, (char *)state_names[g_state], 12, 1);

    /* 第3行: 前方距离 */
    OLED_ShowString(0, 32, (char *)"Dist:", 12, 1);
    OLED_ShowFloat(42, 32, g_front_dist, 3, 1, 12, 1);
    OLED_ShowString(90, 32, (char *)"cm", 12, 1);

    /* 第4行: 扫描结果 (仅在SCAN状态显示) */
    if (g_state == STATE_SCAN || g_prev_state == STATE_SCAN) {
        OLED_ShowString(0, 48, (char *)"L:", 12, 1);
        OLED_ShowFloat(14, 48, g_scan.dist_left, 2, 0, 12, 1);
        OLED_ShowString(48, 48, (char *)"R:", 12, 1);
        OLED_ShowFloat(62, 48, g_scan.dist_right, 2, 0, 12, 1);
    }

    OLED_Refresh();
}

/* ══════════════════════════════════════════════════════════════
 *  系统初始化
 * ══════════════════════════════════════════════════════════════ */

static void System_Init(void)
{
    SYSCFG_DL_init();

    /* 初始化电机 */
    Motor_Init(motor_cfg);

    /* 初始化超声波 */
    Ultrasonic_Init(&ultra_cfg);

    /* 初始化舵机 (PA8, TIMA0 CH0) */
    Servo_Init(PIN_SERVO_TIM, PIN_SERVO_C0_IDX);
    Servo_SetAngle(SERVO_CENTER);   /* 初始朝正前方 */

    /* 初始化OLED */
    OLED_Init(I2C_0_INST);
    OLED_Clear();
    OLED_ShowString(10, 24, (char *)"Avoid Robot", 16, 1);
    OLED_Refresh();

    /* LED指示 */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_22);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_22);

    delay_cycles(16000000);  /* 等待传感器稳定 */
}

/* ══════════════════════════════════════════════════════════════
 *  主函数
 * ══════════════════════════════════════════════════════════════ */

int main(void)
{
    System_Init();

    while (1) {
        /* 1. 触发超声波测量 (每次循环) */
        Ultrasonic_Measure();

        /* 2. 状态机更新 */
        StateMachine_Update();

        /* 3. OLED刷新 */
        if ((g_sys_tick - g_last_oled_tick) >= OLED_REFRESH_MS) {
            g_last_oled_tick = g_sys_tick;
            OLED_UpdateDisplay();
        }

        /* 4. 控制周期 ~20ms */
        delay_cycles(640000);   /* ~20ms @32MHz */
        g_sys_tick += 20;
    }
}
