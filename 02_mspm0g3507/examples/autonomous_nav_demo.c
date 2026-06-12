/**
 * @file    autonomous_nav_demo.c
 * @brief   自主导航小车示例 — MSPM0G3507
 *
 * 功能概述:
 *   1. 超声波传感器测距避障 (前方+左侧+右侧)
 *   2. 8路灰度传感器循迹
 *   3. 有限状态机管理导航行为
 *   4. 电机差速转向
 *   5. OLED显示导航状态
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │                    硬件接线说明                           │
 * ├──────────────────────────────────────────────────────────┤
 * │ 超声波传感器 (3个SR04):                                  │
 * │   前方: MSPM0 PB6 → Trig   PB7 → Echo                   │
 * │   左侧: MSPM0 PA28→ Trig   PA29→ Echo                   │
 * │   右侧: MSPM0 PA30→ Trig   PA31→ Echo                   │
 * │                                                          │
 * │ 8路灰度传感器 (感为无MCU版):                              │
 * │   MSPM0 PB0 → AD0 (ADC输入)                              │
 * │   MSPM0 PB27→ OUT (数字输出, 可选)                        │
 * │   MSPM0 PA25→ 地址位0                                    │
 * │   MSPM0 PA26→ 地址位1                                    │
 * │   MSPM0 PA27→ 地址位2                                    │
 * │                                                          │
 * │ TB6612 电机驱动:                                         │
 * │   MSPM0 PA0 → AIN1   PA1 → AIN2   PA12(PWM) → PWMA    │
 * │   MSPM0 PA2 → BIN1   PA3 → BIN2   PA13(PWM) → PWMB    │
 * │   MSPM0 PA4 → STBY                                      │
 * │                                                          │
 * │ N20 编码器电机:                                          │
 * │   MSPM0 PB0 → 左轮A相   PB1 → 左轮B相                   │
 * │   MSPM0 PB4 → 右轮A相   PB5 → 右轮B相                   │
 * │                                                          │
 * │ OLED 显示:                                               │
 * │   MSPM0 PB2(SCL) → SCL   PB3(SDA) → SDA                │
 * │                                                          │
 * │ 按键 (可选):                                             │
 * │   MSPM0 PA18 → 启动按键 (接地触发)                       │
 * └──────────────────────────────────────────────────────────┘
 *
 * 导航策略:
 *   巡线模式: 沿黑线前进, 遇到十字路口/丁字路口执行转向决策
 *   避障模式: 检测到前方障碍物时切换到避障路径
 *   迷宫模式: 右手法则/左手法则走迷宫
 *
 * 依赖驱动: ultrasonic_mspm0, grayscale_mspm0, motor_mspm0,
 *           encoder_gpio_mspm0, oled_ssd1306_mspm0, state_machine,
 *           advanced_pid
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/ultrasonic_mspm0.h"
#include "drivers/grayscale_mspm0.h"
#include "drivers/motor_mspm0.h"
#include "drivers/encoder_gpio_mspm0.h"
#include "drivers/oled_ssd1306_mspm0.h"
#include "drivers/state_machine.h"
#include "drivers/advanced_pid.h"

/* ══════════════════════════════════════════════════════════════
 *  配置参数
 * ══════════════════════════════════════════════════════════════ */

#define PWM_PERIOD          1000
#define PWM_MAX             800
#define ENCODER_SAMPLE_MS   10

/* 超声波避障阈值 (cm) */
#define OBSTACLE_DIST_FRONT 20.0f   /* 前方障碍物距离 */
#define OBSTACLE_DIST_SIDE  15.0f   /* 侧方障碍物距离 */
#define WALL_FOLLOW_DIST    15.0f   /* 沿墙行走距离 */

/* 循迹基础速度 */
#define TRACK_SPEED_SLOW    200
#define TRACK_SPEED_NORMAL  350
#define TRACK_SPEED_FAST    500

/* 超声波传感器数量 */
#define ULTRASONIC_FRONT    0
#define ULTRASONIC_LEFT     1
#define ULTRASONIC_RIGHT    2
#define ULTRASONIC_COUNT    3

/* ══════════════════════════════════════════════════════════════
 *  导航模式
 * ══════════════════════════════════════════════════════════════ */
typedef enum {
    NAV_LINE_TRACK = 0,     /* 循线模式 */
    NAV_OBSTACLE_AVOID,     /* 避障模式 */
    NAV_MAZE_RIGHT,         /* 迷宫右手法则 */
    NAV_MAZE_LEFT,          /* 迷宫左手法则 */
    NAV_FREE_ROAM           /* 自由漫游 */
} NavMode;

/* ══════════════════════════════════════════════════════════════
 *  状态机定义
 * ══════════════════════════════════════════════════════════════ */

enum {
    STATE_TRACK_LINE = 0,   /* 巡线前进 */
    STATE_TURN_LEFT,        /* 左转 */
    STATE_TURN_RIGHT,       /* 右转 */
    STATE_U_TURN,           /* 掉头 */
    STATE_OBSTACLE_STOP,    /* 遇障碍停车 */
    STATE_OBSTACLE_TURN,    /* 避障转向 */
    STATE_WALL_FOLLOW,      /* 沿墙行走 */
    STATE_SEARCH_LINE,      /* 搜索黑线 */
    STATE_STOP,             /* 停车 */
    NAV_STATE_COUNT
};

/* ══════════════════════════════════════════════════════════════
 *  全局变量
 * ══════════════════════════════════════════════════════════════ */

static SM_Machine nav_sm;
static PID_Controller pid_track;    /* 循迹PID */
static PID_Controller pid_wall;     /* 沿墙PID */

static volatile uint32_t sys_tick_ms = 0;
static volatile NavMode nav_mode = NAV_LINE_TRACK;

/* 传感器数据 */
static float dist[ULTRASONIC_COUNT] = {0};  /* 超声波距离 */
static int16_t track_error = 0;              /* 循迹偏差 */
static uint8_t grayscale_digital = 0;        /* 灰度数字量 */

/* 避障转向方向 (0=左, 1=右) */
static uint8_t avoid_turn_right = 1;

/* ══════════════════════════════════════════════════════════════
 *  定时器中断
 * ══════════════════════════════════════════════════════════════ */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        sys_tick_ms++;
        if (sys_tick_ms % ENCODER_SAMPLE_MS == 0) {
            EncoderGpio_Update();
        }
        SM_Tick(&nav_sm);
    }
}

/* ══════════════════════════════════════════════════════════════
 *  传感器读取
 * ══════════════════════════════════════════════════════════════ */

/* 超声波传感器实例 */
static UltrasonicConfig ultrasonic_cfg[ULTRASONIC_COUNT] = {
    [ULTRASONIC_FRONT] = {
        .port = GPIOB,
        .trig_pin = DL_GPIO_PIN_6,
        .echo_pin = DL_GPIO_PIN_7,
        .type = ULTRASONIC_SR04,
        .filter_size = 5
    },
    [ULTRASONIC_LEFT] = {
        .port = GPIOA,
        .trig_pin = DL_GPIO_PIN_28,
        .echo_pin = DL_GPIO_PIN_29,
        .type = ULTRASONIC_SR04,
        .filter_size = 5
    },
    [ULTRASONIC_RIGHT] = {
        .port = GPIOA,
        .trig_pin = DL_GPIO_PIN_30,
        .echo_pin = DL_GPIO_PIN_31,
        .type = ULTRASONIC_SR04,
        .filter_size = 5
    }
};

/**
 * @brief 更新所有传感器数据
 *        在主循环中周期性调用
 */
static void Update_Sensors(void)
{
    /* 读取超声波 (注意: 阻塞式测量, 需合理安排间隔) */
    dist[ULTRASONIC_FRONT] = Ultrasonic_GetFilteredDistance();  /* 使用最近一次结果 */
    /* 多路超声波需要分时触发, 避免互相干扰 */

    /* 读取灰度 */
    Grayscale_Read();
    track_error = Grayscale_GetTrackError();
    grayscale_digital = Grayscale_GetDigital();
}

/* ══════════════════════════════════════════════════════════════
 *  电机控制辅助函数
 * ══════════════════════════════════════════════════════════════ */

static void Motor_Forward(int16_t speed)
{
    Motor_SetSpeed(MOTOR_A, speed);
    Motor_SetSpeed(MOTOR_B, speed);
}

static void Motor_TurnLeft(int16_t speed)
{
    Motor_SetSpeed(MOTOR_A, -speed / 2);
    Motor_SetSpeed(MOTOR_B, speed);
}

static void Motor_TurnRight(int16_t speed)
{
    Motor_SetSpeed(MOTOR_A, speed);
    Motor_SetSpeed(MOTOR_B, -speed / 2);
}

static void Motor_UTurn(int16_t speed)
{
    Motor_SetSpeed(MOTOR_A, -speed);
    Motor_SetSpeed(MOTOR_B, speed);
}

static void Motor_Stop(void)
{
    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);
}

/* ══════════════════════════════════════════════════════════════
 *  循线控制
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 循线PID控制
 *
 * @param base_speed  基础前进速度
 *
 * 偏差来源: Grayscale_GetTrackError()
 *   -1000 ~ +1000, 0=居中, 负=偏左, 正=偏右
 */
static void TrackLine_Control(int16_t base_speed)
{
    int16_t steer = (int16_t)PID_Calc(&pid_track, 0.0f, (float)track_error);

    int16_t pwm_left  = base_speed - steer;
    int16_t pwm_right = base_speed + steer;

    /* 限幅 */
    if (pwm_left > PWM_MAX)  pwm_left = PWM_MAX;
    if (pwm_left < -PWM_MAX) pwm_left = -PWM_MAX;
    if (pwm_right > PWM_MAX)  pwm_right = PWM_MAX;
    if (pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;

    Motor_SetSpeed(MOTOR_A, pwm_left);
    Motor_SetSpeed(MOTOR_B, pwm_right);
}

/* ══════════════════════════════════════════════════════════════
 *  沿墙控制
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 沿墙PID控制
 *
 * @param base_speed  基础前进速度
 * @param side_dist   侧方距离
 * @param wall_side   0=左墙, 1=右墙
 */
static void WallFollow_Control(int16_t base_speed, float side_dist, uint8_t wall_side)
{
    float error = side_dist - WALL_FOLLOW_DIST;
    int16_t steer = (int16_t)PID_Calc(&pid_wall, 0.0f, error);

    int16_t pwm_left, pwm_right;
    if (wall_side == 0) {
        /* 沿左墙: 偏离时右转 */
        pwm_left  = base_speed + steer;
        pwm_right = base_speed - steer;
    } else {
        /* 沿右墙: 偏离时左转 */
        pwm_left  = base_speed - steer;
        pwm_right = base_speed + steer;
    }

    if (pwm_left > PWM_MAX)  pwm_left = PWM_MAX;
    if (pwm_left < -PWM_MAX) pwm_left = -PWM_MAX;
    if (pwm_right > PWM_MAX)  pwm_right = PWM_MAX;
    if (pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;

    Motor_SetSpeed(MOTOR_A, pwm_left);
    Motor_SetSpeed(MOTOR_B, pwm_right);
}

/* ══════════════════════════════════════════════════════════════
 *  状态机回调
 * ══════════════════════════════════════════════════════════════ */

/* ── TRACK_LINE: 巡线前进 ───────────────────────────────── */
static bool TrackLine_OnEnter(SM_Machine *sm)
{
    (void)sm;
    return true;
}

static bool TrackLine_OnEvent(SM_Machine *sm, const SM_Event_t *event)
{
    (void)sm;
    (void)event;
    return false;
}

/* ── TURN_LEFT: 左转 ───────────────────────────────────── */
static bool TurnLeft_OnEnter(SM_Machine *sm)
{
    (void)sm;
    Motor_TurnLeft(TRACK_SPEED_SLOW);
    return true;
}

/* ── TURN_RIGHT: 右转 ──────────────────────────────────── */
static bool TurnRight_OnEnter(SM_Machine *sm)
{
    (void)sm;
    Motor_TurnRight(TRACK_SPEED_SLOW);
    return true;
}

/* ── U_TURN: 掉头 ──────────────────────────────────────── */
static bool UTurn_OnEnter(SM_Machine *sm)
{
    (void)sm;
    Motor_UTurn(TRACK_SPEED_SLOW);
    return true;
}

/* ── OBSTACLE_STOP: 遇障碍停车 ─────────────────────────── */
static bool ObstacleStop_OnEnter(SM_Machine *sm)
{
    (void)sm;
    Motor_Stop();
    return true;
}

/* ── OBSTACLE_TURN: 避障转向 ───────────────────────────── */
static bool ObstacleTurn_OnEnter(SM_Machine *sm)
{
    (void)sm;
    if (avoid_turn_right) {
        Motor_TurnRight(TRACK_SPEED_SLOW);
    } else {
        Motor_TurnLeft(TRACK_SPEED_SLOW);
    }
    return true;
}

/* ── WALL_FOLLOW: 沿墙行走 ─────────────────────────────── */
static bool WallFollow_OnEnter(SM_Machine *sm)
{
    (void)sm;
    return true;
}

/* ── SEARCH_LINE: 搜索黑线 ─────────────────────────────── */
static bool SearchLine_OnEnter(SM_Machine *sm)
{
    (void)sm;
    /* 慢速旋转搜索 */
    Motor_TurnRight(TRACK_SPEED_SLOW);
    return true;
}

/* ── STOP: 停车 ────────────────────────────────────────── */
static bool Stop_OnEnter(SM_Machine *sm)
{
    (void)sm;
    Motor_Stop();
    return true;
}

/* ══════════════════════════════════════════════════════════════
 *  状态描述表
 * ══════════════════════════════════════════════════════════════ */
static const SM_StateDesc_t nav_state_table[NAV_STATE_COUNT] = {
    [STATE_TRACK_LINE] = {
        .parent = SM_NO_PARENT,
        .on_enter = TrackLine_OnEnter,
        .on_exit = NULL,
        .on_event = TrackLine_OnEvent
    },
    [STATE_TURN_LEFT] = {
        .parent = SM_NO_PARENT,
        .on_enter = TurnLeft_OnEnter,
        .on_exit = NULL,
        .on_event = NULL
    },
    [STATE_TURN_RIGHT] = {
        .parent = SM_NO_PARENT,
        .on_enter = TurnRight_OnEnter,
        .on_exit = NULL,
        .on_event = NULL
    },
    [STATE_U_TURN] = {
        .parent = SM_NO_PARENT,
        .on_enter = UTurn_OnEnter,
        .on_exit = NULL,
        .on_event = NULL
    },
    [STATE_OBSTACLE_STOP] = {
        .parent = SM_NO_PARENT,
        .on_enter = ObstacleStop_OnEnter,
        .on_exit = NULL,
        .on_event = NULL
    },
    [STATE_OBSTACLE_TURN] = {
        .parent = SM_NO_PARENT,
        .on_enter = ObstacleTurn_OnEnter,
        .on_exit = NULL,
        .on_event = NULL
    },
    [STATE_WALL_FOLLOW] = {
        .parent = SM_NO_PARENT,
        .on_enter = WallFollow_OnEnter,
        .on_exit = NULL,
        .on_event = NULL
    },
    [STATE_SEARCH_LINE] = {
        .parent = SM_NO_PARENT,
        .on_enter = SearchLine_OnEnter,
        .on_exit = NULL,
        .on_event = NULL
    },
    [STATE_STOP] = {
        .parent = SM_NO_PARENT,
        .on_enter = Stop_OnEnter,
        .on_exit = NULL,
        .on_event = NULL
    }
};

/* ══════════════════════════════════════════════════════════════
 *  导航决策逻辑
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 循线模式决策
 *
 * 优先级:
 *   1. 前方障碍物 → 停车+避障
 *   2. 十字路口 → 直行(或根据任务转弯)
 *   3. 丁字路口 → 直行(或根据任务转弯)
 *   4. 脱线 → 搜索黑线
 *   5. 正常循线
 */
static void LineTrack_Decision(void)
{
    StateId_t state = SM_GetState(&nav_sm);
    uint32_t ticks = SM_GetStateTicks(&nav_sm);

    /* 前方障碍物检测 */
    if (dist[ULTRASONIC_FRONT] > 0 && dist[ULTRASONIC_FRONT] < OBSTACLE_DIST_FRONT) {
        if (state == STATE_TRACK_LINE) {
            SM_Transition(&nav_sm, STATE_OBSTACLE_STOP);
            return;
        }
    }

    /* 状态自动转换 */
    switch (state) {
    case STATE_TRACK_LINE:
        /* 正常循线 */
        TrackLine_Control(TRACK_SPEED_NORMAL);

        /* 检测路口 */
        if (Grayscale_DetectCross()) {
            /* 十字路口: 默认直行, 任务可覆盖 */
            printf("检测到十字路口\n");
        }
        if (Grayscale_DetectTJunction()) {
            /* 丁字路口: 默认右转 */
            printf("检测到丁字路口, 右转\n");
            SM_Transition(&nav_sm, STATE_TURN_RIGHT);
        }
        if (Grayscale_IsOffTrack()) {
            printf("脱线, 搜索黑线\n");
            SM_Transition(&nav_sm, STATE_SEARCH_LINE);
        }
        break;

    case STATE_TURN_LEFT:
        Motor_TurnLeft(TRACK_SPEED_SLOW);
        /* 转弯完成后回到循线 */
        if (ticks > 500 && !Grayscale_IsOffTrack()) {
            SM_Transition(&nav_sm, STATE_TRACK_LINE);
        }
        break;

    case STATE_TURN_RIGHT:
        Motor_TurnRight(TRACK_SPEED_SLOW);
        if (ticks > 500 && !Grayscale_IsOffTrack()) {
            SM_Transition(&nav_sm, STATE_TRACK_LINE);
        }
        break;

    case STATE_U_TURN:
        Motor_UTurn(TRACK_SPEED_SLOW);
        if (ticks > 800 && !Grayscale_IsOffTrack()) {
            SM_Transition(&nav_sm, STATE_TRACK_LINE);
        }
        break;

    case STATE_OBSTACLE_STOP:
        Motor_Stop();
        if (ticks > 300) {
            /* 决定避障方向 */
            if (dist[ULTRASONIC_LEFT] > dist[ULTRASONIC_RIGHT]) {
                avoid_turn_right = 0;  /* 左边空间大, 左转 */
            } else {
                avoid_turn_right = 1;  /* 右边空间大, 右转 */
            }
            SM_Transition(&nav_sm, STATE_OBSTACLE_TURN);
        }
        break;

    case STATE_OBSTACLE_TURN:
        if (ticks > 600) {
            /* 转弯后进入沿墙模式 */
            SM_Transition(&nav_sm, STATE_WALL_FOLLOW);
        }
        break;

    case STATE_WALL_FOLLOW:
        {
            /* 选择沿哪面墙 */
            float side_dist = avoid_turn_right ? dist[ULTRASONIC_RIGHT] : dist[ULTRASONIC_LEFT];
            uint8_t wall_side = avoid_turn_right ? 1 : 0;
            WallFollow_Control(TRACK_SPEED_SLOW, side_dist, wall_side);

            /* 检测是否回到线上 */
            if (!Grayscale_IsOffTrack()) {
                printf("回到线上, 恢复循线\n");
                SM_Transition(&nav_sm, STATE_TRACK_LINE);
            }
            /* 前方再次有障碍 */
            if (dist[ULTRASONIC_FRONT] > 0 && dist[ULTRASONIC_FRONT] < OBSTACLE_DIST_FRONT) {
                SM_Transition(&nav_sm, STATE_OBSTACLE_STOP);
            }
        }
        break;

    case STATE_SEARCH_LINE:
        Motor_TurnRight(TRACK_SPEED_SLOW);
        /* 找到线则回到循线 */
        if (!Grayscale_IsOffTrack()) {
            SM_Transition(&nav_sm, STATE_TRACK_LINE);
        }
        /* 超时则掉头 */
        if (ticks > 2000) {
            SM_Transition(&nav_sm, STATE_U_TURN);
        }
        break;

    default:
        break;
    }
}

/* ══════════════════════════════════════════════════════════════
 *  OLED显示
 * ══════════════════════════════════════════════════════════════ */
static void Display_Update(void)
{
    static uint32_t last_update = 0;
    if (sys_tick_ms - last_update < 200) return;
    last_update = sys_tick_ms;

    OLED_Clear();

    /* 模式 */
    const char *mode_str[] = {"LINE", "OBS", "MAZE_R", "MAZE_L", "FREE"};
    OLED_ShowString(0, 0, (char *)mode_str[nav_mode], 12, 1);

    /* 状态 */
    const char *state_str[] = {
        "TRACK", "T_L", "T_R", "UTURN",
        "STOP", "OBS_T", "WALL", "SEARCH", "STOP"
    };
    StateId_t state = SM_GetState(&nav_sm);
    OLED_ShowString(60, 0, (char *)state_str[state], 12, 1);

    /* 超声波距离 */
    OLED_ShowString(0, 2, "F:", 12, 1);
    OLED_ShowFloat(16, 2, dist[ULTRASONIC_FRONT], 3, 1, 12, 1);
    OLED_ShowString(60, 2, "L:", 12, 1);
    OLED_ShowFloat(76, 2, dist[ULTRASONIC_LEFT], 3, 1, 12, 1);

    OLED_ShowString(0, 3, "R:", 12, 1);
    OLED_ShowFloat(16, 3, dist[ULTRASONIC_RIGHT], 3, 1, 12, 1);

    /* 循迹偏差 */
    OLED_ShowString(0, 4, "Err:", 12, 1);
    OLED_ShowNum(30, 4, (uint32_t)(track_error < 0 ? -track_error : track_error), 4, 12, 1);

    /* 灰度数字量 */
    OLED_ShowString(0, 5, "Gray:", 12, 1);
    OLED_ShowNum(40, 5, grayscale_digital, 3, 12, 1);

    OLED_Refresh();
}

/* ══════════════════════════════════════════════════════════════
 *  硬件初始化
 * ══════════════════════════════════════════════════════════════ */
static void Motor_Init_HW(void)
{
    MotorConfig cfg[MOTOR_MAX] = {
        [MOTOR_A] = {
            .port_in1 = GPIOA, .pin_in1 = DL_GPIO_PIN_0,
            .port_in2 = GPIOA, .pin_in2 = DL_GPIO_PIN_1,
            .pwm_timer = TIMA0, .pwm_channel = DL_TIMER_CC_0_INDEX,
            .pwm_period = PWM_PERIOD
        },
        [MOTOR_B] = {
            .port_in1 = GPIOA, .pin_in1 = DL_GPIO_PIN_2,
            .port_in2 = GPIOA, .pin_in2 = DL_GPIO_PIN_3,
            .pwm_timer = TIMA0, .pwm_channel = DL_TIMER_CC_3_INDEX,
            .pwm_period = PWM_PERIOD
        }
    };
    Motor_Init(cfg);

    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_4);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_4);
}

static void Encoder_Init_HW(void)
{
    EncoderGpioConfig cfg[ENCODER_GPIO_MAX] = {
        [ENCODER_GPIO_LEFT] = {
            .port = GPIOB, .pin_a = DL_GPIO_PIN_0, .pin_b = DL_GPIO_PIN_1,
            .inverted = 0
        },
        [ENCODER_GPIO_RIGHT] = {
            .port = GPIOB, .pin_a = DL_GPIO_PIN_4, .pin_b = DL_GPIO_PIN_5,
            .inverted = 1
        }
    };
    EncoderGpio_Init(cfg);
}

static void Ultrasonic_Init_HW(void)
{
    /* 只初始化前方超声波, 左右可后续添加 */
    Ultrasonic_Init(&ultrasonic_cfg[ULTRASONIC_FRONT]);
}

static void Grayscale_Init_HW(void)
{
    GrayscaleConfig cfg = {
        .addr_port = GPIOA,
        .addr_pin_0 = DL_GPIO_PIN_25,
        .addr_pin_1 = DL_GPIO_PIN_26,
        .addr_pin_2 = DL_GPIO_PIN_27,
        .adc = ADC0,
        .adc_channel = DL_ADC_INPUT_CHAN_0,
        .direction = 0
    };
    Grayscale_Init(&cfg);
}

static void PID_Init_All(void)
{
    PID_Param track_param = {
        .kp = 1.2f, .ki = 0.01f, .kd = 0.5f,
        .output_min = -300, .output_max = 300,
        .integral_max = 500.0f, .dead_zone = 20
    };
    PID_Init(&pid_track, &track_param);

    PID_Param wall_param = {
        .kp = 3.0f, .ki = 0.02f, .kd = 1.0f,
        .output_min = -200, .output_max = 200,
        .integral_max = 300.0f, .dead_zone = 1
    };
    PID_Init(&pid_wall, &wall_param);
}

/* ══════════════════════════════════════════════════════════════
 *  主函数
 * ══════════════════════════════════════════════════════════════ */
int main(void)
{
    System_Init();

    Motor_Init_HW();
    Encoder_Init_HW();
    Ultrasonic_Init_HW();
    Grayscale_Init_HW();
    PID_Init_All();

    OLED_Init(I2C_0_INST);
    OLED_Clear();
    OLED_ShowString(0, 0, "AutoNav Init", 16, 1);
    OLED_Refresh();

    /* 初始化状态机 */
    SM_Init(&nav_sm, nav_state_table, NAV_STATE_COUNT, STATE_TRACK_LINE, NULL);
    SM_Start(&nav_sm);

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    printf("=== 自主导航小车启动 ===\n");
    printf("模式: 循线导航\n");

    /* 等待启动按键 */
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_21,
                                      DL_GPIO_INVERSION_DISABLE,
                                      DL_GPIO_RESISTOR_PULL_UP,
                                      DL_GPIO_HYSTERESIS_DISABLE,
                                      DL_GPIO_WAKEUP_DISABLE);
    printf("按下PA18启动...\n");
    while (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_21) != 0) {
        DELAY_MS(100);
    }
    DELAY_MS(500);  /* 消抖 */
    printf("启动!\n");

    /* ── 主循环 ─────────────────────────────────────────── */
    uint32_t last_sensor_tick = 0;

    while (1) {
        /* 每50ms更新传感器 */
        if (sys_tick_ms - last_sensor_tick >= 50) {
            last_sensor_tick = sys_tick_ms;
            Update_Sensors();
        }

        /* 导航决策 */
        LineTrack_Decision();

        /* OLED显示 */
        Display_Update();

        DELAY_MS(10);
    }

    return 0;
}
