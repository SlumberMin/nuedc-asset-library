/**
 * @file    color_sorting_demo.c
 * @brief   颜色分拣系统示例 — MSPM0G3507
 *
 * 功能概述:
 *   1. TCS34725颜色传感器识别物体颜色 (红/绿/蓝/白/黑)
 *   2. PCA9685驱动多路舵机执行分拣动作
 *   3. TB6612驱动传送带电机
 *   4. 状态机管理分拣流程
 *   5. OLED显示实时颜色和分拣统计
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │                    硬件接线说明                           │
 * ├──────────────────────────────────────────────────────────┤
 * │ I2C总线 (共用):                                          │
 * │   MSPM0 PB2 → SCL (需4.7K上拉)                          │
 * │   MSPM0 PB3 → SDA (需4.7K上拉)                          │
 * │   TCS34725 I2C地址: 0x29                                 │
 * │   PCA9685  I2C地址: 0x40                                 │
 * │                                                          │
 * │ TCS34725 颜色传感器:                                     │
 * │   VCC → 3.3V    GND → GND                               │
 * │   SCL → PB2     SDA → PB3                               │
 * │   LED → 3.3V (板载LED使能) 或 NC (关闭LED)               │
 * │                                                          │
 * │ PCA9685 舵机驱动板:                                      │
 * │   VCC → 5V (舵机电源)   GND → GND                       │
 * │   SCL → PB2             SDA → PB3                        │
 * │   V+ → 外部电源 (舵机供电, 如5V/2A)                      │
 * │   CH0 → 分拣舵机1 (左)                                  │
 * │   CH1 → 分拣舵机2 (右)                                  │
 * │   CH2 → 入料舵机 (可选)                                  │
 * │                                                          │
 * │ TB6612 传送带电机:                                       │
 * │   MSPM0 PA0 → AIN1   PA1 → AIN2   PA12(PWM) → PWMA    │
 * │   MSPM0 PA4 → STBY                                      │
 * │                                                          │
 * │ 光电传感器 (检测物体到位, 可选):                          │
 * │   MSPM0 PB4 → 光电传感器输出 (低电平=有物体)              │
 * │                                                          │
 * │ OLED 显示:                                               │
 * │   MSPM0 PB2(SCL) → SCL   PB3(SDA) → SDA                │
 * │   (与TCS34725/PCA9685共享I2C总线)                        │
 * └──────────────────────────────────────────────────────────┘
 *
 * 工作流程:
 *   [空闲] → 物体到位 → [读取颜色] → 判断颜色 → [分拣动作] → [统计] → [空闲]
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/tcs34725.h"
#include "drivers/pca9685.h"
#include "drivers/motor_mspm0.h"
#include "drivers/oled_ssd1306_mspm0.h"
#include "drivers/state_machine.h"

/* ══════════════════════════════════════════════════════════════
 *  配置参数
 * ══════════════════════════════════════════════════════════════ */

/* 舵机通道 */
#define SERVO_SORT_LEFT     0   /* 左侧分拣舵机 (PCA9685 CH0) */
#define SERVO_SORT_RIGHT    1   /* 右侧分拣舵机 (PCA9685 CH1) */
#define SERVO_GATE          2   /* 入料口闸门舵机 (PCA9685 CH2) */

/* 舵机角度 */
#define SERVO_IDLE_ANGLE    90  /* 中位(不动作) */
#define SERVO_LEFT_OPEN     45  /* 左侧分拣角度 */
#define SERVO_RIGHT_OPEN    135 /* 右侧分拣角度 */
#define SERVO_GATE_OPEN     30  /* 闸门打开 */
#define SERVO_GATE_CLOSE    90  /* 闸门关闭 */

/* 传送带速度 */
#define CONVEYOR_SPEED      300
#define CONVEYOR_PWM_PERIOD 1000

/* 光电传感器 */
#define SENSOR_PORT         GPIOB
#define SENSOR_PIN          DL_GPIO_PIN_4

/* 颜色分类阈值 (根据实际环境校准) */
/* 判据: R/G/B各通道占总和的比例 */
#define RED_DOMINANT_THRESH     0.40f   /* R/(R+G+B) > 0.4 → 红色 */
#define GREEN_DOMINANT_THRESH   0.40f   /* G/(R+G+B) > 0.4 → 绿色 */
#define BLUE_DOMINANT_THRESH    0.40f   /* B/(R+G+B) > 0.4 → 蓝色 */
#define WHITE_THRESH            500     /* Clear通道 > 500 → 白色 */
#define BLACK_THRESH            50      /* Clear通道 < 50 → 黑色 */

/* ══════════════════════════════════════════════════════════════
 *  颜色枚举
 * ══════════════════════════════════════════════════════════════ */
typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_WHITE,
    COLOR_BLACK,
    COLOR_COUNT   /* 颜色种类数 */
} ColorType;

static const char *color_names[] = {
    "???", "RED", "GRN", "BLU", "WHT", "BLK"
};

/* ══════════════════════════════════════════════════════════════
 *  状态机定义
 * ══════════════════════════════════════════════════════════════ */

/* 状态ID */
enum {
    STATE_IDLE = 0,         /* 空闲等待 */
    STATE_DETECT,           /* 检测颜色 */
    STATE_SORT,             /* 执行分拣 */
    STATE_WAIT_DROP,        /* 等待物体掉落 */
    STATE_RESET,            /* 舵机复位 */
    STATE_COUNT
};

/* 事件ID */
enum {
    EVENT_OBJECT_ARRIVED = 1,   /* 物体到位 */
    EVENT_COLOR_READY,          /* 颜色识别完成 */
    EVENT_SORT_DONE,            /* 分拣动作完成 */
    EVENT_DROP_DONE,            /* 掉落完成 */
    EVENT_RESET_DONE            /* 复位完成 */
};

/* ══════════════════════════════════════════════════════════════
 *  全局变量
 * ══════════════════════════════════════════════════════════════ */

static SM_Machine sorter_sm;    /* 分拣状态机 */
static volatile uint32_t sys_tick_ms = 0;

/* 当前检测到的颜色 */
static ColorType detected_color = COLOR_UNKNOWN;

/* TCS34725 数据 */
static TCS34725_RGBC rgbc_data;

/* 分拣统计 */
static uint32_t sort_count[COLOR_COUNT] = {0};  /* 各颜色分拣计数 */
static uint32_t total_sorted = 0;

/* 定时器 */
static uint32_t sort_start_tick = 0;

/* ══════════════════════════════════════════════════════════════
 *  定时器中断
 * ══════════════════════════════════════════════════════════════ */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        sys_tick_ms++;
        SM_Tick(&sorter_sm);  /* 状态机时钟 */
    }
}

/* ══════════════════════════════════════════════════════════════
 *  传送带控制
 * ══════════════════════════════════════════════════════════════ */
static void Conveyor_Start(void)
{
    Motor_SetSpeed(MOTOR_A, CONVEYOR_SPEED);
}

static void Conveyor_Stop(void)
{
    Motor_SetSpeed(MOTOR_A, 0);
}

/* ══════════════════════════════════════════════════════════════
 *  舵机动作
 * ══════════════════════════════════════════════════════════════ */
static void SortServo_Idle(void)
{
    PCA9685_SetAngle(SERVO_SORT_LEFT, SERVO_IDLE_ANGLE);
    PCA9685_SetAngle(SERVO_SORT_RIGHT, SERVO_IDLE_ANGLE);
}

static void SortServo_Direct(ColorType color)
{
    /* 根据颜色选择分拣方向 */
    switch (color) {
    case COLOR_RED:
        PCA9685_SetAngle(SERVO_SORT_LEFT, SERVO_LEFT_OPEN);
        break;
    case COLOR_GREEN:
        PCA9685_SetAngle(SERVO_SORT_RIGHT, SERVO_RIGHT_OPEN);
        break;
    case COLOR_BLUE:
        PCA9685_SetAngle(SERVO_SORT_LEFT, SERVO_LEFT_OPEN);
        PCA9685_SetAngle(SERVO_SORT_RIGHT, SERVO_RIGHT_OPEN);
        break;
    default:
        /* 白色/黑色/未知 → 不动作, 直接通过 */
        break;
    }
}

/* ══════════════════════════════════════════════════════════════
 *  颜色识别算法
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 根据RGBC数据判断颜色
 *
 * 算法:
 *   1. 先检查Clear通道判断黑白
 *   2. 计算R/G/B各通道占比
 *   3. 占比最大的通道即为主色
 */
static ColorType Identify_Color(const TCS34725_RGBC *data)
{
    /* 黑白判断 */
    if (data->clear < BLACK_THRESH) {
        return COLOR_BLACK;
    }
    if (data->clear > WHITE_THRESH &&
        data->red > 400 && data->green > 400 && data->blue > 400) {
        return COLOR_WHITE;
    }

    /* 归一化: 计算各通道占比 */
    float total = (float)(data->red + data->green + data->blue);
    if (total < 1.0f) return COLOR_UNKNOWN;

    float r_ratio = (float)data->red / total;
    float g_ratio = (float)data->green / total;
    float b_ratio = (float)data->blue / total;

    /* 判断主色 */
    if (r_ratio > RED_DOMINANT_THRESH && r_ratio > g_ratio && r_ratio > b_ratio) {
        return COLOR_RED;
    }
    if (g_ratio > GREEN_DOMINANT_THRESH && g_ratio > r_ratio && g_ratio > b_ratio) {
        return COLOR_GREEN;
    }
    if (b_ratio > BLUE_DOMINANT_THRESH && b_ratio > r_ratio && b_ratio > g_ratio) {
        return COLOR_BLUE;
    }

    return COLOR_UNKNOWN;
}

/* ══════════════════════════════════════════════════════════════
 *  光电传感器检测
 * ══════════════════════════════════════════════════════════════ */
static uint8_t IsObjectPresent(void)
{
    /* 低电平=有物体 (根据传感器型号可能相反) */
    return (DL_GPIO_readPins(SENSOR_PORT, SENSOR_PIN) == 0) ? 1 : 0;
}

/* ══════════════════════════════════════════════════════════════
 *  状态机回调函数
 * ══════════════════════════════════════════════════════════════ */

/* ── IDLE: 空闲等待 ─────────────────────────────────────── */
static bool State_Idle_OnEvent(SM_Machine *sm, const SM_Event_t *event)
{
    (void)sm;
    if (event->id == EVENT_OBJECT_ARRIVED) {
        Conveyor_Stop();  /* 暂停传送带 */
        return true;
    }
    return false;
}

static bool State_Idle_OnEnter(SM_Machine *sm)
{
    (void)sm;
    Conveyor_Start();  /* 启动传送带 */
    return true;
}

/* ── DETECT: 检测颜色 ──────────────────────────────────── */
static bool State_Detect_OnEnter(SM_Machine *sm)
{
    (void)sm;
    /* 多次采样取平均 */
    TCS34725_RGBC avg = {0};
    const int samples = 5;
    for (int i = 0; i < samples; i++) {
        TCS34725_RGBC tmp;
        if (TCS34725_ReadRGBC(&tmp)) {
            avg.clear += tmp.clear;
            avg.red   += tmp.red;
            avg.green += tmp.green;
            avg.blue  += tmp.blue;
        }
        DELAY_MS(10);
    }
    avg.clear /= samples;
    avg.red   /= samples;
    avg.green /= samples;
    avg.blue  /= samples;

    rgbc_data = avg;
    detected_color = Identify_Color(&avg);

    printf("颜色检测: R=%d G=%d B=%d C=%d → %s\n",
           avg.red, avg.green, avg.blue, avg.clear,
           color_names[detected_color]);

    /* 检测完成, 发送事件 */
    SM_Event_t evt = { .id = EVENT_COLOR_READY };
    SM_Dispatch(sm, &evt);
    return true;
}

/* ── SORT: 执行分拣 ─────────────────────────────────────── */
static bool State_Sort_OnEnter(SM_Machine *sm)
{
    (void)sm;
    sort_start_tick = sys_tick_ms;
    SortServo_Direct(detected_color);
    Conveyor_Start();  /* 重新启动传送带 */
    return true;
}

static bool State_Sort_OnEvent(SM_Machine *sm, const SM_Event_t *event)
{
    (void)sm;
    if (event->id == EVENT_SORT_DONE) {
        return true;
    }
    return false;
}

/* ── WAIT_DROP: 等待物体掉落 ────────────────────────────── */
static bool State_WaitDrop_OnEnter(SM_Machine *sm)
{
    (void)sm;
    sort_start_tick = sys_tick_ms;
    return true;
}

static bool State_WaitDrop_OnEvent(SM_Machine *sm, const SM_Event_t *event)
{
    (void)sm;
    /* 超时或物体离开传感器 */
    if (event->id == EVENT_DROP_DONE ||
        (sys_tick_ms - sort_start_tick > 1000)) {
        /* 更新统计 */
        if (detected_color < COLOR_COUNT) {
            sort_count[detected_color]++;
        }
        total_sorted++;
        return true;
    }
    return false;
}

/* ── RESET: 舵机复位 ────────────────────────────────────── */
static bool State_Reset_OnEnter(SM_Machine *sm)
{
    (void)sm;
    SortServo_Idle();
    sort_start_tick = sys_tick_ms;
    return true;
}

static bool State_Reset_OnEvent(SM_Machine *sm, const SM_Event_t *event)
{
    (void)sm;
    (void)event;
    /* 等待200ms让舵机到位 */
    if (sys_tick_ms - sort_start_tick > 200) {
        return true;
    }
    return false;
}

/* ══════════════════════════════════════════════════════════════
 *  状态描述表
 * ══════════════════════════════════════════════════════════════ */
static const SM_StateDesc_t state_table[STATE_COUNT] = {
    [STATE_IDLE] = {
        .parent = SM_NO_PARENT,
        .on_enter = State_Idle_OnEnter,
        .on_exit = NULL,
        .on_event = State_Idle_OnEvent
    },
    [STATE_DETECT] = {
        .parent = SM_NO_PARENT,
        .on_enter = State_Detect_OnEnter,
        .on_exit = NULL,
        .on_event = NULL
    },
    [STATE_SORT] = {
        .parent = SM_NO_PARENT,
        .on_enter = State_Sort_OnEnter,
        .on_exit = NULL,
        .on_event = State_Sort_OnEvent
    },
    [STATE_WAIT_DROP] = {
        .parent = SM_NO_PARENT,
        .on_enter = State_WaitDrop_OnEnter,
        .on_exit = NULL,
        .on_event = State_WaitDrop_OnEvent
    },
    [STATE_RESET] = {
        .parent = SM_NO_PARENT,
        .on_enter = State_Reset_OnEnter,
        .on_exit = NULL,
        .on_event = State_Reset_OnEvent
    }
};

/* ══════════════════════════════════════════════════════════════
 *  OLED显示
 * ══════════════════════════════════════════════════════════════ */
static void Display_Update(void)
{
    static uint32_t last_update = 0;
    if (sys_tick_ms - last_update < 300) return;
    last_update = sys_tick_ms;

    OLED_Clear();

    /* 标题 */
    OLED_ShowString(0, 0, "Color Sorter", 16, 1);

    /* 当前颜色 */
    OLED_ShowString(0, 2, "Color:", 12, 1);
    OLED_ShowString(50, 2, (char *)color_names[detected_color], 12, 1);

    /* RGBC原始值 */
    OLED_ShowString(0, 3, "R:", 12, 1);
    OLED_ShowNum(16, 3, rgbc_data.red, 4, 12, 1);
    OLED_ShowString(56, 3, "G:", 12, 1);
    OLED_ShowNum(72, 3, rgbc_data.green, 4, 12, 1);

    /* 统计 */
    OLED_ShowString(0, 5, "Total:", 12, 1);
    OLED_ShowNum(50, 5, total_sorted, 5, 12, 1);

    OLED_ShowString(0, 6, "R:", 12, 1);
    OLED_ShowNum(16, 6, sort_count[COLOR_RED], 3, 12, 1);
    OLED_ShowString(40, 6, "G:", 12, 1);
    OLED_ShowNum(56, 6, sort_count[COLOR_GREEN], 3, 12, 1);
    OLED_ShowString(80, 6, "B:", 12, 1);
    OLED_ShowNum(96, 6, sort_count[COLOR_BLUE], 3, 12, 1);

    OLED_Refresh();
}

/* ══════════════════════════════════════════════════════════════
 *  主函数
 * ══════════════════════════════════════════════════════════════ */
int main(void)
{
    System_Init();

    /* 初始化I2C总线上的设备 */
    if (!TCS34725_Init()) {
        printf("TCS34725初始化失败!\n");
        while (1);
    }
    printf("TCS34725 OK\n");

    if (!PCA9685_Init()) {
        printf("PCA9685初始化失败!\n");
        while (1);
    }
    printf("PCA9685 OK\n");

    /* 舵机初始位置 */
    SortServo_Idle();

    /* 初始化电机 (传送带) */
    MotorConfig motor_cfg[MOTOR_MAX] = {
        [MOTOR_A] = {
            .port_in1 = GPIOA, .pin_in1 = DL_GPIO_PIN_0,
            .port_in2 = GPIOA, .pin_in2 = DL_GPIO_PIN_1,
            .pwm_timer = TIMA0, .pwm_channel = DL_TIMER_CC_0_INDEX,
            .pwm_period = CONVEYOR_PWM_PERIOD
        }
    };
    Motor_Init(motor_cfg);

    /* STBY使能 */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_4);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_4);

    /* 光电传感器输入 */
    DL_GPIO_initDigitalInputFeatures(SENSOR_PORT, SENSOR_PIN,
                                      DL_GPIO_INVERSION_DISABLE,
                                      DL_GPIO_RESISTOR_PULL_UP,
                                      DL_GPIO_HYSTERESIS_DISABLE,
                                      DL_GPIO_WAKEUP_DISABLE);

    /* OLED初始化 */
    OLED_Init(I2C_0_INST);
    OLED_Clear();
    OLED_ShowString(0, 0, "Color Sorter Init", 12, 1);
    OLED_Refresh();

    /* 初始化状态机 */
    SM_Init(&sorter_sm, state_table, STATE_COUNT, STATE_IDLE, NULL);
    SM_Start(&sorter_sm);

    /* 使能定时器中断 */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    printf("=== 颜色分拣系统启动 ===\n");

    /* ── 主循环 ─────────────────────────────────────────── */
    uint8_t prev_object = 0;

    while (1) {
        /* 检测物体到位 (上升沿触发) */
        uint8_t now_object = IsObjectPresent();
        if (now_object && !prev_object) {
            SM_Event_t evt = { .id = EVENT_OBJECT_ARRIVED };
            SM_Dispatch(&sorter_sm, &evt);
        }
        prev_object = now_object;

        /* 状态机超时/自动转换处理 */
        StateId_t state = SM_GetState(&sorter_sm);
        uint32_t ticks = SM_GetStateTicks(&sorter_sm);

        switch (state) {
        case STATE_DETECT:
            /* 检测完成后自动转到SORT */
            if (detected_color != COLOR_UNKNOWN) {
                SM_Transition(&sorter_sm, STATE_SORT);
            } else {
                /* 未识别, 直接回IDLE */
                SM_Transition(&sorter_sm, STATE_IDLE);
            }
            break;

        case STATE_SORT:
            /* 等待舵机动作完成 (500ms) */
            if (ticks > 500) {
                SM_Transition(&sorter_sm, STATE_WAIT_DROP);
            }
            break;

        case STATE_WAIT_DROP:
            /* 物体已掉落或超时 */
            if (!now_object || ticks > 1500) {
                SM_Transition(&sorter_sm, STATE_RESET);
            }
            break;

        case STATE_RESET:
            /* 舵机复位等待 */
            if (ticks > 300) {
                SM_Transition(&sorter_sm, STATE_IDLE);
            }
            break;

        default:
            break;
        }

        /* OLED更新 */
        Display_Update();

        DELAY_MS(10);
    }

    return 0;
}
