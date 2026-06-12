/**
 * @file    bluetooth_remote_car.c
 * @brief   蓝牙遥控小车完整示例 — MSPM0G3507
 *
 * 功能概述:
 *   1. 手机APP通过HC-05蓝牙发送控制指令
 *   2. 支持前进/后退/左转/右转/停止
 *   3. 速度实时调节 (10级)
 *   4. 模式切换: 手动遥控 / 自动巡线 / 自动避障
 *   5. OLED实时显示状态信息
 *   6. 蓝牙回传速度、模式等状态
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │                    硬件接线说明                           │
 * ├──────────────────────────────────────────────────────────┤
 * │ TB6612 电机驱动:                                         │
 * │   MSPM0 PA0 → AIN1    PA1 → AIN2   PA12(PWM) → PWMA   │
 * │   MSPM0 PA2 → BIN1    PA3 → BIN2   PA13(PWM) → PWMB   │
 * │                                                          │
 * │ HC-05 蓝牙 (UART1):                                     │
 * │   MSPM0 PA17(U1TX) → HC-05 RX                          │
 * │   MSPM0 PA18(U1RX) ← HC-05 TX                          │
 * │   MSPM0 PA16       → HC-05 EN (低=透传模式)              │
 * │                                                          │
 * │ OLED (I2C0):                                             │
 * │   MSPM0 PB2(SCL) → OLED SCL                             │
 * │   MSPM0 PB3(SDA) → OLED SDA                             │
 * │                                                          │
 * │ 状态LED:                                                 │
 * │   MSPM0 PA22 → LED (模式指示)                            │
 * └──────────────────────────────────────────────────────────┘
 *
 * 蓝牙协议 (ASCII单字符):
 *   'F' = 前进    'B' = 后退    'L' = 左转    'R' = 右转
 *   'S' = 停止    '+' = 加速    '-' = 减速
 *   '1' = 手动模式  '2' = 巡线模式  '3' = 避障模式
 *
 * 依赖驱动: motor_mspm0, bluetooth_hc05_mspm0, oled_ssd1306_mspm0,
 *           encoder_gpio_mspm0, advanced_pid, pin_config
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
#include "drivers/bluetooth_hc05_mspm0.h"
#include "drivers/oled_ssd1306_mspm0.h"
#include "drivers/encoder_gpio_mspm0.h"
#include "drivers/advanced_pid.h"
#include "drivers/pin_config.h"

/* ══════════════════════════════════════════════════════════════
 *  配置参数
 * ══════════════════════════════════════════════════════════════ */

/* 速度档位配置 */
#define SPEED_MIN           100     /* 最低速度 */
#define SPEED_MAX           900     /* 最高速度 */
#define SPEED_STEP          100     /* 每档步进 */
#define SPEED_DEFAULT       400     /* 默认速度 */
#define PWM_PERIOD          1000    /* PWM周期 */

/* 转弯差速比例 (0.0~1.0) */
#define TURN_RATIO          0.6f

/* 蓝牙接收缓冲区 */
#define BT_BUF_SIZE         64

/* OLED刷新间隔 (ms) */
#define OLED_REFRESH_MS     200

/* ══════════════════════════════════════════════════════════════
 *  运行模式枚举
 * ══════════════════════════════════════════════════════════════ */

typedef enum {
    MODE_MANUAL = 0,        /* 手动遥控模式 */
    MODE_LINE_FOLLOW,       /* 自动巡线模式 */
    MODE_OBSTACLE_AVOID,    /* 自动避障模式 */
    MODE_COUNT
} RunMode;

/* 运动方向枚举 */
typedef enum {
    MOVE_STOP = 0,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT
} MoveDir;

/* ══════════════════════════════════════════════════════════════
 *  全局变量
 * ══════════════════════════════════════════════════════════════ */

/* 当前运行状态 */
static volatile RunMode   g_mode = MODE_MANUAL;
static volatile MoveDir   g_dir  = MOVE_STOP;
static volatile int16_t   g_speed = SPEED_DEFAULT;

/* 蓝牙接收缓冲 */
static uint8_t  g_bt_buf[BT_BUF_SIZE];
static uint16_t g_bt_len = 0;

/* OLED刷新计时 */
static volatile uint32_t  g_sys_tick = 0;
static uint32_t  g_last_oled_tick = 0;

/* 状态字符串 */
static const char *mode_names[] = {"Manual", "LineFol", "ObsAvoi"};
static const char *dir_names[]  = {"STOP", "FWD", "BWD", "LEFT", "RIGHT"};

/* ══════════════════════════════════════════════════════════════
 *  硬件配置结构 (根据pin_config.h)
 * ══════════════════════════════════════════════════════════════ */

/* TB6612电机配置 */
static const MotorConfig motor_cfg[MOTOR_MAX] = {
    [MOTOR_A] = {
        .port_in1   = PIN_TB6612_PORT,
        .pin_in1    = PIN_TB6612_AIN1,
        .port_in2   = PIN_TB6612_PORT,
        .pin_in2    = PIN_TB6612_AIN2,
        .pwm_timer  = PIN_TB6612_PWM_TIM,
        .pwm_channel = PIN_TB6612_PWM_C0_IDX,
        .pwm_period  = PWM_PERIOD
    },
    [MOTOR_B] = {
        .port_in1   = PIN_TB6612_PORT,
        .pin_in1    = PIN_TB6612_BIN1,
        .port_in2   = PIN_TB6612_PORT,
        .pin_in2    = PIN_TB6612_BIN2,
        .pwm_timer  = PIN_TB6612_PWM_TIM,
        .pwm_channel = PIN_TB6612_PWM_C3_IDX,
        .pwm_period  = PWM_PERIOD
    }
};

/* HC-05蓝牙配置 */
static const BT_HC05_Config bt_cfg = {
    .uart       = UART_1_INST,
    .state_port = PIN_BT_EN_PORT,
    .state_pin  = PIN_BT_EN_PIN,
    .baudrate   = 9600
};

/* ══════════════════════════════════════════════════════════════
 *  运动控制函数
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 设置小车运动方向和速度
 * @param dir    运动方向
 * @param speed  速度值 (0~PWM_PERIOD)
 */
static void Car_Move(MoveDir dir, int16_t speed)
{
    /* 速度限幅 — 错误经验#1: 除零保护不适用，但限幅防越界 */
    if (speed < 0) speed = 0;
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;

    int16_t turn_speed = (int16_t)(speed * TURN_RATIO);
    /* TURN_RATIO为常量，无除零风险 */

    switch (dir) {
    case MOVE_FORWARD:
        Motor_SetSpeed(MOTOR_A,  speed);
        Motor_SetSpeed(MOTOR_B,  speed);
        break;
    case MOVE_BACKWARD:
        Motor_SetSpeed(MOTOR_A, -speed);
        Motor_SetSpeed(MOTOR_B, -speed);
        break;
    case MOVE_LEFT:
        Motor_SetSpeed(MOTOR_A,  turn_speed);   /* 左轮慢 */
        Motor_SetSpeed(MOTOR_B,  speed);         /* 右轮快 */
        break;
    case MOVE_RIGHT:
        Motor_SetSpeed(MOTOR_A,  speed);         /* 左轮快 */
        Motor_SetSpeed(MOTOR_B,  turn_speed);    /* 右轮慢 */
        break;
    case MOVE_STOP:
    default:
        Motor_SetSpeed(MOTOR_A, 0);
        Motor_SetSpeed(MOTOR_B, 0);
        break;
    }
}

/**
 * @brief 紧急停车
 */
static void Car_EmergencyStop(void)
{
    Motor_Brake(MOTOR_A);
    Motor_Brake(MOTOR_B);
    g_dir = MOVE_STOP;
}

/* ══════════════════════════════════════════════════════════════
 *  蓝牙指令解析
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 解析蓝牙接收到的单字符指令
 * @param cmd  指令字符
 */
static void Parse_BtCommand(uint8_t cmd)
{
    switch (cmd) {
    /* ── 运动控制 ── */
    case 'F': case 'f':     /* 前进 */
        g_dir = MOVE_FORWARD;
        break;
    case 'B': case 'b':     /* 后退 */
        g_dir = MOVE_BACKWARD;
        break;
    case 'L': case 'l':     /* 左转 */
        g_dir = MOVE_LEFT;
        break;
    case 'R': case 'r':     /* 右转 */
        g_dir = MOVE_RIGHT;
        break;
    case 'S': case 's':     /* 停止 */
        g_dir = MOVE_STOP;
        Car_EmergencyStop();
        break;

    /* ── 速度调节 ── */
    case '+':               /* 加速 */
        g_speed += SPEED_STEP;
        if (g_speed > SPEED_MAX) g_speed = SPEED_MAX;
        break;
    case '-':               /* 减速 */
        g_speed -= SPEED_STEP;
        if (g_speed < SPEED_MIN) g_speed = SPEED_MIN;
        break;

    /* ── 模式切换 ── */
    case '1':               /* 手动模式 */
        g_mode = MODE_MANUAL;
        Car_EmergencyStop();
        break;
    case '2':               /* 巡线模式 */
        g_mode = MODE_LINE_FOLLOW;
        break;
    case '3':               /* 避障模式 */
        g_mode = MODE_OBSTACLE_AVOID;
        break;

    default:
        /* 未知指令，忽略 */
        break;
    }
}

/* ══════════════════════════════════════════════════════════════
 *  蓝牙数据处理
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 轮询检查蓝牙接收并解析指令
 *
 * 错误经验#5: BT_HC05_Data中的rx_flag在ISR中设置，
 * 已在驱动头文件中声明为volatile，此处安全读取。
 */
static void BT_Process(void)
{
    if (BT_HC05_IsDataReceived()) {
        g_bt_len = BT_HC05_GetReceivedData(g_bt_buf, BT_BUF_SIZE);
        /* 错误经验#7: 缓冲区长度已在GetReceivedData中校验 */

        for (uint16_t i = 0; i < g_bt_len; i++) {
            Parse_BtCommand(g_bt_buf[i]);
        }
        BT_HC05_ClearRxBuffer();
    }
}

/**
 * @brief 蓝牙回传当前状态
 */
static void BT_SendStatus(void)
{
    char status[64];
    int len = snprintf(status, sizeof(status),
                       "M:%s D:%s SPD:%d\r\n",
                       mode_names[g_mode],
                       dir_names[g_dir],
                       g_speed);
    /* 错误经验#7: snprintf限制长度，不会溢出 */
    if (len > 0 && len < (int)sizeof(status)) {
        BT_HC05_SendString(status);
    }
}

/* ══════════════════════════════════════════════════════════════
 *  OLED显示
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 更新OLED显示内容
 */
static void OLED_UpdateDisplay(void)
{
    OLED_Clear();

    /* 第1行: 模式 */
    OLED_ShowString(0, 0, (char *)"Mode:", 12, 1);
    OLED_ShowString(48, 0, (char *)mode_names[g_mode], 12, 1);

    /* 第2行: 方向 */
    OLED_ShowString(0, 16, (char *)"Dir: ", 12, 1);
    OLED_ShowString(48, 16, (char *)dir_names[g_dir], 12, 1);

    /* 第3行: 速度 */
    OLED_ShowString(0, 32, (char *)"Speed:", 12, 1);
    OLED_ShowNum(60, 32, (uint32_t)g_speed, 4, 12, 1);

    /* 第4行: 蓝牙连接状态 */
    OLED_ShowString(0, 48, (char *)"BT:", 12, 1);
    if (BT_HC05_IsConnected()) {
        OLED_ShowString(30, 48, (char *)"Connected ", 12, 1);
    } else {
        OLED_ShowString(30, 48, (char *)"Waiting... ", 12, 1);
    }

    OLED_Refresh();
}

/* ══════════════════════════════════════════════════════════════
 *  系统初始化
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 初始化所有外设
 */
static void System_Init(void)
{
    /* 初始化系统时钟 — SYSCFG_DL_init由SysConfig生成 */
    SYSCFG_DL_init();

    /* 初始化电机驱动 (TIMA0, PA12/PA13 PWM) */
    Motor_Init(motor_cfg);

    /* 初始化蓝牙 (UART1, PA17/PA18) */
    BT_HC05_Init(&bt_cfg);

    /* 初始化OLED (I2C0, PB2/PB3) */
    OLED_Init(I2C_0_INST);
    OLED_Clear();
    OLED_ShowString(20, 24, (char *)"BT Car Ready", 16, 1);
    OLED_Refresh();

    /* 配置LED引脚 (PA22) 作为模式指示 */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_22);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_22);
}

/* ══════════════════════════════════════════════════════════════
 *  主函数
 * ══════════════════════════════════════════════════════════════ */

int main(void)
{
    System_Init();

    /* 主循环 */
    while (1) {
        /* 1. 处理蓝牙接收 */
        BT_Process();

        /* 2. 根据模式执行运动控制 */
        if (g_mode == MODE_MANUAL) {
            /* 手动模式: 直接执行遥控指令 */
            if (g_dir != MOVE_STOP) {
                Car_Move(g_dir, g_speed);
            } else {
                Car_EmergencyStop();
            }
        }
        /*
         * MODE_LINE_FOLLOW 和 MODE_OBSTACLE_AVOID
         * 可在此扩展: 集成灰度传感器/超声波传感器驱动
         * 参见 line_following_pid.c 和 obstacle_avoidance_robot.c
         */

        /* 3. 定期更新OLED (避免过于频繁刷新) */
        /* 错误经验: g_sys_tick在定时器ISR中递增，已声明volatile */
        if ((g_sys_tick - g_last_oled_tick) >= OLED_REFRESH_MS) {
            g_last_oled_tick = g_sys_tick;
            OLED_UpdateDisplay();
            BT_SendStatus();
        }

        /* 4. 短延时，降低CPU负载 */
        delay_cycles(32000);   /* ~1ms @32MHz */
        g_sys_tick++;
    }
}
