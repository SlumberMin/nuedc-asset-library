/**
 * @file    balance_car_demo.c
 * @brief   两轮自平衡车完整示例 — MSPM0G3507
 *
 * 功能概述:
 *   1. JY901S IMU读取倾角 → 直立环PID
 *   2. 编码器读取轮速   → 速度环PID
 *   3. 编码器差速       → 转向环PID
 *   4. 三环输出叠加驱动电机
 *   5. HC-05蓝牙实时调参/监控
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │                    硬件接线说明                           │
 * ├──────────────────────────────────────────────────────────┤
 * │ JY901S IMU (UART):                                      │
 * │   MSPM0 PA9(U2RX) ← JY901S TX                          │
 * │   MSPM0 PA8(U2TX) → JY901S RX                          │
 * │                                                          │
 * │ TB6612 电机驱动:                                         │
 * │   MSPM0 PA0 → AIN1    PA1 → AIN2   PA12(PWM) → PWMA   │
 * │   MSPM0 PA2 → BIN1    PA3 → BIN2   PA13(PWM) → PWMB   │
 * │   MSPM0 PA4 → STBY (使能, 高电平有效)                    │
 * │                                                          │
 * │ N20 编码器电机:                                          │
 * │   MSPM0 PB0 → 左轮A相   PB1 → 左轮B相                   │
 * │   MSPM0 PB2 → 右轮A相   PB3 → 右轮B相                   │
 * │                                                          │
 * │ HC-05 蓝牙:                                             │
 * │   MSPM0 PA9(U1RX) ← HC-05 TX   (注意: 与IMU分时复用或   │
 * │   MSPM0 PA8(U1TX) → HC-05 RX    使用不同UART)           │
 * │   实际示例中IMU用UART0, 蓝牙用UART1                      │
 * │                                                          │
 * │ OLED (可选):                                             │
 * │   MSPM0 PB2(SCL) → OLED SCL                             │
 * │   MSPM0 PB3(SDA) → OLED SDA                             │
 * └──────────────────────────────────────────────────────────┘
 *
 * 依赖驱动: jy901s_mspm0, motor_mspm0, encoder_gpio_mspm0,
 *           bluetooth_hc05_mspm0, oled_ssd1306_mspm0, advanced_pid
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/jy901s_mspm0.h"
#include "drivers/motor_mspm0.h"
#include "drivers/encoder_gpio_mspm0.h"
#include "drivers/bluetooth_hc05_mspm0.h"
#include "drivers/advanced_pid.h"

/* ══════════════════════════════════════════════════════════════
 *  配置参数
 * ══════════════════════════════════════════════════════════════ */

/* 机械零点偏移 (度) — 需根据实际安装校准 */
#define BALANCE_ANGLE_OFFSET    0.0f

/* 三环PID参数初始值 (可通过蓝牙在线调整) */
#define BAL_KP                  45.0f   /* 直立环 Kp */
#define BAL_KI                  0.0f    /* 直立环 Ki */
#define BAL_KD                  28.0f   /* 直立环 Kd */

#define SPD_KP                  0.5f    /* 速度环 Kp */
#define SPD_KI                  0.005f  /* 速度环 Ki */
#define SPD_KD                  0.0f    /* 速度环 Kd */

#define TURN_KP                 2.0f    /* 转向环 Kp */
#define TURN_KI                 0.0f    /* 转向环 Ki */
#define TURN_KD                 0.5f    /* 转向环 Kd */

/* PWM限幅 */
#define PWM_MAX                 900
#define PWM_PERIOD              1000

/* 编码器采样周期 */
#define ENCODER_SAMPLE_MS       10

/* 蓝牙数据包发送间隔 (ms) */
#define BT_SEND_INTERVAL_MS     50

/* 蓝牙协议命令字符 */
#define CMD_SET_BAL_KP      'A'
#define CMD_SET_BAL_KD      'B'
#define CMD_SET_SPD_KP      'C'
#define CMD_SET_SPD_KI      'D'
#define CMD_SET_TURN_KP     'E'
#define CMD_SET_ANGLE_OFF   'F'
#define CMD_GET_STATUS      'S'
#define CMD_EMERGENCY_STOP  'X'

/* ══════════════════════════════════════════════════════════════
 *  全局变量
 * ══════════════════════════════════════════════════════════════ */

/* PID控制器实例 */
static PID_Controller pid_balance;     /* 直立环 */
static PID_Controller pid_speed;       /* 速度环 */
static PID_Controller pid_turn;        /* 转向环 */

/* 系统状态 */
static volatile uint32_t sys_tick_ms = 0;  /* 毫秒时钟 */
static volatile uint8_t  emergency_stop = 0; /* 急停标志 */

/* 速度积分 (用于速度环) */
static int32_t speed_integral = 0;

/* 转向目标 (0=直行, 正值=右转, 负值=左转) */
static volatile int16_t turn_target = 0;

/* 蓝牙接收缓冲 */
static uint8_t bt_rx_buf[64];
static uint16_t bt_rx_len = 0;

/* ══════════════════════════════════════════════════════════════
 *  定时器中断 — 1ms 系统心跳
 * ══════════════════════════════════════════════════════════════ */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        sys_tick_ms++;

        /* 每 ENCODER_SAMPLE_MS 更新编码器 */
        if (sys_tick_ms % ENCODER_SAMPLE_MS == 0) {
            EncoderGpio_Update();
        }
    }
}

/* ══════════════════════════════════════════════════════════════
 *  硬件初始化
 * ══════════════════════════════════════════════════════════════ */

/* 电机初始化 */
static void Motor_Init_HW(void)
{
    MotorConfig cfg[MOTOR_MAX] = {
        [MOTOR_A] = {  /* 左电机 */
            .port_in1 = GPIOA, .pin_in1 = DL_GPIO_PIN_0,
            .port_in2 = GPIOA, .pin_in2 = DL_GPIO_PIN_1,
            .pwm_timer = TIMA0, .pwm_channel = DL_TIMER_CC_0_INDEX,
            .pwm_period = PWM_PERIOD
        },
        [MOTOR_B] = {  /* 右电机 */
            .port_in1 = GPIOA, .pin_in1 = DL_GPIO_PIN_2,
            .port_in2 = GPIOA, .pin_in2 = DL_GPIO_PIN_3,
            .pwm_timer = TIMA0, .pwm_channel = DL_TIMER_CC_3_INDEX,
            .pwm_period = PWM_PERIOD
        }
    };
    Motor_Init(cfg);

    /* STBY使能引脚 */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_4);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_4);  /* 拉高使能 */
}

/* 编码器初始化 */
static void Encoder_Init_HW(void)
{
    EncoderGpioConfig cfg[ENCODER_GPIO_MAX] = {
        [ENCODER_GPIO_LEFT] = {
            .port = GPIOB, .pin_a = DL_GPIO_PIN_0, .pin_b = DL_GPIO_PIN_1,
            .inverted = 0
        },
        [ENCODER_GPIO_RIGHT] = {
            .port = GPIOB, .pin_a = DL_GPIO_PIN_2, .pin_b = DL_GPIO_PIN_3,
            .inverted = 1  /* 右轮安装方向相反 */
        }
    };
    EncoderGpio_Init(cfg);
}

/* IMU初始化 */
static void IMU_Init_HW(void)
{
    JY901S_Config cfg = {
        .uart = UART_0_INST,   /* UART0 用于IMU */
        .baudrate = 9600,
        .auto_calib = 1
    };
    JY901S_Init(&cfg);
}

/* 蓝牙初始化 */
static void BT_Init_HW(void)
{
    BT_HC05_Config cfg = {
        .uart = UART_1_INST,   /* UART1 用于蓝牙 */
        .state_port = GPIOA,
        .state_pin = DL_GPIO_PIN_7,
        .baudrate = 9600
    };
    BT_HC05_Init(&cfg);
}

/* PID控制器初始化 */
static void PID_Init_All(void)
{
    /* 直立环 — 需要快速响应, 中等微分 */
    PID_Param bal_param = {
        .kp = BAL_KP, .ki = BAL_KI, .kd = BAL_KD,
        .output_min = -PWM_MAX, .output_max = PWM_MAX,
        .integral_max = 300.0f, .dead_zone = 0
    };
    PID_Init(&pid_balance, &bal_param);

    /* 速度环 — 较慢响应, 需要积分消除稳态误差 */
    PID_Param spd_param = {
        .kp = SPD_KP, .ki = SPD_KI, .kd = SPD_KD,
        .output_min = -300, .output_max = 300,
        .integral_max = 2000.0f, .dead_zone = 3
    };
    PID_Init(&pid_speed, &spd_param);

    /* 转向环 — 纯比例即可 */
    PID_Param turn_param = {
        .kp = TURN_KP, .ki = TURN_KI, .kd = TURN_KD,
        .output_min = -200, .output_max = 200,
        .integral_max = 500.0f, .dead_zone = 0
    };
    PID_Init(&pid_turn, &turn_param);
}

/* ══════════════════════════════════════════════════════════════
 *  核心控制算法 — 三环PID
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 平衡车控制主函数 (每10ms调用一次)
 *
 * 控制框图:
 *   ┌─────────┐
 *   │ IMU角度 │──→ [直立环PID] ──→ +
 *   └─────────┘                     │
 *   ┌─────────┐                     ├──→ 限幅 → 左电机PWM
 *   │编码器速度│──→ [速度环PID] ──→ +        ├──→ 右电机PWM
 *   └─────────┘                     │
 *   ┌─────────┐                     │
 *   │转向指令 │──→ [转向环PID] ──→ ±
 *   └─────────┘
 *
 * 左电机PWM  = 直立 + 速度 + 转向
 * 右电机PWM  = 直立 + 速度 - 转向
 */
static void Balance_Control(void)
{
    /* 1. 读取IMU角度 */
    float pitch, roll, yaw;
    JY901S_GetAngle(&pitch, &roll, &yaw);
    float angle = pitch - BALANCE_ANGLE_OFFSET;

    /* 2. 读取编码器速度 */
    int32_t speed_left  = EncoderGpio_GetSpeed(ENCODER_GPIO_LEFT);
    int32_t speed_right = EncoderGpio_GetSpeed(ENCODER_GPIO_RIGHT);
    int32_t speed_avg   = (speed_left + speed_right) / 2;

    /* 3. 直立环 — 角度PID, 输出叠加到电机 */
    int16_t bal_output = (int16_t)PID_Calc(&pid_balance, 0.0f, angle);

    /* 4. 速度环 — 速度PID, 目标为0(保持静止) */
    int16_t spd_output = (int16_t)PID_Calc(&pid_speed, 0.0f, (float)speed_avg);

    /* 5. 转向环 — 转向PID, 使用yaw角或直接差速 */
    int16_t turn_output = turn_target; /* 直接使用遥控指令 */

    /* 6. 三环叠加 */
    int16_t pwm_left  = bal_output + spd_output + turn_output;
    int16_t pwm_right = bal_output + spd_output - turn_output;

    /* 7. PWM限幅 */
    if (pwm_left > PWM_MAX)   pwm_left = PWM_MAX;
    if (pwm_left < -PWM_MAX)  pwm_left = -PWM_MAX;
    if (pwm_right > PWM_MAX)  pwm_right = PWM_MAX;
    if (pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;

    /* 8. 倾角过大 → 紧急刹车 (车已倒下) */
    if (fabsf(angle) > 45.0f) {
        Motor_SetSpeed(MOTOR_A, 0);
        Motor_SetSpeed(MOTOR_B, 0);
        speed_integral = 0;  /* 清除积分 */
        return;
    }

    /* 9. 急停检查 */
    if (emergency_stop) {
        Motor_SetSpeed(MOTOR_A, 0);
        Motor_SetSpeed(MOTOR_B, 0);
        return;
    }

    /* 10. 输出到电机 */
    Motor_SetSpeed(MOTOR_A, pwm_left);
    Motor_SetSpeed(MOTOR_B, pwm_right);
}

/* ══════════════════════════════════════════════════════════════
 *  蓝牙协议解析 — 接收手机APP调参指令
 *
 *  协议格式: 单字符命令 + 浮点数值 + '\n'
 *  例: "A35.0\n" → 设置直立环Kp=35.0
 * ══════════════════════════════════════════════════════════════ */
static void BT_ParseCommand(void)
{
    if (!BT_HC05_IsDataReceived()) return;

    bt_rx_len = BT_HC05_GetReceivedData(bt_rx_buf, sizeof(bt_rx_buf) - 1);
    if (bt_rx_len == 0) return;
    bt_rx_buf[bt_rx_len] = '\0';

    char cmd = bt_rx_buf[0];
    float value = 0.0f;

    if (bt_rx_len > 1) {
        value = (float)atof((const char *)&bt_rx_buf[1]);
    }

    char reply[64];

    switch (cmd) {
    case CMD_SET_BAL_KP:
        PID_SetKp(&pid_balance, value);
        snprintf(reply, sizeof(reply), "BAL_KP=%.2f\n", value);
        BT_HC05_SendString(reply);
        break;

    case CMD_SET_BAL_KD:
        PID_SetKd(&pid_balance, value);
        snprintf(reply, sizeof(reply), "BAL_KD=%.2f\n", value);
        BT_HC05_SendString(reply);
        break;

    case CMD_SET_SPD_KP:
        PID_SetKp(&pid_speed, value);
        snprintf(reply, sizeof(reply), "SPD_KP=%.3f\n", value);
        BT_HC05_SendString(reply);
        break;

    case CMD_SET_SPD_KI:
        PID_SetKi(&pid_speed, value);
        snprintf(reply, sizeof(reply), "SPD_KI=%.4f\n", value);
        BT_HC05_SendString(reply);
        break;

    case CMD_SET_TURN_KP:
        PID_SetKp(&pid_turn, value);
        snprintf(reply, sizeof(reply), "TURN_KP=%.2f\n", value);
        BT_HC05_SendString(reply);
        break;

    case CMD_SET_ANGLE_OFF:
        /* 动态修改零点偏移 — 写入全局即可, 下次循环生效 */
        snprintf(reply, sizeof(reply), "OFFSET=%.2f\n", value);
        BT_HC05_SendString(reply);
        break;

    case CMD_GET_STATUS:
        /* 立即发送一帧状态 */
        {
            float p, r, y;
            JY901S_GetAngle(&p, &r, &y);
            int32_t sl = EncoderGpio_GetSpeed(ENCODER_GPIO_LEFT);
            int32_t sr = EncoderGpio_GetSpeed(ENCODER_GPIO_RIGHT);
            snprintf(reply, sizeof(reply),
                     "ANG:%.1f,SPD:%ld,%ld,PWM:%d,%d\n",
                     p, sl, sr,
                     Motor_GetPWM(MOTOR_A), Motor_GetPWM(MOTOR_B));
            BT_HC05_SendString(reply);
        }
        break;

    case CMD_EMERGENCY_STOP:
        emergency_stop = 1;
        BT_HC05_SendString("STOPPED\n");
        break;

    case 'R':  /* 恢复运行 */
        emergency_stop = 0;
        speed_integral = 0;
        BT_HC05_SendString("RUNNING\n");
        break;

    case 'L':  /* 左转 */
        turn_target = (int16_t)value;
        break;

    case 'N':  /* 右转 */
        turn_target = -(int16_t)value;
        break;

    case 'Z':  /* 直行 */
        turn_target = 0;
        break;

    default:
        BT_HC05_SendString("ERR:Unknown cmd\n");
        break;
    }

    BT_HC05_ClearRxBuffer();
}

/* ══════════════════════════════════════════════════════════════
 *  蓝牙状态上报 — 周期性发送运行数据
 * ══════════════════════════════════════════════════════════════ */
static void BT_SendStatus(void)
{
    static uint32_t last_send_tick = 0;

    if (sys_tick_ms - last_send_tick < BT_SEND_INTERVAL_MS) return;
    last_send_tick = sys_tick_ms;

    if (!BT_HC05_IsConnected()) return;

    /* 数据帧格式: $ANGLE,PWM_L,PWM_R,SPEED_L,SPEED_R* */
    float pitch, roll, yaw;
    JY901S_GetAngle(&pitch, &roll, &yaw);

    int32_t spd_l = EncoderGpio_GetSpeed(ENCODER_GPIO_LEFT);
    int32_t spd_r = EncoderGpio_GetSpeed(ENCODER_GPIO_RIGHT);

    char buf[80];
    snprintf(buf, sizeof(buf), "$%.1f,%d,%d,%ld,%ld*\n",
             pitch,
             Motor_GetPWM(MOTOR_A),
             Motor_GetPWM(MOTOR_B),
             spd_l, spd_r);

    BT_HC05_SendIfConnected((uint8_t *)buf, strlen(buf));
}

/* ══════════════════════════════════════════════════════════════
 *  主函数
 * ══════════════════════════════════════════════════════════════ */
int main(void)
{
    /* ── 1. 系统初始化 ─────────────────────────────────── */
    System_Init();

    /* ── 2. 外设初始化 ─────────────────────────────────── */
    Motor_Init_HW();
    Encoder_Init_HW();
    IMU_Init_HW();
    BT_Init_HW();
    PID_Init_All();

    /* ── 3. 使能定时器中断 ─────────────────────────────── */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* ── 4. 启动提示 ───────────────────────────────────── */
    printf("=== 平衡车示例启动 ===\n");
    printf("蓝牙调参协议:\n");
    printf("  A<val> 设置直立Kp   B<val> 设置直立Kd\n");
    printf("  C<val> 设置速度Kp   D<val> 设置速度Ki\n");
    printf("  E<val> 设置转向Kp   S 查询状态\n");
    printf("  X 急停  R 恢复  L左转 N右转 Z直行\n");
    printf("等待IMU数据稳定...\n");

    /* 等待IMU数据就绪 */
    while (!JY901S_IsDataReady()) {
        DELAY_MS(100);
    }
    printf("IMU就绪, 开始平衡控制!\n");

    BT_HC05_SendString("BALANCE_CAR_READY\n");

    /* ── 5. 主循环 ─────────────────────────────────────── */
    uint32_t last_ctrl_tick = 0;

    while (1) {
        /* 每10ms执行一次平衡控制 */
        if (sys_tick_ms - last_ctrl_tick >= ENCODER_SAMPLE_MS) {
            last_ctrl_tick = sys_tick_ms;
            Balance_Control();
        }

        /* 处理蓝牙命令 */
        BT_ParseCommand();

        /* 周期性上报状态 */
        BT_SendStatus();
    }

    return 0;
}
