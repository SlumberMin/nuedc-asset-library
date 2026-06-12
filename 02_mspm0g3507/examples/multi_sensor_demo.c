/**
 * @file    multi_sensor_demo.c
 * @brief   多传感器融合演示示例 — MSPM0G3507
 *
 * 功能概述:
 *   同时使用库中所有传感器模块，实现综合监控与控制:
 *   1. JY901S IMU  — 姿态角 (Pitch/Roll/Yaw)
 *   2. 编码器      — 轮速和里程
 *   3. TCS34725    — 颜色识别
 *   4. 超声波      — 距离测量
 *   5. 灰度传感器  — 循线/地面检测
 *   6. OLED        — 实时数据显示
 *   7. HC-05蓝牙   — 远程监控与控制
 *   8. TB6612电机  — 驱动执行
 *   9. PCA9685舵机 — 多路执行
 *   10. 状态机     — 任务流程管理
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │                    硬件接线说明                           │
 * ├──────────────────────────────────────────────────────────┤
 * │                                                          │
 * │ 【I2C总线 (共用)】                                       │
 * │   MSPM0 PB2 → SCL (4.7K上拉)                            │
 * │   MSPM0 PB3 → SDA (4.7K上拉)                            │
 * │   ├─ TCS34725 颜色传感器 (0x29)                          │
 * │   ├─ PCA9685  舵机驱动板 (0x40)                          │
 * │   └─ OLED SSD1306 显示屏 (0x3C)                         │
 * │                                                          │
 * │ 【UART0 — JY901S IMU】                                   │
 * │   MSPM0 PA9(U0RX) ← JY901S TX                           │
 * │   MSPM0 PA8(U0TX) → JY901S RX                           │
 * │                                                          │
 * │ 【UART1 — HC-05 蓝牙】                                   │
 * │   MSPM0 PB0(U1RX) ← HC-05 TX                            │
 * │   MSPM0 PB1(U1TX) → HC-05 RX                            │
 * │   MSPM0 PA7       → HC-05 STATE                          │
 * │                                                          │
 * │ 【超声波传感器】                                         │
 * │   MSPM0 PA0 → Trig   PA1 → Echo                         │
 * │                                                          │
 * │ 【8路灰度传感器】                                        │
 * │   MSPM0 PA25→ ADR0  PA26→ ADR1  PA27→ ADR2             │
 * │   MSPM0 PB6 → AD0 (ADC输入)                              │
 * │                                                          │
 * │ 【TB6612 电机驱动】                                      │
 * │   MSPM0 PA2 → AIN1   PA3 → AIN2   PA12(PWM) → PWMA    │
 * │   MSPM0 PA4 → BIN1   PA5 → BIN2   PA13(PWM) → PWMB    │
 * │   MSPM0 PA10→ STBY                                      │
 * │                                                          │
 * │ 【N20 编码器电机】                                       │
 * │   MSPM0 PB0 → 左轮A相   PB1 → 左轮B相                   │
 * │   MSPM0 PB4 → 右轮A相   PB5 → 右轮B相                   │
 * │   注意: 编码器与灰度传感器引脚互斥, 不能同时使用           │
 * │                                                          │
 * │ 【PCA9685 舵机】                                         │
 * │   CH0 → 舵机1   CH1 → 舵机2   CH2 → 舵机3               │
 * └──────────────────────────────────────────────────────────┘
 *
 * 蓝牙监控协议 (手机APP显示):
 *   上行 (MCU→APP): $IMU,p,r,y|ENC,ls,rs|ULT,d|COL,r,g,b|GRAY,dig\n
 *   下行 (APP→MCU): 单字符命令
 *     'S' 查询状态   'M' 切换模式   'P' 暂停/恢复
 *     'F' 前进       'B' 后退       'L' 左转   'R' 右转   'X' 停止
 *     '0'~'9' 舵机通道控制
 *
 * 依赖驱动: 全部传感器和执行器驱动模块
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
#include "drivers/encoder_gpio_mspm0.h"
#include "drivers/tcs34725.h"
#include "drivers/ultrasonic_mspm0.h"
#include "drivers/grayscale_mspm0.h"
#include "drivers/motor_mspm0.h"
#include "drivers/pca9685.h"
#include "drivers/oled_ssd1306_mspm0.h"
#include "drivers/bluetooth_hc05_mspm0.h"
#include "drivers/advanced_pid.h"
#include "drivers/state_machine.h"

/* ══════════════════════════════════════════════════════════════
 *  配置参数
 * ══════════════════════════════════════════════════════════════ */

#define PWM_PERIOD          1000
#define PWM_MAX             800
#define ENCODER_SAMPLE_MS   10

/* OLED页面 */
#define PAGE_SENSOR_DATA    0   /* 传感器数据页 */
#define PAGE_MOTOR_STATUS   1   /* 电机状态页 */
#define PAGE_SYSTEM_INFO    2   /* 系统信息页 */
#define PAGE_COUNT          3

/* 蓝牙上报间隔 */
#define BT_REPORT_INTERVAL  100  /* ms */

/* 任务模式 */
#define MODE_IDLE           0   /* 空闲 */
#define MODE_LINE_TRACK     1   /* 循线 */
#define MODE_OBSTACLE       2   /* 避障 */
#define MODE_REMOTE         3   /* 遥控 */

/* ══════════════════════════════════════════════════════════════
 *  全局变量
 * ══════════════════════════════════════════════════════════════ */

/* 系统时钟 */
static volatile uint32_t sys_tick_ms = 0;

/* 传感器数据汇总 */
typedef struct {
    /* IMU */
    float pitch, roll, yaw;
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    /* 编码器 */
    int32_t enc_left_speed, enc_right_speed;
    int32_t enc_left_count, enc_right_count;
    /* 颜色 */
    uint16_t color_r, color_g, color_b, color_c;
    /* 超声波 */
    float distance;
    /* 灰度 */
    uint8_t gray_digital;
    int16_t gray_error;
    uint16_t gray_analog[8];
    /* 电机 */
    int16_t motor_left_pwm, motor_right_pwm;
    /* 状态 */
    uint8_t mode;
    uint8_t page;
    uint32_t uptime_sec;
} SystemData;

static SystemData sys_data = {0};

/* PID控制器 */
static PID_Controller pid_track;

/* OLED页面 */
static volatile uint8_t oled_page = PAGE_SENSOR_DATA;

/* 蓝牙接收 */
static uint8_t bt_rx_buf[64];
static uint16_t bt_rx_len = 0;

/* 运行标志 */
static volatile uint8_t system_paused = 0;

/* 舵机角度 (手动控制) */
static uint16_t servo_angles[3] = {90, 90, 90};

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
    }
}

/* ══════════════════════════════════════════════════════════════
 *  传感器数据采集
 * ══════════════════════════════════════════════════════════════ */
static void Collect_All_Sensors(void)
{
    /* IMU */
    JY901S_GetAngle(&sys_data.pitch, &sys_data.roll, &sys_data.yaw);
    JY901S_GetAccel(&sys_data.acc_x, &sys_data.acc_y, &sys_data.acc_z);
    JY901S_GetGyro(&sys_data.gyro_x, &sys_data.gyro_y, &sys_data.gyro_z);

    /* 编码器 */
    sys_data.enc_left_speed  = EncoderGpio_GetSpeed(ENCODER_GPIO_LEFT);
    sys_data.enc_right_speed = EncoderGpio_GetSpeed(ENCODER_GPIO_RIGHT);
    sys_data.enc_left_count  = EncoderGpio_Read(ENCODER_GPIO_LEFT);
    sys_data.enc_right_count = EncoderGpio_Read(ENCODER_GPIO_RIGHT);

    /* 颜色传感器 */
    TCS34725_RGBC rgbc;
    if (TCS34725_ReadRGBC(&rgbc)) {
        sys_data.color_r = rgbc.red;
        sys_data.color_g = rgbc.green;
        sys_data.color_b = rgbc.blue;
        sys_data.color_c = rgbc.clear;
    }

    /* 超声波 */
    sys_data.distance = Ultrasonic_GetFilteredDistance();

    /* 灰度 */
    Grayscale_Read();
    sys_data.gray_digital = Grayscale_GetDigital();
    sys_data.gray_error   = Grayscale_GetTrackError();
    Grayscale_GetAnalog(sys_data.gray_analog);

    /* 电机PWM */
    sys_data.motor_left_pwm  = Motor_GetPWM(MOTOR_A);
    sys_data.motor_right_pwm = Motor_GetPWM(MOTOR_B);

    /* 系统信息 */
    sys_data.page = oled_page;
    sys_data.uptime_sec = sys_tick_ms / 1000;
}

/* ══════════════════════════════════════════════════════════════
 *  OLED多页面显示
 * ══════════════════════════════════════════════════════════════ */
static void OLED_Display_Page0(void)
{
    /* 传感器数据页 */
    OLED_ShowString(0, 0, "== Sensor Data ==", 12, 1);

    /* IMU */
    OLED_ShowString(0, 1, "P:", 12, 1);
    OLED_ShowFloat(16, 1, sys_data.pitch, 3, 1, 12, 1);
    OLED_ShowString(60, 1, "R:", 12, 1);
    OLED_ShowFloat(76, 1, sys_data.roll, 3, 1, 12, 1);

    /* 超声波 */
    OLED_ShowString(0, 2, "Dist:", 12, 1);
    OLED_ShowFloat(36, 2, sys_data.distance, 3, 1, 12, 1);
    OLED_ShowString(80, 2, "cm", 12, 1);

    /* 颜色 */
    OLED_ShowString(0, 3, "R:", 12, 1);
    OLED_ShowNum(16, 3, sys_data.color_r, 4, 12, 1);
    OLED_ShowString(50, 3, "G:", 12, 1);
    OLED_ShowNum(66, 3, sys_data.color_g, 4, 12, 1);

    OLED_ShowString(0, 4, "B:", 12, 1);
    OLED_ShowNum(16, 4, sys_data.color_b, 4, 12, 1);
    OLED_ShowString(50, 4, "C:", 12, 1);
    OLED_ShowNum(66, 4, sys_data.color_c, 4, 12, 1);

    /* 灰度 */
    OLED_ShowString(0, 5, "Gray:", 12, 1);
    OLED_ShowNum(36, 5, sys_data.gray_digital, 3, 12, 1);
    OLED_ShowString(70, 5, "E:", 12, 1);
    OLED_ShowNum(86, 5, (uint32_t)(sys_data.gray_error < 0 ? -sys_data.gray_error : sys_data.gray_error), 4, 12, 1);

    /* 编码器 */
    OLED_ShowString(0, 6, "L:", 12, 1);
    OLED_ShowNum(16, 6, (uint32_t)sys_data.enc_left_speed, 4, 12, 1);
    OLED_ShowString(50, 6, "R:", 12, 1);
    OLED_ShowNum(66, 6, (uint32_t)sys_data.enc_right_speed, 4, 12, 1);
}

static void OLED_Display_Page1(void)
{
    /* 电机状态页 */
    OLED_ShowString(0, 0, "== Motor Status ==", 12, 1);

    OLED_ShowString(0, 1, "Left  PWM:", 12, 1);
    OLED_ShowNum(70, 1, (uint32_t)sys_data.motor_left_pwm, 4, 12, 1);

    OLED_ShowString(0, 2, "Right PWM:", 12, 1);
    OLED_ShowNum(70, 2, (uint32_t)sys_data.motor_right_pwm, 4, 12, 1);

    OLED_ShowString(0, 3, "Left  Cnt:", 12, 1);
    OLED_ShowNum(70, 3, (uint32_t)sys_data.enc_left_count, 5, 12, 1);

    OLED_ShowString(0, 4, "Right Cnt:", 12, 1);
    OLED_ShowNum(70, 4, (uint32_t)sys_data.enc_right_count, 5, 12, 1);

    /* 舵机角度 */
    OLED_ShowString(0, 5, "Servo:", 12, 1);
    OLED_ShowNum(40, 5, servo_angles[0], 3, 12, 1);
    OLED_ShowNum(70, 5, servo_angles[1], 3, 12, 1);
    OLED_ShowNum(100, 5, servo_angles[2], 3, 12, 1);
}

static void OLED_Display_Page2(void)
{
    /* 系统信息页 */
    OLED_ShowString(0, 0, "== System Info ==", 12, 1);

    const char *mode_names[] = {"IDLE", "LINE", "OBS", "REMOTE"};
    OLED_ShowString(0, 1, "Mode:", 12, 1);
    OLED_ShowString(36, 1, (char *)mode_names[sys_data.mode], 12, 1);

    OLED_ShowString(0, 2, "Uptime:", 12, 1);
    OLED_ShowNum(50, 2, sys_data.uptime_sec, 5, 12, 1);
    OLED_ShowString(90, 2, "s", 12, 1);

    /* IMU详细 */
    OLED_ShowString(0, 3, "Yaw:", 12, 1);
    OLED_ShowFloat(30, 3, sys_data.yaw, 3, 1, 12, 1);

    OLED_ShowString(0, 4, "AccZ:", 12, 1);
    OLED_ShowFloat(36, 4, sys_data.acc_z, 2, 2, 12, 1);

    OLED_ShowString(0, 5, "GyroY:", 12, 1);
    OLED_ShowFloat(42, 5, sys_data.gyro_y, 3, 1, 12, 1);

    /* BT状态 */
    OLED_ShowString(0, 6, "BT:", 12, 1);
    OLED_ShowString(24, 6, BT_HC05_IsConnected() ? "ON " : "OFF", 12, 1);
}

static void OLED_Update(void)
{
    static uint32_t last_update = 0;
    if (sys_tick_ms - last_update < 250) return;
    last_update = sys_tick_ms;

    OLED_Clear();

    switch (oled_page) {
    case PAGE_SENSOR_DATA: OLED_Display_Page0(); break;
    case PAGE_MOTOR_STATUS: OLED_Display_Page1(); break;
    case PAGE_SYSTEM_INFO:  OLED_Display_Page2(); break;
    }

    OLED_Refresh();
}

/* ══════════════════════════════════════════════════════════════
 *  蓝牙数据上报
 *
 *  数据帧格式:
 *  $IMU,pitch,roll,yaw|ENC,spd_l,spd_r|ULT,dist|COL,r,g,b,clr|GRAY,dig,err|PWM,l,r\n
 * ══════════════════════════════════════════════════════════════ */
static void BT_Report(void)
{
    static uint32_t last_report = 0;
    if (sys_tick_ms - last_report < BT_REPORT_INTERVAL) return;
    last_report = sys_tick_ms;

    if (!BT_HC05_IsConnected()) return;

    char buf[160];
    int len = snprintf(buf, sizeof(buf),
        "$IMU,%.1f,%.1f,%.1f|ENC,%ld,%ld|ULT,%.1f|COL,%u,%u,%u,%u|GRAY,%u,%d|PWM,%d,%d\n",
        sys_data.pitch, sys_data.roll, sys_data.yaw,
        sys_data.enc_left_speed, sys_data.enc_right_speed,
        sys_data.distance,
        sys_data.color_r, sys_data.color_g, sys_data.color_b, sys_data.color_c,
        sys_data.gray_digital, sys_data.gray_error,
        sys_data.motor_left_pwm, sys_data.motor_right_pwm
    );

    BT_HC05_SendIfConnected((uint8_t *)buf, (uint16_t)len);
}

/* ══════════════════════════════════════════════════════════════
 *  蓝牙命令解析
 * ══════════════════════════════════════════════════════════════ */
static void BT_ProcessCommand(void)
{
    if (!BT_HC05_IsDataReceived()) return;

    bt_rx_len = BT_HC05_GetReceivedData(bt_rx_buf, sizeof(bt_rx_buf) - 1);
    if (bt_rx_len == 0) return;
    bt_rx_buf[bt_rx_len] = '\0';

    char cmd = bt_rx_buf[0];
    char reply[80];

    switch (cmd) {
    /* 状态查询 */
    case 'S':
    case 's':
        snprintf(reply, sizeof(reply),
            "MODE=%d UPT=%lus P=%.1f R=%.1f D=%.1f GRAY=%02X\n",
            sys_data.mode, sys_data.uptime_sec,
            sys_data.pitch, sys_data.roll,
            sys_data.distance, sys_data.gray_digital);
        BT_HC05_SendString(reply);
        break;

    /* 切换模式 */
    case 'M':
    case 'm':
        if (bt_rx_len > 1) {
            sys_data.mode = bt_rx_buf[1] - '0';
            snprintf(reply, sizeof(reply), "MODE=%d\n", sys_data.mode);
            BT_HC05_SendString(reply);
            Motor_SetSpeed(MOTOR_A, 0);
            Motor_SetSpeed(MOTOR_B, 0);
        }
        break;

    /* 暂停/恢复 */
    case 'P':
    case 'p':
        system_paused = !system_paused;
        if (system_paused) {
            Motor_SetSpeed(MOTOR_A, 0);
            Motor_SetSpeed(MOTOR_B, 0);
        }
        BT_HC05_SendString(system_paused ? "PAUSED\n" : "RUNNING\n");
        break;

    /* 遥控: 前进 */
    case 'F':
    case 'f':
        sys_data.mode = MODE_REMOTE;
        Motor_SetSpeed(MOTOR_A, 400);
        Motor_SetSpeed(MOTOR_B, 400);
        BT_HC05_SendString("FWD\n");
        break;

    /* 遥控: 后退 */
    case 'B':
    case 'b':
        sys_data.mode = MODE_REMOTE;
        Motor_SetSpeed(MOTOR_A, -400);
        Motor_SetSpeed(MOTOR_B, -400);
        BT_HC05_SendString("BWD\n");
        break;

    /* 遥控: 左转 */
    case 'L':
    case 'l':
        sys_data.mode = MODE_REMOTE;
        Motor_SetSpeed(MOTOR_A, -200);
        Motor_SetSpeed(MOTOR_B, 400);
        BT_HC05_SendString("LEFT\n");
        break;

    /* 遥控: 右转 */
    case 'R':
    case 'r':
        sys_data.mode = MODE_REMOTE;
        Motor_SetSpeed(MOTOR_A, 400);
        Motor_SetSpeed(MOTOR_B, -200);
        BT_HC05_SendString("RIGHT\n");
        break;

    /* 停止 */
    case 'X':
    case 'x':
        Motor_SetSpeed(MOTOR_A, 0);
        Motor_SetSpeed(MOTOR_B, 0);
        BT_HC05_SendString("STOP\n");
        break;

    /* 切换OLED页面 */
    case 'D':
    case 'd':
        oled_page = (oled_page + 1) % PAGE_COUNT;
        snprintf(reply, sizeof(reply), "PAGE=%d\n", oled_page);
        BT_HC05_SendString(reply);
        break;

    /* 舵机控制: '0'~'2' + 角度 */
    case '0': case '1': case '2':
        {
            uint8_t ch = cmd - '0';
            if (bt_rx_len > 1) {
                uint16_t angle = (uint16_t)atoi((const char *)&bt_rx_buf[1]);
                if (angle <= 180) {
                    servo_angles[ch] = angle;
                    PCA9685_SetAngle(ch, angle);
                    snprintf(reply, sizeof(reply), "SERVO%d=%d\n", ch, angle);
                    BT_HC05_SendString(reply);
                }
            }
        }
        break;

    default:
        BT_HC05_SendString("ERR\n");
        break;
    }

    BT_HC05_ClearRxBuffer();
}

/* ══════════════════════════════════════════════════════════════
 *  自动运行模式
 * ══════════════════════════════════════════════════════════════ */
static void Auto_Mode_LineTrack(void)
{
    int16_t error = sys_data.gray_error;
    int16_t steer = (int16_t)PID_Calc(&pid_track, 0.0f, (float)error);

    int16_t base_speed = 300;
    int16_t pwm_left  = base_speed - steer;
    int16_t pwm_right = base_speed + steer;

    if (pwm_left > PWM_MAX)  pwm_left = PWM_MAX;
    if (pwm_left < -PWM_MAX) pwm_left = -PWM_MAX;
    if (pwm_right > PWM_MAX)  pwm_right = PWM_MAX;
    if (pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;

    Motor_SetSpeed(MOTOR_A, pwm_left);
    Motor_SetSpeed(MOTOR_B, pwm_right);
}

static void Auto_Mode_Obstacle(void)
{
    if (sys_data.distance > 0 && sys_data.distance < 20.0f) {
        /* 前方有障碍, 后退+转向 */
        Motor_SetSpeed(MOTOR_A, -300);
        Motor_SetSpeed(MOTOR_B, -300);
        DELAY_MS(300);
        Motor_SetSpeed(MOTOR_A, 400);
        Motor_SetSpeed(MOTOR_B, -400);
        DELAY_MS(400);
    } else {
        Motor_SetSpeed(MOTOR_A, 350);
        Motor_SetSpeed(MOTOR_B, 350);
    }
}

/* ══════════════════════════════════════════════════════════════
 *  硬件初始化
 * ══════════════════════════════════════════════════════════════ */
static void Motor_Init_HW(void)
{
    MotorConfig cfg[MOTOR_MAX] = {
        [MOTOR_A] = {
            .port_in1 = GPIOA, .pin_in1 = DL_GPIO_PIN_2,
            .port_in2 = GPIOA, .pin_in2 = DL_GPIO_PIN_3,
            .pwm_timer = TIMA0, .pwm_channel = DL_TIMER_CC_0_INDEX,
            .pwm_period = PWM_PERIOD
        },
        [MOTOR_B] = {
            .port_in1 = GPIOA, .pin_in1 = DL_GPIO_PIN_4,
            .port_in2 = GPIOA, .pin_in2 = DL_GPIO_PIN_5,
            .pwm_timer = TIMA0, .pwm_channel = DL_TIMER_CC_3_INDEX,
            .pwm_period = PWM_PERIOD
        }
    };
    Motor_Init(cfg);

    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_10);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_10);
}

static void Encoder_Init_HW(void)
{
    EncoderGpioConfig cfg[ENCODER_GPIO_MAX] = {
        [ENCODER_GPIO_LEFT] = {  /* 与灰度传感器PB0互斥 */
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

static void IMU_Init_HW(void)
{
    JY901S_Config cfg = {
        .uart = UART_0_INST,
        .baudrate = 9600,
        .auto_calib = 1
    };
    JY901S_Init(&cfg);
}

static void BT_Init_HW(void)
{
    BT_HC05_Config cfg = {
        .uart = UART_1_INST,
        .state_port = GPIOA,
        .state_pin = DL_GPIO_PIN_7,
        .baudrate = 9600
    };
    BT_HC05_Init(&cfg);
}

static void Ultrasonic_Init_HW(void)
{
    UltrasonicConfig cfg = {
        .port = GPIOA,
        .trig_pin = DL_GPIO_PIN_0,
        .echo_pin = DL_GPIO_PIN_1,
        .type = ULTRASONIC_SR04,
        .filter_size = 5
    };
    Ultrasonic_Init(&cfg);
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

static void PID_Init_HW(void)
{
    PID_Param param = {
        .kp = 1.0f, .ki = 0.01f, .kd = 0.4f,
        .output_min = -300, .output_max = 300,
        .integral_max = 500.0f, .dead_zone = 20
    };
    PID_Init(&pid_track, &param);
}

/* ══════════════════════════════════════════════════════════════
 *  主函数
 * ══════════════════════════════════════════════════════════════ */
int main(void)
{
    /* ── 1. 系统初始化 ─────────────────────────────────── */
    System_Init();

    printf("=== 多传感器融合演示启动 ===\n");

    /* ── 2. 逐个初始化传感器 ───────────────────────────── */
    printf("初始化电机...");
    Motor_Init_HW();
    printf(" OK\n");

    printf("初始化编码器...");
    Encoder_Init_HW();
    printf(" OK\n");

    printf("初始化IMU...");
    IMU_Init_HW();
    printf(" OK\n");

    printf("初始化蓝牙...");
    BT_Init_HW();
    printf(" OK\n");

    printf("初始化超声波...");
    Ultrasonic_Init_HW();
    printf(" OK\n");

    printf("初始化灰度...");
    Grayscale_Init_HW();
    printf(" OK\n");

    printf("初始化颜色传感器...");
    if (TCS34725_Init()) {
        printf(" OK\n");
    } else {
        printf(" FAIL!\n");
    }

    printf("初始化舵机驱动...");
    if (PCA9685_Init()) {
        printf(" OK\n");
        /* 舵机归中 */
        PCA9685_SetAngle(0, 90);
        PCA9685_SetAngle(1, 90);
        PCA9685_SetAngle(2, 90);
    } else {
        printf(" FAIL!\n");
    }

    printf("初始化OLED...");
    OLED_Init(I2C_0_INST);
    OLED_Clear();
    OLED_ShowString(0, 0, "Multi-Sensor Demo", 16, 1);
    OLED_ShowString(0, 2, "All sensors init", 12, 1);
    OLED_ShowString(0, 3, "Waiting for start", 12, 1);
    OLED_Refresh();
    printf(" OK\n");

    PID_Init_HW();

    /* ── 3. 等待IMU就绪 ───────────────────────────────── */
    printf("等待IMU数据...\n");
    uint32_t wait_start = sys_tick_ms;
    while (!JY901S_IsDataReady() && (sys_tick_ms - wait_start < 3000)) {
        DELAY_MS(100);
    }
    if (JY901S_IsDataReady()) {
        printf("IMU就绪\n");
    } else {
        printf("IMU超时, 继续运行\n");
    }

    /* ── 4. 使能定时器中断 ─────────────────────────────── */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* ── 5. 蓝牙欢迎信息 ──────────────────────────────── */
    BT_HC05_SendString("=== Multi-Sensor Demo ===\n");
    BT_HC05_SendString("Commands: S=Status M=Mode P=Pause\n");
    BT_HC05_SendString("  F=Forward B=Back L=Left R=Right X=Stop\n");
    BT_HC05_SendString("  D=SwitchPage 0~2+angle=Servo\n");

    printf("\n所有传感器初始化完成!\n");
    printf("蓝牙命令:\n");
    printf("  S 查询状态  M<n> 切换模式  P 暂停/恢复\n");
    printf("  F 前进  B 后退  L 左转  R 右转  X 停止\n");
    printf("  D 切换OLED页面  0~2+角度 控制舵机\n");
    printf("\n开始主循环...\n");

    /* ── 6. 主循环 ─────────────────────────────────────── */
    uint32_t last_collect_tick = 0;

    while (1) {
        /* 每20ms采集一次传感器数据 */
        if (sys_tick_ms - last_collect_tick >= 20) {
            last_collect_tick = sys_tick_ms;
            Collect_All_Sensors();
        }

        /* 自动模式执行 */
        if (!system_paused) {
            switch (sys_data.mode) {
            case MODE_LINE_TRACK:
                Auto_Mode_LineTrack();
                break;
            case MODE_OBSTACLE:
                Auto_Mode_Obstacle();
                break;
            case MODE_REMOTE:
            case MODE_IDLE:
            default:
                /* 遥控/空闲模式不自动控制 */
                break;
            }
        }

        /* 处理蓝牙命令 */
        BT_ProcessCommand();

        /* 蓝牙数据上报 */
        BT_Report();

        /* OLED显示 */
        OLED_Update();
    }

    return 0;
}
