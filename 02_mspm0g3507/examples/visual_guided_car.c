/**
 * @file    visual_guided_car.c
 * @brief   视觉引导小车示例 — MSPM0G3507 + Orange Pi 5
 *
 * 功能概述:
 *   1. OPi5通过UART发送目标坐标 (x, y) 和识别结果
 *   2. MSPM0解析坐标, 通过PID控制电机使小车跟踪目标
 *   3. 支持视觉循线、目标追踪、AprilTag导航等模式
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │                    硬件接线说明                           │
 * ├──────────────────────────────────────────────────────────┤
 * │ Orange Pi 5 ←→ MSPM0G3507 (UART通信):                   │
 * │   MSPM0 PA9(UART0 RX) ← OPi5 TX (或USB转TTL TX)        │
 * │   MSPM0 PA8(UART0 TX) → OPi5 RX (或USB转TTL RX)        │
 * │   共地!                                                  │
 * │                                                          │
 * │ TB6612 电机驱动:                                         │
 * │   MSPM0 PA0 → AIN1    PA1 → AIN2   PA12(PWM) → PWMA   │
 * │   MSPM0 PA2 → BIN1    PA3 → BIN2   PA13(PWM) → PWMB   │
 * │   MSPM0 PA4 → STBY                                     │
 * │                                                          │
 * │ N20 编码器电机:                                          │
 * │   MSPM0 PB0 → 左轮A相   PB1 → 左轮B相                   │
 * │   MSPM0 PB2 → 右轮A相   PB3 → 右轮B相                   │
 * │                                                          │
 * │ 舵机 (云台, 可选):                                       │
 * │   MSPM0 PA14(PWM) → 舵机信号线                           │
 * │                                                          │
 * │ OLED 显示 (可选):                                        │
 * │   MSPM0 PB2(SCL) → OLED SCL   PB3(SDA) → OLED SDA     │
 * └──────────────────────────────────────────────────────────┘
 *
 * OPi5端通信协议 (TX → MSPM0):
 *   "$T,x,y,area,type\n"  — 目标信息
 *     x: 0~639 (画面中心320)
 *     y: 0~479 (画面中心240)
 *     area: 目标面积 (越大越近)
 *     type: 目标类型 (0=循线, 1=红色, 2=绿色, 3=蓝色, 9=AprilTag)
 *   "$L,x1,y1,x2,y2\n"    — 循线信息 (线段两端点)
 *   "$N\n"                  — 未检测到目标
 *
 * MSPM0回复 (TX → OPi5):
 *   "$OK\n" — 确认收到
 *   "$S,l_pwm,r_pwm\n" — 状态回传
 *
 * 依赖驱动: motor_mspm0, encoder_gpio_mspm0, advanced_pid,
 *           oled_ssd1306_mspm0, servo_mspm0
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/motor_mspm0.h"
#include "drivers/encoder_gpio_mspm0.h"
#include "drivers/advanced_pid.h"
#include "drivers/oled_ssd1306_mspm0.h"

/* ══════════════════════════════════════════════════════════════
 *  配置参数
 * ══════════════════════════════════════════════════════════════ */

#define PWM_PERIOD          1000
#define PWM_MAX             800
#define ENCODER_SAMPLE_MS   10

/* 摄像头画面尺寸 */
#define IMG_WIDTH           640
#define IMG_HEIGHT          480
#define IMG_CENTER_X        (IMG_WIDTH / 2)   /* 320 */
#define IMG_CENTER_Y        (IMG_HEIGHT / 2)  /* 240 */

/* 视觉引导模式 */
typedef enum {
    MODE_LINE_FOLLOW = 0,   /* 循线模式 */
    MODE_TARGET_TRACK = 1,  /* 目标追踪模式 */
    MODE_APRILTAG = 2       /* AprilTag导航模式 */
} VisionMode;

/* 目标类型 */
#define TARGET_LINE         0
#define TARGET_RED          1
#define TARGET_GREEN        2
#define TARGET_BLUE         3
#define TARGET_APRILTAG     9

/* ══════════════════════════════════════════════════════════════
 *  全局变量
 * ══════════════════════════════════════════════════════════════ */

static PID_Controller pid_forward;  /* 前进/后退PID */
static PID_Controller pid_steer;    /* 转向PID */

static volatile uint32_t sys_tick_ms = 0;
static volatile VisionMode current_mode = MODE_LINE_FOLLOW;

/* 视觉目标数据 (由UART中断更新) */
typedef struct {
    int16_t  x;         /* 目标中心x */
    int16_t  y;         /* 目标中心y */
    uint32_t area;      /* 目标面积 */
    uint8_t  type;      /* 目标类型 */
    uint8_t  detected;  /* 是否检测到 */
    /* 循线模式: 线段端点 */
    int16_t  line_x1, line_y1;
    int16_t  line_x2, line_y2;
} VisionTarget;

static volatile VisionTarget vision = {0};
static uint8_t uart_rx_buf[128];
static volatile uint16_t uart_rx_idx = 0;
static volatile uint8_t uart_frame_ready = 0;

/* ══════════════════════════════════════════════════════════════
 *  系统时钟
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
 *  UART中断 — 接收OPi5数据
 * ══════════════════════════════════════════════════════════════ */
void UART_0_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_0_INST) == DL_UART_IIDX_RX) {
        volatile uint8_t ch = DL_UART_receiveData(UART_0_INST);

        if (ch == '$') {
            /* 帧起始, 重置索引 */
            uart_rx_idx = 0;
        }

        if (uart_rx_idx < sizeof(uart_rx_buf) - 1) {
            uart_rx_buf[uart_rx_idx++] = ch;
        }

        if (ch == '\n') {
            uart_rx_buf[uart_rx_idx] = '\0';
            uart_frame_ready = 1;
        }
    }
}

/* ══════════════════════════════════════════════════════════════
 *  协议解析
 *
 *  帧格式: "$T,x,y,area,type\n" 或 "$L,x1,y1,x2,y2\n" 或 "$N\n"
 * ══════════════════════════════════════════════════════════════ */
static void Parse_Vision_Frame(void)
{
    if (!uart_frame_ready) return;
    uart_frame_ready = 0;

    char *frame = (char *)uart_rx_buf;

    /* 跳过 '$' */
    if (frame[0] != '$') return;
    frame++;

    if (frame[0] == 'T' && frame[1] == ',') {
        /* 目标信息: T,x,y,area,type */
        int16_t x, y;
        uint32_t area;
        uint8_t type;
        if (sscanf(frame + 2, "%hd,%hd,%lu,%hhu", &x, &y, &area, &type) == 4) {
            vision.x = x;
            vision.y = y;
            vision.area = area;
            vision.type = type;
            vision.detected = 1;
        }
    }
    else if (frame[0] == 'L' && frame[1] == ',') {
        /* 循线信息: L,x1,y1,x2,y2 */
        sscanf(frame + 2, "%hd,%hd,%hd,%hd",
               &vision.line_x1, &vision.line_y1,
               &vision.line_x2, &vision.line_y2);
        vision.detected = 1;
        vision.type = TARGET_LINE;
    }
    else if (frame[0] == 'N') {
        /* 未检测到目标 */
        vision.detected = 0;
    }

    /* 回复确认 */
    DL_UART_transmitData(UART_0_INST, 'O');
    DL_UART_transmitData(UART_0_INST, 'K');
    DL_UART_transmitData(UART_0_INST, '\n');
}

/* ══════════════════════════════════════════════════════════════
 *  视觉引导控制算法
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 循线模式
 *
 * 策略: 通过线段中心点的水平偏移量控制转向
 *   - 线在画面左侧 → 左转
 *   - 线在画面右侧 → 右转
 *   - 面积(接近程度)控制速度
 */
static void LineFollow_Control(void)
{
    if (!vision.detected) {
        /* 丢失目标, 原地慢速搜索 */
        Motor_SetSpeed(MOTOR_A, 100);
        Motor_SetSpeed(MOTOR_B, -100);  /* 原地旋转 */
        return;
    }

    /* 计算线段中心 */
    int16_t line_center_x = (vision.line_x1 + vision.line_x2) / 2;
    int16_t error_x = line_center_x - IMG_CENTER_X;  /* 偏差: 负=偏左, 正=偏右 */

    /* 转向PID */
    int16_t steer = (int16_t)PID_Calc(&pid_steer, 0.0f, (float)error_x);

    /* 基础速度 (根据线段斜率可进一步优化) */
    int16_t base_speed = 300;

    /* 差速转向 */
    int16_t pwm_left  = base_speed + steer;
    int16_t pwm_right = base_speed - steer;

    /* 限幅 */
    if (pwm_left > PWM_MAX)  pwm_left = PWM_MAX;
    if (pwm_left < -PWM_MAX) pwm_left = -PWM_MAX;
    if (pwm_right > PWM_MAX)  pwm_right = PWM_MAX;
    if (pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;

    Motor_SetSpeed(MOTOR_A, pwm_left);
    Motor_SetSpeed(MOTOR_B, pwm_right);
}

/**
 * @brief 目标追踪模式
 *
 * 策略: 同时控制转向(水平对准)和速度(接近/远离目标)
 *   - x偏差 → 转向
 *   - y偏差或面积 → 前进/后退
 */
static void TargetTrack_Control(void)
{
    if (!vision.detected) {
        /* 丢失目标, 停车 */
        Motor_SetSpeed(MOTOR_A, 0);
        Motor_SetSpeed(MOTOR_B, 0);
        return;
    }

    /* 水平偏差 → 转向 */
    int16_t error_x = vision.x - IMG_CENTER_X;
    int16_t steer = (int16_t)PID_Calc(&pid_steer, 0.0f, (float)error_x);

    /* 垂直偏差或面积 → 前进/后退 */
    /* 目标在画面上方(y小)表示远, 需前进; 下方(y大)表示近, 需减速 */
    int16_t error_y = vision.y - (int16_t)(IMG_CENTER_Y * 0.8f);  /* 目标位置偏上 */
    int16_t forward = (int16_t)PID_Calc(&pid_forward, 0.0f, (float)error_y);

    /* 叠加 */
    int16_t pwm_left  = forward + steer;
    int16_t pwm_right = forward - steer;

    /* 限幅 */
    if (pwm_left > PWM_MAX)  pwm_left = PWM_MAX;
    if (pwm_left < -PWM_MAX) pwm_left = -PWM_MAX;
    if (pwm_right > PWM_MAX)  pwm_right = PWM_MAX;
    if (pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;

    Motor_SetSpeed(MOTOR_A, pwm_left);
    Motor_SetSpeed(MOTOR_B, pwm_right);
}

/**
 * @brief AprilTag导航模式
 *
 * 策略: 根据Tag ID和位置执行不同动作
 *   - 对准Tag中心
 *   - 根据面积(距离)调整速度
 *   - Tag ID决定行为(左转/右转/停止)
 */
static void AprilTag_Control(void)
{
    if (!vision.detected) {
        Motor_SetSpeed(MOTOR_A, 0);
        Motor_SetSpeed(MOTOR_B, 0);
        return;
    }

    /* 对准Tag */
    int16_t error_x = vision.x - IMG_CENTER_X;
    int16_t steer = (int16_t)PID_Calc(&pid_steer, 0.0f, (float)error_x);

    /* 根据面积判断距离 — 面积越大越近 */
    int16_t forward = 0;
    if (vision.area < 5000) {
        forward = 350;  /* 远, 快速前进 */
    } else if (vision.area < 15000) {
        forward = 200;  /* 中等距离 */
    } else if (vision.area < 30000) {
        forward = 100;  /* 接近, 慢速 */
    } else {
        forward = 0;    /* 到达, 停车 */
    }

    /* 根据Tag ID执行特殊动作 (示例) */
    switch (vision.type) {
    case 0: break;              /* 默认行为 */
    case 1: steer += 150; break; /* Tag1: 偏右 */
    case 2: steer -= 150; break; /* Tag2: 偏左 */
    case 3: forward = 0; break;  /* Tag3: 停车 */
    default: break;
    }

    int16_t pwm_left  = forward + steer;
    int16_t pwm_right = forward - steer;

    if (pwm_left > PWM_MAX)  pwm_left = PWM_MAX;
    if (pwm_left < -PWM_MAX) pwm_left = -PWM_MAX;
    if (pwm_right > PWM_MAX)  pwm_right = PWM_MAX;
    if (pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;

    Motor_SetSpeed(MOTOR_A, pwm_left);
    Motor_SetSpeed(MOTOR_B, pwm_right);
}

/* ══════════════════════════════════════════════════════════════
 *  OLED显示
 * ══════════════════════════════════════════════════════════════ */
static void Display_Status(void)
{
    static uint32_t last_update = 0;
    if (sys_tick_ms - last_update < 200) return;
    last_update = sys_tick_ms;

    OLED_Clear();

    /* 模式 */
    const char *mode_str[] = {"LINE", "TRACK", "APRIL"};
    OLED_ShowString(0, 0, (char *)mode_str[current_mode], 12, 1);

    /* 目标状态 */
    if (vision.detected) {
        OLED_ShowString(60, 0, "DET", 12, 1);
        OLED_ShowString(0, 2, "X:", 12, 1);
        OLED_ShowNum(20, 2, vision.x, 3, 12, 1);
        OLED_ShowString(50, 2, "Y:", 12, 1);
        OLED_ShowNum(70, 2, vision.y, 3, 12, 1);
    } else {
        OLED_ShowString(60, 0, "---", 12, 1);
    }

    /* 电机PWM */
    OLED_ShowString(0, 4, "L:", 12, 1);
    OLED_ShowNum(20, 4, Motor_GetPWM(MOTOR_A), 4, 12, 1);
    OLED_ShowString(60, 4, "R:", 12, 1);
    OLED_ShowNum(80, 4, Motor_GetPWM(MOTOR_B), 4, 12, 1);

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
            .port = GPIOB, .pin_a = DL_GPIO_PIN_2, .pin_b = DL_GPIO_PIN_3,
            .inverted = 1
        }
    };
    EncoderGpio_Init(cfg);
}

static void UART_Init_HW(void)
{
    /* UART0 用于OPi5通信 (115200波特率) */
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

static void PID_Init_All(void)
{
    /* 转向PID — 水平偏差 */
    PID_Param steer_param = {
        .kp = 0.8f, .ki = 0.01f, .kd = 0.3f,
        .output_min = -250, .output_max = 250,
        .integral_max = 500.0f, .dead_zone = 10
    };
    PID_Init(&pid_steer, &steer_param);

    /* 前进PID — 垂直偏差/距离 */
    PID_Param fwd_param = {
        .kp = 0.5f, .ki = 0.005f, .kd = 0.2f,
        .output_min = -400, .output_max = 400,
        .integral_max = 800.0f, .dead_zone = 20
    };
    PID_Init(&pid_forward, &fwd_param);
}

/* ══════════════════════════════════════════════════════════════
 *  主函数
 * ══════════════════════════════════════════════════════════════ */
int main(void)
{
    System_Init();

    Motor_Init_HW();
    Encoder_Init_HW();
    UART_Init_HW();
    PID_Init_All();

    OLED_Init(I2C_0_INST);
    OLED_Clear();
    OLED_ShowString(0, 0, "Vision Car Ready", 12, 1);
    OLED_Refresh();

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    printf("=== 视觉引导小车启动 ===\n");
    printf("等待OPi5视觉数据...\n");
    printf("协议: $T,x,y,area,type  或  $L,x1,y1,x2,y2\n");

    uint32_t last_ctrl_tick = 0;

    while (1) {
        /* 解析视觉帧 */
        Parse_Vision_Frame();

        /* 每20ms执行一次控制 */
        if (sys_tick_ms - last_ctrl_tick >= 20) {
            last_ctrl_tick = sys_tick_ms;

            switch (current_mode) {
            case MODE_LINE_FOLLOW:
                LineFollow_Control();
                break;
            case MODE_TARGET_TRACK:
                TargetTrack_Control();
                break;
            case MODE_APRILTAG:
                AprilTag_Control();
                break;
            }
        }

        /* OLED刷新 */
        Display_Status();
    }

    return 0;
}
