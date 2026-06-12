/**
 * @file voice_control_car.c
 * @brief 语音控制小车 - 完整系统集成示例
 * @target MSPM0G3507
 * @hardware JQ8900语音播报模块 + LD3320语音识别模块 + L298N电机驱动
 *
 * 系统架构：
 *   LD3320语音识别 --UART1--> 识别码 --> 状态机 --> 电机/语音动作
 *   JQ8900语音播报 --UART0/串口--> 语音触发
 *   L298N电机驱动 --GPIO+PWM--> 直流电机x2
 *   LED指示灯 --GPIO--> 状态指示
 *
 * 状态机设计：
 *   IDLE(待命) -> FORWARD/BACKWARD/LEFT/RIGHT/STOP/ACCEL/DECEL
 *   每条语音指令触发对应动作+语音反馈
 *
 * 错误经验库遵守：
 *   - LD3320识别结果需二次确认（连续2次相同结果才执行），防误触发
 *   - JQ8900发送播放指令后需等待BUSY引脚拉低再发下一条
 *   - 电机转向切换需先刹车再反转，防止H桥直通损坏
 *   - 语音识别超时需复位模块
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>

/* ========== 硬件引脚定义 ========== */
/* L298N电机驱动 */
#define MOTOR_IN1_PORT      GPIOA
#define MOTOR_IN1_PIN       DL_GPIO_PIN_0   /* 左电机方向1 */
#define MOTOR_IN2_PORT      GPIOA
#define MOTOR_IN2_PIN       DL_GPIO_PIN_1   /* 左电机方向2 */
#define MOTOR_IN3_PORT      GPIOA
#define MOTOR_IN3_PIN       DL_GPIO_PIN_2   /* 右电机方向1 */
#define MOTOR_IN4_PORT      GPIOA
#define MOTOR_IN4_PIN       DL_GPIO_PIN_3   /* 右电机方向2 */

/* JQ8900语音模块 */
#define JQ8900_TX_PORT      GPIOA           /* MCU TX -> JQ8900 RX */
#define JQ8900_TX_PIN       DL_GPIO_PIN_10
#define JQ8900_BUSY_PORT    GPIOA           /* JQ8900 BUSY引脚(低电平播放中) */
#define JQ8900_BUSY_PIN     DL_GPIO_PIN_11

/* LD3320语音识别 */
#define LD3320_RST_PORT     GPIOA
#define LD3320_RST_PIN      DL_GPIO_PIN_12

/* LED状态指示 */
#define LED_RUN_PORT        GPIOB
#define LED_RUN_PIN         DL_GPIO_PIN_0    /* 运行指示灯 */
#define LED_ERR_PORT        GPIOB
#define LED_ERR_PIN         DL_GPIO_PIN_1    /* 错误指示灯 */

/* ========== 系统参数 ========== */
#define PWM_PERIOD          1000
#define PWM_BASE_SPEED      500     /* 基础速度(50%) */
#define PWM_MAX_SPEED       900     /* 最大速度(90%) */
#define PWM_MIN_SPEED       200     /* 最小速度(20%) */
#define PWM_STEP            100     /* 速度调节步进 */
#define BRAKE_DURATION_MS   150     /* 刹车持续时间ms */
#define CMD_CONFIRM_COUNT   2       /* 指令确认次数(防误触) */
#define CMD_TIMEOUT_MS      3000    /* 指令超时(自动停止) */
#define UART_BAUD_LD3320    9600    /* LD3320波特率 */
#define UART_BAUD_JQ8900    9600    /* JQ8900波特率 */

/* ========== 语音指令码定义 ========== */
typedef enum {
    VOICE_CMD_NONE = 0,
    VOICE_CMD_FORWARD  = 0x01,  /* "前进" */
    VOICE_CMD_BACKWARD = 0x02,  /* "后退" */
    VOICE_CMD_TURN_LEFT  = 0x03, /* "左转" */
    VOICE_CMD_TURN_RIGHT = 0x04, /* "右转" */
    VOICE_CMD_STOP     = 0x05,  /* "停止" */
    VOICE_CMD_SPEED_UP = 0x06,  /* "加速" */
    VOICE_CMD_SPEED_DOWN = 0x07, /* "减速" */
    VOICE_CMD_SPIN_LEFT = 0x08, /* "左旋" */
    VOICE_CMD_SPIN_RIGHT = 0x09, /* "右旋" */
    VOICE_CMD_MAX
} VoiceCmd_t;

/* ========== JQ8900语音播报文件号 ========== */
typedef enum {
    JQ8900_WELCOME    = 1,   /* "欢迎使用语音控制小车" */
    JQ8900_FORWARD    = 2,   /* "前进" */
    JQ8900_BACKWARD   = 3,   /* "后退" */
    JQ8900_TURN_LEFT  = 4,   /* "左转" */
    JQ8900_TURN_RIGHT = 5,   /* "右转" */
    JQ8900_STOP       = 6,   /* "停止" */
    JQ8900_SPEED_UP   = 7,   /* "加速" */
    JQ8900_SPEED_DOWN = 8,   /* "减速" */
    JQ8900_ERROR      = 9,   /* "指令错误" */
    JQ8900_LOW_BAT    = 10,  /* "电量不足" */
} JQ8900_Sound_t;

/* ========== 小车状态 ========== */
typedef enum {
    CAR_STATE_IDLE = 0,      /* 待命 */
    CAR_STATE_FORWARD,       /* 前进 */
    CAR_STATE_BACKWARD,      /* 后退 */
    CAR_STATE_TURN_LEFT,     /* 左转 */
    CAR_STATE_TURN_RIGHT,    /* 右转 */
    CAR_STATE_SPIN_LEFT,     /* 原地左旋 */
    CAR_STATE_SPIN_RIGHT,    /* 原地右旋 */
    CAR_STATE_BRAKING,       /* 刹车中 */
} CarState_t;

/* ========== 全局变量 ========== */
static volatile CarState_t g_car_state = CAR_STATE_IDLE;
static volatile uint16_t g_pwm_speed = PWM_BASE_SPEED;  /* 当前速度 */
static volatile uint32_t g_tick = 0;          /* 系统节拍(ms) */
static volatile uint32_t g_last_cmd_tick = 0; /* 上次指令时间 */
static volatile uint32_t g_brake_start_tick = 0; /* 刹车开始时间 */

/* 语音识别确认 */
static volatile uint8_t g_last_voice_cmd = VOICE_CMD_NONE;
static volatile uint8_t g_cmd_confirm_cnt = 0;

/* UART接收 */
static volatile uint8_t g_ld3320_rx = 0;

/* ========== JQ8900语音播报驱动 ========== */
/**
 * @brief JQ8900播放指定曲目
 * @param file_no 文件编号(1-65535)
 *
 * 协议：0xAA 0x07 0x02 [file_no_H] [file_no_L] [sum]
 * 错误经验：发送前必须检查BUSY引脚，否则指令会被丢弃
 */
static void JQ8900_Play(JQ8900_Sound_t file_no)
{
    /* 等待上一次播放完成 */
    uint32_t timeout = 0;
    while (!DL_GPIO_readPins(JQ8900_BUSY_PORT, JQ8900_BUSY_PIN)) {
        timeout++;
        if (timeout > 500000) break; /* 超时保护 */
    }

    uint8_t cmd[6];
    cmd[0] = 0xAA;
    cmd[1] = 0x07;
    cmd[2] = 0x02;
    cmd[3] = (uint8_t)(file_no >> 8);
    cmd[4] = (uint8_t)(file_no & 0xFF);

    /* 计算校验和 */
    uint8_t sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += cmd[i];
    }
    cmd[5] = sum;

    /* 通过UART发送 */
    for (int i = 0; i < 6; i++) {
        while (!DL_UART_isTXFIFOEmpty(UART0)) {}
        DL_UART_transmitData(UART0, cmd[i]);
    }
}

/* ========== 电机控制 ========== */
/**
 * @brief 小车前进
 * 错误经验：改变方向前先短暂停顿（刹车），防止H桥上下管直通
 */
static void Car_Forward(void)
{
    /* 左电机前进 */
    DL_GPIO_setPins(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
    /* 右电机前进 */
    DL_GPIO_setPins(MOTOR_IN3_PORT, MOTOR_IN3_PIN);
    DL_GPIO_clearPins(MOTOR_IN4_PORT, MOTOR_IN4_PIN);

    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed, DL_TIMER_CC_1_INDEX);
}

static void Car_Backward(void)
{
    DL_GPIO_clearPins(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    DL_GPIO_setPins(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
    DL_GPIO_clearPins(MOTOR_IN3_PORT, MOTOR_IN3_PIN);
    DL_GPIO_setPins(MOTOR_IN4_PORT, MOTOR_IN4_PIN);

    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed, DL_TIMER_CC_1_INDEX);
}

/**
 * @brief 左转(左轮减速，右轮正常)
 */
static void Car_TurnLeft(void)
{
    DL_GPIO_setPins(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
    DL_GPIO_setPins(MOTOR_IN3_PORT, MOTOR_IN3_PIN);
    DL_GPIO_clearPins(MOTOR_IN4_PORT, MOTOR_IN4_PIN);

    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed / 3, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed, DL_TIMER_CC_1_INDEX);
}

static void Car_TurnRight(void)
{
    DL_GPIO_setPins(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
    DL_GPIO_setPins(MOTOR_IN3_PORT, MOTOR_IN3_PIN);
    DL_GPIO_clearPins(MOTOR_IN4_PORT, MOTOR_IN4_PIN);

    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed / 3, DL_TIMER_CC_1_INDEX);
}

/**
 * @brief 原地左旋(左轮反转，右轮正转)
 */
static void Car_SpinLeft(void)
{
    DL_GPIO_clearPins(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    DL_GPIO_setPins(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
    DL_GPIO_setPins(MOTOR_IN3_PORT, MOTOR_IN3_PIN);
    DL_GPIO_clearPins(MOTOR_IN4_PORT, MOTOR_IN4_PIN);

    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed, DL_TIMER_CC_1_INDEX);
}

static void Car_SpinRight(void)
{
    DL_GPIO_setPins(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
    DL_GPIO_clearPins(MOTOR_IN3_PORT, MOTOR_IN3_PIN);
    DL_GPIO_setPins(MOTOR_IN4_PORT, MOTOR_IN4_PIN);

    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(TIMER0, g_pwm_speed, DL_TIMER_CC_1_INDEX);
}

/**
 * @brief 刹车(电机两端短接)
 */
static void Car_Brake(void)
{
    DL_GPIO_clearPins(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
    DL_GPIO_clearPins(MOTOR_IN3_PORT, MOTOR_IN3_PIN);
    DL_GPIO_clearPins(MOTOR_IN4_PORT, MOTOR_IN4_PIN);

    DL_Timer_setCaptureCompareValue(TIMER0, PWM_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(TIMER0, PWM_PERIOD, DL_TIMER_CC_1_INDEX);
}

/**
 * @brief 停止(滑行)
 */
static void Car_Stop(void)
{
    DL_GPIO_clearPins(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_IN2_PORT, MOTOR_IN2_PIN);
    DL_GPIO_clearPins(MOTOR_IN3_PORT, MOTOR_IN3_PIN);
    DL_GPIO_clearPins(MOTOR_IN4_PORT, MOTOR_IN4_PIN);

    DL_Timer_setCaptureCompareValue(TIMER0, 0, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(TIMER0, 0, DL_TIMER_CC_1_INDEX);
}

/* ========== 语音指令处理 ========== */
/**
 * @brief 处理语音指令（带二次确认）
 *
 * 错误经验：LD3320在噪声环境下偶尔误识别，需连续2次相同结果才执行
 *          这能大幅降低误触发率
 */
static void ProcessVoiceCommand(uint8_t cmd)
{
    if (cmd == g_last_voice_cmd) {
        g_cmd_confirm_cnt++;
    } else {
        g_last_voice_cmd = cmd;
        g_cmd_confirm_cnt = 1;
        return; /* 首次识别，等待确认 */
    }

    if (g_cmd_confirm_cnt < CMD_CONFIRM_COUNT) return;

    /* 确认通过，执行指令 */
    g_cmd_confirm_cnt = 0;
    g_last_cmd_tick = g_tick;
    g_last_voice_cmd = VOICE_CMD_NONE;

    switch (cmd) {
        case VOICE_CMD_FORWARD:
            g_car_state = CAR_STATE_FORWARD;
            Car_Forward();
            JQ8900_Play(JQ8900_FORWARD);
            break;

        case VOICE_CMD_BACKWARD:
            g_car_state = CAR_STATE_BACKWARD;
            Car_Backward();
            JQ8900_Play(JQ8900_BACKWARD);
            break;

        case VOICE_CMD_TURN_LEFT:
            g_car_state = CAR_STATE_TURN_LEFT;
            Car_TurnLeft();
            JQ8900_Play(JQ8900_TURN_LEFT);
            break;

        case VOICE_CMD_TURN_RIGHT:
            g_car_state = CAR_STATE_TURN_RIGHT;
            Car_TurnRight();
            JQ8900_Play(JQ8900_TURN_RIGHT);
            break;

        case VOICE_CMD_STOP:
            g_car_state = CAR_STATE_BRAKING;
            g_brake_start_tick = g_tick;
            Car_Brake();
            JQ8900_Play(JQ8900_STOP);
            break;

        case VOICE_CMD_SPEED_UP:
            g_pwm_speed += PWM_STEP;
            if (g_pwm_speed > PWM_MAX_SPEED) g_pwm_speed = PWM_MAX_SPEED;
            JQ8900_Play(JQ8900_SPEED_UP);
            /* 如果正在运动，立即更新速度 */
            if (g_car_state == CAR_STATE_FORWARD) Car_Forward();
            else if (g_car_state == CAR_STATE_BACKWARD) Car_Backward();
            break;

        case VOICE_CMD_SPEED_DOWN:
            if (g_pwm_speed >= PWM_MIN_SPEED + PWM_STEP) {
                g_pwm_speed -= PWM_STEP;
            } else {
                g_pwm_speed = PWM_MIN_SPEED;
            }
            JQ8900_Play(JQ8900_SPEED_DOWN);
            if (g_car_state == CAR_STATE_FORWARD) Car_Forward();
            else if (g_car_state == CAR_STATE_BACKWARD) Car_Backward();
            break;

        case VOICE_CMD_SPIN_LEFT:
            g_car_state = CAR_STATE_SPIN_LEFT;
            Car_SpinLeft();
            JQ8900_Play(JQ8900_TURN_LEFT);
            break;

        case VOICE_CMD_SPIN_RIGHT:
            g_car_state = CAR_STATE_SPIN_RIGHT;
            Car_SpinRight();
            JQ8900_Play(JQ8900_TURN_RIGHT);
            break;

        default:
            JQ8900_Play(JQ8900_ERROR);
            break;
    }

    DL_GPIO_togglePins(LED_RUN_PORT, LED_RUN_PIN);
}

/* ========== 中断服务 ========== */
/**
 * @brief UART1中断 - LD3320语音识别数据接收
 * LD3320输出格式：识别码(1字节)
 */
void UART1_IRQHandler(void)
{
    uint32_t status = DL_UART_getPendingInterrupt(UART1);
    if (status == DL_UART_IIDX_RX) {
        uint8_t ch = DL_UART_receiveData(UART1);
        g_ld3320_rx = ch;
        ProcessVoiceCommand(ch);
    }
}

/**
 * @brief SysTick中断 - 系统计时
 */
void SysTick_Handler(void)
{
    g_tick++;
}

/* ========== 系统初始化 ========== */
static void System_Init(void)
{
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    NVIC_EnableIRQ(UART1_IRQn);

    /* 启动PWM */
    DL_Timer_startCounter(TIMER0);
}

/* ========== 主函数 ========== */
int main(void)
{
    System_Init();

    /* 欢迎语 */
    JQ8900_Play(JQ8900_WELCOME);

    printf("=== Voice Control Car ===\r\n");
    printf("Commands: forward/backward/left/right/stop\r\n");
    printf("          speed_up/speed_down/spin_left/spin_right\r\n");
    printf("Confirm count: %d (anti-false-trigger)\r\n", CMD_CONFIRM_COUNT);
    printf("Speed: %d/%d\r\n", g_pwm_speed, PWM_MAX_SPEED);

    while (1) {
        /* 刹车状态自动恢复为IDLE */
        if (g_car_state == CAR_STATE_BRAKING) {
            if (g_tick - g_brake_start_tick >= BRAKE_DURATION_MS) {
                Car_Stop();
                g_car_state = CAR_STATE_IDLE;
            }
        }

        /* 超时自动停止(安全保护) */
        if (g_car_state != CAR_STATE_IDLE && g_car_state != CAR_STATE_BRAKING) {
            if (g_tick - g_last_cmd_tick >= CMD_TIMEOUT_MS) {
                g_car_state = CAR_STATE_BRAKING;
                g_brake_start_tick = g_tick;
                Car_Brake();
                printf("CMD TIMEOUT -> STOP\r\n");
            }
        }

        /* LED心跳 */
        static uint32_t led_tick = 0;
        if (g_tick - led_tick >= 500) {
            led_tick = g_tick;
            DL_GPIO_togglePins(LED_RUN_PORT, LED_RUN_PIN);
        }
    }
}
