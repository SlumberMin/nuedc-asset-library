/**
 * @file ir_remote_car.c
 * @brief MSPM0G3507 红外遥控小车
 *
 * 功能：NEC协议红外遥控 + L298N双路电机驱动 + 速度调节 + 方向控制
 * 红外遥控按键映射(标准NEC遥控器):
 *   CH-: 停止    CH:  前进    CH+: 后退
 *   |<<: 左转    >>|: 右转    >||: 直行
 *   VOL+: 加速   VOL-: 减速   EQ:  原地旋转
 *   0~9: 设置速度级别(0=停止, 9=最快)
 *
 * 硬件连接：
 *   IR接收: OUT=PA14 (E18-38K)
 *   L298N电机驱动:
 *     左电机: IN1=PA0, IN2=PA1, ENA=PA4 (PWM)
 *     右电机: IN3=PA2, IN4=PA3, ENB=PA5 (PWM)
 *   LED指示: PB0 (状态)
 *
 * @author 电赛资产库
 * @date 2026
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>

/* ========== 红外遥控 NEC协议解码 ========== */

/* NEC协议参数 (单位: 微秒) */
#define IR_LEADER_HIGH_MIN   8500
#define IR_LEADER_HIGH_MAX   9500
#define IR_LEADER_LOW_MIN    4000
#define IR_LEADER_LOW_MAX    5000
#define IR_BIT_HIGH_MIN      400
#define IR_BIT_HIGH_MAX      700
#define IR_BIT_0_LOW_MIN     400
#define IR_BIT_0_LOW_MAX     700
#define IR_BIT_1_LOW_MIN     1500
#define IR_BIT_1_LOW_MAX     1800

/* 遥控器按键码 (通用NEC协议, 根据实际遥控器修改) */
#define IR_ADDR         0x00     /* 地址码 */
#define IR_KEY_CH_MINUS 0x45
#define IR_KEY_CH       0x46
#define IR_KEY_CH_PLUS  0x47
#define IR_KEY_PREV     0x44
#define IR_KEY_NEXT     0x40
#define IR_KEY_PLAY     0x43
#define IR_KEY_VOL_MINUS 0x07
#define IR_KEY_VOL_PLUS  0x15
#define IR_KEY_EQ       0x09
#define IR_KEY_0        0x16
#define IR_KEY_1        0x0C
#define IR_KEY_2        0x18
#define IR_KEY_3        0x5E
#define IR_KEY_4        0x08
#define IR_KEY_5        0x1C
#define IR_KEY_6        0x5A
#define IR_KEY_7        0x42
#define IR_KEY_8        0x52
#define IR_KEY_9        0x4A

/* IR接收状态机 */
typedef enum {
    IR_STATE_IDLE,
    IR_STATE_LEADER_HIGH,
    IR_STATE_LEADER_LOW,
    IR_STATE_DATA_HIGH,
    IR_STATE_DATA_LOW
} IR_State_t;

static volatile IR_State_t g_irState = IR_STATE_IDLE;
static volatile uint32_t g_irTimestamp = 0;
static volatile uint32_t g_irData = 0;
static volatile uint8_t  g_irBitCount = 0;
static volatile bool     g_irReady = false;
static volatile uint32_t g_irCode = 0;
static volatile uint32_t g_irLastTime = 0;

/* 微秒计时器 */
static volatile uint32_t g_tickUs = 0;

/* 定时器中断: 1us计数 */
void TIMG0_IRQHandler(void) {
    if (DL_TimerG_getPendingInterrupt(TIMG0) == DL_TIMER_IIDX_ZERO) {
        g_tickUs++;
    }
}

static uint32_t micros(void) {
    return g_tickUs;
}

/* ========== IR解码中断 ========== */

/**
 * @brief 外部中断: IR信号边沿触发
 * 使用GPIO中断捕获NEC协议的高低电平持续时间
 */
void GROUP1_IRQHandler(void) {
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(GPIOA);

    if (flags & DL_GPIO_PIN_14) {
        uint32_t now = micros();
        bool level = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_14) ? true : false;
        uint32_t duration = now - g_irTimestamp;
        g_irTimestamp = now;

        if (level) {
            /* 上升沿: 低电平结束 */
            switch (g_irState) {
                case IR_STATE_LEADER_LOW:
                    if (duration >= IR_LEADER_LOW_MIN && duration <= IR_LEADER_LOW_MAX) {
                        g_irState = IR_STATE_DATA_HIGH;
                        g_irData = 0;
                        g_irBitCount = 0;
                    } else {
                        g_irState = IR_STATE_IDLE;
                    }
                    break;

                case IR_STATE_DATA_LOW:
                    /* 判断数据位: 0或1 */
                    if (duration >= IR_BIT_0_LOW_MIN && duration <= IR_BIT_0_LOW_MAX) {
                        /* 逻辑0: 560us低 */
                        g_irData <<= 1;
                        g_irBitCount++;
                    } else if (duration >= IR_BIT_1_LOW_MIN && duration <= IR_BIT_1_LOW_MAX) {
                        /* 逻辑1: 1690us低 */
                        g_irData = (g_irData << 1) | 1;
                        g_irBitCount++;
                    } else {
                        g_irState = IR_STATE_IDLE;
                    }

                    if (g_irBitCount >= 32) {
                        g_irCode = g_irData;
                        g_irReady = true;
                        g_irState = IR_STATE_IDLE;
                    } else if (g_irState != IR_STATE_IDLE) {
                        g_irState = IR_STATE_DATA_HIGH;
                    }
                    break;

                default:
                    break;
            }
        } else {
            /* 下降沿: 高电平开始 */
            if (duration >= IR_LEADER_HIGH_MIN && duration <= IR_LEADER_HIGH_MAX) {
                g_irState = IR_STATE_LEADER_LOW;
            } else if (g_irState == IR_STATE_DATA_HIGH &&
                       duration >= IR_BIT_HIGH_MIN && duration <= IR_BIT_HIGH_MAX) {
                g_irState = IR_STATE_DATA_LOW;
            } else {
                g_irState = IR_STATE_IDLE;
            }
        }

        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_14);
    }
}

/**
 * @brief 解析NEC码
 * @param code 32位NEC数据
 * @param addr 输出地址码
 * @param cmd 输出命令码
 * @return true=有效(重复码检测)
 */
static bool ir_decode(uint32_t code, uint8_t *addr, uint8_t *cmd) {
    uint8_t a  = (code >> 24) & 0xFF;
    uint8_t an = (code >> 16) & 0xFF;
    uint8_t c  = (code >> 8) & 0xFF;
    uint8_t cn = code & 0xFF;

    /* 验证反码 */
    if ((a ^ an) != 0xFF || (c ^ cn) != 0xFF) return false;

    *addr = a;
    *cmd = c;
    return true;
}

/* ========== 电机驱动 ========== */

/* L298N引脚 */
#define MOTOR_L_IN1_PORT    GPIOA
#define MOTOR_L_IN1_PIN     DL_GPIO_PIN_0
#define MOTOR_L_IN2_PORT    GPIOA
#define MOTOR_L_IN2_PIN     DL_GPIO_PIN_1
#define MOTOR_R_IN3_PORT    GPIOA
#define MOTOR_R_IN3_PIN     DL_GPIO_PIN_2
#define MOTOR_R_IN4_PORT    GPIOA
#define MOTOR_R_IN4_PIN     DL_GPIO_PIN_3

/* PWM周期值 (假设8MHz定时器, 1kHz PWM) */
#define PWM_PERIOD          7999

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_SPIN_LEFT,
    MOTOR_SPIN_RIGHT
} MotorAction_t;

static uint8_t g_speed = 60;       /* 速度百分比 0~100 */
#define SPEED_MIN  20
#define SPEED_MAX  100
#define SPEED_STEP 10

/**
 * @brief 设置电机PWM占空比
 */
static void motor_set_pwm(bool left, uint8_t duty) {
    uint32_t compare = (uint32_t)PWM_PERIOD * duty / 100;
    if (left) {
        DL_TimerG_setCaptureCompareValue(TIMER0, compare, DL_TIMER_CC_0_INDEX);
    } else {
        DL_TimerG_setCaptureCompareValue(TIMER0, compare, DL_TIMER_CC_1_INDEX);
    }
}

/**
 * @brief 电机动作控制
 */
static void motor_control(MotorAction_t action) {
    switch (action) {
        case MOTOR_STOP:
            DL_GPIO_clearPins(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN);
            DL_GPIO_clearPins(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN);
            DL_GPIO_clearPins(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN);
            DL_GPIO_clearPins(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN);
            motor_set_pwm(true, 0);
            motor_set_pwm(false, 0);
            break;

        case MOTOR_FORWARD:
            DL_GPIO_setPins(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN);
            DL_GPIO_clearPins(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN);
            DL_GPIO_setPins(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN);
            DL_GPIO_clearPins(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN);
            motor_set_pwm(true, g_speed);
            motor_set_pwm(false, g_speed);
            break;

        case MOTOR_BACKWARD:
            DL_GPIO_clearPins(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN);
            DL_GPIO_setPins(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN);
            DL_GPIO_clearPins(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN);
            DL_GPIO_setPins(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN);
            motor_set_pwm(true, g_speed);
            motor_set_pwm(false, g_speed);
            break;

        case MOTOR_LEFT:
            /* 左轮减速/停, 右轮正常 */
            DL_GPIO_setPins(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN);
            DL_GPIO_clearPins(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN);
            DL_GPIO_setPins(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN);
            DL_GPIO_clearPins(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN);
            motor_set_pwm(true, g_speed / 3);   /* 内轮减速 */
            motor_set_pwm(false, g_speed);
            break;

        case MOTOR_RIGHT:
            DL_GPIO_setPins(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN);
            DL_GPIO_clearPins(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN);
            DL_GPIO_setPins(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN);
            DL_GPIO_clearPins(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN);
            motor_set_pwm(true, g_speed);
            motor_set_pwm(false, g_speed / 3);  /* 内轮减速 */
            break;

        case MOTOR_SPIN_LEFT:
            /* 原地左转: 左轮后退, 右轮前进 */
            DL_GPIO_clearPins(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN);
            DL_GPIO_setPins(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN);
            DL_GPIO_setPins(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN);
            DL_GPIO_clearPins(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN);
            motor_set_pwm(true, g_speed);
            motor_set_pwm(false, g_speed);
            break;

        case MOTOR_SPIN_RIGHT:
            DL_GPIO_setPins(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN);
            DL_GPIO_clearPins(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN);
            DL_GPIO_clearPins(MOTOR_R_IN3_PORT, MOTOR_R_IN3_PIN);
            DL_GPIO_setPins(MOTOR_R_IN4_PORT, MOTOR_R_IN4_PIN);
            motor_set_pwm(true, g_speed);
            motor_set_pwm(false, g_speed);
            break;
    }
}

/* ========== LED指示 ========== */
static void led_blink(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_0);
        for (volatile uint32_t j = 0; j < 200000; j++);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0);
        for (volatile uint32_t j = 0; j < 200000; j++);
    }
}

/* ========== 主函数 ========== */
int main(void) {
    /* 初始化系统 */
    SYSCFG_DL_init();

    /* 启动微秒定时器 */
    NVIC_EnableIRQ(TIMG0_IRQn);
    DL_TimerG_startCounter(TIMG0);

    /* 配置IR接收GPIO中断 */
    DL_GPIO_enableInterrupt(GPIOA, DL_GPIO_PIN_14);
    NVIC_EnableIRQ(GROUP1_IRQn);

    /* 启动PWM (电机速度控制) */
    DL_TimerG_startCounter(TIMER0);

    /* 初始状态: 电机停止 */
    motor_control(MOTOR_STOP);

    /* 启动指示 */
    led_blink(3);

    /* IR解码变量 */
    uint8_t addr, cmd;
    MotorAction_t currentAction = MOTOR_STOP;
    uint32_t lastCmdTime = 0;
    const uint32_t TIMEOUT_MS = 200;  /* 遥控超时自动停止 */

    /* 主循环 */
    while (1) {
        /* 处理红外接收 */
        if (g_irReady) {
            g_irReady = false;
            uint32_t code = g_irCode;

            if (ir_decode(code, &addr, &cmd)) {
                lastCmdTime = g_tickUs / 1000;  /* 转毫秒 */

                switch (cmd) {
                    case IR_KEY_CH:
                        currentAction = MOTOR_FORWARD;
                        break;
                    case IR_KEY_CH_PLUS:
                        currentAction = MOTOR_BACKWARD;
                        break;
                    case IR_KEY_PREV:
                        currentAction = MOTOR_LEFT;
                        break;
                    case IR_KEY_NEXT:
                        currentAction = MOTOR_RIGHT;
                        break;
                    case IR_KEY_CH_MINUS:
                        currentAction = MOTOR_STOP;
                        break;
                    case IR_KEY_PLAY:
                        currentAction = MOTOR_STOP;
                        break;
                    case IR_KEY_EQ:
                        currentAction = MOTOR_SPIN_LEFT;
                        break;
                    case IR_KEY_VOL_PLUS:
                        g_speed += SPEED_STEP;
                        if (g_speed > SPEED_MAX) g_speed = SPEED_MAX;
                        break;
                    case IR_KEY_VOL_MINUS:
                        if (g_speed > SPEED_MIN)
                            g_speed -= SPEED_STEP;
                        else
                            g_speed = SPEED_MIN;
                        break;

                    /* 数字键: 直接设置速度级别 */
                    case IR_KEY_0: g_speed = 0;   currentAction = MOTOR_STOP; break;
                    case IR_KEY_1: g_speed = 10;  break;
                    case IR_KEY_2: g_speed = 20;  break;
                    case IR_KEY_3: g_speed = 30;  break;
                    case IR_KEY_4: g_speed = 40;  break;
                    case IR_KEY_5: g_speed = 50;  break;
                    case IR_KEY_6: g_speed = 60;  break;
                    case IR_KEY_7: g_speed = 70;  break;
                    case IR_KEY_8: g_speed = 80;  break;
                    case IR_KEY_9: g_speed = 90;  break;

                    default:
                        break;
                }

                motor_control(currentAction);
            }
        }

        /* 超时检测: 超过一段时间无指令自动停车 */
        if (currentAction != MOTOR_STOP &&
            (g_tickUs / 1000 - lastCmdTime) > TIMEOUT_MS) {
            currentAction = MOTOR_STOP;
            motor_control(MOTOR_STOP);
        }

        /* 重复码处理: NEC协议按住不放会发重复码 */
        if (g_irReady == false && g_irCode == 0xFFFFFFFF) {
            /* 重复码: 维持当前动作, 刷新超时 */
            lastCmdTime = g_tickUs / 1000;
            g_irCode = 0;
        }
    }
}
