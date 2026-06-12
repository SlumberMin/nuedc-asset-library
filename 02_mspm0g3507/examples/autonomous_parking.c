/**
 * @file autonomous_parking.c
 * @brief 自动泊车系统 - 完整系统集成示例
 * @target MSPM0G3507
 * @hardware HC-SR04超声波x3(前/左/右) + TB6612电机驱动 + 舵机转向 + OLED显示
 *
 * 系统架构：
 *   超声波(前) --Trig/Echo--> 前方障碍物距离
 *   超声波(左) --Trig/Echo--> 左侧距离(检测车位)
 *   超声波(右) --Trig/Echo--> 右侧距离
 *   舵机 --PWM--> 前轮转向
 *   TB6612 --PWM+GPIO--> 后轮驱动
 *   OLED --I2C--> 状态显示
 *
 * 泊车策略(侧方位停车)：
 *   1. 沿路缓行，左侧超声波检测车位(距离突增>车位阈值)
 *   2. 车位长度满足要求后，执行侧方停车动作序列
 *   3. 前进+右打 -> 后退+左打 -> 后退回正 -> 微调对齐
 *
 * 错误经验库遵守：
 *   - 超声波测量需设置超时(~30ms)，否则无回波时会死等
 *   - 多个超声波不能同时触发，会互相干扰，需顺序测量
 *   - 转向舵机需中间位置校准，不同舵机中位差异大
 *   - 泊车每步需检查前方距离，防止碰撞
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== 硬件引脚定义 ========== */
/* HC-SR04超声波(前) */
#define US_FRONT_TRIG_PORT      GPIOA
#define US_FRONT_TRIG_PIN       DL_GPIO_PIN_0
#define US_FRONT_ECHO_PORT      GPIOA
#define US_FRONT_ECHO_PIN       DL_GPIO_PIN_1

/* HC-SR04超声波(左) */
#define US_LEFT_TRIG_PORT       GPIOA
#define US_LEFT_TRIG_PIN        DL_GPIO_PIN_2
#define US_LEFT_ECHO_PORT       GPIOA
#define US_LEFT_ECHO_PIN        DL_GPIO_PIN_3

/* HC-SR04超声波(右) */
#define US_RIGHT_TRIG_PORT      GPIOA
#define US_RIGHT_TRIG_PIN       DL_GPIO_PIN_4
#define US_RIGHT_ECHO_PORT      GPIOA
#define US_RIGHT_ECHO_PIN       DL_GPIO_PIN_5

/* TB6612电机驱动 */
#define MOTOR_AIN1_PORT         GPIOA
#define MOTOR_AIN1_PIN          DL_GPIO_PIN_6
#define MOTOR_AIN2_PORT         GPIOA
#define MOTOR_AIN2_PIN          DL_GPIO_PIN_7

/* 转向舵机(通过TIMER0 CCR0输出PWM) */
#define SERVO_STEER_CHANNEL     DL_TIMER_CC_0_INDEX

/* ========== 系统参数 ========== */
#define SYSTEM_CLOCK_HZ         32000000UL  /* 32MHz系统时钟 */
#define SOUND_SPEED_CM_US       58          /* 声速: 1cm = 58us往返 */
#define US_TIMEOUT_US           30000       /* 超声波超时30ms (~5m) */
#define US_MEASURE_INTERVAL_MS  60          /* 测量间隔(避免串扰) */

#define SERVO_CENTER            1500        /* 舵机中位(1.5ms) */
#define SERVO_LEFT_MAX          2100        /* 左打最大(2.1ms) */
#define SERVO_RIGHT_MAX         900         /* 右打最大(0.9ms) */
#define SERVO_STEP              50          /* 转向步进 */

#define MOTOR_SPEED_CRUISE      400         /* 巡航速度 */
#define MOTOR_SPEED_PARKING     350         /* 泊车速度 */
#define MOTOR_SPEED_REVERSE     350         /* 倒车速度 */
#define PWM_PERIOD              1000

/* 泊车参数 */
#define PARKING_SLOT_DEPTH_CM   40          /* 车位深度阈值(cm) */
#define PARKING_SLOT_LENGTH_CM  60          /* 车位最小长度(cm) */
#define PARKING_DISTANCE_CM     20          /* 泊车安全距离(cm) */
#define FRONT_STOP_CM           15          /* 前方紧急停止距离 */

/* ========== 泊车状态机 ========== */
typedef enum {
    PARK_IDLE = 0,              /* 待命 */
    PARK_CRUISING,              /* 巡航找车位 */
    PARK_SLOT_FOUND,            /* 发现车位 */
    PARK_STEP1_FORWARD_RIGHT,   /* 步骤1: 前进+右打 */
    PARK_STEP2_REVERSE_LEFT,    /* 步骤2: 倒车+左打 */
    PARK_STEP3_REVERSE_STRAIGHT,/* 步骤3: 倒车回正 */
    PARK_STEP4_ADJUST,          /* 步骤4: 微调对齐 */
    PARK_COMPLETE,              /* 泊车完成 */
    PARK_ABORT,                 /* 泊车中止(障碍物) */
    PARK_EXITING                /* 驶出车位 */
} ParkState_t;

/* ========== 全局变量 ========== */
static volatile uint32_t g_tick = 0;

/* 超声波距离(cm) */
static volatile float g_dist_front = 999.0f;
static volatile float g_dist_left  = 999.0f;
static volatile float g_dist_right = 999.0f;

/* 舵机转向角度(中位=1500) */
static volatile uint16_t g_steer_pos = SERVO_CENTER;

/* 泊车状态 */
static volatile ParkState_t g_park_state = PARK_IDLE;

/* 车位检测 */
static volatile float g_slot_start_left = 0;   /* 进入车位时左侧距离 */
static volatile uint32_t g_slot_start_tick = 0; /* 进入车位时时间戳 */
static volatile uint8_t g_slot_detected = 0;

/* 泊车步骤计时 */
static volatile uint32_t g_step_start_tick = 0;

/* ========== 微秒级延时 ========== */
/**
 * @brief 微秒级延时(SysTick轮询)
 * 错误经验：微秒延时不能用SysTick中断做，中断延迟太大
 *          必须用忙等循环或TIMER计数
 */
static void delay_us(uint32_t us)
{
    uint32_t start = SysTick->VAL;
    uint32_t ticks = us * (SYSTEM_CLOCK_HZ / 1000000);
    while (1) {
        uint32_t now = SysTick->VAL;
        uint32_t elapsed;
        if (start >= now) {
            elapsed = start - now;
        } else {
            elapsed = start + (SysTick->LOAD + 1) - now;
        }
        if (elapsed >= ticks) break;
    }
}

/* ========== HC-SR04超声波 ========== */
/**
 * @brief 测量单个超声波距离
 * @param trig_port/trig_pin Trig引脚
 * @param echo_port/echo_pin Echo引脚
 * @return 距离(cm)，超时返回999.0
 *
 * 错误经验：
 *   1. Trig脉冲必须>=10us，建议15us确保可靠
 *   2. Echo高电平持续时间=往返时间，除以58得cm
 *   3. 必须设超时(30ms)，否则无回波时死等阻塞系统
 *   4. 多个超声波必须顺序测量，不能同时触发
 */
static float Ultrasonic_Measure(GPIO_Regs *trig_port, uint32_t trig_pin,
                                 GPIO_Regs *echo_port, uint32_t echo_pin)
{
    /* 发送Trig脉冲(15us) */
    DL_GPIO_clearPins(trig_port, trig_pin);
    delay_us(2);
    DL_GPIO_setPins(trig_port, trig_pin);
    delay_us(15);
    DL_GPIO_clearPins(trig_port, trig_pin);

    /* 等待Echo上升沿 */
    uint32_t timeout_start = g_tick;
    while (!DL_GPIO_readPins(echo_port, echo_pin)) {
        if (g_tick - timeout_start > 3) return 999.0f; /* 3ms超时 */
    }

    /* 记录Echo高电平开始时间 */
    uint32_t echo_start = SysTick->VAL;

    /* 等待Echo下降沿 */
    timeout_start = g_tick;
    while (DL_GPIO_readPins(echo_port, echo_pin)) {
        if (g_tick - timeout_start > 30) return 999.0f; /* 30ms超时 */
    }

    /* 计算高电平持续时间(us) */
    uint32_t echo_end = SysTick->VAL;
    uint32_t pulse_ticks;
    if (echo_start >= echo_end) {
        pulse_ticks = echo_start - echo_end;
    } else {
        pulse_ticks = echo_start + (SysTick->LOAD + 1) - echo_end;
    }
    uint32_t pulse_us = pulse_ticks / (SYSTEM_CLOCK_HZ / 1000000);

    /* 距离 = 脉宽(us) / 58 */
    float distance = (float)pulse_us / (float)SOUND_SPEED_CM_US;

    /* 合理性检查 */
    if (distance > 400.0f || distance < 2.0f) return 999.0f;

    return distance;
}

/**
 * @brief 测量所有超声波(顺序测量，避免串扰)
 */
static void Ultrasonic_MeasureAll(void)
{
    /* 前方 */
    g_dist_front = Ultrasonic_Measure(US_FRONT_TRIG_PORT, US_FRONT_TRIG_PIN,
                                       US_FRONT_ECHO_PORT, US_FRONT_ECHO_PIN);
    delay_us(500); /* 测量间隔 */

    /* 左侧 */
    g_dist_left = Ultrasonic_Measure(US_LEFT_TRIG_PORT, US_LEFT_TRIG_PIN,
                                      US_LEFT_ECHO_PORT, US_LEFT_ECHO_PIN);
    delay_us(500);

    /* 右侧 */
    g_dist_right = Ultrasonic_Measure(US_RIGHT_TRIG_PORT, US_RIGHT_TRIG_PIN,
                                       US_RIGHT_ECHO_PORT, US_RIGHT_ECHO_PIN);
}

/* ========== 舵机转向控制 ========== */
/**
 * @brief 设置转向舵机位置
 * @param pulse_us 脉宽(us): 900(右满) ~ 1500(中) ~ 2100(左满)
 *
 * 错误经验：舵机脉宽范围因型号而异，SG90是500-2400us
 *          设置前需限幅，超出范围可能烧舵机
 */
static void Steer_Set(uint16_t pulse_us)
{
    if (pulse_us < SERVO_RIGHT_MAX) pulse_us = SERVO_RIGHT_MAX;
    if (pulse_us > SERVO_LEFT_MAX) pulse_us = SERVO_LEFT_MAX;
    g_steer_pos = pulse_us;

    /* 更新PWM比较值 (假设PWM周期20ms = 20000计数) */
    /* 转换: pulse_us -> 计数值 */
    uint32_t compare = (uint32_t)pulse_us * PWM_PERIOD / 20000;
    DL_Timer_setCaptureCompareValue(TIMER0, compare, SERVO_STEER_CHANNEL);
}

static void Steer_Center(void) { Steer_Set(SERVO_CENTER); }
static void Steer_Left(void)   { Steer_Set(SERVO_LEFT_MAX); }
static void Steer_Right(void)  { Steer_Set(SERVO_RIGHT_MAX); }

/* ========== 电机驱动 ========== */
static void Motor_Forward(uint16_t speed)
{
    DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    DL_Timer_setCaptureCompareValue(TIMER0, speed, DL_TIMER_CC_1_INDEX);
}

static void Motor_Reverse(uint16_t speed)
{
    DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    DL_Timer_setCaptureCompareValue(TIMER0, speed, DL_TIMER_CC_1_INDEX);
}

static void Motor_Stop(void)
{
    DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    DL_Timer_setCaptureCompareValue(TIMER0, 0, DL_TIMER_CC_1_INDEX);
}

/* ========== 中断服务 ========== */
void SysTick_Handler(void)
{
    g_tick++;
}

/* ========== 泊车状态机 ========== */
/**
 * @brief 泊车控制主循环
 *
 * 侧方停车算法详解：
 *   Step1: 前进+右打轮，使车尾向车位方向摆入
 *   Step2: 倒车+左打轮，车身逐渐正对车位
 *   Step3: 倒车回正，继续深入车位
 *   Step4: 微调前进，使车辆居中
 *
 * 每步持续时间需根据车位宽度调整，这里用固定时间示意
 */
static void Parking_Control(void)
{
    switch (g_park_state) {
    case PARK_IDLE:
        /* 等待启动 */
        break;

    case PARK_CRUISING:
        /* 巡航模式：前进，检测左侧车位 */
        Steer_Center();
        Motor_Forward(MOTOR_SPEED_CRUISE);

        /* 左侧距离检测车位 */
        if (g_dist_left > PARKING_SLOT_DEPTH_CM && !g_slot_detected) {
            /* 左侧突然变远，可能是车位开始 */
            g_slot_start_left = g_dist_left;
            g_slot_start_tick = g_tick;
            g_slot_detected = 1;
            printf("Potential slot start, L=%.1f\r\n", g_dist_left);
        }

        if (g_slot_detected) {
            if (g_dist_left < PARKING_SLOT_DEPTH_CM) {
                /* 左侧恢复近距，车位结束 */
                uint32_t slot_duration = g_tick - g_slot_start_tick;
                /* 通过时间和速度估算车位长度 */
                /* 简化: 假设巡航速度约20cm/s, 100ms=2cm */
                float slot_length = (float)slot_duration * 0.2f;

                if (slot_length >= PARKING_SLOT_LENGTH_CM) {
                    /* 车位足够大，开始泊车 */
                    printf("Slot found! Length=%.1f cm\r\n", slot_length);
                    Motor_Stop();
                    g_park_state = PARK_SLOT_FOUND;
                } else {
                    /* 车位太小，继续巡航 */
                    printf("Slot too small: %.1f cm\r\n", slot_length);
                    g_slot_detected = 0;
                }
            }
        }

        /* 前方紧急停止 */
        if (g_dist_front < FRONT_STOP_CM) {
            Motor_Stop();
            g_park_state = PARK_ABORT;
            printf("ABORT: front obstacle %.1f cm\r\n", g_dist_front);
        }
        break;

    case PARK_SLOT_FOUND:
        /* 停稳后开始泊车 */
        Motor_Stop();
        Steer_Center();
        g_step_start_tick = g_tick;
        g_park_state = PARK_STEP1_FORWARD_RIGHT;
        printf("Park Step1: Forward+Right\r\n");
        break;

    case PARK_STEP1_FORWARD_RIGHT:
        /* 步骤1: 前进+右打轮(约1.5秒) */
        Steer_Right();
        Motor_Forward(MOTOR_SPEED_PARKING);

        if (g_tick - g_step_start_tick >= 1500) {
            Motor_Stop();
            g_step_start_tick = g_tick;
            g_park_state = PARK_STEP2_REVERSE_LEFT;
            printf("Park Step2: Reverse+Left\r\n");
        }

        /* 安全检查 */
        if (g_dist_front < FRONT_STOP_CM) {
            Motor_Stop();
            g_park_state = PARK_ABORT;
        }
        break;

    case PARK_STEP2_REVERSE_LEFT:
        /* 步骤2: 倒车+左打轮(约2秒) */
        Steer_Left();
        Motor_Reverse(MOTOR_SPEED_REVERSE);

        if (g_tick - g_step_start_tick >= 2000) {
            Motor_Stop();
            g_step_start_tick = g_tick;
            g_park_state = PARK_STEP3_REVERSE_STRAIGHT;
            printf("Park Step3: Reverse straight\r\n");
        }
        break;

    case PARK_STEP3_REVERSE_STRAIGHT:
        /* 步骤3: 回正倒车(接近后方障碍物) */
        Steer_Center();
        Motor_Reverse(MOTOR_SPEED_REVERSE);

        /* 后方安全距离(用右侧超声波近似) */
        if (g_dist_right < PARKING_DISTANCE_CM || 
            g_tick - g_step_start_tick >= 1500) {
            Motor_Stop();
            g_step_start_tick = g_tick;
            g_park_state = PARK_STEP4_ADJUST;
            printf("Park Step4: Adjust\r\n");
        }
        break;

    case PARK_STEP4_ADJUST:
        /* 步骤4: 前进微调，使车身居中 */
        Steer_Center();
        Motor_Forward(MOTOR_SPEED_PARKING / 2);

        /* 前方距离合适时停车 */
        if (g_dist_front < PARKING_DISTANCE_CM * 2 ||
            g_tick - g_step_start_tick >= 800) {
            Motor_Stop();
            g_park_state = PARK_COMPLETE;
            printf("Parking COMPLETE!\r\n");
        }
        break;

    case PARK_COMPLETE:
        /* 泊车完成，等待驶出指令 */
        Motor_Stop();
        break;

    case PARK_ABORT:
        /* 中止泊车 */
        Motor_Stop();
        Steer_Center();
        break;

    case PARK_EXITING:
        /* 驶出车位: 倒车->右打->前进->回正 */
        /* 简化实现 */
        Motor_Stop();
        g_park_state = PARK_IDLE;
        break;
    }
}

/* ========== 系统初始化 ========== */
static void System_Init(void)
{
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    /* 舵机初始中位 */
    Steer_Center();
}

/* ========== 主函数 ========== */
int main(void)
{
    System_Init();

    printf("=== Autonomous Parking System ===\r\n");
    printf("Press 'P' to start parking scan\r\n");
    printf("Press 'S' to stop\r\n");
    printf("Press 'E' to exit slot\r\n\r\n");

    /* 启动巡航 */
    g_park_state = PARK_CRUISING;

    uint32_t last_measure_tick = 0;
    uint32_t last_print_tick = 0;

    while (1) {
        /* 周期性超声波测量 */
        if (g_tick - last_measure_tick >= US_MEASURE_INTERVAL_MS) {
            last_measure_tick = g_tick;
            Ultrasonic_MeasureAll();
        }

        /* 泊车状态机控制 */
        Parking_Control();

        /* 状态打印 */
        if (g_tick - last_print_tick >= 200) {
            last_print_tick = g_tick;
            printf("F:%.0f L:%.0f R:%.0f S:%d\r\n",
                   g_dist_front, g_dist_left, g_dist_right, g_park_state);
        }
    }
}
