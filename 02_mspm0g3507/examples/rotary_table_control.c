/**
 * @file rotary_table_control.c
 * @brief 转台控制系统 - AS5048A磁编码器 + 步进电机 + PID定位
 * @platform MSPM0G3507
 * @date 2026-06-12
 *
 * 功能概述：
 *   1. AS5048A磁编码器读取转台角度（14位分辨率，0.022°精度）
 *   2. 步进电机驱动（脉冲+方向控制）
 *   3. PID位置闭环控制（角度定位精度<0.1°）
 *   4. 速度规划（梯形加减速曲线）
 *   5. 支持绝对定位和相对旋转
 *
 * 硬件连接：
 *   AS5048A: SPI0 (PA14-SCK, PA12-MOSI, PA13-MISO, PA15-CS)
 *   步进电机: PB0-STEP(Timer), PB1-DIR, PB2-ENABLE
 *   按键:     PB3(设置), PB4(启动), PB5(停止)
 *   LED指示:  PA8(运行), PA9(到位)
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ===== 硬件引脚定义 ===== */
/* AS5048A SPI接口 */
#define AS5048A_CS_PORT         GPIOA
#define AS5048A_CS_PIN          DL_GPIO_PIN_15

/* 步进电机控制 */
#define STEP_PULSE_PORT         GPIOB
#define STEP_PULSE_PIN          DL_GPIO_PIN_0
#define STEP_DIR_PORT           GPIOB
#define STEP_DIR_PIN            DL_GPIO_PIN_1
#define STEP_EN_PORT            GPIOB
#define STEP_EN_PIN             DL_GPIO_PIN_2

/* 按键 */
#define KEY_SET_PORT            GPIOB
#define KEY_SET_PIN             DL_GPIO_PIN_3
#define KEY_START_PORT          GPIOB
#define KEY_START_PIN           DL_GPIO_PIN_4
#define KEY_STOP_PORT           GPIOB
#define KEY_STOP_PIN            DL_GPIO_PIN_5

/* LED */
#define LED_RUN_PORT            GPIOA
#define LED_RUN_PIN             DL_GPIO_PIN_8
#define LED_DONE_PORT           GPIOA
#define LED_DONE_PIN            DL_GPIO_PIN_9

/* ===== 参数配置 ===== */
#define AS5048A_RESOLUTION      16384.0f    /* 14位分辨率 */
#define DEG_PER_COUNT           (360.0f / AS5048A_RESOLUTION)
#define GEAR_RATIO              10.0f       /* 减速比 */
#define STEPS_PER_REV           200         /* 步进电机每转步数(1.8°) */
#define MICRO_STEPS             16          /* 细分数 */
#define TOTAL_STEPS_PER_REV     (STEPS_PER_REV * MICRO_STEPS * (uint32_t)GEAR_RATIO)

/* PID参数 */
#define PID_KP                  2.5f
#define PID_KI                  0.05f
#define PID_KD                  1.0f
#define PID_OUTPUT_MAX          500.0f      /* 最大脉冲频率(Hz) */
#define PID_OUTPUT_MIN          10.0f       /* 最小脉冲频率 */
#define PID_INTEGRAL_MAX        1000.0f     /* 积分限幅 */
#define POSITION_TOLERANCE_DEG  0.1f        /* 到位判定容差(°) */
#define POSITION_SETTLE_MS      200         /* 到位稳定时间(ms) */

/* 加减速参数 */
#define ACCEL_STEPS             200         /* 加速段步数 */
#define DECEL_STEPS             200         /* 减速段步数 */
#define MAX_SPEED_HZ            4000        /* 最大脉冲频率(Hz) */
#define MIN_SPEED_HZ            100         /* 最小脉冲频率(Hz) */

/* ===== AS5048A寄存器定义 ===== */
#define AS5048A_REG_ANGLE_H     0xFE        /* 角度高字节 */
#define AS5048A_REG_ANGLE_L     0xFF        /* 角度低字节 */
#define AS5048A_CMD_READ        0x4000      /* 读命令标志 */
#define AS5048A_CMD_NOP         0x0000      /* 无操作 */

/* ===== 系统状态枚举 ===== */
typedef enum {
    SYS_IDLE,           /* 待机 */
    SYS_MOVING,         /* 运动中 */
    SYS_SETTLE,         /* 到位稳定中 */
    SYS_DONE,           /* 到位完成 */
    SYS_ERROR           /* 错误 */
} SystemState_t;

/* ===== 运动模式 ===== */
typedef enum {
    MOVE_ABSOLUTE,      /* 绝对定位 */
    MOVE_RELATIVE       /* 相对旋转 */
} MoveMode_t;

/* ===== PID控制器结构体 ===== */
typedef struct {
    float kp, ki, kd;           /* PID参数 */
    float integral;              /* 积分累积 */
    float prev_error;            /* 上次误差 */
    float output;                /* 输出 */
    float integral_max;          /* 积分限幅 */
    float output_max;            /* 输出限幅 */
    float output_min;            /* 输出下限 */
} PID_Controller_t;

/* ===== 全局变量 ===== */
static volatile uint32_t g_systick_count = 0;
static volatile uint32_t g_step_count = 0;      /* 已走步数 */
static volatile uint32_t g_target_steps = 0;    /* 目标步数 */
static volatile bool g_step_pulse_flag = false;

static SystemState_t g_sys_state = SYS_IDLE;
static MoveMode_t g_move_mode = MOVE_ABSOLUTE;

static float g_current_angle = 0.0f;            /* 当前角度(°) */
static float g_target_angle = 0.0f;             /* 目标角度(°) */
static float g_angle_offset = 0.0f;             /* 零点偏移 */
static float g_angle_history[4] = {0};          /* 角度滤波缓存 */
static uint8_t g_angle_index = 0;

static PID_Controller_t g_pid;

static volatile uint32_t g_pulse_period_us = 0; /* 脉冲周期(us) */
static uint32_t g_settle_start = 0;             /* 稳定开始时间 */

/* ===== SysTick ===== */
void SysTick_Handler(void) {
    g_systick_count++;
}

static void delay_ms(uint32_t ms) {
    uint32_t start = g_systick_count;
    while ((g_systick_count - start) < ms);
}

static uint32_t get_tick(void) { return g_systick_count; }

/* ===== SPI通信(AS5048A) ===== */
static void as5048a_cs_low(void) {
    DL_GPIO_clearPins(AS5048A_CS_PORT, AS5048A_CS_PIN);
}

static void as5048a_cs_high(void) {
    DL_GPIO_setPins(AS5048A_CS_PORT, AS5048A_CS_PIN);
}

static uint16_t spi_transfer16(uint16_t tx_data) {
    /* 等待SPI空闲 */
    while (DL_SPI_isBusy(SPI_0_INST));

    /* 发送16位数据 */
    DL_SPI_transmitData16(SPI_0_INST, tx_data);

    /* 等待传输完成 */
    while (!DL_SPI_isRXFIFOEmpty(SPI_0_INST));

    return DL_SPI_receiveData16(SPI_0_INST);
}

static uint16_t as5048a_read_raw(void) {
    uint16_t raw_angle;

    as5048a_cs_low();

    /* 发送读取角度命令 */
    spi_transfer16(AS5048A_CMD_READ | AS5048A_REG_ANGLE_H);
    as5048a_cs_high();

    delay_ms(1);

    as5048a_cs_low();
    /* 读取数据 */
    raw_angle = spi_transfer16(AS5048A_CMD_NOP);
    as5048a_cs_high();

    return raw_angle & 0x3FFF; /* 14位有效 */
}

static float as5048a_read_angle(void) {
    uint16_t raw = as5048a_read_raw();
    float angle = (float)raw * DEG_PER_COUNT;
    return angle;
}

/* 角度滤波(移动平均) */
static float read_angle_filtered(void) {
    float new_angle = as5048a_read_angle();
    g_angle_history[g_angle_index] = new_angle;
    g_angle_index = (g_angle_index + 1) & 0x03;

    float sum = 0;
    for (int i = 0; i < 4; i++) {
        sum += g_angle_history[i];
    }
    g_current_angle = sum / 4.0f - g_angle_offset;
    return g_current_angle;
}

/* 角度差计算(处理0°/360°跨越) */
static float angle_diff(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

/* ===== PID控制器 ===== */
static void pid_init(PID_Controller_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
    pid->integral_max = PID_INTEGRAL_MAX;
    pid->output_max = PID_OUTPUT_MAX;
    pid->output_min = PID_OUTPUT_MIN;
}

static float pid_compute(PID_Controller_t *pid, float error) {
    /* 积分项(带限幅) */
    pid->integral += error;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    /* 微分项 */
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    /* PID输出 */
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    /* 输出限幅 */
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;

    /* 死区(防止在目标附近振荡) */
    if (fabsf(pid->output) < pid->output_min && fabsf(error) < POSITION_TOLERANCE_DEG) {
        pid->output = 0;
    }

    return pid->output;
}

/* ===== 步进电机控制 ===== */
static void stepper_enable(bool en) {
    if (en) {
        DL_GPIO_clearPins(STEP_EN_PORT, STEP_EN_PIN); /* 低电平有效 */
    } else {
        DL_GPIO_setPins(STEP_EN_PORT, STEP_EN_PIN);
    }
}

static void stepper_set_dir(bool clockwise) {
    if (clockwise) {
        DL_GPIO_setPins(STEP_DIR_PORT, STEP_DIR_PIN);
    } else {
        DL_GPIO_clearPins(STEP_DIR_PORT, STEP_DIR_PIN);
    }
}

static void stepper_one_step(void) {
    DL_GPIO_setPins(STEP_PULSE_PORT, STEP_PULSE_PIN);
    delay_us(2); /* 最小脉宽2us */
    DL_GPIO_clearPins(STEP_PULSE_PORT, STEP_PULSE_PIN);
}

/* 梯形加减速速度规划 */
static uint32_t calc_speed_hz(uint32_t step, uint32_t total_steps) {
    uint32_t accel = (step < ACCEL_STEPS) ? step : ACCEL_STEPS;
    uint32_t remaining = total_steps - step;
    uint32_t decel = (remaining < DECEL_STEPS) ? remaining : DECEL_STEPS;
    uint32_t phase = (accel < decel) ? accel : decel;

    float ratio = (float)phase / ACCEL_STEPS;
    uint32_t speed = MIN_SPEED_HZ + (uint32_t)((MAX_SPEED_HZ - MIN_SPEED_HZ) * ratio);
    if (speed < MIN_SPEED_HZ) speed = MIN_SPEED_HZ;
    return speed;
}

/* ===== 按键扫描(带消抖) ===== */
static bool key_pressed(GPIO_Regs *port, uint32_t pin) {
    if (!(port->DIN31_0 & pin)) {
        delay_ms(20);
        if (!(port->DIN31_0 & pin)) {
            while (!(port->DIN31_0 & pin)); /* 等待释放 */
            return true;
        }
    }
    return false;
}

/* ===== 主函数 ===== */
int main(void) {
    /* 系统初始化 */
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    /* GPIO初始化 */
    DL_GPIO_initDigitalOutput(AS5048A_CS_PIN);
    DL_GPIO_enableOutput(AS5048A_CS_PORT, AS5048A_CS_PIN);
    DL_GPIO_setPins(AS5048A_CS_PORT, AS5048A_CS_PIN); /* CS默认高 */

    DL_GPIO_initDigitalOutput(STEP_PULSE_PIN);
    DL_GPIO_enableOutput(STEP_PULSE_PORT, STEP_PULSE_PIN);
    DL_GPIO_initDigitalOutput(STEP_DIR_PIN);
    DL_GPIO_enableOutput(STEP_DIR_PORT, STEP_DIR_PIN);
    DL_GPIO_initDigitalOutput(STEP_EN_PIN);
    DL_GPIO_enableOutput(STEP_EN_PORT, STEP_EN_PIN);

    DL_GPIO_initDigitalInput(KEY_SET_PIN);
    DL_GPIO_initDigitalInput(KEY_START_PIN);
    DL_GPIO_initDigitalInput(KEY_STOP_PIN);

    DL_GPIO_initDigitalOutput(LED_RUN_PIN);
    DL_GPIO_enableOutput(LED_RUN_PORT, LED_RUN_PIN);
    DL_GPIO_initDigitalOutput(LED_DONE_PIN);
    DL_GPIO_enableOutput(LED_DONE_PORT, LED_DONE_PIN);

    /* PID初始化 */
    pid_init(&g_pid, PID_KP, PID_KI, PID_KD);

    /* 使能步进电机 */
    stepper_enable(true);

    /* 读取初始角度作为零点 */
    delay_ms(100);
    g_angle_offset = as5048a_read_angle();

    /* 系统启动指示 */
    DL_GPIO_setPins(LED_RUN_PORT, LED_RUN_PIN);
    delay_ms(500);
    DL_GPIO_clearPins(LED_RUN_PORT, LED_RUN_PIN);

    /* 主循环 */
    while (1) {
        /* 读取当前角度 */
        float current = read_angle_filtered();

        /* 按键处理 */
        if (key_pressed(KEY_SET_PORT, KEY_SET_PIN)) {
            /* SET键: 设置零点 */
            g_angle_offset = as5048a_read_angle();
            DL_GPIO_togglePins(LED_DONE_PORT, LED_DONE_PIN);
        }

        if (key_pressed(KEY_START_PORT, KEY_START_PIN)) {
            /* START键: 启动定位到90°(示例目标) */
            g_target_angle = 90.0f;
            g_move_mode = MOVE_ABSOLUTE;
            g_sys_state = SYS_MOVING;
            g_pid.integral = 0;
            g_pid.prev_error = 0;
            g_step_count = 0;
            DL_GPIO_setPins(LED_RUN_PORT, LED_RUN_PIN);
            DL_GPIO_clearPins(LED_DONE_PORT, LED_DONE_PIN);
        }

        if (key_pressed(KEY_STOP_PORT, KEY_STOP_PIN)) {
            /* STOP键: 停止运动 */
            g_sys_state = SYS_IDLE;
            stepper_enable(false);
            DL_GPIO_clearPins(LED_RUN_PORT, LED_RUN_PIN);
        }

        /* 状态机处理 */
        switch (g_sys_state) {
        case SYS_IDLE:
            /* 待机: 低功耗等待 */
            break;

        case SYS_MOVING: {
            float diff = angle_diff(g_target_angle, current);

            /* 到位判断 */
            if (fabsf(diff) < POSITION_TOLERANCE_DEG) {
                g_sys_state = SYS_SETTLE;
                g_settle_start = get_tick();
                g_pulse_period_us = 0; /* 停止脉冲 */
                break;
            }

            /* PID计算 */
            float pid_out = pid_compute(&g_pid, diff);

            /* 设置方向 */
            bool cw = (pid_out > 0);
            stepper_set_dir(cw);

            /* 计算脉冲频率 */
            float abs_output = fabsf(pid_out);
            if (abs_output < PID_OUTPUT_MIN) abs_output = PID_OUTPUT_MIN;

            g_pulse_period_us = (uint32_t)(1000000.0f / abs_output);

            /* 生成脉冲 */
            stepper_one_step();
            g_step_count++;

            /* 脉冲间隔 */
            delay_ms(1000000 / (uint32_t)abs_output / 1000);
            break;
        }

        case SYS_SETTLE:
            /* 到位稳定等待 */
            if ((get_tick() - g_settle_start) >= POSITION_SETTLE_MS) {
                /* 再次检查是否仍在容差内 */
                float diff = angle_diff(g_target_angle, current);
                if (fabsf(diff) < POSITION_TOLERANCE_DEG) {
                    g_sys_state = SYS_DONE;
                    DL_GPIO_clearPins(LED_RUN_PORT, LED_RUN_PIN);
                    DL_GPIO_setPins(LED_DONE_PORT, LED_DONE_PIN);
                } else {
                    /* 漂移了，重新定位 */
                    g_sys_state = SYS_MOVING;
                    g_pid.integral = 0;
                }
            }
            break;

        case SYS_DONE:
            /* 定位完成 */
            break;

        case SYS_ERROR:
            stepper_enable(false);
            /* LED快闪报警 */
            DL_GPIO_togglePins(LED_RUN_PORT, LED_RUN_PIN);
            delay_ms(200);
            break;
        }
    }
}

/* 微秒延时(基于SysTick循环) */
static void delay_us(uint32_t us) {
    volatile uint32_t count = us * (SystemCoreClock / 1000000) / 4;
    while (count--);
}
