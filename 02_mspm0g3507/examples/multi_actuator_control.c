/**
 * @file multi_actuator_control.c
 * @brief MSPM0G3507 多执行器协调控制系统
 *
 * 功能：舵机 + 步进电机 + 直流电机 + 蜂鸣器 联动控制
 * 演示场景：自动化生产线模拟
 *   1. 舵机夹取工件 -> 2. 步进电机传输 -> 3. 直流电机加工
 *   4. 蜂鸣器状态提示 -> 5. 循环执行
 *
 * 支持UART命令控制各执行器独立动作，也支持自动序列模式
 *
 * 硬件连接：
 *   舵机SG90: PWM=PA0 (TIMER0 CH0)
 *   步进电机28BYJ-48 + ULN2003: IN1~4=PA4~PA7
 *   直流电机+L298N: IN1=PB0, IN2=PB1, EN=PB2 (PWM TIMER1 CH0)
 *   蜂鸣器: PWM=PB10 (软件PWM)
 *   LED状态: PB15
 *   按键启动: PB14
 *   UART: TX=PA10, RX=PA11
 *
 * @author 电赛资产库
 * @date 2026
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ========== 系统参数 ========== */
#define SYS_CLK_HZ     32000000UL
#define PWM_SERVO_HZ   50       /* 舵机PWM频率50Hz */
#define SERVO_MIN_US    500     /* 0°脉宽(微秒) */
#define SERVO_MAX_US    2500    /* 180°脉宽(微秒) */
#define SERVO_PERIOD_US 20000   /* 20ms周期 */

/* 步进电机参数 (28BYJ-48, 半步驱动) */
#define STEPS_PER_REV   4096    /* 半步一圈步数 */
#define STEP_DELAY_MS   2       /* 步进延时(ms) */

/* ========== 舵机控制 ========== */

/**
 * @brief 设置舵机角度
 * @param angle 0~180度
 */
static void servo_set_angle(uint8_t angle) {
    if (angle > 180) angle = 180;

    /* 线性映射: angle -> 脉宽(us) */
    uint32_t pulse_us = SERVO_MIN_US + (uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US) / 180;

    /* 计算定时器比较值 */
    uint32_t period = DL_TimerG_getLoadValue(TIMER0);
    uint32_t compare = period * pulse_us / SERVO_PERIOD_US;

    DL_TimerG_setCaptureCompareValue(TIMER0, compare, DL_TIMER_CC_0_INDEX);
}

/* ========== 步进电机控制 (28BYJ-48 半步序列) ========== */

/* 半步驱动序列 (8相) */
static const uint8_t step_sequence[8] = {
    0x01,  /* 0001 */
    0x03,  /* 0011 */
    0x02,  /* 0010 */
    0x06,  /* 0110 */
    0x04,  /* 0100 */
    0x0C,  /* 1100 */
    0x08,  /* 1000 */
    0x09,  /* 1001 */
};

static volatile int32_t g_stepperPos = 0;   /* 当前步进位置 */
static uint8_t g_stepPhase = 0;             /* 当前相位 */

/**
 * @brief 设置步进电机相位输出
 */
static void stepper_set_phase(uint8_t phase) {
    /* PA4~PA7 对应4相输出 */
    uint8_t bits = step_sequence[phase & 0x07];

    DL_GPIO_writePins(GPIOA, DL_GPIO_PIN_4, (bits & 0x01) ? DL_GPIO_PIN_4 : 0);
    DL_GPIO_writePins(GPIOA, DL_GPIO_PIN_5, (bits & 0x02) ? DL_GPIO_PIN_5 : 0);
    DL_GPIO_writePins(GPIOA, DL_GPIO_PIN_6, (bits & 0x04) ? DL_GPIO_PIN_6 : 0);
    DL_GPIO_writePins(GPIOA, DL_GPIO_PIN_7, (bits & 0x08) ? DL_GPIO_PIN_7 : 0);
}

/**
 * @brief 步进电机移动指定步数
 * @param steps 正=顺时针, 负=逆时针
 * @param delay_ms 每步延时
 */
static void stepper_move(int32_t steps, uint32_t delay_ms) {
    int32_t abs_steps = steps > 0 ? steps : -steps;
    int8_t  dir = steps > 0 ? 1 : -1;

    for (int32_t i = 0; i < abs_steps; i++) {
        g_stepPhase = (g_stepPhase + dir + 8) & 0x07;
        stepper_set_phase(g_stepPhase);
        g_stepperPos += dir;

        for (volatile uint32_t j = 0; j < delay_ms * 8000; j++);
    }
}

/**
 * @brief 步进电机转到指定角度
 * @param target_deg 目标角度(0~360)
 */
static void stepper_goto_angle(uint16_t target_deg) {
    int32_t target_steps = (int32_t)target_deg * STEPS_PER_REV / 360;
    int32_t delta = target_steps - (g_stepperPos % STEPS_PER_REV);
    if (delta < 0) delta += STEPS_PER_REV;
    if (delta > STEPS_PER_REV / 2) delta -= STEPS_PER_REV;

    stepper_move(delta, STEP_DELAY_MS);
}

/**
 * @brief 步进电机停止(断电, 防止过热)
 */
static void stepper_stop(void) {
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_4 | DL_GPIO_PIN_5 |
                              DL_GPIO_PIN_6 | DL_GPIO_PIN_7);
}

/* ========== 直流电机控制 ========== */

typedef enum {
    DC_STOP = 0,
    DC_CW,      /* 顺时针 */
    DC_CCW      /* 逆时针 */
} DC_Dir_t;

/**
 * @brief 直流电机控制
 * @param dir 方向
 * @param speed 速度 0~100%
 */
static void dc_motor_control(DC_Dir_t dir, uint8_t speed) {
    switch (dir) {
        case DC_STOP:
            DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0 | DL_GPIO_PIN_1);
            break;
        case DC_CW:
            DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_0);
            DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_1);
            break;
        case DC_CCW:
            DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0);
            DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_1);
            break;
    }

    /* 设置PWM占空比 */
    uint32_t period = DL_TimerG_getLoadValue(TIMER1);
    uint32_t compare = period * speed / 100;
    DL_TimerG_setCaptureCompareValue(TIMER1, compare, DL_TIMER_CC_0_INDEX);
}

/* ========== 蜂鸣器控制 ========== */

/**
 * @brief 蜂鸣器发声
 * @param freq_hz 频率(Hz), 0=静音
 * @param duration_ms 持续时间(ms)
 */
static void buzzer_beep(uint16_t freq_hz, uint32_t duration_ms) {
    if (freq_hz == 0) {
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_10);
        for (volatile uint32_t i = 0; i < duration_ms * 8000; i++);
        return;
    }

    /* 软件PWM产生方波 */
    uint32_t half_period_us = 500000UL / freq_hz;  /* 半周期(微秒) */
    uint32_t cycles = (uint32_t)freq_hz * duration_ms / 1000;

    for (uint32_t i = 0; i < cycles; i++) {
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_10);
        for (volatile uint32_t j = 0; j < half_period_us * 4; j++);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_10);
        for (volatile uint32_t j = 0; j < half_period_us * 4; j++);
    }
}

/* 预定义提示音 */
static void buzzer_ok(void)    { buzzer_beep(1000, 100); buzzer_beep(1500, 100); }
static void buzzer_error(void) { buzzer_beep(300, 300); buzzer_beep(300, 300); }
static void buzzer_start(void) {
    buzzer_beep(523, 150); /* Do */
    buzzer_beep(659, 150); /* Mi */
    buzzer_beep(784, 200); /* Sol */
}
static void buzzer_done(void) {
    buzzer_beep(784, 150); /* Sol */
    buzzer_beep(659, 150); /* Mi */
    buzzer_beep(523, 200); /* Do */
}

/* ========== LED状态 ========== */
#define LED_PORT    GPIOB
#define LED_PIN     DL_GPIO_PIN_15

static void led_on(void)  { DL_GPIO_setPins(LED_PORT, LED_PIN); }
static void led_off(void) { DL_GPIO_clearPins(LED_PORT, LED_PIN); }
static void led_toggle(void) { DL_GPIO_togglePins(LED_PORT, LED_PIN); }

/* ========== UART通信 ========== */

static char g_uartBuf[64];
static volatile uint8_t g_uartIdx = 0;
static volatile bool g_uartReady = false;

void UART0_IRQHandler(void) {
    if (DL_UART_getPendingInterrupt(UART0) == DL_UART_IIDX_RX) {
        char c = (char)DL_UART_Main_receiveData(UART0);
        if (c == '\n' || c == '\r') {
            if (g_uartIdx > 0) {
                g_uartBuf[g_uartIdx] = '\0';
                g_uartReady = true;
            }
        } else if (g_uartIdx < sizeof(g_uartBuf) - 1) {
            g_uartBuf[g_uartIdx++] = c;
        }
    }
}

static void uart_sendchar(char c) {
    DL_UART_Main_transmitData(UART0, (uint8_t)c);
    while (!DL_UART_isTXFIFOEmpty(UART0));
}

static void uart_printf(const char *fmt, ...) {
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    for (int i = 0; buf[i]; i++) uart_sendchar(buf[i]);
}

/* ========== 自动序列 ========== */

typedef enum {
    SEQ_IDLE,
    SEQ_STEP1_CLAMP,    /* 舵机夹紧 */
    SEQ_STEP2_TRANSFER, /* 步进传输 */
    SEQ_STEP3_PROCESS,  /* 直流电机加工 */
    SEQ_STEP4_RELEASE,  /* 舵机松开 */
    SEQ_STEP5_RETURN,   /* 步进回原点 */
    SEQ_DONE
} SequenceState_t;

static volatile SequenceState_t g_seqState = SEQ_IDLE;
static volatile bool g_autoMode = false;
static volatile uint32_t g_seqTimer = 0;

/**
 * @brief 自动序列状态机 (在定时器中断中推进)
 */
static void sequence_tick(void) {
    if (!g_autoMode || g_seqState == SEQ_IDLE || g_seqState == SEQ_DONE) return;

    g_seqTimer++;

    switch (g_seqState) {
        case SEQ_STEP1_CLAMP:
            /* 舵机从0°缓慢夹到90° */
            if (g_seqTimer < 50) {
                servo_set_angle(g_seqTimer * 90 / 50);
            } else {
                g_seqState = SEQ_STEP2_TRANSFER;
                g_seqTimer = 0;
                buzzer_beep(800, 50);
            }
            break;

        case SEQ_STEP2_TRANSFER:
            /* 步进电机前进1/4圈 */
            if (g_seqTimer == 1) {
                stepper_move(STEPS_PER_REV / 4, STEP_DELAY_MS);
                g_seqState = SEQ_STEP3_PROCESS;
                g_seqTimer = 0;
            }
            break;

        case SEQ_STEP3_PROCESS:
            /* 直流电机运行2秒 */
            if (g_seqTimer < 200) {
                dc_motor_control(DC_CW, 70);
                /* LED快闪指示加工中 */
                if (g_seqTimer % 20 == 0) led_toggle();
            } else {
                dc_motor_control(DC_STOP, 0);
                led_off();
                g_seqState = SEQ_STEP4_RELEASE;
                g_seqTimer = 0;
            }
            break;

        case SEQ_STEP4_RELEASE:
            /* 舵机从90°松开到0° */
            if (g_seqTimer < 50) {
                servo_set_angle(90 - g_seqTimer * 90 / 50);
            } else {
                servo_set_angle(0);
                g_seqState = SEQ_STEP5_RETURN;
                g_seqTimer = 0;
            }
            break;

        case SEQ_STEP5_RETURN:
            /* 步进电机回原点 */
            if (g_seqTimer == 1) {
                stepper_move(-STEPS_PER_REV / 4, STEP_DELAY_MS);
                stepper_stop();
                g_seqState = SEQ_DONE;
                g_seqTimer = 0;
                buzzer_done();
            }
            break;

        default:
            break;
    }
}

/* ========== 定时器中断 ========== */

void TIMG0_IRQHandler(void) {
    if (DL_TimerG_getPendingInterrupt(TIMG0) == DL_TIMER_IIDX_ZERO) {
        /* 每10ms执行一次序列推进 */
        static uint8_t tick = 0;
        tick++;
        if (tick >= 10) {
            tick = 0;
            sequence_tick();
        }
    }
}

/* ========== 命令解析 ========== */

/**
 * @brief 解析并执行UART命令
 *
 * 命令格式:
 *   SERVO <angle>       - 舵机角度 0~180
 *   STEP <steps>        - 步进移动步数
 *   STEP_DEG <degrees>  - 步进转到角度
 *   DC <dir> <speed>    - 直流电机(CW/CCW/STOP, 0~100)
 *   BUZZ <freq> <ms>    - 蜂鸣器
 *   AUTO                - 启动自动序列
 *   STOP                - 停止所有
 *   STATUS              - 报告状态
 */
static void cmd_execute(char *cmd) {
    /* 转大写 */
    for (int i = 0; cmd[i]; i++) {
        if (cmd[i] >= 'a' && cmd[i] <= 'z') cmd[i] -= 32;
    }

    if (strncmp(cmd, "SERVO ", 6) == 0) {
        uint8_t angle = atoi(cmd + 6);
        servo_set_angle(angle);
        uart_printf("OK: Servo -> %d deg\r\n", angle);

    } else if (strncmp(cmd, "STEP_DEG ", 9) == 0) {
        uint16_t deg = atoi(cmd + 9);
        stepper_goto_angle(deg);
        stepper_stop();
        uart_printf("OK: Stepper -> %d deg\r\n", deg);

    } else if (strncmp(cmd, "STEP ", 5) == 0) {
        int32_t steps = atoi(cmd + 5);
        stepper_move(steps, STEP_DELAY_MS);
        stepper_stop();
        uart_printf("OK: Stepper %ld steps\r\n", steps);

    } else if (strncmp(cmd, "DC ", 3) == 0) {
        char *p = cmd + 3;
        DC_Dir_t dir = DC_STOP;
        if (strncmp(p, "CW ", 3) == 0) { dir = DC_CW; p += 3; }
        else if (strncmp(p, "CCW ", 4) == 0) { dir = DC_CCW; p += 4; }
        else { dir = DC_STOP; p += 5; }
        uint8_t speed = atoi(p);
        dc_motor_control(dir, speed);
        uart_printf("OK: DC motor dir=%d speed=%d\r\n", dir, speed);

    } else if (strncmp(cmd, "BUZZ ", 5) == 0) {
        uint16_t freq = atoi(cmd + 5);
        char *p = strchr(cmd + 5, ' ');
        uint32_t dur = p ? atoi(p) : 200;
        buzzer_beep(freq, dur);
        uart_printf("OK: Buzzer %dHz %lums\r\n", freq, dur);

    } else if (strcmp(cmd, "AUTO") == 0) {
        g_autoMode = true;
        g_seqState = SEQ_STEP1_CLAMP;
        g_seqTimer = 0;
        buzzer_start();
        uart_printf("OK: Auto sequence started\r\n");

    } else if (strcmp(cmd, "STOP") == 0) {
        g_autoMode = false;
        g_seqState = SEQ_IDLE;
        servo_set_angle(0);
        stepper_stop();
        dc_motor_control(DC_STOP, 0);
        buzzer_beep(0, 0);
        led_off();
        uart_printf("OK: All stopped\r\n");

    } else if (strcmp(cmd, "STATUS") == 0) {
        uart_printf("=== System Status ===\r\n");
        uart_printf("Auto mode: %s\r\n", g_autoMode ? "ON" : "OFF");
        uart_printf("Seq state: %d\r\n", g_seqState);
        uart_printf("Stepper pos: %ld\r\n", g_stepperPos);
        uart_printf("====================\r\n");

    } else {
        uart_printf("ERR: Unknown cmd. Use: SERVO/STEP/DC/BUZZ/AUTO/STOP/STATUS\r\n");
    }
}

/* ========== 主函数 ========== */
int main(void) {
    /* 初始化系统 */
    SYSCFG_DL_init();

    /* 启动PWM定时器 */
    DL_TimerG_startCounter(TIMER0);  /* 舵机PWM */
    DL_TimerG_startCounter(TIMER1);  /* 直流电机PWM */

    /* 启动序列定时器 */
    NVIC_EnableIRQ(TIMG0_IRQn);
    DL_TimerG_startCounter(TIMG0);

    /* 配置UART中断 */
    DL_UART_enableInterrupt(UART0, DL_UART_IIDX_RX);
    NVIC_EnableIRQ(UART0_IRQn);

    /* 初始状态 */
    servo_set_angle(0);
    stepper_stop();
    dc_motor_control(DC_STOP, 0);

    /* 开机提示 */
    buzzer_start();
    led_on();

    uart_printf("\r\n=== Multi-Actuator Control System v1.0 ===\r\n");
    uart_printf("Commands:\r\n");
    uart_printf("  SERVO <0-180>       - Set servo angle\r\n");
    uart_printf("  STEP <steps>        - Move stepper steps\r\n");
    uart_printf("  STEP_DEG <0-360>    - Move stepper to angle\r\n");
    uart_printf("  DC <CW|CCW|STOP> <0-100> - DC motor\r\n");
    uart_printf("  BUZZ <freq> <ms>    - Buzzer\r\n");
    uart_printf("  AUTO                - Start auto sequence\r\n");
    uart_printf("  STOP                - Stop all\r\n");
    uart_printf("  STATUS              - Report status\r\n");
    uart_printf("Ready.\r\n");

    /* 按键启动 */
    #define BTN_PORT GPIOB
    #define BTN_PIN  DL_GPIO_PIN_14

    /* 主循环 */
    while (1) {
        /* UART命令处理 */
        if (g_uartReady) {
            g_uartReady = false;
            g_uartIdx = 0;
            cmd_execute(g_uartBuf);
        }

        /* 按键触发自动模式 */
        if (!(DL_GPIO_readPins(BTN_PORT, BTN_PIN))) {  /* 低电平有效 */
            for (volatile uint32_t i = 0; i < 100000; i++);  /* 消抖 */
            if (!(DL_GPIO_readPins(BTN_PORT, BTN_PIN))) {
                if (!g_autoMode) {
                    g_autoMode = true;
                    g_seqState = SEQ_STEP1_CLAMP;
                    g_seqTimer = 0;
                    buzzer_start();
                    uart_printf("Auto started (button)\r\n");
                }
                while (!(DL_GPIO_readPins(BTN_PORT, BTN_PIN)));  /* 等松开 */
            }
        }

        /* 自动模式完成检测 */
        if (g_autoMode && g_seqState == SEQ_DONE) {
            g_autoMode = false;
            g_seqState = SEQ_IDLE;
            led_on();
            uart_printf("Auto sequence complete.\r\n");
        }
    }
}
