/**
 * @file stepper_3d_printer.c
 * @brief MSPM0G3507 简易3D打印机控制器
 *
 * 功能：A4988步进电机XYZ三轴控制 + 热端温度PID控制 + 简易G-code解析
 * 硬件：MSPM0G3507 LaunchPad + A4988×3 + NTC热敏电阻 + 加热棒MOSFET
 *
 * 引脚分配：
 *   X轴步进: STEP=PA0, DIR=PA1, EN=PA2
 *   Y轴步进: STEP=PA3, DIR=PA4, EN=PA5
 *   Z轴步进: STEP=PA6, DIR=PA7, EN=PB0
 *   热端加热: MOSFET=PB1 (PWM)
 *   热敏电阻: ADC Channel=PA22 (ADC0_CH5)
 *   限位开关: X_MIN=PB10, Y_MIN=PB11, Z_MIN=PB12
 *   UART调试: TX=PA10, RX=PA11
 *
 * @author 电赛资产库
 * @date 2026
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ========== 硬件参数配置 ========== */
#define SYS_CLK_HZ         32000000UL  /* 系统时钟32MHz */
#define STEP_PULSE_US       5           /* 步进脉冲宽度(微秒) */
#define STEPS_PER_MM_X      80          /* X轴每毫米步数 (1/16微步, GT2皮带) */
#define STEPS_PER_MM_Y      80          /* Y轴每毫米步数 */
#define STEPS_PER_MM_Z      400         /* Z轴每毫米步数 (丝杆) */
#define MAX_FEEDRATE_MM_S   100         /* 最大进给速度 mm/s */
#define DEFAULT_FEEDRATE    30          /* 默认进给速度 mm/s */

/* 温控参数 */
#define NTC_NOMINAL_R       100000      /* NTC标称电阻 100K @25°C */
#define NTC_NOMINAL_TEMP    25.0f       /* 标称温度 */
#define NTC_B_COEFF         3950        /* B系数 */
#define NTC_SERIES_R        4700        /* 串联电阻 4.7K */
#define ADC_RESOLUTION      4096        /* 12位ADC */
#define TEMP_TARGET_PLA     200         /* PLA目标温度(°C) */
#define TEMP_TARGET_BED     60          /* 热床目标温度(°C) */
#define TEMP_MAX            260         /* 安全上限温度 */

/* PID参数 */
#define PID_KP              2.0f
#define PID_KI              0.05f
#define PID_KD              100.0f
#define PID_SAMPLE_TIME_MS  500         /* PID采样周期 */

/* G-code缓冲区 */
#define GCODE_LINE_MAX      128
#define GCODE_QUEUE_SIZE    16

/* ========== 轴定义 ========== */
typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_COUNT
} Axis_t;

/* 步进电机状态 */
typedef struct {
    volatile int32_t  position;      /* 当前位置(步数) */
    int32_t           target;        /* 目标位置(步数) */
    float             feedrate;      /* 当前进给 mm/s */
    bool              homing;        /* 是否正在归零 */
    bool              enabled;       /* 驱动使能 */
} StepperState_t;

/* ========== 全局变量 ========== */
static StepperState_t g_steppers[AXIS_COUNT];

/* 引脚映射表 */
static const struct {
    GPIO_Regs *step_port;
    uint32_t   step_pin;
    GPIO_Regs *dir_port;
    uint32_t   dir_pin;
    GPIO_Regs *en_port;
    uint32_t   en_pin;
} g_pinMap[AXIS_COUNT] = {
    /* X轴 */ { GPIOA, DL_GPIO_PIN_0, GPIOA, DL_GPIO_PIN_1, GPIOA, DL_GPIO_PIN_2 },
    /* Y轴 */ { GPIOA, DL_GPIO_PIN_3, GPIOA, DL_GPIO_PIN_4, GPIOA, DL_GPIO_PIN_5 },
    /* Z轴 */ { GPIOA, DL_GPIO_PIN_6, GPIOA, DL_GPIO_PIN_7, GPIOB, DL_GPIO_PIN_0 },
};

/* 限位开关端口/引脚 */
static GPIO_Regs * const g_limitPorts[] = { GPIOB, GPIOB, GPIOB };
static const uint32_t g_limitPins[] = { DL_GPIO_PIN_10, DL_GPIO_PIN_11, DL_GPIO_PIN_12 };

/* 温控PID */
typedef struct {
    float setpoint;      /* 目标温度 */
    float integral;      /* 积分累积 */
    float prev_error;    /* 上次误差 */
    float output;        /* PWM输出 0~100% */
    float kp, ki, kd;
} PID_Controller_t;

static PID_Controller_t g_hotendPID  = { .kp = PID_KP, .ki = PID_KI, .kd = PID_KD };
static PID_Controller_t g_bedPID     = { .kp = 1.5f,    .ki = 0.03f,  .kd = 80.0f  };
static float g_hotendTemp = 0.0f;
static float g_bedTemp    = 0.0f;

/* G-code队列 */
typedef struct {
    char cmd;           /* G或M */
    int  code;          /* 编号 */
    float x, y, z, e;  /* 坐标参数 */
    float f;            /* 进给速度 */
    float s;            /* S参数(温度/功率) */
    bool  has_x, has_y, has_z, has_e, has_f, has_s;
} GCodeCmd_t;

static GCodeCmd_t g_cmdQueue[GCODE_QUEUE_SIZE];
static volatile uint8_t g_cmdHead = 0, g_cmdTail = 0;

/* 运行状态 */
typedef enum {
    STATE_IDLE,
    STATE_HOMING,
    STATE_PRINTING,
    STATE_ERROR,
    STATE_PAUSED
} PrinterState_t;

static volatile PrinterState_t g_state = STATE_IDLE;
static bool g_absoluteMode = true;  /* G90绝对/G91相对 */
static float g_extruderPos = 0.0f;

/* ========== 延时函数 ========== */
static void delay_us(uint32_t us) {
    /* 粗略微秒延时 (32MHz下) */
    volatile uint32_t count = us * 8;  /* 每循环约4周期, 调整系数 */
    while (count--);
}

/* ========== UART输出 ========== */
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
    for (int i = 0; buf[i]; i++) {
        uart_sendchar(buf[i]);
    }
}

/* ========== 步进电机控制 ========== */

/**
 * @brief 使能/禁用步进驱动器
 */
static void stepper_enable(Axis_t axis, bool en) {
    g_steppers[axis].enabled = en;
    if (en)
        DL_GPIO_clearPins(g_pinMap[axis].en_port, g_pinMap[axis].en_pin);
    else
        DL_GPIO_setPins(g_pinMap[axis].en_port, g_pinMap[axis].en_pin);
}

/**
 * @brief 设置方向
 */
static void stepper_set_dir(Axis_t axis, bool forward) {
    if (forward)
        DL_GPIO_setPins(g_pinMap[axis].dir_port, g_pinMap[axis].dir_pin);
    else
        DL_GPIO_clearPins(g_pinMap[axis].dir_port, g_pinMap[axis].dir_pin);
}

/**
 * @brief 输出单个步进脉冲
 */
static void stepper_pulse(Axis_t axis) {
    DL_GPIO_setPins(g_pinMap[axis].step_port, g_pinMap[axis].step_pin);
    delay_us(STEP_PULSE_US);
    DL_GPIO_clearPins(g_pinMap[axis].step_port, g_pinMap[axis].step_pin);
    delay_us(STEP_PULSE_US);
}

/**
 * @brief 单轴移动指定步数
 * @param axis 轴
 * @param steps 步数(正=正方向, 负=反方向)
 * @param delay_per_step 每步延时(微秒), 控制速度
 */
static void stepper_move(Axis_t axis, int32_t steps, uint32_t delay_per_step) {
    if (steps == 0) return;

    stepper_set_dir(axis, steps > 0);
    uint32_t count = (uint32_t)(steps > 0 ? steps : -steps);

    for (uint32_t i = 0; i < count; i++) {
        /* 检查限位 */
        bool limit_hit = DL_GPIO_readPins(g_limitPorts[axis], g_limitPins[axis]);
        if (steps < 0 && limit_hit) break;  /* 负方向碰到限位停 */

        stepper_pulse(axis);
        g_steppers[axis].position += (steps > 0) ? 1 : -1;
        delay_us(delay_per_step);
    }
}

/**
 * @brief 读取限位开关状态 (true=触发)
 */
static bool stepper_read_limit(Axis_t axis) {
    return DL_GPIO_readPins(g_limitPorts[axis], g_limitPins[axis]) ? true : false;
}

/* ========== 温度测量 ========== */

/**
 * @brief 启动ADC采样
 */
static uint16_t adc_read(uint8_t channel) {
    DL_ADC12_startConversion(ADC0);
    while (!DL_ADC12_isConversionCompleted(ADC0));
    return (uint16_t)DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
}

/**
 * @brief ADC值转温度(NTC查表/公式)
 * @param adc_val 12位ADC值
 * @return 温度(°C)
 */
static float adc_to_temperature(uint16_t adc_val) {
    if (adc_val == 0 || adc_val >= ADC_RESOLUTION - 1) return 0.0f;

    /* NTC分压电路: Vout = Vcc * R_series / (R_ntc + R_series) */
    /* R_ntc = R_series * (ADC_MAX / adc_val - 1) */
    float resistance = NTC_SERIES_R * ((float)ADC_RESOLUTION / adc_val - 1.0f);

    /* Steinhart-Hart简化: 1/T = 1/T0 + (1/B)*ln(R/R0) */
    float steinhart = resistance / NTC_NOMINAL_R;
    steinhart = logf(steinhart);
    steinhart /= NTC_B_COEFF;
    steinhart += 1.0f / (NTC_NOMINAL_TEMP + 273.15f);
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;  /* Kelvin -> Celsius */

    return steinhart;
}

/* ========== PID温控 ========== */

/**
 * @brief PID计算并更新输出
 * @param pid PID控制器指针
 * @param current_temp 当前温度
 * @return PWM占空比 0~100
 */
static float pid_compute(PID_Controller_t *pid, float current_temp) {
    float error = pid->setpoint - current_temp;

    /* 积分累加(带限幅) */
    pid->integral += error * PID_KI;
    if (pid->integral > 100.0f) pid->integral = 100.0f;
    if (pid->integral < 0.0f)   pid->integral = 0.0f;

    /* 微分 */
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    /* PID输出 */
    float output = PID_KP * error + pid->integral + PID_KD * derivative;

    /* 限幅 0~100% */
    if (output > 100.0f) output = 100.0f;
    if (output < 0.0f)   output = 0.0f;

    pid->output = output;
    return output;
}

/**
 * @brief 设置加热PWM占空比
 * @param duty 0~100
 */
static void heater_set_pwm(uint8_t duty) {
    /* 使用PWM外设控制MOSFET */
    uint32_t period = DL_TimerG_getLoadValue(TIMER0);
    uint32_t compare = period * duty / 100;
    DL_TimerG_setCaptureCompareValue(TIMER0, compare, DL_TIMER_CC_0_INDEX);
}

/* ========== G-code解析 ========== */

/**
 * @brief 解析G-code行
 * @param line G-code字符串 (如 "G1 X10 Y20 F1500")
 * @param cmd 输出解析结果
 */
static void gcode_parse(char *line, GCodeCmd_t *cmd) {
    memset(cmd, 0, sizeof(GCodeCmd_t));

    /* 跳过空格 */
    while (*line == ' ') line++;

    /* 解析命令类型和编号 */
    if (*line == 'G' || *line == 'g') {
        cmd->cmd = 'G';
        cmd->code = atoi(line + 1);
    } else if (*line == 'M' || *line == 'm') {
        cmd->cmd = 'M';
        cmd->code = atoi(line + 1);
    } else {
        return;
    }

    /* 解析参数 */
    char *p = line + 1;
    while (*p) {
        if (*p == ' ') { p++; continue; }
        float val = strtof(p + 1, &p);
        switch (*(p - 1 >= line ? p : p)) { /* 安全处理 */
            /* 重新搜索参数字母 */
        }

        /* 更安全的解析方式: 逐字符扫描 */
        p = strchr(p, '\0'); /* 跳过当前数值 */
    }

    /* 简化版参数解析: 逐字符扫描整行 */
    p = line;
    while (*p) {
        char ch = *p++;
        if (ch >= 'a' && ch <= 'z') ch -= 32; /* 转大写 */

        if (ch == 'X') { cmd->x = strtof(p, &p); cmd->has_x = true; }
        else if (ch == 'Y') { cmd->y = strtof(p, &p); cmd->has_y = true; }
        else if (ch == 'Z') { cmd->z = strtof(p, &p); cmd->has_z = true; }
        else if (ch == 'E') { cmd->e = strtof(p, &p); cmd->has_e = true; }
        else if (ch == 'F') { cmd->f = strtof(p, &p); cmd->has_f = true; }
        else if (ch == 'S') { cmd->s = strtof(p, &p); cmd->has_s = true; }
    }
}

/**
 * @brief 执行G-code命令
 */
static void gcode_execute(GCodeCmd_t *cmd) {
    if (cmd->cmd == 'G') {
        switch (cmd->code) {
            case 0:  /* G0: 快速移动 */
            case 1:  /* G1: 直线插补 */
            {
                float feedrate = cmd->has_f ? cmd->f : DEFAULT_FEEDRATE * 60; /* mm/min -> mm/s需/60 */
                feedrate /= 60.0f;  /* 转为mm/s */
                if (feedrate > MAX_FEEDRATE_MM_S) feedrate = MAX_FEEDRATE_MM_S;

                float target[AXIS_COUNT] = {
                    g_steppers[AXIS_X].position / (float)STEPS_PER_MM_X,
                    g_steppers[AXIS_Y].position / (float)STEPS_PER_MM_Y,
                    g_steppers[AXIS_Z].position / (float)STEPS_PER_MM_Z
                };

                if (cmd->has_x) target[AXIS_X] = g_absoluteMode ? cmd->x : target[AXIS_X] + cmd->x;
                if (cmd->has_y) target[AXIS_Y] = g_absoluteMode ? cmd->y : target[AXIS_Y] + cmd->y;
                if (cmd->has_z) target[AXIS_Z] = g_absoluteMode ? cmd->z : target[AXIS_Z] + cmd->z;

                /* 计算各轴步数 */
                int32_t steps[AXIS_COUNT];
                steps[AXIS_X] = (int32_t)(target[AXIS_X] * STEPS_PER_MM_X) - g_steppers[AXIS_X].position;
                steps[AXIS_Y] = (int32_t)(target[AXIS_Y] * STEPS_PER_MM_Y) - g_steppers[AXIS_Y].position;
                steps[AXIS_Z] = (int32_t)(target[AXIS_Z] * STEPS_PER_MM_Z) - g_steppers[AXIS_Z].position;

                /* Bresenham多轴插补 */
                int32_t max_steps = 0;
                for (int i = 0; i < AXIS_COUNT; i++) {
                    int32_t abs_s = steps[i] > 0 ? steps[i] : -steps[i];
                    if (abs_s > max_steps) max_steps = abs_s;
                }

                if (max_steps > 0) {
                    /* 每步延时 = 1/(feedrate * steps_per_mm) 微秒 */
                    float slowest_steps_per_mm = STEPS_PER_MM_X;
                    if (STEPS_PER_MM_Y > slowest_steps_per_mm) slowest_steps_per_mm = STEPS_PER_MM_Y;
                    if (STEPS_PER_MM_Z > slowest_steps_per_mm) slowest_steps_per_mm = STEPS_PER_MM_Z;
                    uint32_t step_delay = (uint32_t)(1000000.0f / (feedrate * slowest_steps_per_mm));
                    if (step_delay < 10) step_delay = 10;

                    int32_t error[AXIS_COUNT] = {0};
                    for (int32_t i = 0; i < max_steps; i++) {
                        for (int a = 0; a < AXIS_COUNT; a++) {
                            error[a] += steps[a];
                            if (error[a] >= max_steps) {
                                error[a] -= max_steps;
                                stepper_pulse((Axis_t)a);
                                g_steppers[a].position++;
                            } else if (error[a] <= -max_steps) {
                                error[a] += max_steps;
                                stepper_set_dir((Axis_t)a, false);
                                stepper_pulse((Axis_t)a);
                                g_steppers[a].position--;
                                stepper_set_dir((Axis_t)a, true);
                            }
                        }
                        delay_us(step_delay);
                    }
                }
                uart_printf("ok\r\n");
                break;
            }

            case 28:  /* G28: 归零 */
                uart_printf("Homing...\r\n");
                for (int a = 0; a < AXIS_COUNT; a++) {
                    stepper_enable((Axis_t)a, true);
                    /* 负方向移动直到触发限位 */
                    stepper_set_dir((Axis_t)a, false);
                    while (!stepper_read_limit((Axis_t)a)) {
                        stepper_pulse((Axis_t)a);
                        delay_us(1000);  /* 慢速归零 */
                    }
                    /* 退回一小段 */
                    stepper_set_dir((Axis_t)a, true);
                    for (int i = 0; i < 50; i++) {
                        stepper_pulse((Axis_t)a);
                        delay_us(500);
                    }
                    g_steppers[a].position = 0;
                }
                uart_printf("Homing done\r\n");
                uart_printf("ok\r\n");
                break;

            case 90:  /* G90: 绝对坐标模式 */
                g_absoluteMode = true;
                uart_printf("ok\r\n");
                break;

            case 91:  /* G91: 相对坐标模式 */
                g_absoluteMode = false;
                uart_printf("ok\r\n");
                break;

            case 92:  /* G92: 设置当前位置 */
                if (cmd->has_x) g_steppers[AXIS_X].position = (int32_t)(cmd->x * STEPS_PER_MM_X);
                if (cmd->has_y) g_steppers[AXIS_Y].position = (int32_t)(cmd->y * STEPS_PER_MM_Y);
                if (cmd->has_z) g_steppers[AXIS_Z].position = (int32_t)(cmd->z * STEPS_PER_MM_Z);
                uart_printf("ok\r\n");
                break;

            default:
                uart_printf("ok\r\n");
                break;
        }
    } else if (cmd->cmd == 'M') {
        switch (cmd->code) {
            case 104:  /* M104: 设置热端温度 */
                if (cmd->has_s) {
                    g_hotendPID.setpoint = cmd->s;
                    if (g_hotendPID.setpoint > TEMP_MAX) g_hotendPID.setpoint = TEMP_MAX;
                    uart_printf("Hotend target: %.0fC\r\n", g_hotendPID.setpoint);
                }
                uart_printf("ok\r\n");
                break;

            case 140:  /* M140: 设置热床温度 */
                if (cmd->has_s) {
                    g_bedPID.setpoint = cmd->s;
                }
                uart_printf("ok\r\n");
                break;

            case 105:  /* M105: 报告温度 */
                uart_printf("T:%.1f /%.0f B:%.1f /%.0f\r\n",
                    g_hotendTemp, g_hotendPID.setpoint,
                    g_bedTemp, g_bedPID.setpoint);
                uart_printf("ok\r\n");
                break;

            case 106:  /* M106: 风扇开 */
            case 107:  /* M107: 风扇关 */
                uart_printf("ok\r\n");
                break;

            case 84:   /* M84: 禁用所有电机 */
                for (int i = 0; i < AXIS_COUNT; i++)
                    stepper_enable((Axis_t)i, false);
                uart_printf("ok\r\n");
                break;

            case 112:  /* M112: 紧急停止 */
                g_state = STATE_ERROR;
                for (int i = 0; i < AXIS_COUNT; i++)
                    stepper_enable((Axis_t)i, false);
                heater_set_pwm(0);
                g_hotendPID.setpoint = 0;
                g_bedPID.setpoint = 0;
                uart_printf("EMERGENCY STOP\r\n");
                break;

            default:
                uart_printf("ok\r\n");
                break;
        }
    }
}

/* ========== UART接收G-code ========== */
static char g_rxBuffer[GCODE_LINE_MAX];
static volatile uint8_t g_rxIndex = 0;

/**
 * @brief UART接收中断处理 (在startup文件中调用)
 */
void UART0_IRQHandler(void) {
    if (DL_UART_getPendingInterrupt(UART0) == DL_UART_IIDX_RX) {
        char c = (char)DL_UART_Main_receiveData(UART0);

        if (c == '\n' || c == '\r') {
            if (g_rxIndex > 0) {
                g_rxBuffer[g_rxIndex] = '\0';

                /* 去除注释 */
                char *comment = strchr(g_rxBuffer, ';');
                if (comment) *comment = '\0';

                /* 解析并入队 */
                if (g_cmdHead != (g_cmdTail + 1) % GCODE_QUEUE_SIZE) {
                    gcode_parse(g_rxBuffer, &g_cmdQueue[g_cmdTail]);
                    g_cmdTail = (g_cmdTail + 1) % GCODE_QUEUE_SIZE;
                }

                g_rxIndex = 0;
            }
        } else if (g_rxIndex < GCODE_LINE_MAX - 1) {
            g_rxBuffer[g_rxIndex++] = c;
        }
    }
}

/* ========== 定时器中断: 温控周期 ========== */
void TIMER1_IRQHandler(void) {
    if (DL_TimerG_getPendingInterrupt(TIMER1) == DL_TIMER_IIDX_ZERO) {
        /* 读取温度 */
        uint16_t adc_hotend = adc_read(5);
        g_hotendTemp = adc_to_temperature(adc_hotend);

        /* PID计算 */
        float duty = pid_compute(&g_hotendPID, g_hotendTemp);

        /* 安全检查: 温度过高强制关断 */
        if (g_hotendTemp > TEMP_MAX + 10) {
            duty = 0;
            g_state = STATE_ERROR;
        }

        heater_set_pwm((uint8_t)duty);
    }
}

/* ========== 主函数 ========== */
int main(void) {
    /* 初始化系统时钟和外设 */
    SYSCFG_DL_init();

    /* 初始化步进电机 - 全部禁用 */
    for (int i = 0; i < AXIS_COUNT; i++) {
        stepper_enable((Axis_t)i, false);
        g_steppers[i].position = 0;
        g_steppers[i].feedrate = DEFAULT_FEEDRATE;
    }

    /* 配置UART中断接收 */
    DL_UART_enableInterrupt(UART0, DL_UART_IIDX_RX);
    NVIC_EnableIRQ(UART0_IRQn);

    /* 配置温控定时器中断 */
    NVIC_EnableIRQ(TIMER1_IRQn);

    /* 启动温控定时器 */
    DL_TimerG_startCounter(TIMER1);

    /* 初始化PID */
    g_hotendPID.setpoint = 0;
    g_hotendPID.integral = 0;
    g_hotendPID.prev_error = 0;
    g_bedPID.setpoint = 0;

    uart_printf("\r\n=== MSPM0G3507 3D Printer Controller v1.0 ===\r\n");
    uart_printf("Commands: G0/G1 (move), G28 (home), G90/G91 (abs/rel)\r\n");
    uart_printf("          M104 (set hotend temp), M105 (report temp)\r\n");
    uart_printf("          M112 (emergency stop)\r\n");
    uart_printf("Ready.\r\n");

    /* 主循环 */
    while (1) {
        /* 处理G-code队列 */
        if (g_cmdHead != g_cmdTail) {
            gcode_execute(&g_cmdQueue[g_cmdHead]);
            g_cmdHead = (g_cmdHead + 1) % GCODE_QUEUE_SIZE;
        }

        /* 错误状态处理 */
        if (g_state == STATE_ERROR) {
            heater_set_pwm(0);
            for (int i = 0; i < AXIS_COUNT; i++)
                stepper_enable((Axis_t)i, false);
        }
    }
}
