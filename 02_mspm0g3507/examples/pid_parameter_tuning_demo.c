/**
 * @file pid_parameter_tuning_demo.c
 * @brief PID参数整定示例（在线调参+蓝牙监控+OLED显示）
 * @platform MSPM0G3507
 *
 * ============================================================
 * 接线说明
 * ============================================================
 * 模块          MSPM0G3507引脚      说明
 * ---------------------------------------------------------------
 * OLED SSD1306 (I2C)
 *   SCL        PB2 (I2C0_SCL)     I2C时钟
 *   SDA        PB3 (I2C0_SDA)     I2C数据
 *   VCC        3.3V
 *   GND        GND
 *
 * 蓝牙 HC-05/06 (UART)
 *   TXD        PA9 (UART0_RX)     蓝牙发送 -> MCU接收
 *   RXD        PA8 (UART0_TX)     MCU发送 -> 蓝牙接收
 *   VCC        5V (或模块自带3.3V稳压)
 *   GND        GND
 *
 * 电机编码器
 *   A相        PA12 (TimerA0_C0)  编码器A相输入
 *   B相        PA13 (TimerA0_C1)  编码器B相输入
 *
 * 电机驱动 L298N/TB6612
 *   PWM        PA0 (TimerA1_C0)   电机PWM输出
 *   DIR1       PA1                 方向控制1
 *   DIR2       PA2                 方向控制2
 *
 * 电位器（手动调参，可选）
 *   旋钮1      PA25 (ADC0_CH0)    调节Kp
 *   旋钮2      PA26 (ADC0_CH1)    调节Ki
 *   旋钮3      PA27 (ADC0_CH2)    调节Kd
 *
 * ============================================================
 * 功能说明
 * ============================================================
 * 1. 增量式PID电机速度闭环控制
 * 2. 三种调参模式：
 *    - 蓝牙串口远程调参（发送命令修改Kp/Ki/Kd/目标速度）
 *    - 电位器实时调参（旋钮映射到PID参数）
 *    - 预设参数快速切换
 * 3. OLED实时显示：目标速度、实际速度、误差、PID输出
 * 4. 蓝牙实时发送：速度曲线数据，可用上位机绘图
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ======================== 引脚定义宏 ======================== */
/* 电机驱动 */
#define MOTOR_PWM_INST          TIMER_0_INST
#define MOTOR_PWM_IDX           DL_TIMER_CC_0_INDEX
#define MOTOR_DIR1_PORT         GPIOA
#define MOTOR_DIR1_PIN          DL_GPIO_PIN_1
#define MOTOR_DIR2_PORT         GPIOA
#define MOTOR_DIR2_PIN          DL_GPIO_PIN_2

/* ======================== PID参数结构体 ======================== */
typedef struct {
    float kp;               /* 比例系数 */
    float ki;               /* 积分系数 */
    float kd;               /* 微分系数 */
    float target;           /* 目标速度 (脉冲/采样周期) */
    float error;            /* 当前误差 */
    float error_last;       /* 上次误差 */
    float error_prev;       /* 上上次误差 */
    float integral;         /* 累积积分 */
    float integral_max;     /* 积分限幅 */
    float output;           /* PID输出 */
    float output_max;       /* 输出限幅 */
    float dead_zone;        /* 死区 */
} PID_t;

/* ======================== 全局变量 ======================== */
static volatile uint32_t gSysTick = 0;          /* 系统滴答(1ms) */
static volatile int32_t gEncoderCount = 0;       /* 编码器计数 */
static volatile int32_t gMotorSpeed = 0;          /* 电机速度(脉冲/周期) */
static volatile bool gSpeedUpdated = false;       /* 速度更新标志 */

static PID_t gPID = {
    .kp = 2.0f,
    .ki = 0.5f,
    .kd = 0.1f,
    .target = 200.0f,
    .error = 0.0f,
    .error_last = 0.0f,
    .error_prev = 0.0f,
    .integral = 0.0f,
    .integral_max = 500.0f,
    .output = 0.0f,
    .output_max = 1000.0f,
    .dead_zone = 2.0f,
};

/* 蓝牙接收缓冲区 */
#define BT_BUF_SIZE 64
static volatile uint8_t gBTRxBuf[BT_BUF_SIZE];
static volatile uint8_t gBTRxIdx = 0;
static volatile bool gBTLineReady = false;

/* 调参模式枚举 */
typedef enum {
    TUNE_MODE_BLUETOOTH = 0,    /* 蓝牙远程调参 */
    TUNE_MODE_POTENTIOMETER,    /* 电位器调参 */
    TUNE_MODE_PRESET,           /* 预设参数 */
} TuneMode_t;

static TuneMode_t gTuneMode = TUNE_MODE_BLUETOOTH;

/* 预设PID参数组 */
typedef struct {
    const char *name;
    float kp, ki, kd;
} PresetParams_t;

static const PresetParams_t gPresets[] = {
    { "Slow-Response",  1.0f, 0.2f, 0.05f },   /* 慢响应，超调小 */
    { "Balanced",       2.0f, 0.5f, 0.10f },   /* 均衡参数 */
    { "Fast-Response",  4.0f, 1.0f, 0.20f },   /* 快响应，可能超调 */
    { "Aggressive",     6.0f, 2.0f, 0.50f },   /* 激进参数 */
};
#define PRESET_COUNT (sizeof(gPresets) / sizeof(gPresets[0]))
static uint8_t gPresetIndex = 1;  /* 默认使用均衡参数 */

/* ======================== OLED驱动（简化版） ======================== */
/* SSD1306 I2C地址 */
#define OLED_ADDR       0x3C
#define OLED_CMD        0x00
#define OLED_DATA       0x40

/* 简易6x8字体（ASCII 32-127）— 只列出部分常用字符 */
static const uint8_t font6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* '!' */
    /* ... 省略中间字符，实际使用需完整字库 ... */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* '9' */
};

/**
 * @brief I2C发送数据到OLED
 */
static void oled_write_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {OLED_CMD, cmd};
    DL_I2C_startTransfer(I2C_0_INST);
    DL_I2C_transmitData(I2C_0_INST, OLED_ADDR);
    DL_I2C_transmitData(I2C_0_INST, buf[0]);
    DL_I2C_transmitData(I2C_0_INST, buf[1]);
    DL_I2C_stopTransfer(I2C_0_INST);
    /* 等待传输完成 */
    volatile uint32_t _i2c_timeout = 100000;  /* V2审计: I2C超时 */
    while (DL_I2C_isBusy(I2C_0_INST)); && --_i2c_timeout
}

static void oled_write_data(uint8_t data)
{
    uint8_t buf[2] = {OLED_DATA, data};
    DL_I2C_startTransfer(I2C_0_INST);
    DL_I2C_transmitData(I2C_0_INST, OLED_ADDR);
    DL_I2C_transmitData(I2C_0_INST, buf[0]);
    DL_I2C_transmitData(I2C_0_INST, buf[1]);
    DL_I2C_stopTransfer(I2C_0_INST);
    volatile uint32_t _i2c_timeout = 100000;  /* V2审计: I2C超时 */
    while (DL_I2C_isBusy(I2C_0_INST)); && --_i2c_timeout
}

/**
 * @brief 设置OLED光标位置
 */
static void oled_set_cursor(uint8_t page, uint8_t col)
{
    oled_write_cmd(0xB0 + page);         /* 页地址 */
    oled_write_cmd(0x00 + (col & 0x0F)); /* 列低4位 */
    oled_write_cmd(0x10 + (col >> 4));   /* 列高4位 */
}

/**
 * @brief 清屏
 */
static void oled_clear(void)
{
    for (uint8_t page = 0; page < 8; page++) {
        oled_set_cursor(page, 0);
        for (uint8_t col = 0; col < 128; col++) {
            oled_write_data(0x00);
        }
    }
}

/**
 * @brief 显示字符串（简化版，仅支持数字和部分符号）
 */
static void oled_show_string(uint8_t page, uint8_t col, const char *str)
{
    oled_set_cursor(page, col);
    while (*str) {
        char c = *str++;
        if (c >= '0' && c <= '9') {
            for (uint8_t i = 0; i < 6; i++) {
                oled_write_data(font6x8[c - '0' + 3][i]);
            }
        } else if (c == '.') {
            oled_write_data(0x00); oled_write_data(0x00);
            oled_write_data(0x60); oled_write_data(0x60);
            oled_write_data(0x00); oled_write_data(0x00);
        } else if (c == '-') {
            oled_write_data(0x08); oled_write_data(0x08);
            oled_write_data(0x08); oled_write_data(0x08);
            oled_write_data(0x08); oled_write_data(0x08);
        } else if (c == ' ') {
            for (uint8_t i = 0; i < 6; i++) oled_write_data(0x00);
        } else {
            /* 其他字符用空白代替 */
            for (uint8_t i = 0; i < 6; i++) oled_write_data(0x00);
        }
    }
}

/**
 * @brief OLED初始化
 */
static void oled_init(void)
{
    /* 延时等待OLED上电稳定 */
    for (volatile uint32_t i = 0; i < 100000; i++);

    oled_write_cmd(0xAE);  /* 关显示 */
    oled_write_cmd(0xD5);  /* 设置时钟分频 */
    oled_write_cmd(0x80);
    oled_write_cmd(0xA8);  /* 设置多路复用率 */
    oled_write_cmd(0x3F);  /* 1/64 */
    oled_write_cmd(0xD3);  /* 设置显示偏移 */
    oled_write_cmd(0x00);
    oled_write_cmd(0x40);  /* 设置起始行 */
    oled_write_cmd(0x8D);  /* 电荷泵 */
    oled_write_cmd(0x14);  /* 使能 */
    oled_write_cmd(0x20);  /* 寻址模式 */
    oled_write_cmd(0x02);  /* 页寻址 */
    oled_write_cmd(0xA1);  /* 段重映射 */
    oled_write_cmd(0xC8);  /* COM扫描方向 */
    oled_write_cmd(0xDA);  /* COM引脚配置 */
    oled_write_cmd(0x12);
    oled_write_cmd(0x81);  /* 对比度 */
    oled_write_cmd(0xCF);
    oled_write_cmd(0xD9);  /* 预充电周期 */
    oled_write_cmd(0xF1);
    oled_write_cmd(0xDB);  /* VCOMH取消选择电平 */
    oled_write_cmd(0x40);
    oled_write_cmd(0xA4);  /* 全局显示开启 */
    oled_write_cmd(0xA6);  /* 正常显示 */
    oled_write_cmd(0xAF);  /* 开显示 */

    oled_clear();
}

/**
 * @brief 在OLED上显示PID调参界面
 */
static void oled_display_pid_info(const PID_t *pid, int32_t speed)
{
    char buf[32];

    /* 第0-1行: 模式和目标速度 */
    oled_show_string(0, 0, "Tgt:       ");
    /* 简化：显示目标速度的整数部分 */
    int32_t tgt = (int32_t)pid->target;
    snprintf(buf, sizeof(buf), "Tgt:%ld", tgt);
    oled_show_string(0, 0, buf);

    /* 第2-3行: 实际速度 */
    snprintf(buf, sizeof(buf), "Spd:%ld", speed);
    oled_show_string(1, 0, buf);

    /* 第4-5行: PID参数 */
    snprintf(buf, sizeof(buf), "P:%d I:%d", (int)(pid->kp * 100), (int)(pid->ki * 100));
    oled_show_string(2, 0, buf);

    /* 第6-7行: D参数和误差 */
    snprintf(buf, sizeof(buf), "D:%d E:%d", (int)(pid->kd * 100), (int)pid->error);
    oled_show_string(3, 0, buf);

    /* 第8-9行: PID输出 */
    snprintf(buf, sizeof(buf), "Out:%d", (int)pid->output);
    oled_show_string(4, 0, buf);

    /* 第10-11行: 模式信息 */
    switch (gTuneMode) {
    case TUNE_MODE_BLUETOOTH: oled_show_string(5, 0, "Mode:BT"); break;
    case TUNE_MODE_POTENTIOMETER: oled_show_string(5, 0, "Mode:POT"); break;
    case TUNE_MODE_PRESET:
        snprintf(buf, sizeof(buf), "Pre:%s", gPresets[gPresetIndex].name);
        oled_show_string(5, 0, buf);
        break;
    }
}

/* ======================== 蓝牙/串口通信 ======================== */

/**
 * @brief 蓝牙发送字节
 */
static void bt_send_byte(uint8_t byte)
{
    while (!DL_UART_isTXFIFOEmpty(UART_0_INST));
    DL_UART_transmitDataBlocking(UART_0_INST, byte);
}

/**
 * @brief 蓝牙发送字符串
 */
static void bt_send_string(const char *str)
{
    while (*str) {
        bt_send_byte(*str++);
    }
}

/**
 * @brief 发送速度曲线数据（供上位机绘图）
 *
 * 格式: "DATA,<目标>,<实际>,<输出>\r\n"
 */
static void bt_send_plot_data(const PID_t *pid, int32_t speed)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "DATA,%d,%ld,%d\r\n",
             (int)pid->target, speed, (int)pid->output);
    bt_send_string(buf);
}

/**
 * @brief 解析蓝牙收到的调参命令
 *
 * 支持的命令格式:
 *   "P123\n"     -> 设置Kp = 1.23
 *   "I050\n"     -> 设置Ki = 0.50
 *   "D020\n"     -> 设置Kd = 0.20
 *   "S300\n"     -> 设置目标速度 = 300
 *   "M0\n"       -> 切换到蓝牙调参模式
 *   "M1\n"       -> 切换到电位器调参模式
 *   "M2\n"       -> 切换到预设参数模式
 *   "N\n"        -> 下一组预设参数
 *   "R\n"        -> 重置PID
 *   "Q\n"        -> 查询当前参数
 */
static void bt_parse_command(const char *cmd)
{
    if (cmd[0] == 'P' && strlen(cmd) >= 2) {
        /* 解析Kp: "P" + 3位整数 = 实际值/100 */
        float val = (float)atoi(&cmd[1]) / 100.0f;
        if (val >= 0.0f && val <= 50.0f) {
            gPID.kp = val;
            bt_send_string("OK Kp set\r\n");
        }
    } else if (cmd[0] == 'I' && strlen(cmd) >= 2) {
        float val = (float)atoi(&cmd[1]) / 100.0f;
        if (val >= 0.0f && val <= 20.0f) {
            gPID.ki = val;
            bt_send_string("OK Ki set\r\n");
        }
    } else if (cmd[0] == 'D' && strlen(cmd) >= 2) {
        float val = (float)atoi(&cmd[1]) / 100.0f;
        if (val >= 0.0f && val <= 10.0f) {
            gPID.kd = val;
            bt_send_string("OK Kd set\r\n");
        }
    } else if (cmd[0] == 'S' && strlen(cmd) >= 2) {
        int32_t spd = atoi(&cmd[1]);
        if (spd >= 0 && spd <= 2000) {
            gPID.target = (float)spd;
            bt_send_string("OK Target set\r\n");
        }
    } else if (cmd[0] == 'M' && strlen(cmd) >= 2) {
        uint8_t mode = cmd[1] - '0';
        if (mode <= TUNE_MODE_PRESET) {
            gTuneMode = (TuneMode_t)mode;
            bt_send_string("OK Mode set\r\n");
        }
    } else if (cmd[0] == 'N') {
        /* 切换下一组预设参数 */
        if (gTuneMode == TUNE_MODE_PRESET) {
            gPresetIndex = (gPresetIndex + 1) % PRESET_COUNT;
            gPID.kp = gPresets[gPresetIndex].kp;
            gPID.ki = gPresets[gPresetIndex].ki;
            gPID.kd = gPresets[gPresetIndex].kd;
            bt_send_string("OK Preset next\r\n");
        }
    } else if (cmd[0] == 'R') {
        /* 重置PID */
        gPID.error = 0;
        gPID.error_last = 0;
        gPID.error_prev = 0;
        gPID.integral = 0;
        gPID.output = 0;
        bt_send_string("OK PID reset\r\n");
    } else if (cmd[0] == 'Q') {
        /* 查询当前参数 */
        char buf[64];
        snprintf(buf, sizeof(buf), "PARAMS Kp=%d Ki=%d Kd=%d Tgt=%d\r\n",
                 (int)(gPID.kp * 100), (int)(gPID.ki * 100),
                 (int)(gPID.kd * 100), (int)gPID.target);
        bt_send_string(buf);
    } else {
        bt_send_string("ERR Unknown cmd\r\n");
    }
}

/* ======================== UART中断 ======================== */
void UART_0_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_0_INST) == DL_UART_IIDX_RX) {
        volatile uint8_t byte = DL_UART_receiveDataBlocking(UART_0_INST);

        /* 行缓冲：收到换行符时标记命令就绪 */
        if (byte == '\n' || byte == '\r') {
            if (gBTRxIdx > 0) {
                gBTRxBuf[gBTRxIdx] = '\0';
                gBTLineReady = true;
            }
        } else {
            if (gBTRxIdx < BT_BUF_SIZE - 1) {
                gBTRxBuf[gBTRxIdx++] = byte;
            }
        }
    }
}

/* ======================== 增量式PID算法 ======================== */

/**
 * @brief 增量式PID计算
 * @param pid PID结构体指针
 * @param measurement 当前测量值（实际速度）
 * @return PID输出增量
 *
 * 增量式PID公式:
 * Δu = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2*e[k-1]+e[k-2])
 *
 * 优点：
 * - 不需要累积积分，避免积分饱和
 * - 输出增量有限，更安全
 * - 适合数字实现
 */
static float pid_incremental_calculate(PID_t *pid, float measurement)
{
    pid->error = pid->target - measurement;

    /* 死区处理 */
    if (fabsf(pid->error) < pid->dead_zone) {
        pid->error = 0.0f;
    }

    /* 计算增量 */
    float delta = pid->kp * (pid->error - pid->error_last)
                + pid->ki * pid->error
                + pid->kd * (pid->error - 2.0f * pid->error_last + pid->error_prev);

    /* 更新历史误差 */
    pid->error_prev = pid->error_last;
    pid->error_last = pid->error;

    /* 累加到输出 */
    pid->output += delta;

    /* 输出限幅 */
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;

    return pid->output;
}

/* ======================== 电机控制 ======================== */

/**
 * @brief 设置电机输出（方向 + PWM占空比）
 * @param value PID输出值（-1000 ~ +1000）
 */
static void motor_set_output(float value)
{
    if (value > 0) {
        /* 正转 */
        DL_GPIO_setPins(MOTOR_DIR1_PORT, MOTOR_DIR1_PIN);
        DL_GPIO_clearPins(MOTOR_DIR2_PORT, MOTOR_DIR2_PIN);
    } else if (value < 0) {
        /* 反转 */
        DL_GPIO_clearPins(MOTOR_DIR1_PORT, MOTOR_DIR1_PIN);
        DL_GPIO_setPins(MOTOR_DIR2_PORT, MOTOR_DIR2_PIN);
        value = -value;
    } else {
        /* 停止 */
        DL_GPIO_clearPins(MOTOR_DIR1_PORT, MOTOR_DIR1_PIN);
        DL_GPIO_clearPins(MOTOR_DIR2_PORT, MOTOR_DIR2_PIN);
    }

    /* 限幅 */
    if (value > 1000.0f) value = 1000.0f;

    /* 设置PWM占空比 */
    uint32_t pwm = (uint32_t)(value * 100 / 1000);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, pwm, MOTOR_PWM_IDX);
}

/* ======================== 电位器调参 ======================== */

/**
 * @brief 读取电位器并映射到PID参数
 *
 * 电位器0: Kp范围 0.0 ~ 10.0
 * 电位器1: Ki范围 0.0 ~ 5.0
 * 电位器2: Kd范围 0.0 ~ 2.0
 */
static void potentiometer_tune(void)
{
    if (gTuneMode != TUNE_MODE_POTENTIOMETER) return;

    /* 读取ADC通道0 -> Kp */
    DL_ADC12_startConversion(ADC12_0_INST);
    while (!(DL_ADC12_getStatus(ADC12_0_INST) & DL_ADC12_STATUS_CONVERSION_DONE));
    uint16_t adc0 = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
    gPID.kp = (float)adc0 / 4096.0f * 10.0f;

    /* 读取ADC通道1 -> Ki */
    DL_ADC12_startConversion(ADC12_0_INST);
    while (!(DL_ADC12_getStatus(ADC12_0_INST) & DL_ADC12_STATUS_CONVERSION_DONE));
    uint16_t adc1 = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
    gPID.ki = (float)adc1 / 4096.0f * 5.0f;

    /* 读取ADC通道2 -> Kd */
    DL_ADC12_startConversion(ADC12_0_INST);
    while (!(DL_ADC12_getStatus(ADC12_0_INST) & DL_ADC12_STATUS_CONVERSION_DONE));
    uint16_t adc2 = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_2);
    gPID.kd = (float)adc2 / 4096.0f * 2.0f;
}

/* ======================== 编码器中断 ======================== */
void TIMER_0_INST_IRQHandler(void)
{
    /* 正交编码器模式下自动计数，读取计数值 */
    if (DL_Timer_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO_EVENT) {
        /* 定时器溢出，可以在这里处理上溢/下溢 */
    }
}

/* ======================== SysTick中断（1ms） ======================== */
void SysTick_Handler(void)
{
    gSysTick++;
}

/* ======================== PID控制定时中断 ======================== */

/**
 * @brief 定时执行PID控制（10ms周期，使用Timer中断）
 */
static volatile bool gPIDTick = false;

/* 可以用另一个定时器触发，这里用SysTick分频实现 */
static void pid_timer_check(void)
{
    static uint32_t last_tick = 0;
    if (gSysTick - last_tick >= 10) {  /* 10ms周期 */
        last_tick = gSysTick;

        /* 读取编码器计数作为当前速度 */
        gMotorSpeed = DL_Timer_getTimerCount(TIMER_0_INST);
        DL_Timer_setTimerCount(TIMER_0_INST, 0);  /* 清零 */

        gPIDTick = true;
    }
}

/* ======================== 主函数 ======================== */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    /* SysTick 1ms */
    SysTick_Config(SystemCoreClock / 1000);

    /* 初始化OLED */
    oled_init();
    oled_show_string(0, 0, "PID Tuning");
    oled_show_string(1, 0, "Initializing...");

    /* 初始化UART（蓝牙） */
    NVIC_EnableIRQ(UART_0_INST_IRQ);

    /* 发送欢迎信息 */
    bt_send_string("=== PID Tuning Tool ===\r\n");
    bt_send_string("Commands:\r\n");
    bt_send_string("  P<val> - Set Kp (val/100)\r\n");
    bt_send_string("  I<val> - Set Ki (val/100)\r\n");
    bt_send_string("  D<val> - Set Kd (val/100)\r\n");
    bt_send_string("  S<val> - Set target speed\r\n");
    bt_send_string("  M<0|1|2> - Tune mode\r\n");
    bt_send_string("  N - Next preset\r\n");
    bt_send_string("  R - Reset PID\r\n");
    bt_send_string("  Q - Query params\r\n");

    /* 显示初始参数 */
    bt_send_string("Q\r\n");  /* 触发查询显示 */

    uint32_t display_update = 0;
    uint32_t plot_update = 0;

    /* ======================== 主循环 ======================== */
    while (1) {
        /* 处理蓝牙命令 */
        if (gBTLineReady) {
            __disable_irq();
            char cmd[BT_BUF_SIZE];
            strcpy(cmd, (const char *)gBTRxBuf);
            gBTRxIdx = 0;
            gBTLineReady = false;
            __enable_irq();

            bt_parse_command(cmd);
        }

        /* 电位器调参（仅在对应模式下） */
        if (gTuneMode == TUNE_MODE_POTENTIOMETER) {
            potentiometer_tune();
        }

        /* PID定时执行 */
        pid_timer_check();
        if (gPIDTick) {
            gPIDTick = false;

            /* 执行PID计算 */
            float pid_out = pid_incremental_calculate(&gPID, (float)gMotorSpeed);

            /* 设置电机输出 */
            motor_set_output(pid_out);
        }

        /* OLED显示更新（每200ms） */
        if (gSysTick - display_update >= 200) {
            display_update = gSysTick;
            oled_display_pid_info(&gPID, gMotorSpeed);
        }

        /* 蓝牙绘图数据发送（每50ms） */
        if (gSysTick - plot_update >= 50) {
            plot_update = gSysTick;
            bt_send_plot_data(&gPID, gMotorSpeed);
        }
    }

    return 0;
}
