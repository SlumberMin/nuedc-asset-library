/**
 * @file precision_voltage_source.c
 * @brief MSPM0G3507 精密电压源示例（MCP4921 + PID稳压 + 电压/电流监测）
 *
 * 硬件连接：
 *   MCP4921 DAC（SPI）：
 *     SCK  -> PA0 (SPI0_SCK)
 *     MOSI -> PA1 (SPI0_MOSI)
 *     CS   -> PA2 (GPIO，片选)
 *     LDAC -> PA3 (GPIO，同步锁存)
 *
 *   电压采样（ADC）：
 *     输出电压 -> PA22 (ADC0_CH0)  经分压电阻（10K/10K）后采样
 *     输出电流 -> PA23 (ADC0_CH1)  经采样电阻（0.1Ω）和运放后采样
 *
 *   OLED显示屏（I2C，0x3C，128x64）：
 *     SCL -> PB2 (I2C0_SCL)
 *     SDA -> PB3 (I2C0_SDA)
 *
 *   按键：
 *     目标电压+  -> PA11
 *     目标电压-  -> PA12
 *     步进切换   -> PA13
 *     输出使能   -> PA14
 *
 * 功能：
 *   - MCP4921产生0-4.096V基准电压
 *   - PID闭环控制输出电压精确跟踪目标值
 *   - 实时监测输出电压和电流
 *   - 过流保护（软件限流）
 *   - OLED显示电压、电流、功率和PID状态
 *   - 支持1mV/10mV/100mV三种步进调节
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== MCP4921引脚定义 ========== */
#define MCP4921_CS_PORT         GPIOA
#define MCP4921_CS_PIN          DL_GPIO_PIN_2
#define MCP4921_LDAC_PORT       GPIOA
#define MCP4921_LDAC_PIN        DL_GPIO_PIN_3

/* ========== 按键引脚定义 ========== */
#define BTN_UP_PORT             GPIOA
#define BTN_UP_PIN              DL_GPIO_PIN_11
#define BTN_DOWN_PORT           GPIOA
#define BTN_DOWN_PIN            DL_GPIO_PIN_12
#define BTN_STEP_PORT           GPIOA
#define BTN_STEP_PIN            DL_GPIO_PIN_13
#define BTN_ENABLE_PORT         GPIOA
#define BTN_ENABLE_PIN          DL_GPIO_PIN_14

/* ========== ADC通道 ========== */
#define ADC_VOLTAGE_CH          DL_ADC12_MEM_IDX_0    /* PA22 */
#define ADC_CURRENT_CH          DL_ADC12_MEM_IDX_1    /* PA23 */

/* ========== DAC参数 ========== */
#define DAC_RESOLUTION          4096      /* 12位DAC */
#define DAC_VREF_MV             4096      /* 外部参考电压4.096V */
#define DAC_MAX_OUTPUT_MV       4096      /* 最大输出电压mV */

/* ========== 电压采样参数 ========== */
#define VOLTAGE_DIVIDER_RATIO   2.0f      /* 分压比 (R1+R2)/R2 = 20K/10K */
#define ADC_RESOLUTION          4096
#define ADC_VREF_MV             3300      /* ADC参考电压3.3V */
#define CURRENT_SENSE_R_OHM     0.1f      /* 采样电阻 */
#define CURRENT_GAIN            50.0f     /* 运放增益 */

/* ========== PID参数 ========== */
#define PID_KP                  2.5f
#define PID_KI                  0.8f
#define PID_KD                  0.1f
#define PID_OUTPUT_MAX          4095.0f
#define PID_OUTPUT_MIN          0.0f
#define PID_INTEGRAL_MAX        1000.0f   /* 积分限幅 */
#define PID_INTEGRAL_MIN       -1000.0f

/* ========== 保护参数 ========== */
#define OVERCURRENT_MA          500       /* 过流阈值 500mA */
#define OVERVOLTAGE_MV          4200      /* 过压阈值 4200mV */

/* ========== PID控制器结构体 ========== */
typedef struct {
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float kd;           /* 微分系数 */
    float integral;     /* 积分累计 */
    float prevError;    /* 上次误差 */
    float outputMax;    /* 输出上限 */
    float outputMin;    /* 输出下限 */
    float intMax;       /* 积分上限 */
    float intMin;       /* 积分下限 */
} PID_Controller_t;

/* ========== 电源状态结构体 ========== */
typedef struct {
    float    targetVoltage_mV;    /* 目标电压(mV) */
    float    actualVoltage_mV;    /* 实际输出电压(mV) */
    float    outputCurrent_mA;    /* 输出电流(mA) */
    float    outputPower_mW;      /* 输出功率(mW) */
    uint16_t dacCode;             /* DAC当前输出值 */
    bool     outputEnabled;       /* 输出使能 */
    bool     overcurrentFault;    /* 过流故障 */
    uint8_t  stepIndex;           /* 当前步进档位 */
} PowerSupply_t;

/* ========== 全局变量 ========== */
static PID_Controller_t g_pid;
static PowerSupply_t    g_ps;
static volatile uint32_t g_systickCount = 0;

/* 步进档位表(mV) */
static const uint16_t g_stepTable[] = {1, 10, 100, 1000};
#define STEP_TABLE_SIZE  4

/* OLED显存缓冲区 */
static uint8_t g_oledBuffer[128 * 64 / 8];

/* 基本字体 5x8 */
static const uint8_t g_font5x8[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00},
    {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14},
    {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x01,0x01}, {0x3E,0x41,0x41,0x51,0x32},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F},
    {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03},
    {0x61,0x51,0x49,0x45,0x43},
};

/* ========== 函数声明 ========== */
/* 延时 */
void SysTick_Handler(void);
static void Delay_ms(uint32_t ms);

/* MCP4921驱动 */
static void     MCP4921_Init(void);
static void     MCP4921_WriteDAC(uint16_t value);

/* ADC采样 */
static uint16_t ADC_ReadVoltage(void);
static uint16_t ADC_ReadCurrent(void);
static float    ADC_ToVoltage_mV(uint16_t adcVal);
static float    ADC_ToCurrent_mA(uint16_t adcVal);

/* PID控制 */
static void     PID_Init(PID_Controller_t *pid, float kp, float ki, float kd);
static float    PID_Compute(PID_Controller_t *pid, float setpoint, float measured);
static void     PID_Reset(PID_Controller_t *pid);

/* OLED驱动 */
static void     OLED_WriteCmd(uint8_t cmd);
static void     OLED_WriteData(uint8_t data);
static void     OLED_Init(void);
static void     OLED_Clear(void);
static void     OLED_SetPixel(int16_t x, int16_t y, uint8_t color);
static void     OLED_FillRect(int16_t x, int16_t y, int16_t w, int16_t h);
static void     OLED_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h);
static void     OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
static void     OLED_DrawChar(int16_t x, int16_t y, char ch, uint8_t size);
static void     OLED_DrawString(int16_t x, int16_t y, const char *str, uint8_t size);
static void     OLED_Update(void);

/* UI */
static void     UI_UpdateDisplay(void);
static bool     Button_IsPressed(GPIO_Regs *port, uint32_t pin);
static void     PowerSupply_SetVoltage(float target_mV);
static void     PowerSupply_Update(void);

/* ========== 延时函数 ========== */
void SysTick_Handler(void) {
    g_systickCount++;
}

static void Delay_ms(uint32_t ms) {
    uint32_t start = g_systickCount;
    while ((g_systickCount - start) < ms);
}

/* ========== MCP4921驱动 ========== */
/*
 * MCP4921 12位DAC SPI协议：
 * 16位数据格式：[A/B][BUF][GA][SHDN][D11:D0]
 *   bit15: A/B=0选择通道A
 *   bit14: BUF=1缓冲
 *   bit13: GA=1增益1x(=VREF), GA=0增益2x(=2*VREF)
 *   bit12: SHDN=1输出使能
 */
static void MCP4921_Init(void) {
    DL_GPIO_setPins(MCP4921_CS_PORT, MCP4921_CS_PIN);   /* CS默认高 */
    DL_GPIO_setPins(MCP4921_LDAC_PORT, MCP4921_LDAC_PIN); /* LDAC高 */
}

static void MCP4921_WriteDAC(uint16_t value) {
    uint16_t cmd = 0;
    cmd |= (0 << 15);       /* Channel A */
    cmd |= (1 << 14);       /* Buffered */
    cmd |= (1 << 13);       /* Gain = 1x (VOUT = VREF * D/4096) */
    cmd |= (1 << 12);       /* Output enabled (SHDN=1) */
    cmd |= (value & 0x0FFF); /* 12位数据 */

    uint8_t txBuf[2];
    txBuf[0] = (cmd >> 8) & 0xFF;
    txBuf[1] = cmd & 0xFF;

    /* CS拉低，发送数据 */
    DL_GPIO_clearPins(MCP4921_CS_PORT, MCP4921_CS_PIN);
    DL_SPI_fillControllerTXFIFO(SPI_0_INST, txBuf, 2);
    while (DL_SPI_isBusy(SPI_0_INST));
    DL_GPIO_setPins(MCP4921_CS_PORT, MCP4921_CS_PIN);

    /* LDAC脉冲：拉低触发同步更新 */
    DL_GPIO_clearPins(MCP4921_LDAC_PORT, MCP4921_LDAC_PIN);
    __NOP(); __NOP(); __NOP(); __NOP();
    DL_GPIO_setPins(MCP4921_LDAC_PORT, MCP4921_LDAC_PIN);

    g_ps.dacCode = value;
}

/* ========== ADC采样 ========== */
static uint16_t ADC_ReadVoltage(void) {
    DL_ADC12_startConversion(ADC12_0_INST);
    while (!DL_ADC12_isConversionComplete(ADC12_0_INST));
    return DL_ADC12_getMemResult(ADC12_0_INST, ADC_VOLTAGE_CH);
}

static uint16_t ADC_ReadCurrent(void) {
    DL_ADC12_startConversion(ADC12_0_INST);
    while (!DL_ADC12_isConversionComplete(ADC12_0_INST));
    return DL_ADC12_getMemResult(ADC12_0_INST, ADC_CURRENT_CH);
}

static float ADC_ToVoltage_mV(uint16_t adcVal) {
    /* ADC值转换为实际输出电压(mV) */
    float adc_mV = (float)adcVal * ADC_VREF_MV / ADC_RESOLUTION;
    return adc_mV * VOLTAGE_DIVIDER_RATIO;  /* 还原分压前电压 */
}

static float ADC_ToCurrent_mA(uint16_t adcVal) {
    /* ADC值转换为实际电流(mA) */
    float amp_mV = (float)adcVal * ADC_VREF_MV / ADC_RESOLUTION;
    float current_A = amp_mV / (CURRENT_SENSE_R_OHM * CURRENT_GAIN * 1000.0f);
    return current_A * 1000.0f;  /* 转mA */
}

/* ========== PID控制器 ========== */
static void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->outputMax = PID_OUTPUT_MAX;
    pid->outputMin = PID_OUTPUT_MIN;
    pid->intMax = PID_INTEGRAL_MAX;
    pid->intMin = PID_INTEGRAL_MIN;
}

static float PID_Compute(PID_Controller_t *pid, float setpoint, float measured) {
    float error = setpoint - measured;

    /* 比例项 */
    float pTerm = pid->kp * error;

    /* 积分项（带限幅） */
    pid->integral += error;
    if (pid->integral > pid->intMax) pid->integral = pid->intMax;
    if (pid->integral < pid->intMin) pid->integral = pid->intMin;
    float iTerm = pid->ki * pid->integral;

    /* 微分项 */
    float dTerm = pid->kd * (error - pid->prevError);
    pid->prevError = error;

    /* 输出叠加 */
    float output = pTerm + iTerm + dTerm;

    /* 输出限幅 */
    if (output > pid->outputMax) output = pid->outputMax;
    if (output < pid->outputMin) output = pid->outputMin;

    return output;
}

static void PID_Reset(PID_Controller_t *pid) {
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

/* ========== OLED驱动 ========== */
static void OLED_WriteCmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, 0x3C,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static void OLED_WriteData(uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, 0x3C,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static void OLED_Init(void) {
    Delay_ms(100);
    OLED_WriteCmd(0xAE); OLED_WriteCmd(0xD5); OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F); OLED_WriteCmd(0xD3);
    OLED_WriteCmd(0x00); OLED_WriteCmd(0x40); OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14); OLED_WriteCmd(0x20); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xA1); OLED_WriteCmd(0xC8); OLED_WriteCmd(0xDA);
    OLED_WriteCmd(0x12); OLED_WriteCmd(0x81); OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0xF1); OLED_WriteCmd(0xDB);
    OLED_WriteCmd(0x30); OLED_WriteCmd(0xA4); OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0xAF);
    OLED_Clear();
    OLED_Update();
}

static void OLED_Clear(void) {
    memset(g_oledBuffer, 0, sizeof(g_oledBuffer));
}

static void OLED_SetPixel(int16_t x, int16_t y, uint8_t color) {
    if (x < 0 || x >= 128 || y < 0 || y >= 64) return;
    uint16_t idx = (y / 8) * 128 + x;
    uint8_t bit = (1 << (y & 7));
    if (color) g_oledBuffer[idx] |= bit;
    else       g_oledBuffer[idx] &= ~bit;
}

static void OLED_FillRect(int16_t x, int16_t y, int16_t w, int16_t h) {
    for (int16_t i = x; i < x + w; i++)
        for (int16_t j = y; j < y + h; j++)
            OLED_SetPixel(i, j, 1);
}

static void OLED_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h) {
    OLED_DrawLine(x, y, x+w-1, y);
    OLED_DrawLine(x+w-1, y, x+w-1, y+h-1);
    OLED_DrawLine(x+w-1, y+h-1, x, y+h-1);
    OLED_DrawLine(x, y+h-1, x, y);
}

static void OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    int16_t dx = abs(x1-x0), dy = abs(y1-y0);
    int16_t sx = (x0<x1)?1:-1, sy = (y0<y1)?1:-1;
    int16_t err = dx - dy;
    while (1) {
        OLED_SetPixel(x0, y0, 1);
        if (x0==x1 && y0==y1) break;
        int16_t e2 = 2*err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

static void OLED_DrawChar(int16_t x, int16_t y, char ch, uint8_t size) {
    if (ch < ' ' || ch > 'Z') return;
    uint8_t idx = ch - ' ';
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = g_font5x8[idx][i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (1 << j)) {
                if (size == 1) OLED_SetPixel(x+i, y+j, 1);
                else OLED_FillRect(x+i*size, y+j*size, size, size);
            }
        }
    }
}

static void OLED_DrawString(int16_t x, int16_t y, const char *str, uint8_t size) {
    while (*str) {
        OLED_DrawChar(x, y, *str, size);
        x += (size==1) ? 6 : (6*size);
        if (x >= 123) { x = 0; y += (size==1)?8:(8*size); }
        str++;
    }
}

static void OLED_Update(void) {
    OLED_WriteCmd(0x21); OLED_WriteCmd(0); OLED_WriteCmd(127);
    OLED_WriteCmd(0x22); OLED_WriteCmd(0); OLED_WriteCmd(7);
    for (uint16_t i = 0; i < sizeof(g_oledBuffer); i++)
        OLED_WriteData(g_oledBuffer[i]);
}

/* ========== 按键检测 ========== */
static bool Button_IsPressed(GPIO_Regs *port, uint32_t pin) {
    if (DL_GPIO_readPins(port, pin) == 0) {
        Delay_ms(20);
        if (DL_GPIO_readPins(port, pin) == 0) {
            while (DL_GPIO_readPins(port, pin) == 0);
            return true;
        }
    }
    return false;
}

/* ========== 电源控制 ========== */
static void PowerSupply_SetVoltage(float target_mV) {
    if (target_mV < 0) target_mV = 0;
    if (target_mV > DAC_MAX_OUTPUT_MV) target_mV = DAC_MAX_OUTPUT_MV;
    g_ps.targetVoltage_mV = target_mV;
    PID_Reset(&g_pid);
}

/* 主更新函数（10ms调用一次） */
static void PowerSupply_Update(void) {
    /* 读取实际电压和电流 */
    uint16_t adcV = ADC_ReadVoltage();
    uint16_t adcI = ADC_ReadCurrent();

    g_ps.actualVoltage_mV = ADC_ToVoltage_mV(adcV);
    g_ps.outputCurrent_mA = ADC_ToCurrent_mA(adcI);
    g_ps.outputPower_mW = g_ps.actualVoltage_mV * g_ps.outputCurrent_mA / 1000.0f;

    /* 过流保护 */
    if (g_ps.outputCurrent_mA > OVERCURRENT_MA) {
        g_ps.overcurrentFault = true;
        g_ps.outputEnabled = false;
        MCP4921_WriteDAC(0);  /* 立即关闭输出 */
        PID_Reset(&g_pid);
        return;
    }

    /* 过压保护 */
    if (g_ps.actualVoltage_mV > OVERVOLTAGE_MV) {
        g_ps.outputEnabled = false;
        MCP4921_WriteDAC(0);
        PID_Reset(&g_pid);
        return;
    }

    /* PID稳压控制 */
    if (g_ps.outputEnabled) {
        float dacOutput = PID_Compute(&g_pid, g_ps.targetVoltage_mV, g_ps.actualVoltage_mV);
        uint16_t dacCode = (uint16_t)dacOutput;
        if (dacCode > 4095) dacCode = 4095;
        MCP4921_WriteDAC(dacCode);
    } else {
        MCP4921_WriteDAC(0);
    }
}

/* ========== UI显示 ========== */
static void UI_UpdateDisplay(void) {
    char buf[32];
    OLED_Clear();

    /* 标题栏 */
    OLED_DrawString(4, 0, "PRECISION V-SRC", 1);
    OLED_DrawLine(0, 9, 127, 9);

    /* 目标电压 */
    snprintf(buf, sizeof(buf), "SET: %d.%03dV",
             (int)(g_ps.targetVoltage_mV/1000), (int)((int)g_ps.targetVoltage_mV % 1000));
    OLED_DrawString(0, 12, buf, 1);

    /* 实际电压（大号） */
    snprintf(buf, sizeof(buf), "%d.%03d",
             (int)(g_ps.actualVoltage_mV/1000), (int)((int)g_ps.actualVoltage_mV % 1000));
    OLED_DrawString(8, 24, buf, 2);
    OLED_DrawString(88, 28, "V", 2);

    /* 电流 */
    snprintf(buf, sizeof(buf), "I: %dmA", (int)g_ps.outputCurrent_mA);
    OLED_DrawString(0, 44, buf, 1);

    /* 功率 */
    snprintf(buf, sizeof(buf), "P: %dmW", (int)g_ps.outputPower_mW);
    OLED_DrawString(64, 44, buf, 1);

    /* 状态栏 */
    OLED_DrawLine(0, 54, 127, 54);
    if (g_ps.overcurrentFault) {
        OLED_DrawString(0, 56, "FAULT:OVERCURR", 1);
    } else if (g_ps.outputEnabled) {
        OLED_DrawString(0, 56, "OUT:ON", 1);
        /* 步进显示 */
        snprintf(buf, sizeof(buf), "STEP:%dmV", g_stepTable[g_ps.stepIndex]);
        OLED_DrawString(72, 56, buf, 1);
    } else {
        OLED_DrawString(0, 56, "OUT:OFF", 1);
    }

    OLED_Update();
}

/* ========== 主函数 ========== */
int main(void) {
    /* 系统初始化 */
    DL_SYSCTL_initSYSCTL();
    SysTick_Config(SystemCoreClock / 1000);

    /* 外设初始化 */
    DL_I2C_initController(I2C_0_INST, 400000);
    DL_SPI_enable(SPI_0_INST);

    /* GPIO */
    DL_GPIO_initDigitalOutput(MCP4921_CS_PORT | MCP4921_CS_PIN);
    DL_GPIO_initDigitalOutput(MCP4921_LDAC_PORT | MCP4921_LDAC_PIN);
    DL_GPIO_initDigitalInput(BTN_UP_PORT | BTN_UP_PIN);
    DL_GPIO_initDigitalInput(BTN_DOWN_PORT | BTN_DOWN_PIN);
    DL_GPIO_initDigitalInput(BTN_STEP_PORT | BTN_STEP_PIN);
    DL_GPIO_initDigitalInput(BTN_ENABLE_PORT | BTN_ENABLE_PIN);

    /* MCP4921和OLED初始化 */
    MCP4921_Init();
    OLED_Init();

    /* PID初始化 */
    PID_Init(&g_pid, PID_KP, PID_KI, PID_KD);

    /* 电源状态初始化 */
    g_ps.targetVoltage_mV = 2500.0f;  /* 默认2.5V */
    g_ps.outputEnabled = false;
    g_ps.overcurrentFault = false;
    g_ps.stepIndex = 1;  /* 默认10mV步进 */

    /* 开机画面 */
    OLED_Clear();
    OLED_DrawString(16, 20, "Precision V-Source", 1);
    OLED_DrawString(32, 36, "MCP4921+PID", 1);
    OLED_Update();
    Delay_ms(1500);

    uint32_t lastControlTime = 0;
    uint32_t lastDisplayTime = 0;

    while (1) {
        /* 按键处理 */
        if (Button_IsPressed(BTN_UP_PORT, BTN_UP_PIN)) {
            if (g_ps.outputEnabled) {
                PowerSupply_SetVoltage(g_ps.targetVoltage_mV + g_stepTable[g_ps.stepIndex]);
            }
        }
        if (Button_IsPressed(BTN_DOWN_PORT, BTN_DOWN_PIN)) {
            if (g_ps.outputEnabled) {
                PowerSupply_SetVoltage(g_ps.targetVoltage_mV - g_stepTable[g_ps.stepIndex]);
            }
        }
        if (Button_IsPressed(BTN_STEP_PORT, BTN_STEP_PIN)) {
            g_ps.stepIndex = (g_ps.stepIndex + 1) % STEP_TABLE_SIZE;
        }
        if (Button_IsPressed(BTN_ENABLE_PORT, BTN_ENABLE_PIN)) {
            if (g_ps.overcurrentFault) {
                g_ps.overcurrentFault = false;  /* 清除故障 */
            }
            g_ps.outputEnabled = !g_ps.outputEnabled;
            if (!g_ps.outputEnabled) {
                MCP4921_WriteDAC(0);
                PID_Reset(&g_pid);
            }
        }

        /* 10ms PID控制周期 */
        if ((g_systickCount - lastControlTime) >= 10) {
            PowerSupply_Update();
            lastControlTime = g_systickCount;
        }

        /* 200ms显示刷新 */
        if ((g_systickCount - lastDisplayTime) >= 200) {
            UI_UpdateDisplay();
            lastDisplayTime = g_systickCount;
        }
    }
}
