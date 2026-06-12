/**
 * @file dds_signal_generator.c
 * @brief DDS信号发生器 - AD9833 + OLED显示
 * @platform MSPM0G3507
 * @description
 *   基于AD9833 DDS芯片实现可编程信号发生器：
 *   - 正弦波/三角波/方波输出
 *   - 频率范围 0.1Hz ~ 12.5MHz
 *   - 频率分辨率 0.1Hz
 *   - 按键切换波形和频率
 *   - OLED实时显示当前波形参数
 *
 * 硬件连接：
 *   AD9833:  SPI接口 (SCLK, SDATA, FSYNC)
 *   OLED:    I2C接口 (SCL, SDA)
 *   按键:    PA0(波形切换), PA1(频率+), PA2(频率-), PA3(频率步进切换)
 *   LED:     PA27(输出使能指示)
 */

#include "ti_msp_dl_config.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ========== AD9833 DDS 驱动 ========== */

/* AD9833控制寄存器位定义 */
#define AD9833_B28      (1 << 13)   /* 28位频率字传输模式 */
#define AD9833_HLB      (1 << 12)   /* 高/低14位选择 */
#define AD9833_FSELECT  (1 << 11)   /* 频率寄存器选择 */
#define AD9833_PSELECT  (1 << 10)   /* 相位寄存器选择 */
#define AD9833_RESET    (1 << 8)    /* 复位 */
#define AD9833_SLEEP1   (1 << 7)    /* DAC关断 */
#define AD9833_SLEEP12  (1 << 6)    /* 内部时钟关断 */
#define AD9833_OPBITEN  (1 << 5)    /* 方波输出使能 */
#define AD9833_DIV2     (1 << 3)    /* 方波分频 */
#define AD9833_MODE     (1 << 1)    /* 三角波模式 */

#define AD9833_FREQ0_REG  0x4000    /* 频率寄存器0 */
#define AD9833_FREQ1_REG  0x8000    /* 频率寄存器1 */
#define AD9833_PHASE_REG  0xC000    /* 相位寄存器 */

#define AD9833_MCLK       25000000UL  /* 主时钟频率25MHz */
#define AD9833_FREQ_RES   ((double)AD9833_MCLK / (1UL << 28))

/* 波形类型枚举 */
typedef enum {
    WAVE_SINE = 0,      /* 正弦波 */
    WAVE_TRIANGLE,      /* 三角波 */
    WAVE_SQUARE,        /* 方波 */
    WAVE_COUNT
} WaveformType_t;

/* 频率步进档位 */
static const double freq_steps[] = {
    0.1, 1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0, 1000000.0
};
#define FREQ_STEP_COUNT  8

/* 波形名称 */
static const char *wave_names[] = {"Sine", "Triangle", "Square"};

/* 当前信号参数 */
static WaveformType_t current_wave = WAVE_SINE;
static double current_freq = 1000.0;    /* 默认1kHz */
static uint8_t current_step_idx = 3;    /* 默认步进100Hz */
static double current_phase = 0.0;      /* 相位0~360度 */

/* ========== SPI驱动 - AD9833通信 ========== */

/* SPI片选引脚定义 (根据实际硬件修改) */
#define AD9833_FSYNC_PORT   GPIOA
#define AD9833_FSYNC_PIN    DL_GPIO_PIN_15

/**
 * @brief 初始化SPI用于AD9833通信
 */
static void AD9833_SPI_Init(void)
{
    /* 配置FSYNC引脚为GPIO输出模式 */
    DL_GPIO_initDigitalOutput(AD9833_FSYNC_PIN);
    DL_GPIO_setPins(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN);  /* FSYNC高电平(空闲) */
    DL_GPIO_enableOutput(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN);
}

/**
 * @brief 通过SPI发送16位数据到AD9833
 * @param data 16位控制字
 */
static void AD9833_Write16(uint16_t data)
{
    /* 拉低FSYNC开始传输 */
    DL_GPIO_clearPins(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN);

    /* 发送16位数据，MSB先行 */
    for (int i = 15; i >= 0; i--) {
        /* 设置数据位 */
        if (data & (1 << i)) {
            DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_14);   /* SDATA高 */
        } else {
            DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14);  /* SDATA低 */
        }
        /* 时钟脉冲 */
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_13);        /* SCLK高 */
        __NOP(); __NOP(); __NOP(); __NOP();
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_13);      /* SCLK低 */
        __NOP(); __NOP(); __NOP(); __NOP();
    }

    /* 拉高FSYNC结束传输 */
    DL_GPIO_setPins(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN);
}

/**
 * @brief 设置AD9833输出频率
 * @param freq_hz 目标频率(Hz)
 */
static void AD9833_SetFrequency(double freq_hz)
{
    /* 计算频率字: freq_word = freq * 2^28 / MCLK */
    uint32_t freq_word = (uint32_t)(freq_hz * (double)(1UL << 28) / (double)AD9833_MCLK);

    /* 低14位和高14位 */
    uint16_t freq_lsb = (uint16_t)(freq_word & 0x3FFF) | AD9833_FREQ0_REG;
    uint16_t freq_msb = (uint16_t)((freq_word >> 14) & 0x3FFF) | AD9833_FREQ0_REG;

    /* 复位并设置B28模式 */
    AD9833_Write16(AD9833_B28 | AD9833_RESET);
    /* 写入频率字(先低后高) */
    AD9833_Write16(freq_lsb);
    AD9833_Write16(freq_msb);
}

/**
 * @brief 设置AD9833输出相位
 * @param phase_deg 相位角度(0~360度)
 */
static void AD9833_SetPhase(double phase_deg)
{
    /* 相位字 = phase_deg / 360 * 4096 */
    uint16_t phase_word = (uint16_t)(phase_deg / 360.0 * 4096.0) & 0x0FFF;
    AD9833_Write16(phase_word | AD9833_PHASE_REG);
}

/**
 * @brief 设置AD9833输出波形
 * @param wave 波形类型
 */
static void AD9833_SetWaveform(WaveformType_t wave)
{
    uint16_t control = AD9833_B28;

    switch (wave) {
    case WAVE_SINE:
        /* 正弦波: 默认模式 */
        break;
    case WAVE_TRIANGLE:
        /* 三角波: MODE位 */
        control |= AD9833_MODE;
        break;
    case WAVE_SQUARE:
        /* 方波: OPBITEN位 */
        control |= AD9833_OPBITEN | AD9833_DIV2;
        break;
    default:
        break;
    }

    /* 写入控制字并清除复位 */
    AD9833_Write16(control);
}

/**
 * @brief 更新AD9833全部参数
 */
static void AD9833_Update(void)
{
    AD9833_SetFrequency(current_freq);
    AD9833_SetPhase(current_phase);
    AD9833_SetWaveform(current_wave);
}

/* ========== OLED显示驱动 (I2C) ========== */

/* OLED I2C地址 */
#define OLED_ADDR  0x3C

/* 简化的OLED缓冲区 */
static char oled_lines[4][22];  /* 4行，每行21字符 */

/**
 * @brief I2C发送单字节
 */
static void I2C_WriteByte(uint8_t addr, uint8_t reg, uint8_t data)
{
    /* 使用MSPM0 I2C外设发送 */
    DL_I2C_fillControllerTXFIFO(I2C0, &reg, 1);
    DL_I2C_fillControllerTXFIFO(I2C0, &data, 1);
    DL_I2C_startControllerTransfer(I2C0, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C0) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
}

/**
 * @brief 在OLED指定行显示文本
 */
static void OLED_ShowLine(uint8_t line, const char *text)
{
    if (line < 4) {
        snprintf(oled_lines[line], 22, "%-21s", text);
    }
}

/**
 * @brief 刷新OLED显示内容
 * @note 实际项目中应使用OLED驱动库的刷新函数
 */
static void OLED_Refresh(void)
{
    /* 此处调用OLED驱动的刷新函数 */
    /* OLED_FlushBuffer(oled_lines, 4); */
}

/**
 * @brief 更新OLED显示内容
 */
static void Display_Update(void)
{
    char buf[22];

    /* 第一行: 波形类型 */
    snprintf(buf, sizeof(buf), "Wave: %s", wave_names[current_wave]);
    OLED_ShowLine(0, buf);

    /* 第二行: 频率 */
    if (current_freq >= 1000000.0) {
        snprintf(buf, sizeof(buf), "Freq: %.3f MHz", current_freq / 1000000.0);
    } else if (current_freq >= 1000.0) {
        snprintf(buf, sizeof(buf), "Freq: %.3f kHz", current_freq / 1000.0);
    } else {
        snprintf(buf, sizeof(buf), "Freq: %.1f Hz", current_freq);
    }
    OLED_ShowLine(1, buf);

    /* 第三行: 相位 */
    snprintf(buf, sizeof(buf), "Phase: %.1f deg", current_phase);
    OLED_ShowLine(2, buf);

    /* 第四行: 频率步进 */
    double step = freq_steps[current_step_idx];
    if (step >= 1000.0) {
        snprintf(buf, sizeof(buf), "Step: %.0f kHz", step / 1000.0);
    } else {
        snprintf(buf, sizeof(buf), "Step: %.1f Hz", step);
    }
    OLED_ShowLine(3, buf);

    OLED_Refresh();
}

/* ========== 按键驱动 ========== */

/* 按键防抖时间(ms) */
#define DEBOUNCE_MS  200

/* 按键GPIO定义 */
#define KEY_WAVE_PORT   GPIOA
#define KEY_WAVE_PIN    DL_GPIO_PIN_0
#define KEY_FREQ_UP_PORT GPIOA
#define KEY_FREQ_UP_PIN DL_GPIO_PIN_1
#define KEY_FREQ_DN_PORT GPIOA
#define KEY_FREQ_DN_PIN DL_GPIO_PIN_3
#define KEY_STEP_PORT   GPIOA
#define KEY_STEP_PIN    DL_GPIO_PIN_4

/* 上次按键时间戳 */
static volatile uint32_t last_key_time[4] = {0};
static volatile uint32_t sys_tick_ms = 0;

/**
 * @brief SysTick中断处理 - 1ms定时
 */
void SysTick_Handler(void)
{
    sys_tick_ms++;
}

/**
 * @brief 按键扫描与处理
 */
static void Key_ScanProcess(void)
{
    uint32_t now = sys_tick_ms;

    /* 波形切换键 */
    if (DL_GPIO_readPins(KEY_WAVE_PORT, KEY_WAVE_PIN) == 0) {
        if (now - last_key_time[0] > DEBOUNCE_MS) {
            last_key_time[0] = now;
            current_wave = (WaveformType_t)((current_wave + 1) % WAVE_COUNT);
            AD9833_SetWaveform(current_wave);
            Display_Update();
        }
    }

    /* 频率增加键 */
    if (DL_GPIO_readPins(KEY_FREQ_UP_PORT, KEY_FREQ_UP_PIN) == 0) {
        if (now - last_key_time[1] > DEBOUNCE_MS) {
            last_key_time[1] = now;
            current_freq += freq_steps[current_step_idx];
            if (current_freq > 12500000.0) current_freq = 12500000.0;
            AD9833_SetFrequency(current_freq);
            Display_Update();
        }
    }

    /* 频率减少键 */
    if (DL_GPIO_readPins(KEY_FREQ_DN_PORT, KEY_FREQ_DN_PIN) == 0) {
        if (now - last_key_time[2] > DEBOUNCE_MS) {
            last_key_time[2] = now;
            if (current_freq > freq_steps[current_step_idx]) {
                current_freq -= freq_steps[current_step_idx];
            } else {
                current_freq = 0.1;
            }
            AD9833_SetFrequency(current_freq);
            Display_Update();
        }
    }

    /* 步进切换键 */
    if (DL_GPIO_readPins(KEY_STEP_PORT, KEY_STEP_PIN) == 0) {
        if (now - last_key_time[3] > DEBOUNCE_MS) {
            last_key_time[3] = now;
            current_step_idx = (current_step_idx + 1) % FREQ_STEP_COUNT;
            Display_Update();
        }
    }
}

/* ========== GPIO引脚初始化 ========== */

/**
 * @brief 初始化按键GPIO
 */
static void Key_GPIO_Init(void)
{
    /* 配置按键引脚为输入，内部上拉 */
    DL_GPIO_initDigitalInputFeatures(KEY_WAVE_PIN,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_FREQ_UP_PIN,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_FREQ_DN_PIN,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_STEP_PIN,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/**
 * @brief 初始化SPI引脚(SCLK, SDATA)为GPIO输出
 */
static void SPI_GPIO_Init(void)
{
    /* SCLK - PA13 */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_13);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_13);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_13);

    /* SDATA - PA14 */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_14);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_14);
}

/* ========== 预设频率模式 ========== */

/* 常用测试频率预设 */
static const double preset_freqs[] = {
    100.0,      /* 100Hz */
    1000.0,     /* 1kHz */
    10000.0,    /* 10kHz */
    100000.0,   /* 100kHz */
    500000.0,   /* 500kHz */
    1000000.0,  /* 1MHz */
    5000000.0,  /* 5MHz */
    10000000.0  /* 10MHz */
};
#define PRESET_COUNT  8

/* 当前预设索引 */
static uint8_t preset_idx = 1;

/**
 * @brief 应用预设频率
 * @param idx 预设索引
 */
static void Apply_Preset(uint8_t idx)
{
    if (idx < PRESET_COUNT) {
        current_freq = preset_freqs[idx];
        preset_idx = idx;
        AD9833_SetFrequency(current_freq);
        Display_Update();
    }
}

/**
 * @brief 频率扫描模式
 * @param start_hz 起始频率
 * @param stop_hz  结束频率
 * @param points   扫描点数
 * @param delay_ms 每点停留时间(ms)
 */
static void Frequency_Sweep(double start_hz, double stop_hz,
                             uint32_t points, uint32_t delay_ms)
{
    double step = (stop_hz - start_hz) / (double)points;

    for (uint32_t i = 0; i <= points; i++) {
        double freq = start_hz + step * (double)i;
        AD9833_SetFrequency(current_freq);

        char buf[22];
        snprintf(buf, sizeof(buf), "Sweep: %.1f Hz", freq);
        OLED_ShowLine(1, buf);
        OLED_Refresh();

        /* 延时 */
        for (volatile uint32_t d = 0; d < delay_ms * 1000; d++) {}
    }

    /* 恢复原始频率 */
    AD9833_SetFrequency(current_freq);
    Display_Update();
}

/* ========== 主函数 ========== */

int main(void)
{
    /* 系统初始化 */
    DL_SYSCFG_init();
    SysTick_Config(32000000 / 1000);  /* 1ms SysTick */

    /* GPIO初始化 */
    SPI_GPIO_Init();
    Key_GPIO_Init();
    AD9833_SPI_Init();

    /* LED指示灯 */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_27);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_27);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27);  /* 点亮LED */

    /* I2C初始化 */
    DL_I2C_reset(I2C0);
    DL_I2C_enablePower(I2C0);
    DL_I2C_setClockConfig(I2C0, DL_I2C_CLOCK_DIVIDE_100KHZ);
    DL_I2C_enableController(I2C0);

    /* 初始AD9833配置 */
    AD9833_Update();

    /* 初始OLED显示 */
    Display_Update();

    /* 主循环 */
    while (1) {
        /* 按键扫描 */
        Key_ScanProcess();

        /* 低功耗等待 */
        __WFI();
    }
}
