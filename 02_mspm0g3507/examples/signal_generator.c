/**
 * @file signal_generator.c
 * @brief MSPM0G3507 信号发生器示例
 *
 * 硬件连接（MCP4725 12位DAC模块）：
 *   SDA -> PB0 (I2C0 SDA)
 *   SCL -> PB1 (I2C0 SCL)
 *   ADDR-> GND (I2C地址 0x60)
 *   VOUT-> 示波器探头 / DAC输出端
 *
 *   按键：
 *     波形切换 -> PA3 (正弦/方波/三角/锯齿)
 *     频率增加 -> PA4
 *     频率减少 -> PA5
 *     幅度调节 -> PA6
 *
 *   OLED显示（I2C，地址0x3C）：
 *     SDA -> PB0 (共享I2C总线)
 *     SCL -> PB1
 *
 * 功能：
 *   - 4种波形：正弦波、方波、三角波、锯齿波
 *   - 频率可调：1Hz ~ 10kHz
 *   - 幅度可调：0 ~ 3.3V（12位分辨率）
 *   - 可设置直流偏置
 *   - DAC输出通过MCP4725 I2C接口
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ========== MCP4725 I2C配置 ========== */
#define MCP4725_ADDR        0x60  /* ADDR接GND时的地址 */
#define MCP4725_CMD_WRITE   0x40  /* 写DAC寄存器 */
#define MCP4725_CMD_FAST    0x00  /* 快速写命令 */

/* 波形类型枚举 */
typedef enum {
    WAVE_SINE = 0,    /* 正弦波 */
    WAVE_SQUARE,      /* 方波 */
    WAVE_TRIANGLE,    /* 三角波 */
    WAVE_SAWTOOTH,    /* 锯齿波 */
    WAVE_DC,          /* 直流输出 */
    WAVE_COUNT        /* 波形数量 */
} WaveformType_t;

/* 波形名称 */
static const char *wave_names[] = {
    "Sine", "Square", "Triangle", "Sawtooth", "DC"
};

/* 波形中文名 */
static const char *wave_names_cn[] = {
    "正弦波", "方波", "三角波", "锯齿波", "直流"
};

/* ========== 预计算波形查表（256点/周期）========== */
#define WAVE_TABLE_SIZE  256
static uint16_t sine_table[WAVE_TABLE_SIZE];
static uint16_t square_table[WAVE_TABLE_SIZE];
static uint16_t triangle_table[WAVE_TABLE_SIZE];
static uint16_t sawtooth_table[WAVE_TABLE_SIZE];

/* ========== 全局参数 ========== */
static WaveformType_t g_waveform = WAVE_SINE;
static float g_frequency = 1000.0f;    /* 频率Hz */
static float g_amplitude = 1.65f;      /* 幅度V（峰峰值的一半） */
static float g_offset = 1.65f;         /* 直流偏置V */
static uint32_t g_phase_step = 0;      /* 相位步进值（DDS） */
static uint32_t g_phase_acc = 0;       /* 相位累加器 */
static volatile bool g_output_enable = true;

/* 频率预设值表 */
static const float freq_presets[] = {
    1.0f, 10.0f, 50.0f, 100.0f, 200.0f, 500.0f,
    1000.0f, 2000.0f, 5000.0f, 10000.0f
};
#define NUM_FREQ_PRESETS  10
static uint8_t g_freq_idx = 6;  /* 默认1kHz */

/* I2C句柄 */
extern I2C_Regs *g_i2c;

/* ========== DDS参数计算 ========== */
/* DDS主时钟频率（定时器中断频率） */
#define DDS_CLOCK_FREQ  100000.0f  /* 100kHz采样率 */
#define DDS_PHASE_BITS  32

/* 计算相位步进值 */
static void dds_update_phase_step(void)
{
    /* phase_step = freq * 2^32 / clock_freq */
    g_phase_step = (uint32_t)(g_frequency * 4294967296.0f / DDS_CLOCK_FREQ);
}

/* ========== 波形表初始化 ========== */
static void generate_wave_tables(void)
{
    for (int i = 0; i < WAVE_TABLE_SIZE; i++) {
        float phase = (float)i / WAVE_TABLE_SIZE * 2.0f * 3.14159265f;

        /* 正弦波：sin(x)映射到0~4095 */
        float sin_val = (sinf(phase) + 1.0f) * 0.5f;  /* 0~1 */
        sine_table[i] = (uint16_t)(sin_val * 4095.0f);

        /* 方波：前半周期高电平，后半低电平 */
        square_table[i] = (i < WAVE_TABLE_SIZE / 2) ? 4095 : 0;

        /* 三角波：线性上升再下降 */
        if (i < WAVE_TABLE_SIZE / 2) {
            triangle_table[i] = (uint16_t)((float)i / (WAVE_TABLE_SIZE / 2) * 4095.0f);
        } else {
            triangle_table[i] = (uint16_t)((1.0f - (float)(i - WAVE_TABLE_SIZE / 2) / (WAVE_TABLE_SIZE / 2)) * 4095.0f);
        }

        /* 锯齿波：线性上升 */
        sawtooth_table[i] = (uint16_t)((float)i / WAVE_TABLE_SIZE * 4095.0f);
    }
}

/* ========== MCP4725驱动 ========== */
static bool mcp4725_write_dac(uint16_t value)
{
    uint8_t buf[3];
    /* 快速模式写入：2字节数据 */
    buf[0] = MCP4725_CMD_FAST | ((value >> 8) & 0x0F);  /* 高4位 */
    buf[1] = value & 0xFF;                                /* 低8位 */

    DL_I2C_fillControllerTXFIFO(g_i2c, buf, 2);
    DL_I2C_startControllerTransfer(g_i2c, MCP4725_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(g_i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    DL_I2C_flushControllerTXFIFO(g_i2c);
    return true;
}

/* 写DAC值并保存到EEPROM（掉电保存） */
static bool mcp4725_write_and_save(uint16_t value)
{
    uint8_t buf[3];
    buf[0] = 0x60;  /* 写DAC+EEPROM命令 */
    buf[1] = (value >> 4) & 0xFF;   /* 高8位 */
    buf[2] = (value << 4) & 0xF0;   /* 低4位左对齐 */

    DL_I2C_fillControllerTXFIFO(g_i2c, buf, 3);
    DL_I2C_startControllerTransfer(g_i2c, MCP4725_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 3);
    while (DDS_CLOCK_FREQ) {}  /* 等待 */
    DL_I2C_flushControllerTXFIFO(g_i2c);
    return true;
}

/* 读取DAC当前值和状态 */
static bool mcp4725_read(uint16_t *dac_val, uint16_t *eeprom_val)
{
    uint8_t rx_buf[5];

    DL_I2C_startControllerTransfer(g_i2c, MCP4725_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 5);
    while (DL_I2C_getControllerStatus(g_i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}

    for (int i = 0; i < 5; i++) {
        rx_buf[i] = DL_I2C_receiveControllerData(g_i2c);
    }

    /* 解析：字节0=状态，字节1-2=DAC数据，字节3-4=EEPROM数据 */
    if (dac_val) {
        *dac_val = ((uint16_t)(rx_buf[1] & 0x0F) << 8) | rx_buf[2];
    }
    if (eeprom_val) {
        *eeprom_val = ((uint16_t)(rx_buf[3] & 0x0F) << 8) | rx_buf[4];
    }
    return true;
}

/* ========== DDS信号生成 ========== */
/* 根据当前波形和相位获取DAC值 */
static uint16_t get_dac_value(void)
{
    /* 相位累加器高8位作为查表索引 */
    uint8_t index = (uint8_t)(g_phase_acc >> 24);
    uint16_t raw_value;

    switch (g_waveform) {
        case WAVE_SINE:
            raw_value = sine_table[index];
            break;
        case WAVE_SQUARE:
            raw_value = square_table[index];
            break;
        case WAVE_TRIANGLE:
            raw_value = triangle_table[index];
            break;
        case WAVE_SAWTOOTH:
            raw_value = sawtooth_table[index];
            break;
        case WAVE_DC:
            raw_value = (uint16_t)(g_offset / 3.3f * 4095.0f);
            break;
        default:
            raw_value = 2048;
            break;
    }

    /* 应用幅度和偏置 */
    if (g_waveform != WAVE_DC) {
        float normalized = (float)raw_value / 4095.0f;  /* 0~1 */
        float voltage = g_amplitude * 2.0f * (normalized - 0.5f) + g_offset;
        /* 限幅 */
        if (voltage < 0.0f) voltage = 0.0f;
        if (voltage > 3.3f) voltage = 3.3f;
        raw_value = (uint16_t)(voltage / 3.3f * 4095.0f);
    }

    return raw_value;
}

/* ========== 定时器中断处理（DDS核心）========== */
void TIMER_0_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_0) == DL_TIMER_IIDX_ZERO) {
        if (g_output_enable) {
            /* 更新相位累加器 */
            g_phase_acc += g_phase_step;

            /* 获取并输出DAC值 */
            uint16_t dac_val = get_dac_value();
            mcp4725_write_dac(dac_val);
        }
    }
}

/* ========== 按键扫描 ========== */
static uint8_t scan_buttons(void)
{
    static uint8_t last = 0xFF;
    uint8_t cur = 0;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_3)) cur |= 0x01;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_4)) cur |= 0x02;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_5)) cur |= 0x04;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_6)) cur |= 0x08;
    uint8_t pressed = (~cur) & last;
    last = cur;
    if (pressed & 0x01) return 1;
    if (pressed & 0x02) return 2;
    if (pressed & 0x04) return 3;
    if (pressed & 0x08) return 4;
    return 0;
}

/* ========== 频率显示格式化 ========== */
static void format_frequency(float freq, char *buf, int buf_size)
{
    if (freq >= 1000.0f) {
        /* kHz */
        int len = 0;
        int khz = (int)(freq / 1000.0f);
        int hz = (int)freq % 1000;
        /* 手动格式化避免sprintf */
        if (khz >= 10) {
            buf[len++] = '0' + (khz / 10);
        }
        buf[len++] = '0' + (khz % 10);
        buf[len++] = '.';
        buf[len++] = '0' + (hz / 100);
        buf[len++] = 'k';
        buf[len++] = 'H';
        buf[len++] = 'z';
        buf[len] = '\0';
    } else {
        int hz = (int)freq;
        int len = 0;
        if (hz >= 100) buf[len++] = '0' + (hz / 100);
        if (hz >= 10)  buf[len++] = '0' + ((hz / 10) % 10);
        buf[len++] = '0' + (hz % 10);
        buf[len++] = 'H';
        buf[len++] = 'z';
        buf[len] = '\0';
    }
}

/* ========== 幅度显示格式化 ========== */
static void format_amplitude(float amp, char *buf)
{
    /* amp是半幅值，显示峰峰值 */
    float vpp = amp * 2.0f;
    int whole = (int)vpp;
    int frac = (int)((vpp - whole) * 10);
    int len = 0;
    buf[len++] = '0' + whole;
    buf[len++] = '.';
    buf[len++] = '0' + frac;
    buf[len++] = 'V';
    buf[len++] = 'p';
    buf[len++] = 'p';
    buf[len] = '\0';
}

/* ========== 主函数 ========== */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    /* 生成波形查找表 */
    generate_wave_tables();

    /* 计算DDS相位步进 */
    dds_update_phase_step();

    /* 按键GPIO初始化 */
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_3,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_4,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_5,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_6,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 状态LED */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_8);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8);

    /* 启动定时器（DDS采样时钟） */
    /* SYSCFG中配置Timer0为周期中断，频率=DDS_CLOCK_FREQ */
    NVIC_EnableIRQ(TIMER_0_IRQN);

    /* 主循环：处理按键和显示 */
    while (1) {
        uint8_t btn = scan_buttons();

        switch (btn) {
            case 1:  /* 波形切换 */
                g_waveform = (WaveformType_t)((g_waveform + 1) % WAVE_COUNT);
                /* LED闪烁指示波形切换 */
                DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);
                delay_cycles(50 * 32000);
                DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8);
                break;

            case 2:  /* 频率增加 */
                if (g_freq_idx < NUM_FREQ_PRESETS - 1) {
                    g_freq_idx++;
                    g_frequency = freq_presets[g_freq_idx];
                    dds_update_phase_step();
                }
                break;

            case 3:  /* 频率减少 */
                if (g_freq_idx > 0) {
                    g_freq_idx--;
                    g_frequency = freq_presets[g_freq_idx];
                    dds_update_phase_step();
                }
                break;

            case 4:  /* 幅度切换 */
                /* 循环切换幅度：满幅 -> 3/4 -> 1/2 -> 1/4 */
                {
                    static uint8_t amp_idx = 0;
                    amp_idx = (amp_idx + 1) % 4;
                    switch (amp_idx) {
                        case 0: g_amplitude = 1.65f; break;  /* 3.3Vpp */
                        case 1: g_amplitude = 1.24f; break;  /* 2.48Vpp */
                        case 2: g_amplitude = 0.825f; break; /* 1.65Vpp */
                        case 3: g_amplitude = 0.4125f; break; /* 0.825Vpp */
                    }
                }
                break;
        }

        /* 显示更新（每200ms） */
        static uint32_t display_counter = 0;
        display_counter++;
        if (display_counter >= 200) {
            display_counter = 0;

            /* 格式化显示信息 */
            char freq_buf[16];
            char amp_buf[16];
            format_frequency(g_frequency, freq_buf, sizeof(freq_buf));
            format_amplitude(g_amplitude, amp_buf);

            /* 实际项目中通过OLED或UART显示：
             * 第1行：波形名称（正弦波/方波/三角波/锯齿波）
             * 第2行：频率 xxxHz 或 x.xkHz
             * 第3行：幅度 x.xVpp
             * 第4行：偏置 x.xV
             */

            /* LED状态指示 */
            DL_GPIO_togglePins(GPIOA, DL_GPIO_PIN_8);
        }

        delay_cycles(1000 * 32);  /* 1ms延时 */
    }
}
